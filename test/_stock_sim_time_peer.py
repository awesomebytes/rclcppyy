#!/usr/bin/env python3
"""Unchanged stock rclpy sim-time scenario, for the direct_cpp differential.

Deliberately does not import rclcppyy: this proves what stock rclpy does on
its own, as the reference the direct_cpp scenario is compared against. Run
only after the direct_cpp scenario in the same test has fully torn down --
both use the process-wide, non-configurable "/clock" topic name.
"""

import json
import os
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


EXPECTED_NS = 12_000_000_345
TIMEOUT_S = 5.0

rclpy.init(args=[])
executor = SingleThreadedExecutor()
sim_node = Node(
    "stock_sim_time_%d" % os.getpid(),
    parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
)
pub_node = Node("stock_sim_time_clock_pub_%d" % os.getpid())
executor.add_node(sim_node)
executor.add_node(pub_node)

qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
clock_publisher = pub_node.create_publisher(Clock, "/clock", qos)
message = Clock()
message.clock.sec = 12
message.clock.nanosec = 345

clock = sim_node.get_clock()
deadline = time.monotonic() + TIMEOUT_S
while clock.now().nanoseconds != EXPECTED_NS and time.monotonic() < deadline:
    clock_publisher.publish(message)
    executor.spin_once(timeout_sec=0.05)
if clock.now().nanoseconds != EXPECTED_NS:
    raise AssertionError("stock sim time never reached the published value")

active_before_deactivate = clock.ros_time_is_active
ns_before_deactivate = clock.now().nanoseconds

sim_node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, False)])
deadline = time.monotonic() + TIMEOUT_S
while clock.ros_time_is_active and time.monotonic() < deadline:
    executor.spin_once(timeout_sec=0.05)
if clock.ros_time_is_active:
    raise AssertionError("stock sim time never deactivated")

observations = {
    "active_before_deactivate": active_before_deactivate,
    "ns_before_deactivate": ns_before_deactivate,
    "active_after_deactivate": clock.ros_time_is_active,
    "ns_after_deactivate": clock.now().nanoseconds,
}

pub_node.destroy_publisher(clock_publisher)
sim_node.destroy_node()
pub_node.destroy_node()
rclpy.shutdown()
print("STOCK_SIM_TIME_OBSERVATIONS " + json.dumps(observations))
