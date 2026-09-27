#!/usr/bin/env python3
"""Live use_sim_time + /clock activation proof through the native node clock.

Proves the native rclcpp TimeSource -- not any product Python TimeSource --
owns sim-time under direct_cpp, and that the outcome matches unchanged stock
rclpy exactly.
"""

import importlib
import json
import os
import subprocess
import sys
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp", interfaces=("rosgraph_msgs/msg/Clock",))

import rclpy  # noqa: E402
from rclcppyy.policy import BackendUnavailableError  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from rclpy.time_source import TimeSource  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("conversion or serialization entered clock inspection")


kit = importlib.import_module("rclcpp_kit")
bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
serialization = importlib.import_module("rclcpp_kit.serialization")
rclpy_serialization = importlib.import_module("rclpy.serialization")
kit.convert_python_msg_to_cpp = forbidden_boundary
bringup.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary
rclpy_serialization.serialize_message = forbidden_boundary
rclpy_serialization.deserialize_message = forbidden_boundary


EXPECTED_NS = 12_000_000_345
TIMEOUT_S = 5.0

rclpy.init(args=[])
executor = SingleThreadedExecutor()
sim_node = Node(
    "direct_sim_time_%d" % os.getpid(),
    parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
)
pub_node = Node("direct_sim_time_clock_pub_%d" % os.getpid())
executor.add_node(sim_node)
executor.add_node(pub_node)

qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
clock_publisher = pub_node.create_publisher(Clock, "/clock", qos)
message = Clock()
message.clock.sec = 12
message.clock.nanosec = 345

clock = sim_node.get_clock()
# use_sim_time via parameter_overrides activates synchronously at node
# construction (matching stock exactly) -- no /clock message is needed to
# flip ros_time_is_active, only to advance now() off its zero starting value.
assert clock.ros_time_is_active

deadline = time.monotonic() + TIMEOUT_S
while clock.now().nanoseconds != EXPECTED_NS and time.monotonic() < deadline:
    clock_publisher.publish(message)
    executor.spin_once(timeout_sec=0.05)
if clock.now().nanoseconds != EXPECTED_NS:
    raise AssertionError("direct_cpp sim time never reached the published value")
assert clock.ros_time_is_active
print("DIRECT_CPP_SIM_TIME_ACTIVATION_OK")

active_before_deactivate = clock.ros_time_is_active
ns_before_deactivate = clock.now().nanoseconds

# The Python TimeSource class stays fail-closed: it is not patched, not
# needed, and not implemented for direct_cpp. Attaching it to a direct_cpp
# node is a mechanical no-op (no clock is ever attached to it, so its own
# competing /clock subscription drives nothing) but attaching a clock to it
# explicitly hits the same fail-closed set_ros_time_override every other
# manual-override path raises.
stock_time_source = TimeSource(node=sim_node)
try:
    stock_time_source.attach_clock(clock)
except BackendUnavailableError:
    pass
else:
    raise AssertionError("stock TimeSource.attach_clock succeeded on a direct clock")
finally:
    stock_time_source.detach_node()
print("DIRECT_CPP_SIM_TIME_SOURCE_FAIL_CLOSED_OK")

sim_node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, False)])
deadline = time.monotonic() + TIMEOUT_S
while clock.ros_time_is_active and time.monotonic() < deadline:
    executor.spin_once(timeout_sec=0.05)
if clock.ros_time_is_active:
    raise AssertionError("direct_cpp sim time never deactivated")
assert clock.now().nanoseconds > EXPECTED_NS
print("DIRECT_CPP_SIM_TIME_DEACTIVATION_OK")

direct_observations = {
    "active_before_deactivate": active_before_deactivate,
    "ns_before_deactivate": ns_before_deactivate,
    "active_after_deactivate": clock.ros_time_is_active,
    "ns_after_deactivate": clock.now().nanoseconds,
}

pub_node.destroy_publisher(clock_publisher)
sim_node.destroy_node()
pub_node.destroy_node()
rclpy.shutdown()
print("DIRECT_CPP_SIM_TIME_TEARDOWN_OK")

# Only spawn the stock peer after the direct_cpp scenario has fully torn
# down: sim-time's "/clock" topic name is fixed and process-wide, so the two
# scenarios must not run concurrently on the same domain.
peer_path = os.path.join(os.path.dirname(__file__), "_stock_sim_time_peer.py")
peer = subprocess.run(
    [sys.executable, peer_path],
    capture_output=True,
    text=True,
    timeout=30,
    env=os.environ.copy(),
)
if peer.returncode != 0:
    raise AssertionError(
        "stock sim-time peer failed:\nstdout=%s\nstderr=%s" %
        (peer.stdout, peer.stderr))
marker = "STOCK_SIM_TIME_OBSERVATIONS "
line = next(
    (entry for entry in peer.stdout.splitlines() if entry.startswith(marker)),
    None)
if line is None:
    raise AssertionError("stock sim-time peer produced no observations")
stock_observations = json.loads(line[len(marker):])

assert (
    direct_observations["active_before_deactivate"]
    == stock_observations["active_before_deactivate"] is True
)
assert (
    direct_observations["ns_before_deactivate"]
    == stock_observations["ns_before_deactivate"] == EXPECTED_NS
)
assert (
    direct_observations["active_after_deactivate"]
    == stock_observations["active_after_deactivate"] is False
)
assert direct_observations["ns_after_deactivate"] > EXPECTED_NS
assert stock_observations["ns_after_deactivate"] > EXPECTED_NS
print("DIRECT_CPP_SIM_TIME_STOCK_DIFFERENTIAL_OK")
