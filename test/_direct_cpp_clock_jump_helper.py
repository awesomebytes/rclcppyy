#!/usr/bin/env python3
"""Live proof: DirectClock.create_jump_callback over native pre/post callbacks."""

import importlib
import os
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp", interfaces=("rosgraph_msgs/msg/Clock",))

import rclpy  # noqa: E402
from rclpy.clock import ClockChange, JumpThreshold, TimeJump  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("conversion or serialization entered clock jump inspection")


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


def _settle(predicate, executor):
    """Spin a bit past a jump becoming observable via ``now()``/``ros_time_
    is_active`` until ``predicate()`` also holds: the native jump-callback
    dispatch (post_callback in particular, over the /clock subscription
    path) is not guaranteed to have completed the instant the clock's own
    value updates -- give it a bounded grace window instead of asserting
    the same instant, matching the suite-level proof's own settle sleep."""
    deadline = time.monotonic() + TIMEOUT_S
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        time.sleep(0.005)
    return predicate()


rclpy.init(args=[])
executor = SingleThreadedExecutor()
node = Node("direct_clock_jump_%d" % os.getpid())
pub_node = Node("direct_clock_jump_pub_%d" % os.getpid())
executor.add_node(node)
executor.add_node(pub_node)

clk = node.get_clock()
assert not clk.ros_time_is_active

pre_calls = []
post_calls = []


def pre_callback():
    pre_calls.append(True)


def post_callback(time_jump):
    assert isinstance(time_jump, TimeJump)
    post_calls.append((time_jump.clock_change, time_jump.delta.nanoseconds))


threshold = JumpThreshold(
    min_forward=Duration(nanoseconds=1), min_backward=None, on_clock_change=True)
handle = clk.create_jump_callback(
    threshold, pre_callback=pre_callback, post_callback=post_callback)

node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
deadline = time.monotonic() + TIMEOUT_S
while not clk.ros_time_is_active and time.monotonic() < deadline:
    executor.spin_once(timeout_sec=0.05)
assert clk.ros_time_is_active
assert _settle(lambda: len(pre_calls) == len(post_calls), executor)
assert len(post_calls) >= 1
assert post_calls[-1][0] == ClockChange.ROS_TIME_ACTIVATED
print("DIRECT_CPP_CLOCK_JUMP_ACTIVATED_OK")

qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
clock_publisher = pub_node.create_publisher(Clock, "/clock", qos)
message = Clock()
message.clock.sec = 12
message.clock.nanosec = 345
deadline = time.monotonic() + TIMEOUT_S
while clk.now().nanoseconds != EXPECTED_NS and time.monotonic() < deadline:
    clock_publisher.publish(message)
    executor.spin_once(timeout_sec=0.05)
assert clk.now().nanoseconds == EXPECTED_NS
assert _settle(lambda: len(pre_calls) == len(post_calls), executor)
assert post_calls[-1][0] == ClockChange.ROS_TIME_NO_CHANGE
assert post_calls[-1][1] == EXPECTED_NS
print("DIRECT_CPP_CLOCK_JUMP_FORWARD_OK")

pre_count = len(pre_calls)
post_count = len(post_calls)
handle.unregister()
node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, False)])
deadline = time.monotonic() + TIMEOUT_S
while clk.ros_time_is_active and time.monotonic() < deadline:
    executor.spin_once(timeout_sec=0.05)
assert not clk.ros_time_is_active
assert len(pre_calls) == pre_count
assert len(post_calls) == post_count
print("DIRECT_CPP_CLOCK_JUMP_UNREGISTER_OK")

# A raising post_callback must not crash the process: it is contained at the
# node's exception sink and re-raised by the executor's own spin_once() call
# (matching every other native-dispatched callback, defect A), not allowed to
# cross back into C++ as an uncaught exception on the native worker thread.
raise_count = [0]


def raising_post_callback(time_jump):
    raise_count[0] += 1
    raise RuntimeError("boom from jump post_callback")


threshold2 = JumpThreshold(min_forward=None, min_backward=None, on_clock_change=True)
handle2 = clk.create_jump_callback(threshold2, post_callback=raising_post_callback)

node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
deadline = time.monotonic() + TIMEOUT_S
caught = None
while time.monotonic() < deadline:
    try:
        executor.spin_once(timeout_sec=0.05)
    except RuntimeError as exc:
        caught = exc
        break
    if clk.ros_time_is_active:
        break
if caught is None:
    raise AssertionError("raising jump post_callback exception was never observed")
assert "boom from jump post_callback" in str(caught)
assert raise_count[0] >= 1
assert clk.ros_time_is_active
handle2.unregister()
print("DIRECT_CPP_CLOCK_JUMP_EXCEPTION_CONTAINED_OK")

pub_node.destroy_publisher(clock_publisher)
node.destroy_node()
pub_node.destroy_node()
rclpy.shutdown()
print("DIRECT_CPP_CLOCK_JUMP_TEARDOWN_OK")
