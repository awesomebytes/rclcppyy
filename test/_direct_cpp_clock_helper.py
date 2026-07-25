#!/usr/bin/env python3
"""Live native-node-clock proof for direct_cpp's Node.get_clock()."""

import importlib
import os
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from rclcppyy.direct_clock import DirectClock  # noqa: E402
from rclcppyy.policy import BackendUnavailableError  # noqa: E402
from rclpy.clock import Clock, ROSClock  # noqa: E402
from rclpy.clock_type import ClockType  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.time import Time  # noqa: E402


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


rclpy.init(args=[])
node = Node("direct_clock_%d" % os.getpid())

clk = node.get_clock()
assert isinstance(clk, Clock)
assert isinstance(clk, ROSClock)
assert clk.clock_type == ClockType.ROS_TIME
assert node.get_clock() is clk
print("DIRECT_CPP_CLOCK_IDENTITY_OK")

before_ns = time.time_ns()
first = clk.now()
second = clk.now()
after_ns = time.time_ns()
assert type(first) is Time
assert type(second) is Time
assert second.nanoseconds >= first.nanoseconds
# Node clocks start on wall time (RCL_ROS_TIME with no /clock override yet):
# generous bracket, since spinning up cppyy/rclcpp can itself take a moment.
assert before_ns - 2_000_000_000 <= first.nanoseconds <= after_ns + 2_000_000_000
seconds, nanoseconds = first.seconds_nanoseconds()
assert seconds * 1_000_000_000 + nanoseconds == first.nanoseconds
print("DIRECT_CPP_CLOCK_STOCK_DIFFERENTIAL_OK")

raw_clock = node._direct_cpp_node.get_clock()
assert clk._native_node_clock.address == cppyy.addressof(raw_clock)
del raw_clock
print("DIRECT_CPP_CLOCK_NATIVE_IDENTITY_OK")


def expect_unsupported(operation):
    try:
        operation()
    except BackendUnavailableError:
        pass
    else:
        raise AssertionError("unsupported direct clock operation succeeded")


expect_unsupported(lambda: clk.set_ros_time_override(first))
expect_unsupported(lambda: clk.handle)
# Standalone construction is fail-closed on DirectClock itself -- the stock
# rclpy.clock.Clock name stays unpatched (finding #2: Clock/Duration/
# ROSClock/ClockType/TimeSource are not part of the direct_cpp facade), so a
# plain Clock(...) call legitimately keeps constructing the ordinary stock
# clock and is not part of this surface at all.
expect_unsupported(lambda: DirectClock(clock_type=ClockType.ROS_TIME))
print("DIRECT_CPP_CLOCK_FAIL_CLOSED_OK")

node.destroy_node()
try:
    clk.now()
except RuntimeError as exc:
    assert "closed" in str(exc)
else:
    raise AssertionError("destroyed node left its clock open")
rclpy.shutdown()
print("DIRECT_CPP_CLOCK_TEARDOWN_OK")
