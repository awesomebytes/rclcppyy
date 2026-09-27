#!/usr/bin/env python3
"""Live proof that Time/Duration to_msg/from_msg keep C++ builtin_interfaces
payloads under direct_cpp, with no Python-message or serialization boundary.
"""

import importlib
import os

import rclcppyy


rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp", interfaces=("std_msgs/msg/Header",))

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.time import Time  # noqa: E402
from std_msgs.msg import Header  # noqa: E402


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError(
        "Python conversion or serialization entered a Time/Duration payload")


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


CPP_TIME_TYPE = cppyy.gbl.builtin_interfaces.msg.Time_["std::allocator<void>"]
CPP_DURATION_TYPE = (
    cppyy.gbl.builtin_interfaces.msg.Duration_["std::allocator<void>"])
assert Header is cppyy.gbl.std_msgs.msg.Header_["std::allocator<void>"]

rclpy.init(args=[])
node = Node("direct_time_%d" % os.getpid())

t = node.get_clock().now()
message = t.to_msg()
assert type(message) is CPP_TIME_TYPE
seconds, nanoseconds = t.seconds_nanoseconds()
assert message.sec == seconds
assert message.nanosec == nanoseconds
print("DIRECT_CPP_TIME_TO_MSG_OK")

round_tripped = Time.from_msg(message, clock_type=t.clock_type)
assert type(round_tripped) is Time
assert round_tripped == t
print("DIRECT_CPP_TIME_FROM_MSG_OK")

duration = Duration(seconds=1, nanoseconds=500)
duration_message = duration.to_msg()
assert type(duration_message) is CPP_DURATION_TYPE
duration_seconds, duration_nanoseconds = divmod(duration.nanoseconds, 1_000_000_000)
assert duration_message.sec == duration_seconds
assert duration_message.nanosec == duration_nanoseconds
round_tripped_duration = Duration.from_msg(duration_message)
assert type(round_tripped_duration) is Duration
assert round_tripped_duration.nanoseconds == duration.nanoseconds
print("DIRECT_CPP_DURATION_MSG_ROUNDTRIP_OK")

# A real app timestamp path: assign the payload into a generated C++
# message field and confirm it never becomes a Python message.
header = Header()
header.stamp = message
assert type(header.stamp) is CPP_TIME_TYPE
assert header.stamp.sec == seconds
assert header.stamp.nanosec == nanoseconds
print("DIRECT_CPP_TIME_HEADER_FIELD_OK")

node.destroy_node()
rclpy.shutdown()
print("DIRECT_CPP_TIME_NO_CONVERSION_OK")
