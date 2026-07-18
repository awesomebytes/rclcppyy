#!/usr/bin/env python3
"""Prove unsupported runtime activation has no patch or import side effect."""

from std_msgs.msg import String
from rclpy.executors import Executor
from rclpy.node import Node

import rclcppyy
from rclcppyy import monkey
from rclcppyy.policy import BackendUnavailableError


try:
    rclcppyy.enable_cpp_acceleration(profile="message_facade")
except BackendUnavailableError as exception:
    assert "rmw_cyclonedds_cpp" in str(exception)
else:
    raise AssertionError("unsupported middleware activated message_facade")

from std_msgs.msg import String as after_failure  # noqa: E402

assert after_failure is String
assert Node.create_publisher is monkey._original_create_publisher
assert Executor._take_subscription is monkey._original_take_subscription
records = rclcppyy.status()["operations"]
assert records[-1]["backend"] == "unsupported"
print("MESSAGE_FACADE_GATE_OK")
