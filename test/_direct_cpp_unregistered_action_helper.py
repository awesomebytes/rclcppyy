#!/usr/bin/env python3
"""An installed but unregistered action fails before graph creation."""

import os

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclcppyy_test_interfaces.action import Accumulate  # noqa: E402


rclpy.init(args=[])
node = Node("direct_unregistered_action_%d" % os.getpid())
before = len(node._action_clients)
try:
    ActionClient(node, Accumulate, "/direct_cpp/unregistered_action")
except TypeError as exc:
    assert "not registered" in str(exc)
else:
    raise AssertionError("direct_cpp accepted an unregistered installed action")
assert len(node._action_clients) == before
node.destroy_node()
rclpy.shutdown()
print("DIRECT_CPP_UNREGISTERED_ACTION_FAIL_CLOSED_OK")
