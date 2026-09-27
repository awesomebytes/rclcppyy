#!/usr/bin/env python3
"""A class imported before activation remains correct and visibly stock."""

from std_msgs.msg import String as stale_string

import rclcppyy

rclcppyy.enable_cpp_acceleration(profile="message_facade")

import rclpy  # noqa: E402
from rclpy.context import Context  # noqa: E402
from std_msgs.msg import String as late_string  # noqa: E402


assert stale_string is not late_string
context = Context()
context.init(args=[])
node = rclpy.create_node("facade_stale_import", context=context)
stale_publisher = node.create_publisher(stale_string, "stale", 10)
direct_publisher = node.create_publisher(late_string, "direct", 10)
assert not hasattr(stale_publisher, "_rclcppyy_publish_route")
assert hasattr(direct_publisher, "_rclcppyy_publish_route")
records = rclcppyy.status()["entities"]
assert any(
    item["backend"] == "python" and
    "no active certified C++ facade" in item["reason"]
    for item in records
)
assert node.destroy_publisher(stale_publisher)
assert node.destroy_publisher(direct_publisher)
node.destroy_node()
context.shutdown()
print("MESSAGE_FACADE_STALE_IMPORT_OK")
