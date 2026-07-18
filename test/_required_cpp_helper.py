#!/usr/bin/env python3
"""Required-C++ policy rejects uncertified operations without partial entities."""

import rclcppyy
from rclcppyy import BackendUnavailableError

rclcppyy.enable_cpp_acceleration(profile="required_cpp")

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.publisher import Publisher  # noqa: E402
from std_msgs.msg import String  # noqa: E402


def main():
    context = rclpy.context.Context()
    context.init(args=[])
    node = rclpy.create_node("required_cpp", context=context)
    assert type(node) is Node

    publisher = node.create_publisher(String, "required_cpp", 10)
    assert type(publisher) is Publisher
    assert hasattr(publisher, "_rclcppyy_publish_route")
    print("REQUIRED_PUBLISHER_OK", flush=True)

    before = len(list(node.subscriptions))
    try:
        node.create_subscription(String, "required_cpp", lambda message: None, 10)
    except BackendUnavailableError:
        pass
    else:
        raise AssertionError("required-C++ subscription unexpectedly fell back")
    assert len(list(node.subscriptions)) == before
    print("REQUIRED_FAIL_CLOSED_OK", flush=True)

    status = rclcppyy.status()
    assert status["counts"]["operations"]["unsupported"] >= 1
    node.destroy_publisher(publisher)
    node.destroy_node()
    context.shutdown()


if __name__ == "__main__":
    main()
