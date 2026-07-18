#!/usr/bin/env python3
"""Compatible publish fallback is visible and retains the stock operation."""

import rclcppyy

rclcppyy.enable_cpp_acceleration()

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclcppyy import monkey as monkey_module  # noqa: E402
from std_msgs.msg import String  # noqa: E402


class RejectingRoute:
    def publish(self, _publisher, _message):
        raise TypeError("deliberate route rejection")


def main():
    context = rclpy.context.Context()
    context.init(args=[])
    node = rclpy.create_node("compatible_publish_fallback", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    observed = []
    subscription = monkey_module._original_create_subscription(
        node,
        String,
        "compatible_publish_fallback",
        lambda message: observed.append(message.data),
        10,
    )
    publisher = node.create_publisher(String, "compatible_publish_fallback", 10)
    publisher._rclcppyy_publish_route = RejectingRoute()
    publisher.publish(String(data="stock-fallback"))
    for _ in range(20):
        executor.spin_once(timeout_sec=0.05)
        if observed:
            break
    assert observed == ["stock-fallback"]
    fallbacks = [
        record
        for record in rclcppyy.status()["operations"]
        if record["backend"] == "python"
        and record["metadata"].get("operation") == "publish"
    ]
    assert len(fallbacks) == 1
    assert "deliberate route rejection" in fallbacks[0]["reason"]
    node.destroy_publisher(publisher)
    node.destroy_subscription(subscription)
    executor.remove_node(node)
    executor.shutdown(timeout_sec=1.0)
    node.destroy_node()
    context.shutdown()
    print("COMPATIBLE_PUBLISH_FALLBACK_VISIBLE_OK")


if __name__ == "__main__":
    main()
