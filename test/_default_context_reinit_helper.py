#!/usr/bin/env python3
"""Prove repeated default Context cycles after explicit C++ publishing."""

import os
import time

import rclcppyy


CYCLES = 3
SPIN_DEADLINE_S = 15.0


def main():
    rclcppyy.enable_cpp_acceleration(profile="publisher_cpp")

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String

    for cycle in range(CYCLES):
        rclpy.init(args=[])
        assert rclpy.ok()
        node = rclpy.create_node(
            "rclcppyy_default_reinit_%d_%d" % (os.getpid(), cycle))
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        topic = "/rclcppyy/default_reinit/p%d/c%d" % (os.getpid(), cycle)
        received = []
        subscription = node.create_subscription(
            String, topic, lambda message: received.append(message.data), 10)
        publisher = node.create_publisher(String, topic, 10)
        assert hasattr(publisher, "_rclcppyy_publish_route")

        deadline = time.monotonic() + SPIN_DEADLINE_S
        while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert publisher.get_subscription_count() >= 1

        payload = "default context cycle %d" % cycle
        publisher.publish(String(data=payload))
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert received == [payload], received
        assert publisher._rclcppyy_last_publish_backend == "cpp"
        assert publisher._rclcppyy_publish_tainted is False

        executor.remove_node(node)
        assert executor.shutdown(timeout_sec=2.0)
        assert node.destroy_subscription(subscription)
        assert node.destroy_publisher(publisher)
        node.destroy_node()
        rclpy.shutdown()
        assert not rclpy.ok()

    status = rclcppyy.status()
    assert status["counts"]["operations"]["cpp"] >= CYCLES, status
    print("DEFAULT_CONTEXT_REINIT_AFTER_CPP_PUBLISH_OK", flush=True)


if __name__ == "__main__":
    main()
