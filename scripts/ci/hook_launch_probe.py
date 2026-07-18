#!/usr/bin/env python3
"""Unedited-style rclpy process used by the launch/startup-hook CI proof."""

import json
import os
import subprocess
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String


def main() -> int:
    rclpy.init(args=[])
    node = rclpy.create_node(
        "hook_launch_probe_%d" % os.getpid(),
        namespace="/rclcppyy_contract",
        start_parameter_services=False,
    )
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    received = []
    subscription = node.create_subscription(
        String, "hook_launch_topic", lambda message: received.append(message.data), 10)
    publisher = node.create_publisher(String, "hook_launch_topic", 10)
    try:
        deadline = time.monotonic() + 20.0
        while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert publisher.get_subscription_count() >= 1

        publisher.publish(String(data="launch-contract"))
        deadline = time.monotonic() + 20.0
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert received == ["launch-contract"], received

        cli_environment = os.environ.copy()
        cli_environment["RCLCPPYY_ENABLE_HOOK"] = "0"
        cli = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True,
            env=cli_environment,
            text=True,
            timeout=30,
            check=True,
        )
        full_name = "%s/%s" % (
            node.get_namespace().rstrip("/"), node.get_name())
        assert full_name in cli.stdout.splitlines(), (full_name, cli.stdout, cli.stderr)

        import rclcppyy
        status = rclcppyy.status()
        routes = {
            record["metadata"].get("entity_type"): record["backend"]
            for record in status["entities"]
            if record["metadata"].get("topic") in (
                "hook_launch_topic", "/rclcppyy_contract/hook_launch_topic")
        }
        assert routes == {"publisher": "cpp", "subscription": "python"}, status
        print(json.dumps({
            "node": full_name,
            "routes": routes,
            "message": received[0],
        }, sort_keys=True), flush=True)
        print("HOOK_LAUNCH_CLI_OK", flush=True)
        return 0
    finally:
        node.destroy_publisher(publisher)
        node.destroy_subscription(subscription)
        executor.remove_node(node)
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
