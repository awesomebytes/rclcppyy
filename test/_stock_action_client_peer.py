#!/usr/bin/env python3
"""External stock rclpy client for the direct action-server interop proof."""

import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_msgs.action import LookupTransform


def spin_until(node, predicate, label, timeout=15.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
    if not predicate():
        raise TimeoutError("timed out waiting for " + label)


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: helper.py ACTION_NAME")
    action_name = sys.argv[1]
    rclpy.init(args=[])
    node = Node("stock_client_direct_action_server")
    client = ActionClient(node, LookupTransform, action_name)
    feedback = []
    try:
        if not client.wait_for_server(timeout_sec=15.0):
            raise TimeoutError("direct action server was not discovered")
        goal = LookupTransform.Goal()
        goal.target_frame = "stock-client-target"
        goal.source_frame = "stock-client-source"
        goal.source_time.sec = 17
        goal.source_time.nanosec = 23
        goal_future = client.send_goal_async(
            goal, feedback_callback=feedback.append)
        spin_until(node, goal_future.done, "goal response")
        handle = goal_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("direct action server rejected the stock goal")
        result_future = handle.get_result_async()
        spin_until(
            node,
            lambda: result_future.done() and len(feedback) == 2,
            "feedback and result",
        )
        wrapped = result_future.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError("direct action server returned the wrong status")
        if wrapped.result.transform.child_frame_id != "stock-client-result":
            raise RuntimeError("direct action server returned the wrong payload")
        if len(feedback) != 2:
            raise RuntimeError("direct action server returned the wrong feedback count")
        print("STOCK_ACTION_CLIENT_DIRECT_SERVER_OK", flush=True)
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.try_shutdown()
    print("STOCK_ACTION_CLIENT_TEARDOWN_OK", flush=True)


if __name__ == "__main__":
    main()
