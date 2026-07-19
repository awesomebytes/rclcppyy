#!/usr/bin/env python3
"""Stock rclpy action server used by the direct-C++ client proof."""

import sys
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_msgs.action import LookupTransform


def main():
    action_name = sys.argv[1]
    rclpy.init(args=[])
    node = Node("direct_cpp_stock_action_server")

    def goal_callback(goal):
        if goal.target_frame == "reject":
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(_goal_handle):
        return CancelResponse.ACCEPT

    def execute(goal_handle):
        target = goal_handle.request.target_frame
        # Action service discovery can complete before the separate feedback
        # publisher/subscription pair has matched on a fresh DDS graph.
        time.sleep(0.25)
        for _ in range(3):
            goal_handle.publish_feedback(LookupTransform.Feedback())
            time.sleep(0.001)
        # Feedback and result use separate DDS channels. Give the executor time
        # to dispatch the full deterministic burst before the terminal response.
        time.sleep(0.05)
        if target == "cancel":
            deadline = time.monotonic() + 10.0
            while not goal_handle.is_cancel_requested and time.monotonic() < deadline:
                time.sleep(0.001)
            if not goal_handle.is_cancel_requested:
                raise RuntimeError("direct action cancellation was not received")
            goal_handle.canceled()
            result = LookupTransform.Result()
            result.transform.child_frame_id = "canceled-stock-result"
            return result
        goal_handle.succeed()
        result = LookupTransform.Result()
        result.transform.child_frame_id = "successful-stock-result"
        return result

    server = ActionServer(
        node,
        LookupTransform,
        action_name,
        execute_callback=execute,
        goal_callback=goal_callback,
        cancel_callback=cancel_callback,
        callback_group=ReentrantCallbackGroup(),
    )
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    print("STOCK_ACTION_SERVER_READY", flush=True)
    try:
        executor.spin()
    finally:
        server.destroy()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
