#!/usr/bin/env python3
"""Committed confirm test for Slice 2.5c (PLAN-mte-unlock.md): action
server/client entities are PROVEN-SAFE-BY-DISPATCH-MODEL under a live
MultiThreadedExecutor (goal/cancel decisions are dispatched only on the
creator thread, per the suite's own P0 guard, and an existing
depth-tracked deferred close already prevents severing a callable while a
decision callback is in flight) -- so no native-owned-lifetime fix was
needed for actions (see Slice 2.5a's suite-level confirm test). This is
the PRODUCT-level counterpart: destroying the node while an action goal's
execute callback is genuinely in flight, under a live MultiThreadedExecutor,
must not crash. One iteration suffices per the charter (a confirm/guard,
not a timing-dependent proof).

Two P0 gates shape this test's structure, neither being a bug to work
around but a deliberate guard to respect:
  - direct_cpp ActionServer construction rejects a node already attached to
    a MultiThreadedExecutor (direct_actions.py:1199-1202) -- so the server/
    client are constructed first, on a not-yet-attached node.
  - Goal/cancel decisions are dispatched only on the "creator thread" (the
    thread that constructed the ActionServer) -- so spin is driven via
    spin_once() in a loop on the SAME (main) thread that did the
    construction, never a separate spin thread, or every goal decision
    would be silently rejected as off-creator-thread.
"""
import faulthandler
import os
import sys
import threading
import time

import rclcppyy

WATCHDOG_SECONDS = 60.0
faulthandler.dump_traceback_later(WATCHDOG_SECONDS, file=sys.stderr, exit=True)

rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclcppyy.direct_executors import (  # noqa: E402
    _multi_threaded_construction_test_only,
)
from rclpy.action import ActionClient, ActionServer  # noqa: E402
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from tf2_msgs.action import LookupTransform  # noqa: E402


def main():
    rclpy.init(args=[])
    pid = os.getpid()
    action_name = "/direct_cpp/action_under_mte/p%d" % pid
    node = Node("action_under_mte_%d" % pid)
    group = MutuallyExclusiveCallbackGroup()

    execute_entered = threading.Event()
    execute_release = threading.Event()

    def execute_callback(goal_handle):
        execute_entered.set()
        execute_release.wait(timeout=15.0)
        goal_handle.succeed()
        return LookupTransform.Result()

    # Construct on the (not-yet-attached) node first -- see the module
    # docstring for why.
    server = ActionServer(
        node, LookupTransform, action_name, execute_callback,
        callback_group=group)
    client = ActionClient(node, LookupTransform, action_name)

    with _multi_threaded_construction_test_only():
        executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    assert client.wait_for_server(timeout_sec=15.0)

    goal = LookupTransform.Goal()
    goal.target_frame = "action-under-mte"
    goal.source_frame = "base"
    client.send_goal_async(goal)

    # Drive spin from THIS (the creator/construction) thread -- a separate
    # spin thread would make every goal decision dispatch off-creator-
    # thread, which the suite's P0 guard rejects outright (see docstring).
    deadline = time.monotonic() + 15.0
    while not execute_entered.is_set() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert execute_entered.is_set(), "execute callback never entered"

    # Destroy the node WHILE the action goal's execute callback is
    # genuinely in flight (parked on execute_release), under a live
    # MultiThreadedExecutor.
    node.destroy_node()

    # Let the in-flight execute callback finish (or not -- either is fine;
    # the point is that neither destroying the node nor the callback
    # finishing afterward crashes the process). It runs on the pump's own
    # polling, driven by spin_once below, same as before.
    execute_release.set()
    settle_deadline = time.monotonic() + 2.0
    while time.monotonic() < settle_deadline:
        executor.spin_once(timeout_sec=0.05)

    assert executor.shutdown(timeout_sec=15.0) is True

    rclpy.shutdown()
    assert not rclpy.ok()
    print("ACTION_UNDER_MTE_CONFIRM_OK", flush=True)


if __name__ == "__main__":
    main()
