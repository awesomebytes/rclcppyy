#!/usr/bin/env python3
"""Committed proof for Slice 2.5c (PLAN-mte-unlock.md): the "v1 mandatory
shape" -- a peer callback is genuinely mid-trampoline (sleeping inside its
Python callback, dispatched on a native MultiThreadedExecutor worker) when
``executor.shutdown()`` fires from another thread. Exercises the shutdown
path specifically (``DirectExecutor.shutdown`` -> native thread join ->
``_mark_runtime_shutdown`` for every attached node), not just
``destroy_*``, since shutdown-while-a-peer-is-in-flight is a distinct
teardown path with its own (join-based, not counter-based) safety
argument.

Expected observable (the stop condition): ``shutdown()`` returns True, the
spin thread joins, ``spin_errors == []``, exit 0. N >= 50.
"""
import faulthandler
import os
import sys
import threading
import time

import rclcppyy

WATCHDOG_SECONDS = 250.0
faulthandler.dump_traceback_later(WATCHDOG_SECONDS, file=sys.stderr, exit=True)

rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import UInt64  # noqa: E402

ITERATIONS = 50
SLOW_SLEEP_S = 0.3


def run_iteration(index, pid):
    node = Node("shutdown_under_spin_%d_%d" % (pid, index))
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    group = ReentrantCallbackGroup()

    prefix = "/direct_cpp/shutdown_under_spin/p%d/i%d" % (pid, index)
    slow_topic = prefix + "/slow"
    pub = node.create_publisher(UInt64, slow_topic, 10)

    entered = threading.Event()
    slow_done = threading.Event()

    def slow_callback(_message):
        entered.set()
        time.sleep(SLOW_SLEEP_S)
        slow_done.set()

    node.create_subscription(
        UInt64, slow_topic, slow_callback, 10, callback_group=group)

    deadline = time.monotonic() + 15.0
    while pub.get_subscription_count() < 1 and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert pub.get_subscription_count() == 1

    spin_errors = []

    def _spin_target():
        try:
            executor.spin()
        except BaseException as exc:  # noqa: BLE001 -- captured for the proof
            spin_errors.append(exc)

    spin_thread = threading.Thread(
        target=_spin_target, name="shutdown-under-spin")
    spin_thread.start()

    pub.publish(UInt64(data=1))
    assert entered.wait(timeout=15.0), "slow callback never entered"

    # shutdown() fires WHILE the peer is genuinely still asleep inside its
    # own Python callback, dispatched on a native worker.
    assert not slow_done.is_set(), (
        "slow callback finished before shutdown() fired -- widen the sleep, "
        "this run did not exercise the intended overlap"
    )
    result = executor.shutdown(timeout_sec=15.0)
    assert result is True, "shutdown() did not return True"

    spin_thread.join(timeout=15.0)
    assert not spin_thread.is_alive(), "spin thread hung after shutdown"
    assert spin_errors == [], "unexpected spin() exception(s): %r" % (spin_errors,)

    node.destroy_node()


def main():
    rclpy.init(args=[])
    pid = os.getpid()
    try:
        for index in range(ITERATIONS):
            run_iteration(index, pid)
            print("SHUTDOWN_UNDER_SPIN_ITER_%d_OK" % index, flush=True)
    finally:
        rclpy.shutdown()
    assert not rclpy.ok()
    print("SHUTDOWN_UNDER_SPIN_ALL_OK", flush=True)


if __name__ == "__main__":
    main()
