#!/usr/bin/env python3
"""Committed proof for Slice 2.5c (PLAN-mte-unlock.md): cross-thread
destroy -- an external (non-dispatch) thread destroys an entity while its
own callback is genuinely in flight on a native MultiThreadedExecutor
worker. Unlike the self-destroy scenarios, the destroying thread is NOT
itself dispatching a callback for this node (``_is_dispatching_for``
reads False), so ``destroy_subscription`` runs the synchronous
quiescence-wait path directly (``_quiesce_or_raise``) rather than
deferring to the pump. Expected observable: the call blocks until the
callback returns, then completes cleanly; exit 0.
"""
import faulthandler
import os
import sys
import threading
import time

import rclcppyy

WATCHDOG_SECONDS = 120.0
faulthandler.dump_traceback_later(WATCHDOG_SECONDS, file=sys.stderr, exit=True)

rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclcppyy.direct_executors import (  # noqa: E402
    _multi_threaded_construction_test_only,
)
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import UInt64  # noqa: E402

ITERATIONS = 30
SLOW_SLEEP_S = 0.3


def run_iteration(index, pid):
    node = Node("cross_thread_destroy_%d_%d" % (pid, index))
    with _multi_threaded_construction_test_only():
        executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    group = ReentrantCallbackGroup()

    topic = "/direct_cpp/cross_thread_destroy/p%d/i%d" % (pid, index)
    pub = node.create_publisher(UInt64, topic, 10)

    entered = threading.Event()
    sub_holder = {}

    def slow_callback(_message):
        entered.set()
        time.sleep(SLOW_SLEEP_S)

    sub_holder["sub"] = node.create_subscription(
        UInt64, topic, slow_callback, 10, callback_group=group)

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
        target=_spin_target, name="cross-thread-destroy-spin")
    spin_thread.start()

    pub.publish(UInt64(data=1))
    assert entered.wait(timeout=15.0), "callback never entered"

    # This (main/test) thread is NOT dispatching any callback for this
    # node -- destroy_subscription must take the synchronous
    # quiescence-wait path, blocking here until the callback returns,
    # never severing the callable while it might still be in flight.
    before = time.monotonic()
    result = node.destroy_subscription(sub_holder["sub"])
    elapsed = time.monotonic() - before
    assert result is True
    assert elapsed >= SLOW_SLEEP_S * 0.5, (
        "destroy_subscription returned suspiciously fast (%.3fs) -- did it "
        "actually wait for the in-flight callback?" % elapsed
    )

    assert executor.shutdown(timeout_sec=15.0) is True
    spin_thread.join(timeout=15.0)
    assert not spin_thread.is_alive(), "spin thread hung after teardown"
    assert spin_errors == [], "unexpected spin() exception(s): %r" % (spin_errors,)
    node.destroy_node()


def main():
    rclpy.init(args=[])
    pid = os.getpid()
    try:
        for index in range(ITERATIONS):
            run_iteration(index, pid)
            print("CROSS_THREAD_DESTROY_ITER_%d_OK" % index, flush=True)
    finally:
        rclpy.shutdown()
    assert not rclpy.ok()
    print("CROSS_THREAD_DESTROY_ALL_OK", flush=True)


if __name__ == "__main__":
    main()
