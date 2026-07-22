#!/usr/bin/env python3
"""Committed proof for Slice 2.5c (PLAN-mte-unlock.md): the product-level
counterpart of the suite's gc_after_close/gc_after_quiescent_close proofs.
Unlike the raw suite API, EVERY product-level destroy path
(``destroy_subscription``, whether the caller is dispatching for this node
or not) calls ``_quiesce_or_raise`` before removing the entity from the
node's registry and closing it -- so by construction only the "quiescent"
variant is reachable through the gated API (the racy one is not; confirmed
by recon, PLAN-mte-unlock.md Addendum v3.1 point 1).

Uses a cross-thread destroy (the calling thread is NOT itself dispatching
for this node, so ``destroy_subscription`` takes the synchronous
quiescence-wait path directly) so ``destroy_subscription`` genuinely blocks
until the in-flight callback returns, THEN drops every Python reference to
the destroyed subscription and forces ``gc.collect()`` -- confirming the
whole integrated stack (product gating + suite ManagedSubscription +
reaper) stays crash-free end to end.
"""
import faulthandler
import gc
import os
import sys
import threading
import time

import rclcppyy

WATCHDOG_SECONDS = 250.0
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

ITERATIONS = 50
SLOW_SLEEP_S = 0.3


def run_iteration(index, pid):
    node = Node("gc_after_destroy_%d_%d" % (pid, index))
    with _multi_threaded_construction_test_only():
        executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    group = ReentrantCallbackGroup()

    topic = "/direct_cpp/gc_after_destroy/p%d/i%d" % (pid, index)
    pub = node.create_publisher(UInt64, topic, 10)

    entered = threading.Event()
    holder = {}

    def slow_callback(_message):
        entered.set()
        time.sleep(SLOW_SLEEP_S)

    holder["sub"] = node.create_subscription(
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

    spin_thread = threading.Thread(target=_spin_target, name="gc-after-destroy-spin")
    spin_thread.start()

    pub.publish(UInt64(data=1))
    assert entered.wait(timeout=15.0), "callback never entered"

    # This (main) thread is not dispatching for this node, so
    # destroy_subscription takes the synchronous quiescence-wait path,
    # blocking here until the in-flight callback returns.
    result = node.destroy_subscription(holder["sub"])
    assert result is True
    holder["sub"] = None
    gc.collect()
    gc.collect()

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
            print("GC_AFTER_DESTROY_ITER_%d_OK" % index, flush=True)
    finally:
        rclpy.shutdown()
    assert not rclpy.ok()
    print("GC_AFTER_DESTROY_ALL_OK", flush=True)


if __name__ == "__main__":
    main()
