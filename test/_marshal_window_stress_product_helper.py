#!/usr/bin/env python3
"""Committed proof for Slice 2.5c (PLAN-mte-unlock.md): the product-level
counterpart of the suite's marshal-window stress test (Slice 2.5a3). A
worker that has obtained the executable and committed to dispatch,
mid-marshal (cppyy converting the message into Python arguments), is
invisible to the product's own in-flight counter -- it increments only
once ``_contain_callback_exceptions``'s wrapper is entered, which is AFTER
the marshal step. So when ``destroy_subscription`` (cross-thread, hence
the synchronous path) calls ``_quiesce_or_raise`` while a worker is parked
inside the marshal window, it reads ``in_flight == 0`` and proceeds
immediately -- the product's OWN gating provides zero protection here.
This confirms the suite's reaper (Slice 2.5a2/2.5a3) is what actually
makes this safe, end to end through the full product stack, not the
product's quiescence gate.
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
from rclcpp_kit import direct_entities  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import UInt64  # noqa: E402

ITERATIONS = 50
HOOK_SLEEP_S = 0.2


def run_iteration(index, pid):
    node = Node("marshal_window_product_%d_%d" % (pid, index))
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    group = ReentrantCallbackGroup()

    topic = "/direct_cpp/marshal_window_product/p%d/i%d" % (pid, index)
    pub = node.create_publisher(UInt64, topic, 10)

    entered_marshal = threading.Event()
    proceed = threading.Event()
    holder = {}
    received = []

    def marshal_hook():
        entered_marshal.set()
        proceed.wait(timeout=15.0)

    direct_entities.set_marshal_window_hook(marshal_hook)
    try:
        holder["sub"] = node.create_subscription(
            UInt64, topic, received.append, 10, callback_group=group)

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
            target=_spin_target, name="marshal-window-product-spin")
        spin_thread.start()

        pub.publish(UInt64(data=1))

        assert entered_marshal.wait(timeout=15.0), (
            "worker never entered the marshal window -- the hook did not "
            "fire, the proof did not run"
        )

        # destroy_subscription's own _quiesce_or_raise is blind to this
        # window (in_flight has not incremented yet) -- it will proceed
        # immediately rather than wait. This thread is not dispatching for
        # this node, so it takes the synchronous path directly.
        result = node.destroy_subscription(holder["sub"])
        assert result is True
        holder["sub"] = None
        gc.collect()
        gc.collect()

        # Let the parked worker continue: it will now actually attempt the
        # marshal + invoke the callback of the just-destroyed subscription.
        proceed.set()
        time.sleep(HOOK_SLEEP_S + 0.5)

        assert executor.shutdown(timeout_sec=15.0) is True
        spin_thread.join(timeout=15.0)
        assert not spin_thread.is_alive(), "spin thread hung after teardown"
        assert spin_errors == [], (
            "unexpected spin() exception(s): %r" % (spin_errors,)
        )
    finally:
        direct_entities.clear_marshal_window_hook()
    node.destroy_node()


def main():
    rclpy.init(args=[])
    pid = os.getpid()
    try:
        for index in range(ITERATIONS):
            run_iteration(index, pid)
            print("MARSHAL_WINDOW_PRODUCT_ITER_%d_OK" % index, flush=True)
    finally:
        rclpy.shutdown()
    assert not rclpy.ok()
    print("MARSHAL_WINDOW_PRODUCT_ALL_OK", flush=True)


if __name__ == "__main__":
    main()
