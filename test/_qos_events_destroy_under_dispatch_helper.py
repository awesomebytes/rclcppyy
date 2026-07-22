#!/usr/bin/env python3
"""Destroy an event-bearing subscription while its own event callback is
genuinely in flight on a native MultiThreadedExecutor worker thread
(PLAN-qos-events-product.md S3/S6#1) -- the in-flight-counter half of the
load-bearing MTE-safety proof (the other half,
``_qos_events_mte_raise_helper.py``, proves containment-on-raise).

Before Slice 1, event callbacks did not flow through
``_contain_callback_exceptions``, so the product's in-flight counter (used
by ``destroy_subscription``'s quiescence gate, PLAN-mte-unlock.md Slice
2.5/2.5c) could not see an event callback dispatching -- a destroy racing a
dispatching event callback would get zero protection from that gate. This
repeats the destroy-during-dispatch race N=50 times using the
``deadline``-missed subscription event (chosen because, unlike ``matched``,
it repeats on a fixed period once triggered -- proven by the suite's own
teardown-UAF-guard proof -- so each iteration gets a genuine, synchronized
in-flight window instead of racing a one-shot event): the callback signals
it has started, then sleeps briefly to widen the window; the main thread
waits for that signal and destroys the subscription immediately, while the
callback may still be inside the sleep. Success is simply "no crash, no
executor exception, across all N iterations" (a timing-dependent
use-after-free would not reproduce every time, hence the N>=50 floor).
"""
import os
import threading
import time

os.environ.setdefault("ROS_DOMAIN_ID", "77")

import rclcppyy  # noqa: E402

rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.event_handler import SubscriptionEventCallbacks  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import UInt64  # noqa: E402


ITERATIONS = 50
DEADLINE_NS = 60_000_000  # 60ms -- short enough to repeat quickly.
DISPATCH_SLEEP_S = 0.05  # widens the in-flight window past the destroy call.


def run_once(index, pid):
    node = Node("qos_event_destroy_dispatch_%d_%d" % (pid, index))
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    topic = "/qos_events_product/destroy_dispatch/p%d/i%d" % (pid, index)
    qos = QoSProfile(
        depth=8, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        deadline=Duration(nanoseconds=DEADLINE_NS))

    entered = threading.Event()

    def on_deadline(_info):
        entered.set()
        time.sleep(DISPATCH_SLEEP_S)

    publisher = node.create_publisher(UInt64, topic, qos)
    subscription = node.create_subscription(
        UInt64, topic, lambda _msg: None, qos,
        event_callbacks=SubscriptionEventCallbacks(
            deadline=on_deadline, use_default_callbacks=False))

    spin_errors = []

    def _spin_target():
        try:
            executor.spin()
        except BaseException as exc:  # noqa: BLE001 -- captured for the assertion below
            spin_errors.append(exc)

    thread = threading.Thread(target=_spin_target, name="destroy-dispatch-spin-%d" % index)
    thread.start()

    deadline = time.monotonic() + 10.0
    while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
        time.sleep(0.01)
    assert publisher.get_subscription_count() == 1, (
        "iteration %d: discovery never completed" % index)

    publisher.publish(UInt64(data=1))
    assert entered.wait(timeout=5.0), (
        "iteration %d: deadline event never started firing" % index)

    # The callback is very likely still inside its sleep right now -- destroy
    # immediately, racing the in-flight dispatch on purpose.
    node.destroy_subscription(subscription)

    executor.shutdown(timeout_sec=5.0)
    thread.join(timeout=5.0)
    still_alive = thread.is_alive()
    node.destroy_node()
    return still_alive, spin_errors


def main():
    pid = os.getpid()
    rclpy.init(args=[])
    failures = []
    for index in range(ITERATIONS):
        still_alive, spin_errors = run_once(index, pid)
        if still_alive:
            failures.append("iteration %d: spin thread still alive after shutdown" % index)
        if spin_errors:
            failures.append(
                "iteration %d: executor raised %r" % (index, spin_errors))
    rclpy.shutdown()

    if failures:
        for failure in failures:
            print("QOS_EVENT_DESTROY_DISPATCH_FAILURE: %s" % failure, flush=True)
        raise SystemExit(1)

    print(
        "QOS_EVENT_DESTROY_DISPATCH_ALL_OK iterations=%d" % ITERATIONS, flush=True)


if __name__ == "__main__":
    main()
