#!/usr/bin/env python3
"""Committed proof for Slice 2.5c (PLAN-mte-unlock.md): a callback that
destroys its own subscription/node mid-dispatch while a peer callback is
genuinely concurrently in-flight on a different native MultiThreadedExecutor
worker thread. Two scenarios, each repeated across iterations to catch a
timing-dependent use-after-free:

  A) self-destroy the subscription the callback is running from
  B) self-destroy the whole owning node (the crasher: pre-Slice-2.5 this
     crashed by iteration 7-10 of 20 with 'callable was deleted' /
     'terminate called without an active exception')

This is the promoted, committed form of the scratchpad reference probe
(self_destroy_probe.py) used as the stop-gate throughout the MTE-unlock
engagement, at the charter's required N >= 50 (was 20 in the scratchpad
version). Both scenarios run under a ReentrantCallbackGroup with
num_threads=2 so the "slow" peer and the "destroyer" callback can genuinely
overlap in time. All under the MultiThreadedExecutor construction escape
hatch (public construction is still fail-closed pending Slice 3).
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
from rclcppyy.direct_executors import (  # noqa: E402
    _multi_threaded_construction_test_only,
)
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import UInt64  # noqa: E402

ITERATIONS = 50
SLOW_SLEEP_S = 0.3


def drive_until(executor, predicate, timeout=15.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert predicate(), "did not complete before the deadline"


def spin_in_background(executor):
    errors = []

    def _target():
        try:
            executor.spin()
        except BaseException as exc:  # noqa: BLE001 -- captured for the proof
            errors.append(exc)

    thread = threading.Thread(target=_target, name="self-destroy-spin")
    thread.start()
    return thread, errors


def run_subscription_self_destroy(index, pid):
    node = Node("self_destroy_sub_%d_%d" % (pid, index))
    with _multi_threaded_construction_test_only():
        executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    group = ReentrantCallbackGroup()

    prefix = "/direct_cpp/self_destroy/sub/p%d/i%d" % (pid, index)
    slow_topic = prefix + "/slow"
    destroyer_topic = prefix + "/destroyer"
    slow_pub = node.create_publisher(UInt64, slow_topic, 10)
    destroyer_pub = node.create_publisher(UInt64, destroyer_topic, 10)

    slow_done = threading.Event()
    destroyer_done = threading.Event()
    destroyer_sub_holder = {}

    def slow_callback(_message):
        time.sleep(SLOW_SLEEP_S)
        slow_done.set()

    def destroyer_callback(_message):
        sub = destroyer_sub_holder["sub"]
        node.destroy_subscription(sub)
        destroyer_done.set()

    node.create_subscription(
        UInt64, slow_topic, slow_callback, 10, callback_group=group)
    destroyer_sub_holder["sub"] = node.create_subscription(
        UInt64, destroyer_topic, destroyer_callback, 10, callback_group=group)

    drive_until(
        executor,
        lambda: (
            slow_pub.get_subscription_count() == 1
            and destroyer_pub.get_subscription_count() == 1
        ),
    )

    spin_thread, spin_errors = spin_in_background(executor)
    slow_pub.publish(UInt64(data=1))
    destroyer_pub.publish(UInt64(data=1))

    assert slow_done.wait(timeout=15.0), "slow peer callback never completed"
    assert destroyer_done.wait(timeout=15.0), "destroyer callback never completed"

    assert executor.shutdown(timeout_sec=15.0) is True
    spin_thread.join(timeout=15.0)
    assert not spin_thread.is_alive(), "spin thread hung after shutdown"
    assert spin_errors == [], "unexpected spin() exception(s): %r" % (spin_errors,)
    node.destroy_node()


def run_node_self_destroy(index, pid):
    node = Node("self_destroy_node_%d_%d" % (pid, index))
    with _multi_threaded_construction_test_only():
        executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    group = ReentrantCallbackGroup()

    prefix = "/direct_cpp/self_destroy/node/p%d/i%d" % (pid, index)
    slow_topic = prefix + "/slow"
    destroyer_topic = prefix + "/destroyer"
    slow_pub = node.create_publisher(UInt64, slow_topic, 10)
    destroyer_pub = node.create_publisher(UInt64, destroyer_topic, 10)

    slow_done = threading.Event()
    destroyer_done = threading.Event()

    def slow_callback(_message):
        time.sleep(SLOW_SLEEP_S)
        slow_done.set()

    def destroyer_callback(_message):
        node.destroy_node()
        destroyer_done.set()

    node.create_subscription(
        UInt64, slow_topic, slow_callback, 10, callback_group=group)
    node.create_subscription(
        UInt64, destroyer_topic, destroyer_callback, 10, callback_group=group)

    drive_until(
        executor,
        lambda: (
            slow_pub.get_subscription_count() == 1
            and destroyer_pub.get_subscription_count() == 1
        ),
    )

    spin_thread, spin_errors = spin_in_background(executor)
    slow_pub.publish(UInt64(data=1))
    destroyer_pub.publish(UInt64(data=1))

    assert slow_done.wait(timeout=15.0), "slow peer callback never completed"
    assert destroyer_done.wait(timeout=15.0), "destroyer callback never completed"

    assert executor.shutdown(timeout_sec=15.0) is True
    spin_thread.join(timeout=15.0)
    assert not spin_thread.is_alive(), "spin thread hung after shutdown"
    assert spin_errors == [], "unexpected spin() exception(s): %r" % (spin_errors,)
    # node.destroy_node() already ran once inside the callback; it must be
    # idempotent/harmless to call again during normal teardown.
    node.destroy_node()


def main():
    rclpy.init(args=[])
    pid = os.getpid()
    try:
        for index in range(ITERATIONS):
            run_subscription_self_destroy(index, pid)
            print("SELF_DESTROY_SUBSCRIPTION_ITER_%d_OK" % index, flush=True)
        print("SELF_DESTROY_SUBSCRIPTION_ALL_OK", flush=True)

        for index in range(ITERATIONS):
            run_node_self_destroy(index, pid)
            print("SELF_DESTROY_NODE_ITER_%d_OK" % index, flush=True)
        print("SELF_DESTROY_NODE_ALL_OK", flush=True)
    finally:
        rclpy.shutdown()
    assert not rclpy.ok()
    print("SELF_DESTROY_SCENARIO_ALL_OK", flush=True)


if __name__ == "__main__":
    main()
