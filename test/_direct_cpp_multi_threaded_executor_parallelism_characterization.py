#!/usr/bin/env python3
"""Committed true-parallelism proof for Slice 3 (PLAN-mte-unlock.md
un-fail-close): genuine temporal-overlap proofs for
DirectMultiThreadedExecutor through the public constructor -- reentrant
groups dispatch in real parallel across native ``rclcpp`` worker threads,
mutually-exclusive groups serialize under real concurrency, and exclusion
is per-group (a cross-group pair still runs concurrently). Concurrency is
proven with ``threading.Barrier``, never sleeps: a barrier only releases if
the required number of parties reach it inside the timeout, which happens
only under genuine concurrent execution (the GIL is released by
``Barrier.wait()``, matching real stock rclpy MultiThreadedExecutor's own
GIL reality for pure-Python callbacks).

This was previously a one-time, uncommitted evidence script (not run by
CI) because this dispatch path (``spin()``/``start_executor()``, the native
background thread) was affected by a suite-level (cppyy_kit) zero-dispatch
race under real concurrency -- a ready Python subscription callback could
occasionally never fire at all. Slice 1 of this same plan fixed that race
in the product's own pump (the startup-gate + wake-cancel-window fix in
``_run_native_background``/``wake()``); this is now a committed, non-flaky
test through the public constructor.

A top-level watchdog (``faulthandler.dump_traceback_later(..., exit=True)``)
guarantees a design regression here fails loud within a bounded time instead
of hanging a dev machine forever.
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
from rclpy.callback_groups import (  # noqa: E402
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import UInt64  # noqa: E402


rclpy.init(args=[])
PREFIX = "/direct_cpp/mte/p%d" % os.getpid()


def wait_for_discovery(executor, predicate, timeout=15.0):
    """Discovery/setup only -- always via spin_once(), never the background
    pump (spin() holds the executor's _spin_lock for its whole duration)."""
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert predicate(), "discovery/setup did not complete before the deadline"


def spin_in_background(executor):
    errors = []

    def _target():
        try:
            executor.spin()
        except BaseException as exc:  # noqa: BLE001 -- captured for the test
            errors.append(exc)

    thread = threading.Thread(target=_target, name="mte-helper-spin")
    thread.start()
    return thread, errors


class InFlightTracker:
    """Thread-safe in-flight/peak counter shared across group members."""

    def __init__(self):
        self._lock = threading.Lock()
        self.in_flight = 0
        self.peak = 0

    def enter(self):
        with self._lock:
            self.in_flight += 1
            self.peak = max(self.peak, self.in_flight)
            return self.in_flight

    def exit(self):
        with self._lock:
            self.in_flight -= 1


# ---------------------------------------------------------------------
# 1) Reentrant group: two entities dispatch in real parallel.
# ---------------------------------------------------------------------
reentrant_node = Node("direct_mte_reentrant_%d" % os.getpid())
reentrant_executor = MultiThreadedExecutor(num_threads=4)
reentrant_executor.add_node(reentrant_node)
reentrant_group = ReentrantCallbackGroup()

reentrant_tracker = InFlightTracker()
reentrant_barrier = threading.Barrier(2)
reentrant_results = {}
reentrant_done = threading.Event()


def _reentrant_member(name):
    def _callback(_message):
        reentrant_tracker.enter()
        broken = False
        try:
            reentrant_barrier.wait(timeout=10.0)
        except threading.BrokenBarrierError:
            broken = True
        reentrant_tracker.exit()
        reentrant_results[name] = broken
        if len(reentrant_results) == 2:
            reentrant_done.set()
    return _callback


topic_a = PREFIX + "/reentrant_a"
topic_b = PREFIX + "/reentrant_b"
pub_a = reentrant_node.create_publisher(UInt64, topic_a, 10)
pub_b = reentrant_node.create_publisher(UInt64, topic_b, 10)
reentrant_node.create_subscription(
    UInt64, topic_a, _reentrant_member("a"), 10, callback_group=reentrant_group)
reentrant_node.create_subscription(
    UInt64, topic_b, _reentrant_member("b"), 10, callback_group=reentrant_group)

wait_for_discovery(
    reentrant_executor,
    lambda: pub_a.get_subscription_count() == 1 and pub_b.get_subscription_count() == 1,
)
# Both published before the background pump starts, so both are pending
# when native dispatch begins -- the deterministic way to make "ready at
# once" true rather than relying on timing after the pump is already live.
pub_a.publish(UInt64(data=1))
pub_b.publish(UInt64(data=1))

reentrant_thread, reentrant_spin_errors = spin_in_background(reentrant_executor)
assert reentrant_done.wait(timeout=20.0), "reentrant callbacks never both completed"
assert reentrant_executor.shutdown(timeout_sec=10.0)
reentrant_thread.join(timeout=10.0)
assert not reentrant_thread.is_alive()
assert reentrant_spin_errors == [], reentrant_spin_errors

assert reentrant_tracker.peak == 2, (
    "expected two reentrant callbacks to run concurrently, peak was %d"
    % reentrant_tracker.peak
)
assert not any(reentrant_results.values()), (
    "the reentrant barrier broke instead of releasing: %r" % reentrant_results
)
reentrant_node.destroy_node()
print("DIRECT_CPP_MTE_REENTRANT_PARALLEL_OK", flush=True)


# ---------------------------------------------------------------------
# 2) Mutually-exclusive group: two entities serialize.
# ---------------------------------------------------------------------
exclusive_node = Node("direct_mte_exclusive_%d" % os.getpid())
exclusive_executor = MultiThreadedExecutor(num_threads=4)
exclusive_executor.add_node(exclusive_node)
exclusive_group = MutuallyExclusiveCallbackGroup()

exclusive_tracker = InFlightTracker()
exclusive_barrier = threading.Barrier(2)
exclusive_results = {}
exclusive_done = threading.Event()


def _exclusive_member(name):
    def _callback(_message):
        current = exclusive_tracker.enter()
        assert current == 1, (
            "mutually-exclusive group allowed concurrent entry (in_flight=%d)"
            % current
        )
        broken = False
        try:
            # Short timeout: this MUST time out, because the peer cannot
            # reach the barrier until this callback returns and releases
            # the group -- a broken barrier here is the exclusion proof,
            # not a failure.
            exclusive_barrier.wait(timeout=0.5)
        except threading.BrokenBarrierError:
            broken = True
        time.sleep(0.05)  # brief GIL-releasing hold of the exclusive slot
        exclusive_tracker.exit()
        exclusive_results[name] = broken
        if len(exclusive_results) == 2:
            exclusive_done.set()
    return _callback


topic_c = PREFIX + "/exclusive_c"
topic_d = PREFIX + "/exclusive_d"
pub_c = exclusive_node.create_publisher(UInt64, topic_c, 10)
pub_d = exclusive_node.create_publisher(UInt64, topic_d, 10)
exclusive_node.create_subscription(
    UInt64, topic_c, _exclusive_member("c"), 10, callback_group=exclusive_group)
exclusive_node.create_subscription(
    UInt64, topic_d, _exclusive_member("d"), 10, callback_group=exclusive_group)

wait_for_discovery(
    exclusive_executor,
    lambda: pub_c.get_subscription_count() == 1 and pub_d.get_subscription_count() == 1,
)
pub_c.publish(UInt64(data=1))
pub_d.publish(UInt64(data=1))

exclusive_thread, exclusive_spin_errors = spin_in_background(exclusive_executor)
assert exclusive_done.wait(timeout=20.0), "exclusive callbacks never both completed"
assert exclusive_executor.shutdown(timeout_sec=10.0)
exclusive_thread.join(timeout=10.0)
assert not exclusive_thread.is_alive()
assert exclusive_spin_errors == [], exclusive_spin_errors

assert exclusive_tracker.peak == 1, (
    "expected the exclusive group to serialize, peak was %d"
    % exclusive_tracker.peak
)
assert all(exclusive_results.values()), (
    "expected the exclusive barrier to time out (proving exclusion): %r"
    % exclusive_results
)
assert set(exclusive_results) == {"c", "d"}
exclusive_node.destroy_node()
print("DIRECT_CPP_MTE_EXCLUSIVE_SERIALIZED_OK", flush=True)


# ---------------------------------------------------------------------
# 3) Cross-group control: two different exclusive groups run concurrently.
# ---------------------------------------------------------------------
cross_node = Node("direct_mte_cross_group_%d" % os.getpid())
cross_executor = MultiThreadedExecutor(num_threads=4)
cross_executor.add_node(cross_node)
cross_group_1 = MutuallyExclusiveCallbackGroup()
cross_group_2 = MutuallyExclusiveCallbackGroup()

cross_barrier = threading.Barrier(2)
cross_results = {}
cross_done = threading.Event()


def _cross_member(name):
    def _callback(_message):
        broken = False
        try:
            cross_barrier.wait(timeout=10.0)
        except threading.BrokenBarrierError:
            broken = True
        cross_results[name] = broken
        if len(cross_results) == 2:
            cross_done.set()
    return _callback


topic_e = PREFIX + "/cross_e"
topic_f = PREFIX + "/cross_f"
pub_e = cross_node.create_publisher(UInt64, topic_e, 10)
pub_f = cross_node.create_publisher(UInt64, topic_f, 10)
cross_node.create_subscription(
    UInt64, topic_e, _cross_member("e"), 10, callback_group=cross_group_1)
cross_node.create_subscription(
    UInt64, topic_f, _cross_member("f"), 10, callback_group=cross_group_2)

wait_for_discovery(
    cross_executor,
    lambda: pub_e.get_subscription_count() == 1 and pub_f.get_subscription_count() == 1,
)
pub_e.publish(UInt64(data=1))
pub_f.publish(UInt64(data=1))

cross_thread, cross_spin_errors = spin_in_background(cross_executor)
assert cross_done.wait(timeout=20.0), "cross-group callbacks never both completed"
assert cross_executor.shutdown(timeout_sec=10.0)
cross_thread.join(timeout=10.0)
assert not cross_thread.is_alive()
assert cross_spin_errors == [], cross_spin_errors

assert not any(cross_results.values()), (
    "expected two different exclusive groups to run concurrently: %r"
    % cross_results
)
cross_node.destroy_node()
print("DIRECT_CPP_MTE_CROSS_GROUP_CONCURRENT_OK", flush=True)

# Wake latency and teardown are proven as committed, deterministic tests in
# _direct_cpp_multi_threaded_executor_helper.py instead -- neither depends
# on entity dispatch succeeding, so neither needs this evidence-only script.

rclpy.shutdown()
assert not rclpy.ok()
print("DIRECT_CPP_MTE_PARALLELISM_CHARACTERIZATION_COMPLETE", flush=True)
