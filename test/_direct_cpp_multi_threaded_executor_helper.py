#!/usr/bin/env python3
"""Deterministic DirectMultiThreadedExecutor proofs: dispatch/no-starvation
regression guards, an entity-to-group binding proof, wake latency, and
teardown -- all on the reliable ``spin_once()`` path.

These do NOT claim or require temporal overlap ("true parallelism"), and the
first two do NOT and cannot claim anything about exclusion MECHANICS either
-- see the labeling note on each section below. That is a deliberate,
confirmed scope limit (wave-3 Lane 2 hybrid disposition, see
``docs/plans/PLAN-executor-slice.md``), not an oversight: native ``rclcpp``
dispatch enforces callback-group exclusion entirely in C++ and never calls
back into ``DirectCallbackGroup``'s ``can_execute``/``beginning_execution``/
``ending_execution`` during real dispatch (confirmed empirically --
``_active_entity`` stays ``None`` throughout live dispatch, always; these
hooks are advisory only, invoked solely when a caller explicitly calls them,
as ``_direct_cpp_executor_surface_helper.py`` does with synthetic timers).
So there is no facade-side bookkeeping to observe during live dispatch, and
"exclusive-group serialization" proven purely by driving ``spin_once()`` in
a loop would be vacuous -- it would pass identically even if callback groups
enforced nothing, since ``spin_once()`` only ever runs one thing at a time
regardless (confirmed empirically: two callbacks racing a
``threading.Barrier`` never overlap on this path).

Why not a genuine parallelism proof here: proving actual temporal overlap
needs the background-pump path (``spin()``/``start_executor()``), which has
a genuine suite-level (cppyy_kit) zero-dispatch race under real concurrency;
that proof is a one-time recorded characterization instead of a committed
test -- see
``_direct_cpp_multi_threaded_executor_parallelism_characterization.py``.

Public MultiThreadedExecutor construction is fail-closed this wave (see
``rclcppyy.direct_executors._MULTI_THREADED_FAIL_CLOSED_REASON``), so every
construction below goes through the test-only
``_multi_threaded_construction_test_only()`` guard.
"""

import os
import threading
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclcppyy.direct_executors import (  # noqa: E402
    _multi_threaded_construction_test_only,
)
from rclpy.callback_groups import (  # noqa: E402
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import UInt64  # noqa: E402


rclpy.init(args=[])
PREFIX = "/direct_cpp/mte_correctness/p%d" % os.getpid()


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
        except BaseException as exc:  # noqa: BLE001 -- captured for the test
            errors.append(exc)

    thread = threading.Thread(target=_target, name="mte-helper-spin")
    thread.start()
    return thread, errors


# ---------------------------------------------------------------------
# 1) LABEL: wiring/no-starvation regression guard -- NOT an exclusion-
#    mechanics proof. Both entities in one exclusive group are dispatched
#    exactly once within a bounded wait; a native group-wiring bug could
#    plausibly starve one of them forever, so this is real, non-vacuous
#    coverage of that risk. It does not and cannot claim anything about
#    HOW exclusion is enforced (that lives entirely in native rclcpp).
# ---------------------------------------------------------------------
exclusive_node = Node("direct_mte_exclusive_%d" % os.getpid())
with _multi_threaded_construction_test_only():
    exclusive_executor = MultiThreadedExecutor(num_threads=4)
exclusive_executor.add_node(exclusive_node)
exclusive_group = MutuallyExclusiveCallbackGroup()

exclusive_results = {}


def _exclusive_member(name):
    def _callback(_message):
        exclusive_results[name] = True
    return _callback


topic_c = PREFIX + "/exclusive_c"
topic_d = PREFIX + "/exclusive_d"
pub_c = exclusive_node.create_publisher(UInt64, topic_c, 10)
pub_d = exclusive_node.create_publisher(UInt64, topic_d, 10)
exclusive_node.create_subscription(
    UInt64, topic_c, _exclusive_member("c"), 10, callback_group=exclusive_group)
exclusive_node.create_subscription(
    UInt64, topic_d, _exclusive_member("d"), 10, callback_group=exclusive_group)

drive_until(
    exclusive_executor,
    lambda: pub_c.get_subscription_count() == 1 and pub_d.get_subscription_count() == 1,
)
pub_c.publish(UInt64(data=1))
pub_d.publish(UInt64(data=1))
drive_until(exclusive_executor, lambda: len(exclusive_results) == 2)
assert exclusive_executor.shutdown(timeout_sec=5.0)
exclusive_node.destroy_node()
print("DIRECT_CPP_MTE_EXCLUSIVE_CORRECTNESS_OK", flush=True)


# ---------------------------------------------------------------------
# 2) LABEL: wiring/no-starvation regression guard, same limits as (1). Two
#    entities in TWO DIFFERENT exclusive groups both dispatch within a
#    bounded wait -- neither group's wiring starves the other's entity.
#    This does not and cannot claim "exclusion is per-group, not global"
#    as a mechanics proof; that is a concurrency claim and belongs to the
#    one-time characterization script instead.
# ---------------------------------------------------------------------
cross_node = Node("direct_mte_cross_group_%d" % os.getpid())
with _multi_threaded_construction_test_only():
    cross_executor = MultiThreadedExecutor(num_threads=4)
cross_executor.add_node(cross_node)
cross_group_1 = MutuallyExclusiveCallbackGroup()
cross_group_2 = MutuallyExclusiveCallbackGroup()

cross_results = {}


def _cross_member(name):
    def _callback(_message):
        cross_results[name] = True
    return _callback


topic_e = PREFIX + "/cross_e"
topic_f = PREFIX + "/cross_f"
pub_e = cross_node.create_publisher(UInt64, topic_e, 10)
pub_f = cross_node.create_publisher(UInt64, topic_f, 10)
cross_node.create_subscription(
    UInt64, topic_e, _cross_member("e"), 10, callback_group=cross_group_1)
cross_node.create_subscription(
    UInt64, topic_f, _cross_member("f"), 10, callback_group=cross_group_2)

drive_until(
    cross_executor,
    lambda: pub_e.get_subscription_count() == 1 and pub_f.get_subscription_count() == 1,
)
pub_e.publish(UInt64(data=1))
pub_f.publish(UInt64(data=1))
drive_until(cross_executor, lambda: len(cross_results) == 2)
assert cross_executor.shutdown(timeout_sec=5.0)
cross_node.destroy_node()
print("DIRECT_CPP_MTE_CROSS_GROUP_CORRECTNESS_OK", flush=True)


# ---------------------------------------------------------------------
# 3) LABEL: entity-to-group BINDING proof, not an admission-semantics
#    proof. Commit 1's surface helper already proves the reentrant
#    can_execute truth table in isolation with synthetic timer objects,
#    with no executor spinning at all. This proves the same call succeeds
#    end-to-end on a REAL, live entity (a subscription registered via
#    create_subscription, group.has_entity() true) -- the real
#    entity-to-group wiring path, not exercised by the synthetic version.
# ---------------------------------------------------------------------
reentrant_node = Node("direct_mte_reentrant_%d" % os.getpid())
with _multi_threaded_construction_test_only():
    reentrant_executor = MultiThreadedExecutor(num_threads=4)
reentrant_executor.add_node(reentrant_node)
reentrant_group = ReentrantCallbackGroup()

reentrant_results = {}
reentrant_entities = {}


def _reentrant_member(name):
    def _callback(_message):
        entity = reentrant_entities[name]
        assert reentrant_group.has_entity(entity)
        assert reentrant_group.can_execute(entity) is True, (
            "%s: can_execute did not succeed on a real, live entity" % name
        )
        reentrant_results[name] = True
    return _callback


topic_a = PREFIX + "/reentrant_a"
topic_b = PREFIX + "/reentrant_b"
pub_a = reentrant_node.create_publisher(UInt64, topic_a, 10)
pub_b = reentrant_node.create_publisher(UInt64, topic_b, 10)
reentrant_entities["a"] = reentrant_node.create_subscription(
    UInt64, topic_a, _reentrant_member("a"), 10, callback_group=reentrant_group)
reentrant_entities["b"] = reentrant_node.create_subscription(
    UInt64, topic_b, _reentrant_member("b"), 10, callback_group=reentrant_group)

drive_until(
    reentrant_executor,
    lambda: pub_a.get_subscription_count() == 1 and pub_b.get_subscription_count() == 1,
)
pub_a.publish(UInt64(data=1))
pub_b.publish(UInt64(data=1))
drive_until(reentrant_executor, lambda: len(reentrant_results) == 2)
assert reentrant_executor.shutdown(timeout_sec=5.0)
reentrant_node.destroy_node()
print("DIRECT_CPP_MTE_REENTRANT_ADMISSION_OK", flush=True)


# ---------------------------------------------------------------------
# 4) Wake: create_task() while idle-spinning drives within bounded latency.
#    Task-only -- bypasses the wait set entirely, so it does not depend on
#    entity dispatch and is unaffected by the dispatch-reliability gap.
#    Unchanged by the labeling review above.
# ---------------------------------------------------------------------
wake_node = Node("direct_mte_wake_%d" % os.getpid())
with _multi_threaded_construction_test_only():
    wake_executor = MultiThreadedExecutor(num_threads=2)
wake_executor.add_node(wake_node)

wake_thread, wake_spin_errors = spin_in_background(wake_executor)
deadline = time.monotonic() + 10.0
while wake_executor._background_thread is None and time.monotonic() < deadline:
    time.sleep(0.01)
assert wake_executor._background_thread is not None
assert wake_executor.is_spinning

task_ran = threading.Event()
started = time.monotonic()
wake_executor.create_task(task_ran.set)
assert task_ran.wait(timeout=5.0), "create_task() was never driven"
latency = time.monotonic() - started
assert latency < 2.0, "wake-to-drive latency too high: %.3fs" % latency

assert wake_executor.shutdown(timeout_sec=10.0)
wake_thread.join(timeout=10.0)
assert not wake_thread.is_alive()
assert wake_spin_errors == [], wake_spin_errors
wake_node.destroy_node()
print("DIRECT_CPP_MTE_WAKE_OK", flush=True)


# ---------------------------------------------------------------------
# 5) Teardown, including shutdown-during-spin. No entity dispatch involved.
#    Unchanged by the labeling review above.
# ---------------------------------------------------------------------
teardown_node = Node("direct_mte_teardown_%d" % os.getpid())
with _multi_threaded_construction_test_only():
    teardown_executor = MultiThreadedExecutor(num_threads=2)
teardown_executor.add_node(teardown_node)

teardown_thread, teardown_spin_errors = spin_in_background(teardown_executor)
deadline = time.monotonic() + 10.0
while teardown_executor._background_thread is None and time.monotonic() < deadline:
    time.sleep(0.01)
native_thread = teardown_executor._background_thread
assert native_thread is not None
assert native_thread.running
assert teardown_executor.is_spinning

# Shutdown-during-spin: the pump is blocked in _wake_event.wait(); shutdown()
# must wake it, join the native thread, and return with no error.
assert teardown_executor.shutdown(timeout_sec=10.0) is True
teardown_thread.join(timeout=10.0)
assert not teardown_thread.is_alive()
assert teardown_spin_errors == [], teardown_spin_errors

assert native_thread.closed
assert not native_thread.running
assert native_thread.exceptions == 0
assert teardown_executor.get_nodes() == []
assert teardown_executor.is_spinning is False
assert not any(
    thread.name == "mte-helper-spin" and thread.is_alive()
    for thread in threading.enumerate()
)
teardown_node.destroy_node()
print("DIRECT_CPP_MTE_TEARDOWN_OK", flush=True)

rclpy.shutdown()
assert not rclpy.ok()
print("DIRECT_CPP_MTE_ALL_OK", flush=True)
