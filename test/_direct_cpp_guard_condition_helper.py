#!/usr/bin/env python3
"""Live proof: DirectGuardCondition dispatch, teardown, and exception containment."""

import os
import threading
import time

os.environ.setdefault("ROS_DOMAIN_ID", "82")

import rclcppyy  # noqa: E402


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402


TIMEOUT_S = 5.0


def _settle(predicate):
    deadline = time.monotonic() + TIMEOUT_S
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.01)
    return predicate()


rclpy.init()
node = Node("direct_guard_condition_%d" % os.getpid())

calls = []
guard = node.create_guard_condition(lambda: calls.append(True))

assert list(node.guards) == [guard]
assert guard.callback_group is node.default_callback_group
print("DIRECT_CPP_GUARD_CONDITION_CREATE_OK")


def _trigger_from_thread():
    guard.trigger()


thread = threading.Thread(target=_trigger_from_thread)
thread.start()
thread.join()
assert _settle(lambda: len(calls) == 1)
print("DIRECT_CPP_GUARD_CONDITION_TRIGGER_OK")

guard.trigger()
assert _settle(lambda: len(calls) == 2)
print("DIRECT_CPP_GUARD_CONDITION_RETRIGGER_OK")

# A raising callback must not crash the triggering thread or this guard
# condition's own private dispatch thread -- it is contained the same way
# every other native-dispatched callback is, and surfaces via the same
# executor.spin_once() drain path (defect A, see PLAN-mte-unlock.md), with
# no guard-condition-specific executor integration needed for that to work.
raise_calls = []


def _raising_callback():
    raise_calls.append(True)
    raise RuntimeError("boom from guard condition callback")


raising_guard = node.create_guard_condition(_raising_callback)
executor = SingleThreadedExecutor()
executor.add_node(node)
raising_guard.trigger()
assert _settle(lambda: len(raise_calls) == 1)
caught = None
deadline = time.monotonic() + TIMEOUT_S
while caught is None and time.monotonic() < deadline:
    try:
        executor.spin_once(timeout_sec=0.2)
    except RuntimeError as exc:
        caught = exc
assert caught is not None and "boom from guard condition callback" in str(caught)
print("DIRECT_CPP_GUARD_CONDITION_EXCEPTION_CONTAINED_OK")
node.destroy_guard_condition(raising_guard)

# destroy_guard_condition() stops dispatch: it is removed from node.guards,
# and a post-destroy trigger() fails closed instead of silently doing
# nothing (there is no live native guard condition left to signal).
node.destroy_guard_condition(guard)
assert list(node.guards) == []
try:
    guard.trigger()
except RuntimeError:
    pass
else:
    raise AssertionError("trigger() on a destroyed guard condition did not raise")
print("DIRECT_CPP_GUARD_CONDITION_DESTROY_OK")

executor.remove_node(node)
executor.shutdown()
node.destroy_node()
rclpy.shutdown()
print("DIRECT_CPP_GUARD_CONDITION_TEARDOWN_OK")
