#!/usr/bin/env python3
"""Clock, callback-group, autostart, and error semantics for compiled timers."""

import os
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclcpp_kit import python_callback_entities  # noqa: E402


def spin_until(executor, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    assert predicate(), "compiled timer did not complete before timeout"


def same_smart_pointer(left, right):
    return left.__smartptr__() == right.__smartptr__()


rclpy.init(args=[])
node = Node("direct_compiled_timer_bridge_%d" % os.getpid())
executor = SingleThreadedExecutor()
executor.add_node(node)
group = ReentrantCallbackGroup()
factory_calls = []
original_factory = python_callback_entities.create_python_clock_timer


def tracked_factory(owner, native_node, clock, period_ns, callback, **kwargs):
    factory_calls.append((native_node, clock, period_ns, callback, kwargs))
    return original_factory(
        owner, native_node, clock, period_ns, callback, **kwargs)


python_callback_entities.create_python_clock_timer = tracked_factory
ticks = []
timer = node.create_timer(
    0.005, lambda: ticks.append(True), callback_group=group,
    clock=node.get_clock(), autostart=False)
assert len(factory_calls) == 1
native_node, passed_clock, period_ns, contained_callback, kwargs = factory_calls[0]
assert period_ns == 5_000_000
assert callable(contained_callback)
assert kwargs["autostart"] is False
assert same_smart_pointer(passed_clock, node.get_clock()._raw_native_clock())
assert same_smart_pointer(kwargs["callback_group"], group.native_group)
assert timer.callback_group is group
assert group.has_entity(timer)
assert timer.creation_route == "rclcpp_clock_timer"
assert timer.managed.callback_handoff == "compiled_python_callback"
assert timer.managed.source_id
assert timer.is_canceled()
for _ in range(3):
    executor.spin_once(timeout_sec=0.01)
assert ticks == []
timer.reset()
spin_until(executor, lambda: bool(ticks))
decision = [
    item for item in rclcppyy.status()["entities"]
    if item["metadata"].get("entity_type") == "timer"
][-1]
assert decision["metadata"]["callback_handoff"] == "compiled_python_callback"
assert decision["metadata"]["source_id"] == timer.managed.source_id
assert "compiled_python_callback" in decision["policies"]
assert node.destroy_timer(timer)
assert not group.has_entity(timer)


def fail_callback():
    raise LookupError("compiled timer callback exception sentinel")


failing_timer = node.create_timer(0.001, fail_callback, callback_group=group)
deadline = time.monotonic() + 5.0
while time.monotonic() < deadline:
    try:
        executor.spin_once(timeout_sec=0.05)
    except LookupError as exc:
        assert str(exc) == "compiled timer callback exception sentinel"
        break
else:
    raise AssertionError("compiled timer exception was not re-raised")
assert node.destroy_timer(failing_timer)
assert executor.shutdown(timeout_sec=5.0)
node.destroy_node()
rclpy.shutdown()
print("DIRECT_CPP_COMPILED_TIMER_BRIDGE_OK", flush=True)
