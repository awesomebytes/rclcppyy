#!/usr/bin/env python3
"""Live native-timer, exception, and bounded-spin proof for direct_cpp."""

import gc
import importlib
import math
import os
import threading
import time
import weakref

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclcppyy.policy import BackendUnavailableError  # noqa: E402
from rclpy.node import Node  # noqa: E402


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("conversion or serialization entered timer inspection")


kit = importlib.import_module("rclcpp_kit")
bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
serialization = importlib.import_module("rclcpp_kit.serialization")
rclpy_serialization = importlib.import_module("rclpy.serialization")
kit.convert_python_msg_to_cpp = forbidden_boundary
bringup.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary
rclpy_serialization.serialize_message = forbidden_boundary
rclpy_serialization.deserialize_message = forbidden_boundary


def expect_rejected(node, operation, exceptions=(BackendUnavailableError, TypeError, ValueError)):
    before = len(node.timers)
    try:
        operation()
    except exceptions:
        pass
    else:
        raise AssertionError("unsupported direct timer request succeeded")
    assert len(node.timers) == before


rclpy.init(args=[])
node = Node("direct_timer_%d" % os.getpid())
runtime = importlib.import_module("rclcppyy.direct_cpp")._runtime()

expect_rejected(node, lambda: node.create_timer(0.001, lambda: None, callback_group=object()))
expect_rejected(node, lambda: node.create_timer(0.001, lambda: None, clock=object()))
# node.get_clock() -- the node's own clock -- is the one non-None clock=
# create_timer accepts: it routes to the same node-ROS-clock GenericTimer the
# default (clock=None) uses, so this is a normal, ticking timer, not a
# rejection. Standalone/foreign clocks (clock=object() above) still fail
# closed -- a clock this node cannot ever produce is out of scope, not just
# unimplemented.
clock_timer_ticks = []
clock_timer = node.create_timer(
    0.01, lambda: clock_timer_ticks.append(1), clock=node.get_clock())
assert "GenericTimer" in clock_timer.native_type_name
assert clock_timer.creation_route == "rclcpp_clock_timer"
clock_timer_deadline = time.monotonic() + 5.0
while not clock_timer_ticks and time.monotonic() < clock_timer_deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
assert clock_timer_ticks
assert node.destroy_timer(clock_timer)
expect_rejected(node, lambda: node.create_timer(0.001, lambda: None, autostart=object()))
expect_rejected(
    node, lambda: node.create_timer(0.001, lambda: None, oneshot=False), TypeError)
expect_rejected(
    node, lambda: node.create_timer(0.001, lambda: None, options=object()), TypeError)
expect_rejected(node, lambda: node.create_timer(0.0, lambda: None))
expect_rejected(node, lambda: node.create_timer(-1.0, lambda: None))
expect_rejected(node, lambda: node.create_timer(math.inf, lambda: None))
expect_rejected(node, lambda: node.create_timer(math.nan, lambda: None))
expect_rejected(node, lambda: node.create_timer(True, lambda: None))
expect_rejected(node, lambda: node.create_timer(0.001, object()))
try:
    rclpy.spin(node, executor=object())
except BackendUnavailableError:
    pass
else:
    raise AssertionError("direct spin accepted a public executor")
print("DIRECT_CPP_TIMER_FAIL_CLOSED_OK")


class RetainedCallback:
    def __init__(self):
        self.count = 0

    def __call__(self):
        self.count += 1


callback = RetainedCallback()
callback_ref = weakref.ref(callback)
timer = node.create_timer(0.05, callback, autostart=False)
del callback
gc.collect()
assert callback_ref() is not None
assert timer.timer_period_ns == 50_000_000
assert "GenericTimer" in timer.__cpp_name__
assert timer.creation_route == "rclcpp_clock_timer"
assert timer.is_canceled()
assert not timer.is_ready()
assert timer.time_until_next_call() is None
assert timer.time_since_last_call() >= 0
for _ in range(3):
    rclpy.spin_once(node, timeout_sec=0.002)
assert callback_ref().count == 0
timer.reset()
assert not timer.is_canceled()
reset_until = timer.time_until_next_call()
assert isinstance(reset_until, int) and 0 < reset_until <= timer.timer_period_ns
assert not timer.is_ready()
deadline = time.monotonic() + 5.0
while not timer.is_ready() and time.monotonic() < deadline:
    time.sleep(0.001)
assert timer.is_ready()
assert timer.time_until_next_call() <= 0
rclpy.spin_once(node, timeout_sec=0.05)
assert callback_ref().count == 1
post_until = timer.time_until_next_call()
post_since = timer.time_since_last_call()
assert isinstance(post_until, int) and post_until <= timer.timer_period_ns
assert isinstance(post_since, int) and post_since >= 0
deadline = time.monotonic() + 5.0
while callback_ref().count < 3 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
assert callback_ref().count >= 3
timer.cancel()
assert timer.is_canceled()
assert not timer.is_ready()
assert timer.time_until_next_call() is None
canceled_since = timer.time_since_last_call()
assert canceled_since >= 0
canceled_count = callback_ref().count
for _ in range(3):
    rclpy.spin_once(node, timeout_sec=0.002)
assert callback_ref().count == canceled_count
assert timer.time_since_last_call() >= canceled_since
timer.reset()
assert not timer.is_canceled()
reset_until = timer.time_until_next_call()
assert isinstance(reset_until, int) and 0 < reset_until <= timer.timer_period_ns
assert not timer.is_ready()
deadline = time.monotonic() + 5.0
while callback_ref().count == canceled_count and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
assert callback_ref().count == canceled_count + 1
retained_callback = callback_ref()
assert node.destroy_timer(timer)
assert not node.destroy_timer(timer)
assert len(node.timers) == 0
try:
    timer.reset()
except RuntimeError as exc:
    assert "destroyed" in str(exc)
else:
    raise AssertionError("destroyed direct timer was reset")
for method in (
    timer.is_ready,
    timer.time_until_next_call,
    timer.time_since_last_call,
):
    try:
        method()
    except RuntimeError as exc:
        assert "destroyed" in str(exc)
    else:
        raise AssertionError("destroyed direct timer inspection succeeded")
assert timer.timer_period_ns == 50_000_000
for _ in range(3):
    rclpy.spin_once(node, timeout_sec=0.002)
assert retained_callback.count == canceled_count + 1
print("DIRECT_CPP_TIMER_CONTROL_OK")
print("DIRECT_CPP_TIMER_INSPECTION_OK")


def fail_callback():
    raise RuntimeError("direct timer callback sentinel")


failing_timer = node.create_timer(0.001, fail_callback)
deadline = time.monotonic() + 5.0
while True:
    try:
        rclpy.spin_once(node, timeout_sec=0.05)
    except RuntimeError as exc:
        assert "direct timer callback sentinel" in str(exc)
        break
    assert time.monotonic() < deadline
assert node.destroy_timer(failing_timer)
print("DIRECT_CPP_TIMER_EXCEPTION_OK")


records = [
    item for item in rclcppyy.status()["entities"]
    if item["metadata"].get("entity_type") == "timer"
]
# Three timers were created above: the destroyed clock=node.get_clock() proof,
# then timer, then failing_timer -- record_decision logs at creation time and
# is never retracted on destroy, so the destroyed one still appears.
assert len(records) == 3
assert all(item["backend"] == "cpp" for item in records)
assert all("native_timer_authority" in item["policies"] for item in records)
assert all("no_conversion" in item["policies"] for item in records)
assert all(item["metadata"]["callback_handoff"] == "direct_std_function" for item in records)
assert all("GenericTimer" in item["metadata"]["native_type"] for item in records)
assert [item["metadata"]["autostart"] for item in records] == [True, False, True]
assert all(item["metadata"]["clock"] == "ros" for item in records)
assert all(
    item["metadata"]["ros_clock_support"] == "managed_clock_timers"
    for item in records
)

spin_done = threading.Event()
spin_errors = []


def run_spin():
    try:
        rclpy.spin(node)
    except BaseException as exc:
        spin_errors.append(exc)
    finally:
        spin_done.set()


thread = threading.Thread(target=run_spin, name="direct-cpp-spin")
thread.start()
time.sleep(0.15)
assert thread.is_alive()
rclpy.shutdown()
thread.join(timeout=2.0)
assert spin_done.is_set()
assert not thread.is_alive()
assert spin_errors == [], repr(spin_errors)
assert not rclpy.ok()
assert runtime.session is None
assert runtime.executor is None
assert runtime.nodes == []
assert node._direct_cpp_node is None
print("DIRECT_CPP_SPIN_INTERRUPT_OK")
