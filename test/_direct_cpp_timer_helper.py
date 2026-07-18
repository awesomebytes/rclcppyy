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


def expect_rejected(node, operation):
    before = len(node.timers)
    try:
        operation()
    except (BackendUnavailableError, TypeError, ValueError):
        pass
    else:
        raise AssertionError("unsupported direct timer request succeeded")
    assert len(node.timers) == before


rclpy.init(args=[])
node = Node("direct_timer_%d" % os.getpid())
runtime = importlib.import_module("rclcppyy.direct_cpp")._runtime()

expect_rejected(node, lambda: node.create_timer(0.001, lambda: None, callback_group=object()))
expect_rejected(node, lambda: node.create_timer(0.001, lambda: None, clock=object()))
expect_rejected(node, lambda: node.create_timer(0.001, lambda: None, autostart=False))
expect_rejected(node, lambda: node.create_timer(0.001, lambda: None, oneshot=False))
expect_rejected(node, lambda: node.create_timer(0.001, lambda: None, options=object()))
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
timer = node.create_timer(0.001, callback)
del callback
gc.collect()
assert callback_ref() is not None
assert timer.timer_period_ns == 1_000_000
assert "rclcpp::WallTimer" in timer.__cpp_name__
assert timer.creation_route == "rclcpp_wall_timer"
deadline = time.monotonic() + 5.0
while callback_ref().count < 3 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
assert callback_ref().count >= 3
timer.cancel()
assert timer.is_canceled()
canceled_count = callback_ref().count
for _ in range(3):
    rclpy.spin_once(node, timeout_sec=0.002)
assert callback_ref().count == canceled_count
timer.reset()
assert not timer.is_canceled()
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
for _ in range(3):
    rclpy.spin_once(node, timeout_sec=0.002)
assert retained_callback.count == canceled_count + 1
print("DIRECT_CPP_TIMER_CONTROL_OK")


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
assert len(records) == 2
assert all(item["backend"] == "cpp" for item in records)
assert all("native_timer_authority" in item["policies"] for item in records)
assert all("no_conversion" in item["policies"] for item in records)
assert all(item["metadata"]["callback_handoff"] == "direct_std_function" for item in records)
assert all("rclcpp::WallTimer" in item["metadata"]["native_type"] for item in records)

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
