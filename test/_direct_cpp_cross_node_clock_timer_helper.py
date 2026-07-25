#!/usr/bin/env python3
"""Cross-node ``DirectClock`` timer proof for ``create_timer(clock=...)``.

Covers: a node's timer may honor *another* node's ``DirectClock`` (extracted
via ``Node.get_clock()``); a standalone stock ``Clock(clock_type=...)``
still fails closed; and the borrowed clock outlives the node that produced
it, since the timer holds its own ``shared_ptr<rclcpp::Clock>`` copy
independent of the owning node's.
"""

import importlib
import os
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclcppyy.policy import BackendUnavailableError  # noqa: E402
from rclpy.clock import Clock  # noqa: E402
from rclpy.clock_type import ClockType  # noqa: E402
from rclpy.node import Node  # noqa: E402


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError(
        "conversion or serialization entered cross-node clock timer inspection")


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


rclpy.init(args=[])
node_a = Node("direct_cross_clock_a_%d" % os.getpid())
node_b = Node("direct_cross_clock_b_%d" % os.getpid())

# 0. A genuine standalone stock Clock is still rejected -- fail closed, not
# just the generic clock=object() case already covered elsewhere.
try:
    node_b.create_timer(
        0.001, lambda: None, clock=Clock(clock_type=ClockType.STEADY_TIME))
except BackendUnavailableError:
    pass
else:
    raise AssertionError("standalone stock Clock accepted by create_timer")
print("DIRECT_CPP_CROSS_CLOCK_STANDALONE_REJECTED_OK")

# 1. node_b's timer honors node_a's DirectClock: it fires, and the record
# metadata reflects the foreign-clock path, not the node's own default.
ticks = []
timer = node_b.create_timer(
    0.02, lambda: ticks.append(1), clock=node_a.get_clock())
assert "GenericTimer" in timer.native_type_name
assert timer.creation_route == "rclcpp_clock_timer"
deadline = time.monotonic() + 5.0
while not ticks and time.monotonic() < deadline:
    rclpy.spin_once(node_b, timeout_sec=0.05)
assert ticks, "node_b timer never fired on node_a's borrowed clock"
records = [
    item for item in rclcppyy.status()["entities"]
    if item["metadata"].get("entity_type") == "timer"
]
assert records[-1]["metadata"]["clock"] == "foreign_direct_clock"
print("DIRECT_CPP_CROSS_CLOCK_TICK_OK")

# 2. Destroy node_a while node_b's timer -- which retains its own
# shared_ptr<rclcpp::Clock> copy handed out by node_a's DirectClock -- is
# still running. The clock must outlive node_a: no crash, and the timer
# keeps ticking driven by the (still-alive) borrowed clock object.
node_a.destroy_node()
ticks.clear()
deadline = time.monotonic() + 5.0
while not ticks and time.monotonic() < deadline:
    rclpy.spin_once(node_b, timeout_sec=0.05)
assert ticks, "node_b timer stopped firing after node_a was destroyed"
print("DIRECT_CPP_CROSS_CLOCK_OUTLIVES_NODE_OK")

assert node_b.destroy_timer(timer)
node_b.destroy_node()
rclpy.shutdown()
print("DIRECT_CPP_CROSS_CLOCK_TEARDOWN_OK")
