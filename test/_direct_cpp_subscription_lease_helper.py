#!/usr/bin/env python3
"""Live opt-in proof for retained actual-C++ subscription leases."""

import gc
import importlib
import os
import sys
import time

import rclcppyy


OPTIMIZATION = "subscription_shared_lease"


def rejected_before_activation(profile, optimizations, error_type):
    try:
        rclcppyy.enable_cpp_acceleration(
            profile=profile, optimizations=optimizations)
    except error_type:
        pass
    else:
        raise AssertionError("invalid optimization request activated rclcppyy")
    assert rclcppyy._ACTIVE_PROFILE is None
    assert rclcppyy._ACTIVE_OPTIMIZATIONS == ()
    assert "rclcppyy.direct_cpp" not in sys.modules


rejected_before_activation("compatible", (OPTIMIZATION,), ValueError)
rejected_before_activation("direct_cpp", ("unknown_optimization",), ValueError)
rejected_before_activation("direct_cpp", (OPTIMIZATION, object()), TypeError)
print("DIRECT_CPP_SUBSCRIPTION_LEASE_FAIL_CLOSED_OK", flush=True)

assert rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp",
    optimizations=(OPTIMIZATION, OPTIMIZATION),
)
assert rclcppyy._ACTIVE_OPTIMIZATIONS == (OPTIMIZATION,)
assert rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp", optimizations=OPTIMIZATION)
try:
    rclcppyy.enable_cpp_acceleration(profile="direct_cpp")
except RuntimeError:
    pass
else:
    raise AssertionError("active direct_cpp accepted different optimizations")
print("DIRECT_CPP_SUBSCRIPTION_LEASE_IDEMPOTENCE_OK", flush=True)

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String, UInt64  # noqa: E402


assert String is cppyy.gbl.std_msgs.msg.String
assert UInt64 is cppyy.gbl.std_msgs.msg.UInt64


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("a Python-message conversion or serialization path ran")


bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
serialization = importlib.import_module("rclcpp_kit.serialization")
bringup.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary


class LeasePair(Node):
    def __init__(self, suffix):
        super().__init__("direct_lease_pair_%s_%d" % (suffix, os.getpid()))
        prefix = "/direct_cpp/lease/%s/p%d" % (suffix, os.getpid())
        self.strings = []
        self.integers = []
        self.string_publisher = self.create_publisher(
            String, prefix + "/string", 10)
        self.uint64_publisher = self.create_publisher(
            UInt64, prefix + "/uint64", 10)
        self.string_subscription = self.create_subscription(
            String, prefix + "/string", self.strings.append, 10)
        self.uint64_subscription = self.create_subscription(
            UInt64, prefix + "/uint64", self.integers.append, 10)


def spin_until(node, condition):
    deadline = time.monotonic() + 10.0
    while not condition() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert condition()


def publish_pair(node, string_values, integer_values):
    spin_until(
        node,
        lambda: (
            node.string_publisher.get_subscription_count() >= 1
            and node.uint64_publisher.get_subscription_count() >= 1
        ),
    )
    for value in string_values:
        node.string_publisher.publish(String(data=value))
    for value in integer_values:
        node.uint64_publisher.publish(UInt64(data=value))
    spin_until(
        node,
        lambda: (
            len(node.strings) == len(string_values)
            and len(node.integers) == len(integer_values)
        ),
    )


rclpy.init(args=[])
node = LeasePair("first")
runtime = importlib.import_module("rclcppyy.direct_cpp")._runtime()
assert runtime.optimizations == (OPTIMIZATION,)
publish_pair(node, ("first", "second"), (7, 2**63 + 9))

assert all(type(message) is String for message in node.strings)
assert all(type(message) is UInt64 for message in node.integers)
assert [str(message.data) for message in node.strings] == ["first", "second"]
assert [int(message.data) for message in node.integers] == [7, 2**63 + 9]

wrappers = tuple(node._direct_cpp_subscriptions)
assert len(wrappers) == 2
for wrapper in wrappers:
    stats = wrapper.stats()
    assert wrapper.creation_route == "rclcpp_unique_ptr_subscription_lease"
    assert wrapper.owning_cpp_copy_count == 0
    assert wrapper.lease_count == 2
    assert stats.leases == 2
    assert stats.message_deep_copies == 0
    assert stats.shared_control_blocks == 2
    assert stats.shared_owner_acquisitions == 2
    assert stats.python_boundary_crossings == 2
    assert stats.exceptions == 0

string_wrapper = next(wrapper for wrapper in wrappers if wrapper.cpp_type is String)
integer_wrapper = next(wrapper for wrapper in wrappers if wrapper.cpp_type is UInt64)
assert cppyy.addressof(node.strings[-1]) == string_wrapper.last_message_address
assert cppyy.addressof(node.integers[-1]) == integer_wrapper.last_message_address

records = rclcppyy.status()
operation = next(
    item for item in records["operations"]
    if item["metadata"].get("operation") == "enable_cpp_acceleration"
)
assert operation["metadata"]["optimizations"] == [OPTIMIZATION]
subscription_records = [
    item for item in records["entities"]
    if OPTIMIZATION in item["policies"]
]
assert len(subscription_records) == 2
for record in subscription_records:
    assert {OPTIMIZATION, "actual_cpp_message", "no_conversion"} <= set(
        record["policies"])
    metadata = record["metadata"]
    assert metadata["callback_handoff"] == "shared_cpp_message_lease"
    assert metadata["subscription_creation_route"] == (
        "rclcpp_unique_ptr_subscription_lease")
    assert metadata["message_representation"] == "actual_cpp"
    assert metadata["python_message_conversions"] == 0
    assert metadata["serialization_operations"] == 0
    assert metadata["message_deep_copies_per_callback"] == 0
    assert metadata["shared_control_blocks_per_callback"] == 1
    assert metadata["shared_owner_acquisitions_per_callback"] == 1
    assert metadata["owning_cpp_copy_count_at_creation"] == 0
    assert metadata["lease_count_at_creation"] == 0
print("DIRECT_CPP_SUBSCRIPTION_LEASE_MESSAGES_OK", flush=True)

retained_string = node.strings[0]
retained_integer = node.integers[0]
node.strings.clear()
node.integers.clear()
node.string_subscription = None
node.uint64_subscription = None
node.string_publisher = None
node.uint64_publisher = None
node.destroy_node()
assert all(wrapper.closed for wrapper in wrappers)
gc.collect()
assert str(retained_string.data) == "first"
assert int(retained_integer.data) == 7
rclpy.shutdown()
gc.collect()
retained_string.data = "retained-after-shutdown"
retained_integer.data = 101
assert str(retained_string.data) == "retained-after-shutdown"
assert int(retained_integer.data) == 101
print("DIRECT_CPP_SUBSCRIPTION_LEASE_RETAINED_OK", flush=True)

rclpy.init(args=[])
reinitialized = LeasePair("reinit")
publish_pair(reinitialized, ("reinitialized",), (303,))
assert str(reinitialized.strings[0].data) == "reinitialized"
assert int(reinitialized.integers[0].data) == 303
assert all(
    wrapper.lease_count == 1
    for wrapper in reinitialized._direct_cpp_subscriptions
)
reinitialized.destroy_node()
rclpy.shutdown()
assert not rclpy.ok()
print("DIRECT_CPP_SUBSCRIPTION_LEASE_REINIT_OK", flush=True)
