#!/usr/bin/env python3
"""Live proof for the bounded direct-C++ source-compatible profile."""

import gc
import importlib
import os
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from rclcppyy.policy import BackendUnavailableError  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Float64, String, UInt64  # noqa: E402


assert String is cppyy.gbl.std_msgs.msg.String
assert UInt64 is cppyy.gbl.std_msgs.msg.UInt64
assert String(data="constructor").data == "constructor"
assert UInt64(data=2**63 + 19).data == 2**63 + 19
print("DIRECT_CPP_CONSTRUCTORS_OK")


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("a Python-message conversion or serialization path ran")


bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
serialization = importlib.import_module("rclcpp_kit.serialization")
bringup.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary


class DirectPair(Node):
    def __init__(self):
        super().__init__("direct_pair_%d" % os.getpid())
        prefix = "/direct_cpp/p%d" % os.getpid()
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


def assert_rejected_without_entity(node, operation):
    before = (len(node.publishers), len(node.subscriptions))
    try:
        operation()
    except (BackendUnavailableError, TypeError):
        pass
    else:
        raise AssertionError("unsupported direct_cpp entity request succeeded")
    assert (len(node.publishers), len(node.subscriptions)) == before


rclpy.init(args=[])
node = DirectPair()

direct_module = importlib.import_module("rclcppyy.direct_cpp")
runtime = direct_module._runtime()
assert runtime.nodes == [node]
assert runtime.session.nodes == (node._direct_cpp_node,)
assert node.context is runtime.context
try:
    Node("rejected_options", cli_args=[])
except BackendUnavailableError:
    pass
else:
    raise AssertionError("unsupported node options created a native node")
assert runtime.nodes == [node]
assert runtime.session.nodes == (node._direct_cpp_node,)

prefix = "/direct_cpp/rejected/p%d" % os.getpid()
assert_rejected_without_entity(
    node, lambda: node.create_publisher(Float64, prefix + "/type", 10))
assert_rejected_without_entity(
    node, lambda: node.create_publisher(String, prefix + "/qos", object()))
assert_rejected_without_entity(
    node,
    lambda: node.create_publisher(
        String, prefix + "/group", 10, callback_group=object()),
)
assert_rejected_without_entity(
    node,
    lambda: node.create_publisher(
        String, prefix + "/event", 10, event_callbacks=object()),
)
assert_rejected_without_entity(
    node,
    lambda: node.create_publisher(
        String, prefix + "/override", 10, qos_overriding_options=object()),
)
assert_rejected_without_entity(
    node,
    lambda: node.create_publisher(
        String, prefix + "/custom", 10, publisher_class=object()),
)
assert_rejected_without_entity(
    node,
    lambda: node.create_subscription(
        String, prefix + "/raw", lambda _message: None, 10, raw=True),
)
assert_rejected_without_entity(
    node,
    lambda: node.create_subscription(
        String,
        prefix + "/filter",
        lambda _message: None,
        10,
        content_filter_options=object(),
    ),
)
print("DIRECT_CPP_FAIL_CLOSED_OK")

assert not hasattr(node.string_publisher, "_rclcppyy_publish_route")
assert not hasattr(node.string_subscription, "_rclcppyy_take_route")
spin_until(
    node,
    lambda: (
        node.string_publisher.get_subscription_count() >= 1
        and node.uint64_publisher.get_subscription_count() >= 1
    ),
)

node.string_publisher.publish(String(data="first"))
node.string_publisher.publish(String(data="second"))
node.uint64_publisher.publish(UInt64(data=7))
node.uint64_publisher.publish(UInt64(data=2**63 + 9))
spin_until(node, lambda: len(node.strings) == 2 and len(node.integers) == 2)

assert all(type(message) is String for message in node.strings)
assert all(type(message) is UInt64 for message in node.integers)
assert [str(message.data) for message in node.strings] == ["first", "second"]
assert [int(message.data) for message in node.integers] == [7, 2**63 + 9]
retained_string = node.strings[0]
retained_integer = node.integers[0]
second_string_address = cppyy.addressof(node.strings[1])
gc.collect()
node.strings[1].data = "changed"
node.integers[1].data = 99
assert str(retained_string.data) == "first"
assert int(retained_integer.data) == 7
assert cppyy.addressof(retained_string) != second_string_address
print("DIRECT_CPP_MESSAGES_OK")

records = rclcppyy.status()
assert any("native_node_authority" in item["policies"] for item in records["nodes"])
direct_entities = [
    item for item in records["entities"]
    if item["backend"] == "cpp" and "direct_cpp_message" in item["policies"]
]
assert len(direct_entities) == 4
assert all("no_conversion" in item["policies"] for item in direct_entities)
subscription_records = [
    item for item in direct_entities
    if item["metadata"].get("entity_type") == "subscription"
]
assert len(subscription_records) == 2
assert all(
    "owning_cpp_callback_copy" in item["policies"]
    and item["metadata"].get("callback_handoff") == "one_native_cpp_copy"
    for item in subscription_records
)

node.destroy_node()
assert runtime.nodes == []
assert runtime.session.nodes == ()
rclpy.shutdown()
assert not rclpy.ok()
assert runtime.session is None
print("DIRECT_CPP_TEARDOWN_OK")
