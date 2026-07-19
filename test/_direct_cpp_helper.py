#!/usr/bin/env python3
"""Live proof for the bounded direct-C++ source-compatible profile."""

import gc
import importlib
import inspect
import os
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from rclcppyy.policy import BackendUnavailableError  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.exceptions import InvalidHandle  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.publisher import Publisher  # noqa: E402
from rclpy.qos import QoSProfile  # noqa: E402
from rclpy.subscription import Subscription  # noqa: E402
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
    before = (len(tuple(node.publishers)), len(tuple(node.subscriptions)))
    try:
        operation()
    except (BackendUnavailableError, TypeError):
        pass
    else:
        raise AssertionError("unsupported direct_cpp entity request succeeded")
    assert (
        len(tuple(node.publishers)), len(tuple(node.subscriptions))) == before


rclpy.init(args=["--ros-args", "-r", "remap_from:=remap_to"])
node = DirectPair()

direct_module = importlib.import_module("rclcppyy.direct_cpp")
runtime = direct_module._runtime()
assert runtime.nodes == [node]
assert runtime.session.nodes == (node._direct_cpp_node,)
assert node.context is runtime.context
assert Publisher is direct_module.DirectPublisher
assert Subscription is direct_module.DirectSubscription
try:
    Node("rejected_options", cli_args=("--ros-args",))
except (BackendUnavailableError, TypeError):
    pass
else:
    raise AssertionError("invalid node options created a native node")
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


async def coroutine_callback(_message):
    return None


assert_rejected_without_entity(
    node,
    lambda: node.create_subscription(
        String, prefix + "/coroutine", coroutine_callback, 10),
)
print("DIRECT_CPP_FAIL_CLOSED_OK")

assert isinstance(node.string_publisher, Publisher)
assert isinstance(node.string_subscription, Subscription)
assert tuple(node.publishers) == (
    node.string_publisher, node.uint64_publisher)
assert tuple(node.subscriptions) == (
    node.string_subscription, node.uint64_subscription)
assert node.string_publisher.msg_type is String
assert node.string_subscription.msg_type is String
assert node.string_publisher.topic == node.string_publisher.topic_name
assert node.string_subscription.topic == node.string_subscription.topic_name
assert node.string_publisher.logger_name == node.get_name()
assert node.string_subscription.logger_name == node.get_name()
assert isinstance(node.string_publisher.qos_profile, QoSProfile)
assert node.string_publisher.qos_profile.depth == 10
assert isinstance(node.string_subscription.qos_profile, QoSProfile)
assert node.string_subscription.qos_profile.depth == 10
assert node.string_publisher.event_handlers == []
assert node.string_subscription.event_handlers == []
assert node.string_subscription.callback == node.strings.append
assert node.string_publisher.callback_group is node.default_callback_group
assert node.string_subscription.callback_group is node.default_callback_group
assert node.default_callback_group.has_entity(node.string_publisher)
assert node.default_callback_group.has_entity(node.string_subscription)
assert node.string_subscription.raw is False
assert node.string_subscription._callback_type is (
    Subscription.CallbackType.MessageOnly)
assert "rclcpp::Publisher" in str(
    getattr(type(node.string_publisher.native_entity), "__cpp_name__", ""))
assert "rclcpp::Subscription" in str(
    getattr(type(node.string_subscription.native_entity), "__cpp_name__", ""))
assert not inspect.ismethod(node.string_publisher.publish)
for entity in (node.string_publisher, node.string_subscription):
    try:
        entity.handle
    except BackendUnavailableError:
        pass
    else:
        raise AssertionError("direct facade exposed a stock rclpy handle")
try:
    node.string_subscription.callback = lambda _message: None
except BackendUnavailableError:
    pass
else:
    raise AssertionError("direct subscription callback replacement succeeded")
print("DIRECT_CPP_FACADES_OK")

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
assert node.string_subscription.get_publisher_count() == 1
assert node.string_publisher.assert_liveliness() is None
assert node.string_publisher.wait_for_all_acked(Duration(seconds=2))

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

namespaced = Node(
    "direct_namespaced_%d" % os.getpid(), namespace="/facade_namespace")
remapped_messages = []
remapped_publisher = namespaced.create_publisher(String, "remap_from", 10)
remapped_subscription = namespaced.create_subscription(
    String, "remap_from", remapped_messages.append, 10)
assert remapped_publisher.topic == "/facade_namespace/remap_to"
assert remapped_publisher.topic_name == "/facade_namespace/remap_to"
assert remapped_subscription.topic == "/facade_namespace/remap_to"
assert remapped_subscription.topic_name == "/facade_namespace/remap_to"
assert remapped_publisher.logger_name == namespaced.get_logger().name
assert remapped_subscription.logger_name == namespaced.get_logger().name
spin_until(namespaced, lambda: remapped_publisher.get_subscription_count() == 1)
remapped_publisher.publish(String(data="remapped-cpp"))
spin_until(namespaced, lambda: len(remapped_messages) == 1)
assert type(remapped_messages[0]) is String
assert str(remapped_messages[0].data) == "remapped-cpp"
namespaced.destroy_node()
assert runtime.session.nodes == (node._direct_cpp_node,)
print("DIRECT_CPP_NAMESPACED_REMAP_OK")

foreign = Node("direct_foreign_%d" % os.getpid())
assert not foreign.destroy_publisher(node.string_publisher)
assert not foreign.destroy_subscription(node.string_subscription)
foreign.destroy_node()
assert runtime.session.nodes == (node._direct_cpp_node,)

assert node.destroy_publisher(node.string_publisher)
assert not node.destroy_publisher(node.string_publisher)
assert tuple(node.publishers) == (node.uint64_publisher,)
spin_until(node, lambda: node.string_subscription.get_publisher_count() == 0)
try:
    node.string_publisher.native_entity
except InvalidHandle:
    pass
else:
    raise AssertionError("destroyed publisher exposed its native entity")
try:
    node.string_publisher.publish(String(data="destroyed"))
except Exception as exc:
    assert "destroyed" in str(exc)
else:
    raise AssertionError("destroyed publisher still published")

assert node.destroy_subscription(node.string_subscription)
assert not node.destroy_subscription(node.string_subscription)
assert tuple(node.subscriptions) == (node.uint64_subscription,)
try:
    node.string_subscription.native_entity
except InvalidHandle:
    pass
else:
    raise AssertionError("destroyed subscription exposed its native entity")

assert node.uint64_subscription.destroy() is None
assert tuple(node.subscriptions) == (node.uint64_subscription,)
spin_until(node, lambda: node.uint64_publisher.get_subscription_count() == 0)
assert not node.destroy_subscription(node.uint64_subscription)
assert tuple(node.subscriptions) == ()
assert node.uint64_publisher.destroy() is None
assert tuple(node.publishers) == (node.uint64_publisher,)
assert not node.destroy_publisher(node.uint64_publisher)
assert tuple(node.publishers) == ()
print("DIRECT_CPP_LIFECYCLE_OK")

node.destroy_node()
node.destroy_node()
assert runtime.nodes == []
assert runtime.session.nodes == ()
rclpy.shutdown()
assert not rclpy.ok()
assert runtime.session is None
print("DIRECT_CPP_TEARDOWN_OK")
