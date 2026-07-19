#!/usr/bin/env python3
"""Live product proof for registered nested generated-C++ messages."""

import gc
import importlib
import os
import time

import rclcppyy


INTERFACE = "std_msgs/msg/Header"
OPTIMIZATION = "subscription_shared_lease"


def reject_before_activation(profile, interfaces, error_type):
    try:
        rclcppyy.enable_cpp_acceleration(
            profile=profile, interfaces=interfaces)
    except error_type:
        pass
    else:
        raise AssertionError("invalid direct interface request activated rclcppyy")
    assert rclcppyy._ACTIVE_PROFILE is None
    assert rclcppyy._ACTIVE_INTERFACES == ()


reject_before_activation("compatible", (INTERFACE,), ValueError)
reject_before_activation("direct_cpp", ("std_msgs/Header",), ValueError)
reject_before_activation("direct_cpp", (object(),), TypeError)
print("DIRECT_CPP_GENERIC_INTERFACE_FAIL_CLOSED_OK", flush=True)

assert rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp",
    optimizations=(OPTIMIZATION,),
    interfaces=(INTERFACE, INTERFACE),
)
assert rclcppyy._ACTIVE_INTERFACES == (INTERFACE,)
assert rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp",
    optimizations=OPTIMIZATION,
    interfaces=INTERFACE,
)
try:
    rclcppyy.enable_cpp_acceleration(
        profile="direct_cpp",
        optimizations=OPTIMIZATION,
        interfaces=(INTERFACE, "std_msgs/msg/Float64"),
    )
except RuntimeError:
    pass
else:
    raise AssertionError("active direct_cpp accepted a different interface registry")

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Float64, Header  # noqa: E402


assert Header is cppyy.gbl.std_msgs.msg.Header
assert Time is cppyy.gbl.builtin_interfaces.msg.Time
assert not hasattr(Float64, "__smartptr__")
stamp = Time(sec=-17, nanosec=987654321)
constructed = Header(stamp=stamp, frame_id="constructed-in-cpp")
assert int(constructed.stamp.sec) == -17
assert int(constructed.stamp.nanosec) == 987654321
assert str(constructed.frame_id) == "constructed-in-cpp"
print("DIRECT_CPP_GENERIC_CONSTRUCTORS_OK", flush=True)


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("a conversion or serialization bridge ran")


bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
serialization = importlib.import_module("rclcpp_kit.serialization")
bringup.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary


class HeaderPair(Node):
    def __init__(self):
        super().__init__("direct_generic_header_%d" % os.getpid())
        topic = "/direct_cpp/generic/header/p%d" % os.getpid()
        self.received = []
        self.publisher = self.create_publisher(Header, topic, 10)
        self.subscription = self.create_subscription(
            Header, topic, self.received.append, 10)


def spin_until(node, condition):
    deadline = time.monotonic() + 10.0
    while not condition() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert condition()


rclpy.init(args=[])
node = HeaderPair()
before = (len(tuple(node.publishers)), len(tuple(node.subscriptions)))
try:
    node.create_publisher(Float64, "/direct_cpp/unregistered", 10)
except TypeError:
    pass
else:
    raise AssertionError("unregistered Python message class created a direct publisher")
assert (
    len(tuple(node.publishers)), len(tuple(node.subscriptions))) == before

spin_until(node, lambda: node.publisher.get_subscription_count() == 1)
message = Header()
message.stamp.sec = 31
message.stamp.nanosec = 42
message.frame_id = "exact-generated-cpp"
node.publisher.publish(message)
spin_until(node, lambda: len(node.received) == 1)

received = node.received[0]
assert type(received) is Header
assert int(received.stamp.sec) == 31
assert int(received.stamp.nanosec) == 42
assert str(received.frame_id) == "exact-generated-cpp"
wrapper = node._direct_cpp_subscriptions[0]._native
assert wrapper.creation_route == "rclcpp_unique_ptr_subscription_lease"
assert wrapper.owning_cpp_copy_count == 0
assert wrapper.lease_count == 1
assert cppyy.addressof(received) == wrapper.last_message_address
stats = wrapper.stats()
assert stats.message_deep_copies == 0
assert stats.shared_control_blocks == 1
assert stats.shared_owner_acquisitions == 1
assert stats.python_boundary_crossings == 1
assert stats.exceptions == 0
print("DIRECT_CPP_GENERIC_NESTED_LEASE_OK", flush=True)

status = rclcppyy.status()
activation = next(
    item for item in status["operations"]
    if item["metadata"].get("operation") == "enable_cpp_acceleration"
)
assert activation["metadata"]["requested_message_interfaces"] == [INTERFACE]
assert set(activation["metadata"]["message_types"]) == {
    "std_msgs::msg::Header",
    "builtin_interfaces::msg::Time",
    "std_msgs::msg::String",
    "std_msgs::msg::UInt64",
}
header_entities = [
    item for item in status["entities"]
    if item["metadata"].get("message_type") ==
    "std_msgs::msg::Header_<std::allocator<void>>"
]
assert len(header_entities) == 2
assert all(item["backend"] == "cpp" for item in header_entities)
assert all("no_conversion" in item["policies"] for item in header_entities)
print("DIRECT_CPP_GENERIC_EVIDENCE_OK", flush=True)

node.received.clear()
node.publisher = None
node.subscription = None
node.destroy_node()
gc.collect()
assert wrapper.closed
assert str(received.frame_id) == "exact-generated-cpp"
rclpy.shutdown()
gc.collect()
received.frame_id = "retained-after-shutdown"
received.stamp.sec = -9
assert str(received.frame_id) == "retained-after-shutdown"
assert int(received.stamp.sec) == -9
print("DIRECT_CPP_GENERIC_RETAINED_TEARDOWN_OK", flush=True)
