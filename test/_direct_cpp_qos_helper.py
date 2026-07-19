#!/usr/bin/env python3
"""Live endpoint proof for direct-profile rclpy QoS configuration."""

import importlib
import os
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import UInt64  # noqa: E402


assert UInt64 is cppyy.gbl.std_msgs.msg.UInt64


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("QoS configuration used a message conversion boundary")


bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
serialization = importlib.import_module("rclcpp_kit.serialization")
bringup.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary


def native_profile(qos):
    profile = qos.get_rmw_qos_profile()
    return {
        "history": int(profile.history),
        "depth": int(profile.depth),
        "reliability": int(profile.reliability),
        "durability": int(profile.durability),
        "deadline_ns": int(profile.deadline.sec) * 1_000_000_000 + int(
            profile.deadline.nsec),
        "lifespan_ns": int(profile.lifespan.sec) * 1_000_000_000 + int(
            profile.lifespan.nsec),
        "liveliness": int(profile.liveliness),
        "liveliness_lease_duration_ns": int(
            profile.liveliness_lease_duration.sec) * 1_000_000_000 + int(
                profile.liveliness_lease_duration.nsec),
        "avoid_ros_namespace_conventions": bool(
            profile.avoid_ros_namespace_conventions),
    }


def explicit_profile():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=7,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        deadline=Duration(nanoseconds=50_000_001),
        lifespan=Duration(nanoseconds=90_000_002),
        liveliness=LivelinessPolicy.AUTOMATIC,
        liveliness_lease_duration=Duration(nanoseconds=120_000_003),
        avoid_ros_namespace_conventions=False,
    )


def unknown_profile():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=3,
        reliability=ReliabilityPolicy.UNKNOWN,
        durability=DurabilityPolicy.VOLATILE,
        deadline=Duration(),
        lifespan=Duration(),
        liveliness=LivelinessPolicy.AUTOMATIC,
        liveliness_lease_duration=Duration(),
        avoid_ros_namespace_conventions=False,
    )


def spin_until(node, condition):
    deadline = time.monotonic() + 10.0
    while not condition() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert condition()


def assert_rejected_before_creation(node, topic, operation):
    before = (len(tuple(node.publishers)), len(tuple(node.subscriptions)))
    try:
        operation()
    except (TypeError, ValueError):
        pass
    else:
        raise AssertionError("invalid QoS created a direct entity")
    assert (len(tuple(node.publishers)), len(tuple(node.subscriptions))) == before
    assert int(node._direct_cpp_node.count_publishers(topic)) == 0
    assert int(node._direct_cpp_node.count_subscribers(topic)) == 0


rclpy.init()
node = Node("direct_qos_product_%d" % os.getpid())
prefix = "/direct_cpp/qos/p%d" % os.getpid()

assert_rejected_before_creation(
    node,
    prefix + "/object",
    lambda: node.create_publisher(UInt64, prefix + "/object", object()),
)
assert_rejected_before_creation(
    node,
    prefix + "/bool",
    lambda: node.create_publisher(UInt64, prefix + "/bool", True),
)
assert_rejected_before_creation(
    node,
    prefix + "/negative",
    lambda: node.create_subscription(
        UInt64, prefix + "/negative", lambda _message: None, -1),
)
assert_rejected_before_creation(
    node,
    prefix + "/unknown",
    lambda: node.create_subscription(
        UInt64, prefix + "/unknown", lambda _message: None, unknown_profile()),
)
print("DIRECT_CPP_QOS_FAIL_CLOSED_OK")

explicit_messages = []
requested = explicit_profile()
explicit_topic = prefix + "/explicit"
explicit_publisher = node.create_publisher(UInt64, explicit_topic, requested)
explicit_subscription = node.create_subscription(
    UInt64, explicit_topic, explicit_messages.append, requested)
assert explicit_publisher.qos_profile is requested
assert explicit_subscription.qos_profile is requested
spin_until(node, lambda: explicit_publisher.get_subscription_count() == 1)
explicit_publisher.publish(UInt64(data=41))
spin_until(node, lambda: len(explicit_messages) == 1)
assert type(explicit_messages[0]) is UInt64
assert int(explicit_messages[0].data) == 41

expected = {
    "history": int(HistoryPolicy.KEEP_LAST),
    "depth": 7,
    "reliability": int(ReliabilityPolicy.RELIABLE),
    "durability": int(DurabilityPolicy.VOLATILE),
    "deadline_ns": 50_000_001,
    "lifespan_ns": 90_000_002,
    "liveliness": int(LivelinessPolicy.AUTOMATIC),
    "liveliness_lease_duration_ns": 120_000_003,
    "avoid_ros_namespace_conventions": False,
}
assert native_profile(explicit_publisher.native_entity.get_actual_qos()) == expected
subscription_actual = native_profile(
    explicit_subscription.native_entity.get_actual_qos())
assert subscription_actual == {
    **expected,
    "lifespan_ns": 9_223_372_036_854_775_807,
}
print("DIRECT_CPP_QOS_EXPLICIT_ENDPOINT_OK")

sensor_messages = []
sensor_topic = prefix + "/sensor"
sensor_publisher = node.create_publisher(
    UInt64, sensor_topic, qos_profile_sensor_data)
sensor_subscription = node.create_subscription(
    UInt64, sensor_topic, sensor_messages.append, qos_profile_sensor_data)
assert sensor_publisher.qos_profile is qos_profile_sensor_data
assert sensor_subscription.qos_profile is qos_profile_sensor_data
spin_until(node, lambda: sensor_publisher.get_subscription_count() == 1)
sensor_publisher.publish(UInt64(data=73))
spin_until(node, lambda: len(sensor_messages) == 1)
assert type(sensor_messages[0]) is UInt64
assert int(sensor_messages[0].data) == 73
for actual in (
    native_profile(sensor_publisher.native_entity.get_actual_qos()),
    native_profile(sensor_subscription.native_entity.get_actual_qos()),
):
    assert actual["history"] == int(HistoryPolicy.KEEP_LAST)
    assert actual["depth"] == 5
    assert actual["reliability"] == int(ReliabilityPolicy.BEST_EFFORT)
    assert actual["durability"] == int(DurabilityPolicy.VOLATILE)
print("DIRECT_CPP_QOS_SENSOR_ENDPOINT_OK")

node.destroy_node()
rclpy.shutdown()
print("DIRECT_CPP_QOS_TEARDOWN_OK")
