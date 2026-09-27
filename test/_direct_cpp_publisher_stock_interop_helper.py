#!/usr/bin/env python3
"""Publish one exact generated C++ message to an unchanged stock subscriber."""

import gc
import importlib
import inspect
import os
from pathlib import Path
import select
import subprocess
import sys
import time

import rclcppyy


INTERFACE = "sensor_msgs/msg/JointState"
rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp", interfaces=(INTERFACE,))

import cppyy  # noqa: E402
import rclpy  # noqa: E402
import rclpy.serialization as rclpy_serialization  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402
from rclpy.exceptions import InvalidHandle  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.topic_endpoint_info import TopicEndpointTypeEnum  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Header  # noqa: E402


assert JointState is cppyy.gbl.sensor_msgs.msg.JointState
assert Header is cppyy.gbl.std_msgs.msg.Header
assert Time is cppyy.gbl.builtin_interfaces.msg.Time


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError(
        "Python conversion, serialization, or CDR bridge entered publish")


bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
serialization = importlib.import_module("rclcpp_kit.serialization")
bringup.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary
serialization.serialized_message_from_bytes = forbidden_boundary
serialization.serialized_message_to_bytes = forbidden_boundary
rclpy_serialization.serialize_message = forbidden_boundary
rclpy_serialization.deserialize_message = forbidden_boundary


def spin_until(node, predicate, label, timeout=20.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
    if not predicate():
        raise TimeoutError("timed out waiting for " + label)


def wait_for_marker(node, process, marker, timeout=20.0):
    lines = []
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        ready, _, _ = select.select([process.stdout], [], [], 0)
        if ready:
            line = process.stdout.readline()
            if line:
                lines.append(line)
                if marker in line:
                    return lines
        if process.poll() is not None:
            break
    stderr = process.stderr.read() if process.poll() is not None else ""
    raise AssertionError(
        "timed out waiting for %s; stdout=%r stderr=%r" %
        (marker, "".join(lines), stderr)
    )


rclpy.init(args=[])
suffix = str(os.getpid())
direct_node_name = "direct_joint_state_publisher_" + suffix
stock_node_name = "stock_joint_state_subscriber_" + suffix
node = Node(direct_node_name)
topic = "/direct_cpp/stock_joint_state/p" + suffix
publisher = node.create_publisher(JointState, topic, 10)
assert not inspect.ismethod(publisher.publish)
assert "rclcpp::Publisher" in str(
    getattr(type(publisher.native_entity), "__cpp_name__", ""))

message = JointState()
assert type(message) is cppyy.gbl.sensor_msgs.msg.JointState
message.header.stamp.sec = 1700000017
message.header.stamp.nanosec = 987654321
message.header.frame_id = "direct-cpp/joint-frame alpha"
for name in ("shoulder/joint", "elbow joint", "wrist:joint"):
    message.name.push_back(name)
for value in (1.25, -2.5, 3.75):
    message.position.push_back(value)
for value in (0.125, 0.25, -0.5):
    message.velocity.push_back(value)
for value in (10.0, -20.0, 30.5):
    message.effort.push_back(value)

peer_path = Path(__file__).with_name("_stock_joint_state_subscriber_peer.py")
peer = subprocess.Popen(
    [
        sys.executable,
        str(peer_path),
        topic,
        stock_node_name,
        direct_node_name,
    ],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    text=True,
    env=os.environ.copy(),
)
peer_lines = []
try:
    peer_lines.extend(wait_for_marker(
        node, peer, "STOCK_JOINT_STATE_SUBSCRIBER_GRAPH_OK"))
    assert publisher.get_subscription_count() == 1
    assert node.count_subscribers(topic) == 1
    endpoints = node.get_subscriptions_info_by_topic(topic)
    matching = [
        endpoint for endpoint in endpoints
        if endpoint.node_name == stock_node_name
    ]
    assert len(matching) == 1
    endpoint = matching[0]
    assert endpoint.node_namespace == "/"
    assert endpoint.topic_type == INTERFACE
    assert endpoint.endpoint_type == TopicEndpointTypeEnum.SUBSCRIPTION
    assert len(endpoint.endpoint_gid) == 16
    assert endpoint.topic_type_hash.version > 0
    print("DIRECT_CPP_PUBLISHER_STOCK_GRAPH_OK", flush=True)

    publisher.publish(message)
    deadline = time.monotonic() + 20.0
    while peer.poll() is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
    if peer.poll() is None:
        peer.kill()
    stdout, stderr = peer.communicate(timeout=5.0)
    peer_stdout = "".join(peer_lines) + stdout
    assert peer.returncode == 0, (
        "stock subscriber exit=%s\nstdout:\n%s\nstderr:\n%s" %
        (peer.returncode, peer_stdout, stderr)
    )
finally:
    if peer.poll() is None:
        peer.kill()
        peer.wait(timeout=5.0)

assert "STOCK_JOINT_STATE_SUBSCRIBER_PAYLOAD_OK" in peer_stdout
assert (
    "STOCK_JOINT_STATE_SUBSCRIBER_RETAINED_TEARDOWN_OK"
    in peer_stdout
)
spin_until(
    node,
    lambda: (
        publisher.get_subscription_count() == 0
        and node.count_subscribers(topic) == 0
    ),
    "stock subscriber graph teardown",
)
print("DIRECT_CPP_PUBLISHER_STOCK_PAYLOAD_OK", flush=True)

records = [
    item for item in rclcppyy.status()["entities"]
    if item["metadata"].get("entity_type") == "publisher"
    and item["metadata"].get("topic") == topic
]
assert len(records) == 1
record = records[0]
assert record["backend"] == "cpp"
assert "direct_cpp_message" in record["policies"]
assert "no_conversion" in record["policies"]
assert "sensor_msgs::msg::JointState" in record["metadata"]["message_type"]
print("DIRECT_CPP_PUBLISHER_STOCK_NO_BRIDGE_OK", flush=True)

native = publisher._native
assert node.destroy_publisher(publisher)
assert publisher.closed
assert native.closed()
try:
    publisher.native_entity
except InvalidHandle:
    pass
else:
    raise AssertionError("destroyed direct publisher retained its native entity")
node.destroy_node()
rclpy.shutdown()
gc.collect()
assert type(message) is JointState
assert str(message.header.frame_id) == "direct-cpp/joint-frame alpha"
assert [str(value) for value in message.name] == [
    "shoulder/joint", "elbow joint", "wrist:joint"]
assert [float(value) for value in message.position] == [1.25, -2.5, 3.75]
message.name.push_back("retained-after-shutdown")
assert str(message.name.back()) == "retained-after-shutdown"
print("DIRECT_CPP_PUBLISHER_STOCK_RETAINED_TEARDOWN_OK", flush=True)
