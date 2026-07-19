#!/usr/bin/env python3
"""Unchanged stock rclpy subscriber for direct-publisher interop."""

import gc
import sys
import time

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.topic_endpoint_info import TopicEndpointTypeEnum
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


def spin_until(node, predicate, label, timeout=20.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
    if not predicate():
        raise TimeoutError("timed out waiting for " + label)


def main():
    if len(sys.argv) != 4:
        raise SystemExit(
            "usage: helper.py TOPIC STOCK_NODE_NAME DIRECT_NODE_NAME")
    topic, stock_node_name, direct_node_name = sys.argv[1:]
    assert JointState.__module__ == "sensor_msgs.msg._joint_state"
    assert Header.__module__ == "std_msgs.msg._header"
    assert Time.__module__ == "builtin_interfaces.msg._time"
    assert not hasattr(JointState, "__smartptr__")

    rclpy.init(args=[])
    node = Node(stock_node_name)
    received = []
    subscription = node.create_subscription(
        JointState, topic, received.append, 10)
    try:
        spin_until(
            node,
            lambda: node.count_publishers(topic) == 1,
            "direct publisher discovery",
        )
        endpoints = node.get_publishers_info_by_topic(topic)
        matching = [
            endpoint for endpoint in endpoints
            if endpoint.node_name == direct_node_name
        ]
        assert len(matching) == 1
        endpoint = matching[0]
        assert endpoint.node_namespace == "/"
        assert endpoint.topic_type == "sensor_msgs/msg/JointState"
        assert endpoint.endpoint_type == TopicEndpointTypeEnum.PUBLISHER
        assert len(endpoint.endpoint_gid) == 16
        assert endpoint.topic_type_hash.version > 0
        print("STOCK_JOINT_STATE_SUBSCRIBER_GRAPH_OK", flush=True)

        spin_until(node, lambda: len(received) == 1, "direct C++ message")
        message = received[0]
        assert type(message) is JointState
        assert type(message.header) is Header
        assert type(message.header.stamp) is Time
        assert message.header.stamp.sec == 1700000017
        assert message.header.stamp.nanosec == 987654321
        assert message.header.frame_id == "direct-cpp/joint-frame alpha"
        assert list(message.name) == [
            "shoulder/joint", "elbow joint", "wrist:joint"]
        assert list(message.position) == [1.25, -2.5, 3.75]
        assert list(message.velocity) == [0.125, 0.25, -0.5]
        assert list(message.effort) == [10.0, -20.0, 30.5]
        print("STOCK_JOINT_STATE_SUBSCRIBER_PAYLOAD_OK", flush=True)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.try_shutdown()

    gc.collect()
    assert message.header.frame_id == "direct-cpp/joint-frame alpha"
    assert list(message.name) == [
        "shoulder/joint", "elbow joint", "wrist:joint"]
    message.header.frame_id = "stock-retained-after-shutdown"
    assert message.header.frame_id == "stock-retained-after-shutdown"
    print("STOCK_JOINT_STATE_SUBSCRIBER_RETAINED_TEARDOWN_OK", flush=True)


if __name__ == "__main__":
    main()
