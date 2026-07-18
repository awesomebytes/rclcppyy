#!/usr/bin/env python3
"""End-to-end compatible-profile proof in an isolated interpreter."""

import json
import os
import time

import rclcppyy

rclcppyy.enable_cpp_acceleration()

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.publisher import Publisher  # noqa: E402
from std_msgs.msg import String  # noqa: E402


SPIN_DEADLINE_S = 15.0
N_MESSAGES = 5
NAMESPACE = "/rclcppyy_compatible"
TOPIC = NAMESPACE + "/roundtrip"


def main():
    assert not hasattr(String, "__smartptr__")
    print("MESSAGE_CONTRACT_OK", flush=True)

    context = rclpy.context.Context()
    context.init(args=[])
    node_name = "compatible_%d" % os.getpid()
    node = rclpy.create_node(node_name, namespace=NAMESPACE, context=context)
    assert type(node) is Node
    assert node.context is context
    assert not rclpy.ok(), "default context must remain uninitialized"
    print("NODE_AUTHORITY_OK", flush=True)
    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(node)

    received = []
    subscription = node.create_subscription(
        String, TOPIC, lambda message: received.append(message.data), 10)
    publisher = node.create_publisher(String, TOPIC, 10)
    assert type(publisher) is Publisher

    deadline = time.monotonic() + SPIN_DEADLINE_S
    while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert publisher.get_subscription_count() >= 1

    expected = ["compatible hello %d" % index for index in range(N_MESSAGES)]
    for payload in expected:
        publisher.publish(String(data=payload))
    while len(received) < N_MESSAGES and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert received == expected, received
    print("ROUNDTRIP_OK", flush=True)

    identity = (node_name, NAMESPACE)
    assert node.get_node_names_and_namespaces().count(identity) == 1
    assert not any(name.endswith("_rclcpp") for name, _ in node.get_node_names_and_namespaces())
    endpoint_identities = [
        (item.node_name, item.node_namespace)
        for item in node.get_publishers_info_by_topic(TOPIC)
    ]
    assert endpoint_identities == [identity], endpoint_identities
    print("GRAPH_IDENTITY_OK", flush=True)

    status = rclcppyy.status()
    json.dumps(status)
    assert status["counts"]["nodes"]["python"] >= 1, status
    entity_backends = {
        record["metadata"].get("entity_type"): record["backend"]
        for record in status["entities"]
    }
    assert entity_backends["publisher"] == "cpp", status
    assert entity_backends["subscription"] == "python", status
    assert any(
        record["metadata"].get("operation") == "enable_cpp_acceleration"
        for record in status["operations"]
    )
    print("STATUS_OK", flush=True)

    assert node.destroy_publisher(publisher)
    assert node.destroy_subscription(subscription)
    executor.remove_node(node)
    node.destroy_node()
    executor.shutdown(timeout_sec=1.0)
    context.shutdown()
    print("STOCK_TEARDOWN_OK", flush=True)
    print("MONKEYPATCH_ALL_OK", flush=True)


if __name__ == "__main__":
    main()
