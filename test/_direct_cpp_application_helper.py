#!/usr/bin/env python3
"""Unchanged rclpy-style app slice backed by direct_cpp authority."""

import importlib
import os
import time

import rclcppyy

rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("Python-message conversion or serialization ran")


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


class UnchangedAppNode(Node):
    """Ordinary Node subclass and rclpy-shaped callbacks, with no adapters."""


def spin_until(node, predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert predicate(), "application condition did not complete before timeout"


rclpy.init(args=[])
suffix = "p%d" % os.getpid()
node = UnchangedAppNode("direct_application_" + suffix)
topic = "/direct_cpp/application/" + suffix
service_name = "/direct_cpp/application/set_bool/" + suffix
received = []
timer_ticks = []

publisher = node.create_publisher(String, topic, 10)
subscription = node.create_subscription(
    String, topic, lambda message: received.append(message.data), 10)


def handle_set_bool(request, response):
    response.success = request.data
    response.message = "application:true" if request.data else "application:false"
    return response


service = node.create_service(SetBool, service_name, handle_set_bool)
client = node.create_client(SetBool, service_name)
timer = node.create_timer(0.01, lambda: timer_ticks.append("tick"))

runtime = importlib.import_module("rclcppyy.direct_cpp")._runtime()
assert runtime.nodes == [node]
assert runtime.session.nodes == (node._direct_cpp_node,)
assert node.context is runtime.context
assert "rclcpp::Node" in str(
    getattr(type(node._direct_cpp_node), "__cpp_name__", ""))
assert String is cppyy.gbl.std_msgs.msg.String
assert SetBool.Request is cppyy.gbl.std_srvs.srv.SetBool.Request

spin_until(node, lambda: publisher.get_subscription_count() == 1)
publisher.publish(String(data="unchanged-app"))
spin_until(node, lambda: received == ["unchanged-app"])
assert str(received[0]) == "unchanged-app"

assert client.wait_for_service(timeout_sec=5.0)
future = client.call_async(SetBool.Request(data=True))
spin_until(node, future.done)
response = future.result()
assert type(response) is SetBool.Response
assert response.success is True
assert str(response.message) == "application:true"

spin_until(node, lambda: bool(timer_ticks))

status = rclcppyy.status()
node_records = [
    record for record in status["nodes"]
    if record["metadata"].get("name") == node.get_name()
]
assert len(node_records) == 1
assert node_records[0]["backend"] == "cpp"
assert "native_node_authority" in node_records[0]["policies"]

expected = {
    "publisher": "/direct_cpp/application/" + suffix,
    "subscription": "/direct_cpp/application/" + suffix,
    "service": service_name,
    "client": service_name,
    "timer": None,
}
entities = [
    record for record in status["entities"]
    if record["metadata"].get("entity_type") in expected
]
assert len(entities) == len(expected), status["entities"]
for record in entities:
    kind = record["metadata"]["entity_type"]
    assert record["backend"] == "cpp", (kind, record)
    assert "no_conversion" in record["policies"], (kind, record)
    if "direct_cpp_message" in record["policies"]:
        assert record["metadata"].get("python_message_conversions", 0) == 0
    if expected[kind] is not None:
        assert record["metadata"].get(
            "topic", record["metadata"].get("service_name")) == expected[kind]

assert node.destroy_timer(timer)
assert node.destroy_client(client)
assert node.destroy_service(service)
assert node.destroy_subscription(subscription)
assert node.destroy_publisher(publisher)
node.destroy_node()
rclpy.shutdown()
print("DIRECT_CPP_APPLICATION_AUTHORITY_OK")
print("DIRECT_CPP_APPLICATION_NO_CONVERSION_OK")
print("DIRECT_CPP_APPLICATION_TEARDOWN_OK")
