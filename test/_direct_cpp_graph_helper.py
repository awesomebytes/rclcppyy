#!/usr/bin/env python3
"""Live rclpy-shaped graph proof backed by one native rclcpp graph."""

import importlib
import os

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import cppyy  # noqa: E402
import rclpy  # noqa: E402
import rclpy._rclpy_pybind11 as _rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("a Python-message conversion or serialization path ran")


bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
serialization = importlib.import_module("rclcpp_kit.serialization")
bringup.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary


rclpy.init(args=[
    "--ros-args",
    "-r", "chatter:=renamed",
    "-r", "toggle:=renamed_toggle",
])
suffix = str(os.getpid())
observer = Node("direct_graph_observer_" + suffix)
target = Node("direct_graph_target_" + suffix, namespace="/direct_graph")

publisher = target.create_publisher(String, "chatter", 10)
subscription = observer.create_subscription(String, "/direct_graph/renamed", lambda _m: None, 10)
service = target.create_service(SetBool, "toggle", lambda request, response: response)
client = observer.create_client(SetBool, "/direct_graph/renamed_toggle")

assert String is cppyy.gbl.std_msgs.msg.String
assert SetBool.Request is cppyy.gbl.std_srvs.srv.SetBool_Request
assert observer.resolve_topic_name("chatter") == "/renamed"
assert observer.resolve_topic_name("chatter", only_expand=True) == "/chatter"
assert target.resolve_topic_name("chatter") == "/direct_graph/renamed"
assert target.resolve_topic_name("chatter", only_expand=True) == "/direct_graph/chatter"
assert target.resolve_service_name("toggle") == "/direct_graph/renamed_toggle"
assert target.resolve_service_name("toggle", only_expand=True) == "/direct_graph/toggle"

identities = observer.get_node_names_and_namespaces()
target_identity = (target.get_name(), target.get_namespace())
assert target_identity in identities
assert target.get_name() in observer.get_node_names()
assert target.get_fully_qualified_name() in observer.get_fully_qualified_node_names()
assert any(row[:2] == target_identity and len(row) == 3
           for row in observer.get_node_names_and_namespaces_with_enclaves())
assert observer.wait_for_node(target.get_fully_qualified_name(), timeout=1.0)
assert not observer.wait_for_node("/missing_direct_graph", timeout=0.01)

topics = dict(observer.get_topic_names_and_types())
services = dict(observer.get_service_names_and_types())
assert "std_msgs/msg/String" in topics["/direct_graph/renamed"]
assert "std_srvs/srv/SetBool" in services["/direct_graph/renamed_toggle"]

publisher_topics = dict(observer.get_publisher_names_and_types_by_node(*target_identity))
subscriber_topics = dict(observer.get_subscriber_names_and_types_by_node(
    observer.get_name(), observer.get_namespace()))
service_names = dict(observer.get_service_names_and_types_by_node(*target_identity))
client_names = dict(observer.get_client_names_and_types_by_node(
    observer.get_name(), observer.get_namespace()))
assert "std_msgs/msg/String" in publisher_topics["/direct_graph/renamed"]
assert "std_msgs/msg/String" in subscriber_topics["/direct_graph/renamed"]
assert "std_srvs/srv/SetBool" in service_names["/direct_graph/renamed_toggle"]
assert "std_srvs/srv/SetBool" in client_names["/direct_graph/renamed_toggle"]

assert observer.count_publishers("/direct_graph/renamed") == 1
assert observer.count_subscribers("/direct_graph/renamed") == 1
assert observer.count_services("/direct_graph/renamed_toggle") == 1
assert observer.count_clients("/direct_graph/renamed_toggle") == 1
assert target.count_publishers("chatter") == 0
assert target.count_services("toggle") == 0

for operation in (
    lambda: target.resolve_topic_name("bad name"),
    lambda: target.count_publishers("bad name"),
    lambda: target.resolve_service_name("bad name"),
    lambda: target.count_services("bad name"),
):
    try:
        operation()
    except ValueError:
        pass
    else:
        raise AssertionError("invalid graph name reached the native exception boundary")
print("DIRECT_CPP_GRAPH_INVALID_NAME_FAIL_CLOSED_OK", flush=True)

try:
    observer.get_publisher_names_and_types_by_node("missing", "/")
except _rclpy.NodeNameNonExistentError:
    pass
else:
    raise AssertionError("missing graph node did not preserve the rclpy exception")

print("DIRECT_CPP_GRAPH_CPP_AUTHORITY_OK", flush=True)
print("DIRECT_CPP_GRAPH_RCLPY_SHAPE_OK", flush=True)

assert target.destroy_publisher(publisher)
assert observer.destroy_subscription(subscription)
assert target.destroy_service(service)
assert observer.destroy_client(client)
target.destroy_node()
observer.destroy_node()
rclpy.shutdown()
assert not rclpy.ok()
print("DIRECT_CPP_GRAPH_TEARDOWN_OK", flush=True)
