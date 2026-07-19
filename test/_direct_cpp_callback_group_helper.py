#!/usr/bin/env python3
"""Live callback-group ownership proof for direct C++ entities."""

import importlib
import os
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclcppyy.policy import BackendUnavailableError  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.callback_groups import (  # noqa: E402
    CallbackGroup,
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import UInt64  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402
from tf2_msgs.action import LookupTransform  # noqa: E402


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("a Python-message conversion or serialization path ran")


bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
serialization = importlib.import_module("rclcpp_kit.serialization")
bringup.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary


def cpp_name(value):
    return str(
        getattr(type(value), "__cpp_name__", "")
        or getattr(value, "__cpp_name__", "")
    )


assert issubclass(MutuallyExclusiveCallbackGroup, CallbackGroup)
assert issubclass(ReentrantCallbackGroup, CallbackGroup)
rclpy.init(args=[])
node = Node("direct_groups_%d" % os.getpid())
peer = Node("direct_groups_peer_%d" % os.getpid())
executor = SingleThreadedExecutor(context=node.context)
assert executor.add_node(node)

default_group = node.default_callback_group
native_default = (
    node._direct_cpp_node.get_node_base_interface().get_default_callback_group()
)
assert "rclcpp::CallbackGroup" in cpp_name(default_group.native_group)
assert default_group.native_group.__smartptr__() == native_default.__smartptr__()
assert node.callback_groups == (default_group,)

reentrant = ReentrantCallbackGroup()
exclusive = MutuallyExclusiveCallbackGroup()
prefix = "/direct_cpp/groups/p%d" % os.getpid()
received = []
timer_firings = []

publisher = node.create_publisher(
    UInt64, prefix + "/messages", 10, callback_group=reentrant)
subscription = node.create_subscription(
    UInt64,
    prefix + "/messages",
    lambda message, info: received.append((message, info)),
    10,
    callback_group=reentrant,
)
timer = node.create_timer(
    0.001, lambda: timer_firings.append(time.monotonic_ns()),
    callback_group=exclusive)


def service_callback(request, response):
    assert type(request) is SetBool.Request
    assert type(response) is SetBool.Response
    response.success = request.data
    response.message = "grouped"
    return response


service = node.create_service(
    SetBool, prefix + "/service", service_callback,
    callback_group=exclusive)
client = node.create_client(
    SetBool, prefix + "/service", callback_group=reentrant)
action_client = ActionClient(
    node, LookupTransform, prefix + "/action", callback_group=exclusive)

for entity, group in (
    (publisher, reentrant),
    (subscription, reentrant),
    (timer, exclusive),
    (service, exclusive),
    (client, reentrant),
    (action_client, exclusive),
):
    assert entity.callback_group is group
    assert group.has_entity(entity)
assert node.callback_groups == (default_group, reentrant, exclusive)
assert "rclcpp::CallbackGroup" in cpp_name(reentrant.native_group)
assert "rclcpp::CallbackGroup" in cpp_name(exclusive.native_group)
print("DIRECT_CPP_CALLBACK_GROUP_NATIVE_OWNERSHIP_OK", flush=True)

before = len(tuple(peer.publishers))
try:
    peer.create_publisher(
        UInt64, prefix + "/cross_node", 10, callback_group=reentrant)
except BackendUnavailableError:
    pass
else:
    raise AssertionError("a native callback group was shared across nodes")
assert len(tuple(peer.publishers)) == before
assert peer.callback_groups == (peer.default_callback_group,)
print("DIRECT_CPP_CALLBACK_GROUP_CROSS_NODE_FAIL_CLOSED_OK", flush=True)

deadline = time.monotonic() + 10.0
while (
    publisher.get_subscription_count() < 1
    or not client.service_is_ready()
    or not timer_firings
) and time.monotonic() < deadline:
    executor.spin_once(timeout_sec=0.05)
assert publisher.get_subscription_count() == 1
assert client.service_is_ready()
assert timer_firings
timer.cancel()

publisher.publish(UInt64(data=31))
while not received and time.monotonic() < deadline:
    executor.spin_once(timeout_sec=0.05)
assert len(received) == 1
message, info = received[0]
assert type(message) is UInt64
assert int(message.data) == 31
assert set(info) == {
    "source_timestamp",
    "received_timestamp",
    "publication_sequence_number",
    "reception_sequence_number",
}

future = client.call_async(SetBool.Request(data=True))
executor.spin_until_future_complete(future, timeout_sec=10.0)
assert future.done() and not future.cancelled()
response = future.result()
assert type(response) is SetBool.Response
assert response.success is True
assert str(response.message) == "grouped"

assert exclusive.can_execute(timer)
assert exclusive.beginning_execution(timer)
assert not exclusive.can_execute(service)
exclusive.ending_execution(timer)
assert exclusive.can_execute(service)
assert reentrant.can_execute(subscription)
assert reentrant.beginning_execution(subscription)
reentrant.ending_execution(subscription)
print("DIRECT_CPP_CALLBACK_GROUP_EXECUTION_OK", flush=True)

assert node.destroy_subscription(subscription)
assert node.destroy_publisher(publisher)
assert node.destroy_timer(timer)
assert node.destroy_client(client)
assert node.destroy_service(service)
action_client.destroy()
for entity, group in (
    (publisher, reentrant),
    (subscription, reentrant),
    (timer, exclusive),
    (service, exclusive),
    (client, reentrant),
    (action_client, exclusive),
):
    assert not group.has_entity(entity)

peer.destroy_node()
node.destroy_node()
assert executor.get_nodes() == []
assert reentrant.node is None
assert exclusive.node is None
assert executor.shutdown(timeout_sec=2.0)
rclpy.shutdown()
assert not rclpy.ok()
print("DIRECT_CPP_CALLBACK_GROUP_TEARDOWN_OK", flush=True)
