#!/usr/bin/env python3
"""Focused product proof for the compiled direct callback bridge."""

import os
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import UInt64  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402


def spin_until(executor, predicate, timeout=10.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert predicate(), "callback bridge did not complete before timeout"


rclpy.init(args=[])
node = Node("direct_compiled_callback_bridge_%d" % os.getpid())
executor = SingleThreadedExecutor()
executor.add_node(node)
from rclcpp_kit import direct_entities  # noqa: E402

reaper_pump_calls = []
original_reaper_drain = direct_entities.drain_callable_reaper


def tracked_reaper_drain():
    reaper_pump_calls.append(True)
    return original_reaper_drain()


direct_entities.drain_callable_reaper = tracked_reaper_drain
prefix = "/direct_cpp/compiled_callback_bridge/p%d" % os.getpid()
group = MutuallyExclusiveCallbackGroup()
retained = []
observed_info = []


def on_message(message, message_info):
    assert type(message) is UInt64
    assert message.data == 73
    retained.append(message)
    observed_info.append(message_info)


# The two-argument callback exercises native MessageInfo delivery through the
# same compiled bridge as ordinary typed subscriptions.
info_subscription = node.create_subscription(
    UInt64, prefix + "/message_info", on_message, 10,
    callback_group=group)
assert info_subscription._native.callback_handoff == "compiled_python_callback"
assert info_subscription._callback_type.name == "WithMessageInfo"
info_publisher = node.create_publisher(UInt64, prefix + "/message_info", 10)
spin_until(executor, lambda: info_publisher.get_subscription_count() == 1)
info_publisher.publish(UInt64(data=73))
spin_until(executor, lambda: len(observed_info) == 1)
assert isinstance(observed_info[0], dict)
assert "source_timestamp" in observed_info[0]
assert "received_timestamp" in observed_info[0]
assert reaper_pump_calls
before_destroy_pumps = len(reaper_pump_calls)
assert node.destroy_subscription(info_subscription)
executor.spin_once(timeout_sec=0.01)
assert len(reaper_pump_calls) > before_destroy_pumps
retained.clear()
observed_info.clear()


def on_plain_message(message):
    assert type(message) is UInt64
    assert message.data == 73
    retained.append(message)


subscription = node.create_subscription(
    UInt64, prefix + "/message", on_plain_message, 10,
    callback_group=group)
assert subscription.callback_group is group
assert group.has_entity(subscription)
assert subscription._native.callback_handoff == "compiled_python_callback"

publisher = node.create_publisher(UInt64, prefix + "/message", 10)
spin_until(executor, lambda: publisher.get_subscription_count() == 1)
publisher.publish(UInt64(data=73))
spin_until(executor, lambda: len(retained) == 1)
assert type(retained[0]) is UInt64
assert retained[0].data == 73
retained_message = retained[0]
sub_stats = subscription._native.stats().to_dict()
assert sub_stats["callbacks"] == 1
assert sub_stats["message_cpp_copies"] == 1
assert sub_stats["bridge_errors"] == 0


service_requests = []


def on_service(request, response):
    assert type(request) is SetBool.Request
    assert type(response) is SetBool.Response
    service_requests.append(request)
    response.success = request.data
    response.message = "compiled bridge response"
    return response


service = node.create_service(
    SetBool, prefix + "/service", on_service, callback_group=group)
assert service.callback_group is group
assert group.has_entity(service)
assert service._native.callback_handoff == "compiled_python_callback"
client = node.create_client(SetBool, prefix + "/service", callback_group=group)
spin_until(executor, client.service_is_ready)
future = client.call_async(SetBool.Request(data=True))
spin_until(executor, future.done)
service_response = future.result()
assert type(service_requests[0]) is SetBool.Request
assert type(service_response) is SetBool.Response
assert service_response.success is True
assert service_response.message == "compiled bridge response"
service_stats = service._native.stats().to_dict()
assert service_stats["requests"] == 1
assert service_stats["request_cpp_copies"] == 1
assert service_stats["response_cpp_copies"] == 1
assert service_stats["exceptions"] == 0


bad_service = node.create_service(
    SetBool, prefix + "/bad_response", lambda _request, _response: object())
bad_client = node.create_client(SetBool, prefix + "/bad_response")
spin_until(executor, bad_client.service_is_ready)
bad_future = bad_client.call_async(SetBool.Request(data=True))
deadline = time.monotonic() + 10.0
while time.monotonic() < deadline:
    try:
        executor.spin_once(timeout_sec=0.05)
    except TypeError as exc:
        assert "SetBool.Response" in str(exc)
        break
else:
    raise AssertionError("wrong service response type was not reported")
assert bad_service._native.stats().exceptions == 0


exception_topic = prefix + "/exception"


def raising_callback(_message):
    raise LookupError("compiled callback exception sentinel")


exception_subscription = node.create_subscription(
    UInt64, exception_topic, raising_callback, 10)
exception_publisher = node.create_publisher(UInt64, exception_topic, 10)
spin_until(executor, lambda: exception_publisher.get_subscription_count() == 1)
exception_publisher.publish(UInt64(data=1))
deadline = time.monotonic() + 10.0
while time.monotonic() < deadline:
    try:
        executor.spin_once(timeout_sec=0.05)
    except LookupError as exc:
        assert str(exc) == "compiled callback exception sentinel"
        break
else:
    raise AssertionError("contained callback exception was not re-raised")

assert exception_subscription._native.stats().bridge_errors == 0
assert node.destroy_subscription(exception_subscription)
assert node.destroy_service(service)
assert node.destroy_service(bad_service)
assert node.destroy_subscription(subscription)
assert retained_message.data == 73
assert executor.shutdown(timeout_sec=5.0)
node.destroy_node()
rclpy.shutdown()
print("DIRECT_CPP_COMPILED_CALLBACK_BRIDGE_OK", flush=True)
