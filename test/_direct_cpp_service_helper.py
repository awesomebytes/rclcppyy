#!/usr/bin/env python3
"""Live source-shape proof for direct C++ SetBool services and clients."""

import gc
import importlib
import os
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from rclcppyy.policy import BackendUnavailableError  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, qos_profile_services_default  # noqa: E402
from rclpy.task import Future  # noqa: E402
from std_srvs.srv import SetBool, SetBool_Request, SetBool_Response, Trigger  # noqa: E402


assert SetBool.Request is cppyy.gbl.std_srvs.srv.SetBool.Request
assert SetBool.Response is cppyy.gbl.std_srvs.srv.SetBool.Response
assert SetBool_Request is SetBool.Request
assert SetBool_Response is SetBool.Response
assert SetBool.Request(data=True).data is True
constructed = SetBool.Response(success=True, message="constructed")
assert constructed.success is True
assert str(constructed.message) == "constructed"
try:
    SetBool.Request(unknown=True)
except TypeError:
    pass
else:
    raise AssertionError("direct service constructor accepted an unknown field")
print("DIRECT_CPP_SERVICE_CONSTRUCTORS_OK")


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("a Python-message conversion or serialization path ran")


bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
serialization = importlib.import_module("rclcpp_kit.serialization")
bringup.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary


def wait_for(predicate, node, timeout=10.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
    assert predicate()


def reject_without_entity(node, operation):
    before = (len(node.clients), len(node.services))
    try:
        operation()
    except (BackendUnavailableError, TypeError, ValueError):
        pass
    else:
        raise AssertionError("unsupported direct service operation succeeded")
    assert (len(node.clients), len(node.services)) == before


rclpy.init(args=[])
node = Node("direct_service_%d" % os.getpid())
prefix = "/direct_cpp/service/p%d" % os.getpid()


async def async_callback(request, response):
    return response


reject_without_entity(
    node, lambda: node.create_service(Trigger, prefix + "/type", lambda r, s: s))
reject_without_entity(
    node, lambda: node.create_client(Trigger, prefix + "/client_type"))
reject_without_entity(
    node,
    lambda: node.create_service(
        SetBool, prefix + "/group", lambda r, s: s, callback_group=object()),
)
reject_without_entity(
    node,
    lambda: node.create_client(SetBool, prefix + "/group", callback_group=object()),
)
reject_without_entity(
    node,
    lambda: node.create_service(
        SetBool, prefix + "/qos", lambda r, s: s,
        qos_profile=QoSProfile(depth=1)),
)
reject_without_entity(
    node,
    lambda: node.create_client(
        SetBool, prefix + "/qos", qos_profile=QoSProfile(depth=1)),
)
reject_without_entity(
    node, lambda: node.create_service(SetBool, prefix + "/async", async_callback))
reject_without_entity(
    node, lambda: node.create_service(SetBool, prefix + "/callback", object()))
reject_without_entity(
    node, lambda: node.create_service(SetBool, prefix + "/arity", lambda request: request))
print("DIRECT_CPP_SERVICE_FAIL_CLOSED_OK")


retained_requests = []
retained_responses = []


def callback(request, response):
    assert type(request) is SetBool.Request
    assert type(response) is SetBool.Response
    response.success = bool(request.data)
    response.message = "direct:true" if request.data else "direct:false"
    retained_requests.append(request)
    retained_responses.append(response)
    return response


service_name = prefix + "/set_bool"
service = node.create_service(
    SetBool, service_name, callback,
    qos_profile=qos_profile_services_default)
client = node.create_client(
    SetBool, service_name, qos_profile=qos_profile_services_default)
assert len(node.services) == 1
assert len(node.clients) == 1
assert service.service_name == service_name
assert client.service_name == service_name
assert client.wait_for_service(timeout_sec=5.0)

future = client.call_async(SetBool.Request(data=True))
assert type(future) is Future
done_callbacks = []
future.add_done_callback(done_callbacks.append)
rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
assert future.done() and not future.cancelled()
assert future.exception() is None
response = future.result()
assert type(response) is SetBool.Response
assert response.success is True
assert str(response.message) == "direct:true"
assert done_callbacks == [future]
assert retained_requests[0].data is True
assert retained_responses[0].success is True
gc.collect()
assert response.success is True
assert str(response.message) == "direct:true"
assert retained_requests[0].data is True
assert str(retained_responses[0].message) == "direct:true"
print("DIRECT_CPP_SERVICE_CALL_OK")


def failing_callback(request, response):
    raise RuntimeError("direct service callback sentinel")


error_name = prefix + "/error"
error_service = node.create_service(SetBool, error_name, failing_callback)
error_client = node.create_client(SetBool, error_name)
assert error_client.wait_for_service(timeout_sec=5.0)
error_future = error_client.call_async(SetBool.Request(data=True))
deadline = time.monotonic() + 10.0
while True:
    try:
        rclpy.spin_once(node, timeout_sec=0.05)
    except RuntimeError as exc:
        assert "direct service callback sentinel" in str(exc)
        break
    assert time.monotonic() < deadline
assert not error_future.done() and not error_future.cancelled()
error_stats = error_service.stats()
assert error_stats.requests == 0
assert error_stats.exceptions == 1
assert error_stats.python_callback_crossings == 1
assert error_stats.request_cpp_copies == 1
assert error_stats.response_cpp_copies == 0
assert node.destroy_client(error_client)
assert error_future.cancelled()
assert node.destroy_service(error_service)
print("DIRECT_CPP_SERVICE_EXCEPTION_OK")


missing = node.create_client(SetBool, prefix + "/missing")
canceled = missing.call_async(SetBool.Request(data=False))
canceled.cancel()
assert canceled.cancelled() and not canceled.done()
timed_out = missing.call_async(SetBool.Request(data=True))
rclpy.spin_until_future_complete(node, timed_out, timeout_sec=0.01)
assert not timed_out.done() and not timed_out.cancelled()
missing.remove_pending_request(timed_out)
try:
    client.call(SetBool.Request(data=True), timeout_sec=0.01)
except BackendUnavailableError:
    pass
else:
    raise AssertionError("direct synchronous client call was accepted")
try:
    rclpy.spin_until_future_complete(node, Future(), timeout_sec=0.0)
except BackendUnavailableError:
    pass
else:
    raise AssertionError("direct spin accepted a foreign Future")
missing_stats = missing.stats()
assert missing_stats.requests_sent == 2
assert missing_stats.canceled == 2
assert missing_stats.pending_requests == 0
assert missing_stats.cpp_request_copies == 2
print("DIRECT_CPP_SERVICE_FUTURE_CONTROL_OK")


client_stats = client.stats()
service_stats = service.stats()
assert client_stats.requests_sent == 1
assert client_stats.responses_taken == 1
assert client_stats.python_request_crossings == 1
assert client_stats.python_response_crossings == 1
assert client_stats.cpp_request_copies == 1
assert service_stats.requests == 1
assert service_stats.python_callback_crossings == 1
assert service_stats.request_cpp_copies == 1
assert service_stats.response_cpp_copies == 1
assert service_stats.exceptions == 0

records = [
    item for item in rclcppyy.status()["entities"]
    if "direct_cpp_service" in item["policies"]
]
assert len(records) == 5
assert all(item["backend"] == "cpp" for item in records)
assert all(item["metadata"]["python_message_conversions"] == 0 for item in records)
client_records = [item for item in records if item["metadata"]["entity_type"] == "client"]
service_records = [item for item in records if item["metadata"]["entity_type"] == "service"]
assert len(client_records) == 3 and len(service_records) == 2
assert all(
    item["metadata"]["future_control"] == "per_operation_rclpy_task_future"
    and item["metadata"]["cpp_request_copies_per_call"] == 1
    for item in client_records
)
assert all(
    item["metadata"]["python_callback_crossings_per_request"] == 1
    and item["metadata"]["cpp_request_copies_per_request"] == 1
    and item["metadata"]["cpp_response_copies_per_request"] == 1
    for item in service_records
)
print("DIRECT_CPP_SERVICE_EVIDENCE_OK")


raw_node = node._direct_cpp_node
assert int(raw_node.count_services(service_name)) == 1
assert node.destroy_client(client)
assert node.destroy_client(missing)
assert node.destroy_service(service)
wait_for(lambda: int(raw_node.count_services(service_name)) == 0, node)
assert not node.destroy_client(client)
assert not node.destroy_service(service)
node.destroy_node()
rclpy.shutdown()
assert not rclpy.ok()

rclpy.init(args=[])
second = Node("direct_service_reinit_%d" % os.getpid())
second_service = second.create_service(
    SetBool, prefix + "/reinit", lambda request, response: response)
assert second.destroy_service(second_service)
second.destroy_node()
rclpy.shutdown()
assert not rclpy.ok()
print("DIRECT_CPP_SERVICE_REINIT_TEARDOWN_OK")
