#!/usr/bin/env python3
"""Live exact-C++ and stock-rclpy proof for a registered Trigger service."""

import gc
import importlib
import os
from pathlib import Path
import select
import subprocess
import sys
import time

import rclcppyy


INTERFACE = "std_srvs/srv/Trigger"
rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp", interfaces=(INTERFACE, INTERFACE))

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_srvs.srv import Empty, SetBool, Trigger  # noqa: E402


assert Trigger.Request is cppyy.gbl.std_srvs.srv.Trigger.Request
assert Trigger.Response is cppyy.gbl.std_srvs.srv.Trigger.Response
assert SetBool.Request is cppyy.gbl.std_srvs.srv.SetBool.Request
assert SetBool.Response is cppyy.gbl.std_srvs.srv.SetBool.Response
assert type(Trigger.Request()) is Trigger.Request
constructed = Trigger.Response(success=True, message="constructed")
assert constructed.success is True
assert str(constructed.message) == "constructed"
try:
    Trigger.Request(unknown=True)
except TypeError:
    pass
else:
    raise AssertionError("Trigger.Request accepted an unknown field")
print("DIRECT_CPP_TRIGGER_CONSTRUCTORS_OK")


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("a Python-message conversion or serialization path ran")


bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
serialization = importlib.import_module("rclcpp_kit.serialization")
bringup.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary

rclpy.init(args=[])
node = Node("direct_trigger_%d" % os.getpid())
prefix = "/direct_cpp/trigger/p%d" % os.getpid()
before = (len(node.clients), len(node.services))
try:
    node.create_client(Empty, prefix + "/unregistered")
except TypeError as exc:
    assert "not registered" in str(exc)
else:
    raise AssertionError("an unregistered service type was accepted")
assert (len(node.clients), len(node.services)) == before
print("DIRECT_CPP_TRIGGER_FAIL_CLOSED_OK")

retained_requests = []
retained_responses = []


def callback(request, response):
    assert type(request) is Trigger.Request
    assert type(response) is Trigger.Response
    retained_requests.append(request)
    retained_responses.append(response)
    response.success = True
    response.message = "direct-trigger-response"
    return response


service_name = prefix + "/self"
service = node.create_service(Trigger, service_name, callback)
client = node.create_client(Trigger, service_name)
assert client.wait_for_service(timeout_sec=5.0)
future = client.call_async(Trigger.Request())
rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
assert future.done() and future.exception() is None
response = future.result()
assert type(response) is Trigger.Response
assert response.success is True
assert str(response.message) == "direct-trigger-response"
assert type(retained_requests[0]) is Trigger.Request
assert str(retained_responses[0].message) == "direct-trigger-response"
assert service.stats().request_cpp_copies == 1
assert service.stats().response_cpp_copies == 1
assert client.stats().cpp_request_copies == 1
print("DIRECT_CPP_TRIGGER_CALL_OK")


def fail(_request, _response):
    raise RuntimeError("direct Trigger callback sentinel")


error_service = node.create_service(Trigger, prefix + "/error", fail)
error_client = node.create_client(Trigger, prefix + "/error")
assert error_client.wait_for_service(timeout_sec=5.0)
error_future = error_client.call_async(Trigger.Request())
deadline = time.monotonic() + 10.0
while time.monotonic() < deadline:
    try:
        rclpy.spin_once(node, timeout_sec=0.05)
    except RuntimeError as exc:
        assert "direct Trigger callback sentinel" in str(exc)
        break
else:
    raise AssertionError("Trigger service callback exception was not propagated")
assert not error_future.done()
# A contained raise still commits the untouched response object to the
# native reply (see the "service reply-on-contained-raise" disclosure in
# compatibility/jazzy.json's executor.direct_cpp_multi_threaded entry), so
# nothing reaches the C++ callback lambda uncaught here.
assert error_service.stats().exceptions == 0
assert node.destroy_client(error_client)
assert error_future.cancelled()
assert node.destroy_service(error_service)
print("DIRECT_CPP_TRIGGER_EXCEPTION_OK")


def serve_stock_client(request, response):
    assert type(request) is Trigger.Request
    assert type(response) is Trigger.Response
    retained_requests.append(request)
    retained_responses.append(response)
    response.success = True
    response.message = "direct-cpp-response"
    return response


stock_name = prefix + "/stock_server"
direct_name = prefix + "/direct_server"
interop_service = node.create_service(Trigger, direct_name, serve_stock_client)
interop_client = node.create_client(Trigger, stock_name)
peer_path = Path(__file__).with_name("_stock_trigger_service_peer.py")
peer = subprocess.Popen(
    [sys.executable, str(peer_path), stock_name, direct_name],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    text=True,
    env=os.environ.copy(),
)
peer_stdout = ""
peer_stderr = ""
try:
    readable, _, _ = select.select([peer.stdout], [], [], 10.0)
    assert readable, "stock Trigger peer did not report readiness"
    ready = peer.stdout.readline()
    peer_stdout += ready
    assert ready.strip() == "STOCK_TRIGGER_READY"
    assert interop_client.wait_for_service(timeout_sec=10.0)
    interop_future = interop_client.call_async(Trigger.Request())
    deadline = time.monotonic() + 20.0
    while (
        (not interop_future.done() or peer.poll() is None)
        and time.monotonic() < deadline
    ):
        rclpy.spin_once(node, timeout_sec=0.02)
    assert interop_future.done() and interop_future.exception() is None
    assert interop_future.result().success is True
    assert str(interop_future.result().message) == "stock-rclpy-response"
    remaining_stdout, peer_stderr = peer.communicate(timeout=5.0)
    peer_stdout += remaining_stdout
finally:
    if peer.poll() is None:
        peer.kill()
        remaining_stdout, peer_stderr = peer.communicate()
        peer_stdout += remaining_stdout
assert peer.returncode == 0, (
    "stock peer exit=%s\nstdout:\n%s\nstderr:\n%s" %
    (peer.returncode, peer_stdout, peer_stderr)
)
assert "STOCK_TRIGGER_BIDIRECTIONAL_OK" in peer_stdout
assert "STOCK_TRIGGER_TEARDOWN_OK" in peer_stdout
assert node.destroy_client(interop_client)
assert node.destroy_service(interop_service)
print("DIRECT_CPP_TRIGGER_STOCK_INTEROP_OK")

records = [
    item for item in rclcppyy.status()["entities"]
    if item["metadata"].get("service_interface") == INTERFACE
]
assert records
assert all(item["backend"] == "cpp" for item in records)
assert all(item["metadata"]["service_type"] == "std_srvs::srv::Trigger"
           for item in records)
assert all(item["metadata"]["python_message_conversions"] == 0
           for item in records)
operation = rclcppyy.status()["operations"][-1]
assert operation["metadata"]["requested_service_interfaces"] == [INTERFACE]
assert operation["metadata"]["service_types"] == [
    "std_srvs::srv::SetBool",
    "std_srvs::srv::Trigger",
]
print("DIRECT_CPP_TRIGGER_EVIDENCE_OK")

assert node.destroy_client(client)
assert node.destroy_service(service)
node.destroy_node()
rclpy.shutdown()
gc.collect()
assert response.success is True
assert str(response.message) == "direct-trigger-response"
assert type(retained_requests[0]) is Trigger.Request
assert str(retained_responses[0].message) == "direct-trigger-response"

rclpy.init(args=[])
second = Node("direct_trigger_reinit_%d" % os.getpid())
second_service = second.create_service(
    Trigger, prefix + "/reinit", lambda request, response: response)
assert second.destroy_service(second_service)
second.destroy_node()
rclpy.shutdown()
print("DIRECT_CPP_TRIGGER_RETAINED_REINIT_TEARDOWN_OK")
