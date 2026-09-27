#!/usr/bin/env python3
"""Bidirectional interop between registered C++ Trigger aliases and AOT rclcpp."""

import importlib
import os
from pathlib import Path
import select
import subprocess
import sys
import time

import rclcppyy


if len(sys.argv) != 2:
    raise SystemExit("usage: helper.py PATH_TO_AOT_PEER")
peer_path = Path(sys.argv[1]).resolve()
assert peer_path.is_file() and os.access(peer_path, os.X_OK)

rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp", interfaces=("std_srvs/srv/Trigger",))

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402


assert Trigger.Request is cppyy.gbl.std_srvs.srv.Trigger.Request
assert Trigger.Response is cppyy.gbl.std_srvs.srv.Trigger.Response


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("a Python-message conversion or serialization path ran")


bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
serialization = importlib.import_module("rclcpp_kit.serialization")
bringup.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary

rclpy.init(args=[])
node = Node("direct_trigger_aot_%d" % os.getpid())
prefix = "/direct_cpp/trigger_aot/p%d" % os.getpid()


def run_aot_server():
    service_name = prefix + "/aot_server"
    process = subprocess.Popen(
        [str(peer_path), "server", service_name],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=os.environ.copy(),
    )
    stdout = ""
    stderr = ""
    try:
        readable, _, _ = select.select([process.stdout], [], [], 10.0)
        assert readable, "AOT Trigger server did not report readiness"
        ready = process.stdout.readline()
        stdout += ready
        assert ready.strip() == "AOT_TRIGGER_SERVER_READY"
        client = node.create_client(Trigger, service_name)
        assert client.wait_for_service(timeout_sec=10.0)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        assert future.done() and future.exception() is None
        assert type(future.result()) is Trigger.Response
        assert future.result().success is True
        assert str(future.result().message) == "aot-trigger-response"
        assert client.stats().cpp_request_copies == 1
        assert node.destroy_client(client)
        remaining_stdout, stderr = process.communicate(timeout=10.0)
        stdout += remaining_stdout
    finally:
        if process.poll() is None:
            process.kill()
            remaining_stdout, stderr = process.communicate()
            stdout += remaining_stdout
    assert process.returncode == 0, (
        "AOT server exit=%s\nstdout:\n%s\nstderr:\n%s" %
        (process.returncode, stdout, stderr)
    )
    assert "AOT_TRIGGER_SERVER_OK" in stdout
    assert "AOT_TRIGGER_TEARDOWN_OK" in stdout


retained = []


def run_aot_client():
    service_name = prefix + "/direct_server"

    def callback(request, response):
        assert type(request) is Trigger.Request
        assert type(response) is Trigger.Response
        retained.extend((request, response))
        response.success = True
        response.message = "direct-trigger-aot-response"
        return response

    service = node.create_service(Trigger, service_name, callback)
    process = subprocess.Popen(
        [str(peer_path), "client", service_name],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=os.environ.copy(),
    )
    deadline = time.monotonic() + 20.0
    while process.poll() is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
    if process.poll() is None:
        process.kill()
    stdout, stderr = process.communicate()
    assert process.returncode == 0, (
        "AOT client exit=%s\nstdout:\n%s\nstderr:\n%s" %
        (process.returncode, stdout, stderr)
    )
    assert "AOT_TRIGGER_CLIENT_OK" in stdout
    assert "AOT_TRIGGER_TEARDOWN_OK" in stdout
    assert service.stats().requests == 1
    assert service.stats().request_cpp_copies == 1
    assert service.stats().response_cpp_copies == 1
    assert node.destroy_service(service)


run_aot_server()
run_aot_client()
node.destroy_node()
rclpy.shutdown()
assert type(retained[0]) is Trigger.Request
assert type(retained[1]) is Trigger.Response
assert str(retained[1].message) == "direct-trigger-aot-response"
print("DIRECT_CPP_TRIGGER_AOT_BIDIRECTIONAL_OK")
print("DIRECT_CPP_TRIGGER_AOT_RETAINED_TEARDOWN_OK")
