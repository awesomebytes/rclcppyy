#!/usr/bin/env python3
"""Exercise remote parameter callbacks on DirectMultiThreadedExecutor workers."""
import gc
import os
import threading
import time

import rclcppyy

PARAMETER_INTERFACES = (
    "rcl_interfaces/msg/ParameterEvent",
    "rcl_interfaces/srv/DescribeParameters",
    "rcl_interfaces/srv/GetParameters",
    "rcl_interfaces/srv/GetParameterTypes",
    "rcl_interfaces/srv/ListParameters",
    "rcl_interfaces/srv/SetParameters",
    "rcl_interfaces/srv/SetParametersAtomically",
)
rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp", interfaces=PARAMETER_INTERFACES)

import rclpy  # noqa: E402
from rcl_interfaces.msg import SetParametersResult  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402
from rclcppyy.direct_executors import (  # noqa: E402
    DirectMultiThreadedExecutor,
)
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.parameter_client import AsyncParameterClient  # noqa: E402


def wait_future(future, timeout=15.0):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.005)
    assert future.done(), "remote SetParameters future timed out"
    exception = future.exception()
    assert exception is None, "remote SetParameters failed: %r" % (exception,)
    return future.result()


def main():
    rclpy.init(args=[])
    suffix = "p%d" % os.getpid()
    server = Node("remote_param_server_" + suffix)
    client_node = Node(
        "remote_param_client_" + suffix,
        start_parameter_services=False,
        enable_rosout=False,
    )
    server.declare_parameter("remote_value", 0)
    server.declare_parameter("exception_value", 0)
    client = AsyncParameterClient(
        client_node, server.get_fully_qualified_name())
    assert client.wait_for_services(timeout_sec=10.0)

    executor = DirectMultiThreadedExecutor(num_threads=3)
    spin_thread = None
    spin_started = threading.Event()
    spin_errors = []

    def spin():
        spin_started.set()
        try:
            executor.spin()
        except BaseException as exc:  # noqa: BLE001 -- report across thread
            spin_errors.append(exc)

    try:
        assert executor.add_node(server)
        assert executor.add_node(client_node)
        main_thread_id = threading.get_ident()
        spin_thread = threading.Thread(
            target=spin, name="remote-parameter-direct-mte-spin")
        spin_thread.start()
        assert spin_started.wait(timeout=5.0)

        callback_order = []
        callback_thread_ids = {}
        holders = {kind: {} for kind in ("pre", "on", "post")}
        bridge_refs = {}
        bridge_baselines = {}
        bridge_error_baselines = {}

        def register_self_removing(kind):
            holder = holders[kind]
            remove = getattr(server, "remove_%s_set_parameters_callback" % kind)

            def callback(parameters):
                callback_order.append(kind)
                callback_thread_ids[kind] = threading.get_ident()
                callback_ref = holder.pop("callback")
                remove(callback_ref)
                del callback_ref
                gc.collect()
                gc.collect()
                if kind == "pre":
                    return parameters
                if kind == "on":
                    return SetParametersResult(successful=True)
                return None

            holder["callback"] = callback
            getattr(server, "add_%s_set_parameters_callback" % kind)(callback)
            bridge_refs[kind] = server._direct_cpp_parameter_callback_bridges[kind]
            assert bridge_refs[kind].callback_handoff == (
                "compiled_python_callback")
            bridge_baselines[kind] = bridge_refs[kind].stats().to_dict()
            bridge_error_baselines[kind] = bridge_refs[kind].bridge_errors

        for kind in ("pre", "on", "post"):
            register_self_removing(kind)

        first_response = wait_future(client.set_parameters([
            Parameter("remote_value", value=11),
        ]))
        assert type(first_response) is SetParameters.Response
        assert len(first_response.results) == 1
        assert type(first_response.results[0]) is SetParametersResult
        assert first_response.results[0].successful
        assert server.get_parameter("remote_value").value == 11
        assert callback_order == ["pre", "on", "post"]
        assert all(
            callback_thread_ids[kind] != main_thread_id
            for kind in ("pre", "on", "post")
        )
        assert all(
            bridge_refs[kind].stats().calls
            == bridge_baselines[kind]["calls"] + 1
            and bridge_refs[kind].stats().exceptions
            == bridge_baselines[kind]["exceptions"]
            and bridge_refs[kind].stats().rejections
            == bridge_baselines[kind]["rejections"]
            and bridge_refs[kind].bridge_errors
            == bridge_error_baselines[kind]
            for kind in ("pre", "on", "post")
        )
        assert all(holders[kind].get("callback") is None for kind in holders)
        assert bridge_refs["pre"].closed
        assert bridge_refs["on"].closed
        # The node's parameter cache keeps the shared post bridge installed
        # after the user callback removes itself.
        assert not bridge_refs["post"].closed
        assert server._post_set_parameters_callbacks == []
        gc.collect()
        gc.collect()

        exception_calls = []

        def raising_on_set(_parameters):
            exception_calls.append(threading.get_ident())
            raise RuntimeError("remote on-set callback failure")

        server.add_on_set_parameters_callback(raising_on_set)
        exception_bridge = server._direct_cpp_parameter_callback_bridges["on"]
        assert exception_bridge.callback_handoff == "compiled_python_callback"
        exception_baseline = exception_bridge.stats().to_dict()
        exception_bridge_errors = exception_bridge.bridge_errors
        failed_response = wait_future(client.set_parameters([
            Parameter("exception_value", value=22),
        ]))
        assert type(failed_response) is SetParameters.Response
        assert len(failed_response.results) == 1
        assert not failed_response.results[0].successful
        assert failed_response.results[0].reason == "parameter callback raised"
        assert exception_calls and exception_calls[0] != main_thread_id
        assert exception_bridge.stats().calls == exception_baseline["calls"] + 1
        assert exception_bridge.stats().exceptions == (
            exception_baseline["exceptions"] + 1)
        assert exception_bridge.stats().rejections == (
            exception_baseline["rejections"] + 1)
        assert exception_bridge.bridge_errors == exception_bridge_errors
        server.remove_on_set_parameters_callback(raising_on_set)

        recovered_response = wait_future(client.set_parameters([
            Parameter("exception_value", value=23),
        ]))
        assert recovered_response.results[0].successful
        assert server.get_parameter("exception_value").value == 23
        assert spin_errors == [], "executor errors: %r" % (spin_errors,)
        print("DIRECT_CPP_REMOTE_PARAMETER_CALLBACKS_MTE_OK", flush=True)
    finally:
        if spin_thread is not None:
            executor.shutdown(timeout_sec=10.0)
            spin_thread.join(timeout=10.0)
            assert not spin_thread.is_alive(), "MTE spin thread failed to stop"
        server.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
