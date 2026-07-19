#!/usr/bin/env python3
"""Run one fresh-process remote AsyncParameterClient benchmark sample."""

from __future__ import annotations

import argparse
import hashlib
import importlib
import inspect
import json
import os
import time

from _remote_parameter_benchmark_protocol import (
    OPERATION_ROUTES,
    POST_INIT_SETTLE_NS,
    PRIMARY_METRIC,
    QOS,
    RMW,
    ROS_DISTRO,
    SAMPLE_SCHEMA,
    SERVICE_COUNT,
    VARIANTS,
    WORKLOADS,
    latency_summary,
)


PREFIX = "@@RCLCPPYY_REMOTE_PARAMETER_V1@@"
VALUE = 73
PARAMETER_INTERFACES = (
    "rcl_interfaces/msg/ParameterEvent",
    "rcl_interfaces/srv/DescribeParameters",
    "rcl_interfaces/srv/GetParameters",
    "rcl_interfaces/srv/GetParameterTypes",
    "rcl_interfaces/srv/ListParameters",
    "rcl_interfaces/srv/SetParameters",
    "rcl_interfaces/srv/SetParametersAtomically",
)
CLIENT_ATTRIBUTES = (
    "_get_parameter_client",
    "_list_parameter_client",
    "_set_parameter_client",
    "_get_parameter_types_client",
    "_describe_parameters_client",
    "_set_parameters_atomically_client",
)


def _arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", choices=VARIANTS, required=True)
    parser.add_argument("--workload", choices=WORKLOADS, required=True)
    parser.add_argument("--warmup-operations", type=int, required=True)
    parser.add_argument("--operations", type=int, required=True)
    parser.add_argument("--repetition", type=int, required=True)
    parser.add_argument("--order-index", type=int, required=True)
    parser.add_argument("--run-token", required=True)
    return parser.parse_args(argv)


def _loaded_rmw():
    from rclpy.utilities import get_rmw_implementation_identifier

    loaded = get_rmw_implementation_identifier()
    if os.environ.get("ROS_DISTRO") != ROS_DISTRO or loaded != RMW:
        raise RuntimeError("remote parameter benchmark requires Jazzy/CycloneDDS")
    return loaded


def _cpp_name(value):
    return str(
        getattr(type(value), "__cpp_name__", "")
        or getattr(value, "__cpp_name__", "")
    )


def _install_boundary_poison():
    counters = {
        "application_message_conversions": 0,
        "serialization_calls": 0,
        "cdr_calls": 0,
    }

    def conversion(*_args, **_kwargs):
        counters["application_message_conversions"] += 1
        raise AssertionError("conversion entered remote parameter benchmark")

    def serialization_call(*_args, **_kwargs):
        counters["serialization_calls"] += 1
        raise AssertionError("serialization entered remote parameter benchmark")

    def cdr_call(*_args, **_kwargs):
        counters["cdr_calls"] += 1
        raise AssertionError("CDR entered remote parameter benchmark")

    kit = importlib.import_module("rclcpp_kit")
    bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
    serialization = importlib.import_module("rclcpp_kit.serialization")
    rclpy_serialization = importlib.import_module("rclpy.serialization")
    kit.convert_python_msg_to_cpp = conversion
    bringup.convert_python_msg_to_cpp = conversion
    serialization.serialize_message = serialization_call
    serialization.deserialize_message = serialization_call
    serialization.serialized_message_from_bytes = cdr_call
    serialization.serialized_message_to_bytes = cdr_call
    rclpy_serialization.serialize_message = serialization_call
    rclpy_serialization.deserialize_message = serialization_call
    return counters


def _setup(variant, suffix):
    if variant == "direct-rclcppyy":
        import rclcppyy

        rclcppyy.enable_cpp_acceleration(
            profile="direct_cpp", interfaces=PARAMETER_INTERFACES)

    import rclpy
    from rcl_interfaces.msg import Parameter as ParameterMsg
    from rcl_interfaces.msg import ParameterValue
    from rcl_interfaces.msg import SetParametersResult
    from rcl_interfaces.srv import DescribeParameters
    from rcl_interfaces.srv import GetParameters
    from rcl_interfaces.srv import GetParameterTypes
    from rcl_interfaces.srv import ListParameters
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.srv import SetParametersAtomically
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.parameter_client import AsyncParameterClient
    from rclpy.qos import qos_profile_services_default

    rclpy.init(args=[])
    server = Node("remote_parameter_server_" + suffix, enable_rosout=False)
    client_node = Node(
        "remote_parameter_client_" + suffix,
        start_parameter_services=False,
        enable_rosout=False,
    )
    server.declare_parameter("target", VALUE)
    remote_name = server.get_fully_qualified_name()
    client = AsyncParameterClient(
        client_node, remote_name, qos_profile=qos_profile_services_default)
    executor = SingleThreadedExecutor(context=server.context)
    if not executor.add_node(server) or not executor.add_node(client_node):
        raise RuntimeError("remote parameter executor rejected benchmark nodes")
    if not client.wait_for_services(timeout_sec=10.0):
        raise RuntimeError("remote parameter services did not become ready")

    service_types = (
        DescribeParameters, GetParameters, GetParameterTypes, ListParameters,
        SetParameters, SetParametersAtomically,
    )
    service_names = {
        str(name): tuple(str(value) for value in values)
        for name, values in server.get_service_names_and_types()
    }
    expected_services = {
        remote_name + "/" + suffix_name: "rcl_interfaces/srv/" + type_name
        for suffix_name, type_name in (
            ("describe_parameters", "DescribeParameters"),
            ("get_parameters", "GetParameters"),
            ("get_parameter_types", "GetParameterTypes"),
            ("list_parameters", "ListParameters"),
            ("set_parameters", "SetParameters"),
            ("set_parameters_atomically", "SetParametersAtomically"),
        )
    }
    parameter_service_count = sum(
        name in service_names for name in expected_services)
    graph_verified = all(
        service_names.get(name) == (service_type,)
        for name, service_type in expected_services.items()
    ) and parameter_service_count == SERVICE_COUNT
    clients = tuple(getattr(client, name) for name in CLIENT_ATTRIBUTES)
    qos_verified = all(
        value.qos_profile == qos_profile_services_default for value in clients)
    source_path = inspect.getsourcefile(AsyncParameterClient)
    source = inspect.getsource(AsyncParameterClient)
    if source_path is None or not source_path.endswith("rclpy/parameter_client.py"):
        raise RuntimeError("AsyncParameterClient did not come from installed rclpy")
    if AsyncParameterClient.__module__ != "rclpy.parameter_client":
        raise RuntimeError("AsyncParameterClient module identity changed")
    source_shape_verified = all(token in source for token in (
        "request = GetParameters.Request()",
        "request = SetParametersAtomically.Request()",
        "self._get_parameter_client.call_async(request)",
        "self._set_parameters_atomically_client.call_async(request)",
    ))

    return {
        "rclpy": rclpy,
        "server": server,
        "client_node": client_node,
        "client": client,
        "executor": executor,
        "parameter_class": Parameter,
        "parameter_msg_class": ParameterMsg,
        "parameter_value_class": ParameterValue,
        "set_result_class": SetParametersResult,
        "get_service": GetParameters,
        "set_atomic_service": SetParametersAtomically,
        "service_types": service_types,
        "clients": clients,
        "remote_name": remote_name,
        "parameter_service_count": parameter_service_count,
        "graph_verified": graph_verified,
        "qos_verified": qos_verified,
        "source_path": source_path,
        "source_sha256": hashlib.sha256(source.encode("utf-8")).hexdigest(),
        "source_shape_verified": source_shape_verified,
    }


def _complete(state, future):
    state["executor"].spin_until_future_complete(future, timeout_sec=10.0)
    if not future.done():
        raise TimeoutError("remote parameter operation timed out")
    exception = future.exception()
    if exception is not None:
        raise exception
    return future.result()


def _parameter_integer(value):
    type_value = value.type
    if isinstance(type_value, str):
        type_code = ord(type_value)
    elif isinstance(type_value, bytes):
        type_code = type_value[0]
    else:
        type_code = int(type_value)
    if type_code != 2:
        raise RuntimeError("remote parameter response changed type")
    return int(value.integer_value)


def _operation(state, args):
    parameter_class = state["parameter_class"]
    setters = (
        parameter_class("target", value=VALUE),
        parameter_class("target", value=VALUE + 1),
    )

    def invoke(index):
        if args.workload == "get-one":
            response = _complete(
                state, state["client"].get_parameters(["target"]))
            if len(response.values) != 1 or _parameter_integer(
                    response.values[0]) not in (VALUE, VALUE + 1):
                raise RuntimeError("remote get response was incorrect")
            return response
        response = _complete(
            state,
            state["client"].set_parameters_atomically([setters[index % 2]]),
        )
        if not bool(response.result.successful):
            raise RuntimeError("remote atomic set was rejected")
        return response

    last_response = None
    for index in range(args.warmup_operations):
        last_response = invoke(index)

    latencies = []
    completed = 0
    cpu_started = time.process_time_ns()
    wall_started = time.perf_counter_ns()
    for index in range(args.operations):
        operation_started = time.perf_counter_ns()
        last_response = invoke(index)
        latencies.append(time.perf_counter_ns() - operation_started)
        completed += 1
    wall_time_ns = time.perf_counter_ns() - wall_started
    process_cpu_time_ns = time.process_time_ns() - cpu_started

    final_value = state["server"].get_parameter("target").value
    expected = VALUE if args.workload == "get-one" else (
        VALUE + ((args.operations - 1) % 2))
    return {
        "completed": completed,
        "exceptions": 0,
        "responses_verified": completed,
        "pending_futures": 0,
        "final_value": int(final_value),
        "verified": completed == args.operations and int(final_value) == expected,
        "process_cpu_time_ns": process_cpu_time_ns,
        "wall_time_ns": wall_time_ns,
        "latency_ns": latencies,
        "last_response": last_response,
        "setters": setters,
    }


def _topology(state):
    return {
        "process_count": 1,
        "node_count": 2,
        "server_node": state["server"].get_name(),
        "client_node": state["client_node"].get_name(),
        "remote_node_name": state["remote_name"],
        "executor": "rclpy.executors.SingleThreadedExecutor",
        "one_outstanding_request": True,
        "server_parameter_service_count": state["parameter_service_count"],
        "client_parameter_client_count": len(state["clients"]),
        "same_context": state["server"].context is state["client_node"].context,
        "graph_verified": state["graph_verified"],
        "service_qos": QOS if state["qos_verified"] else None,
    }


def _backend_evidence(state, args, measured):
    common = {
        "verified": True,
        "operation_route": OPERATION_ROUTES[args.workload],
        "async_parameter_client_source": (
            "rclpy.parameter_client.AsyncParameterClient"),
        "source_path": state["source_path"],
        "source_sha256": state["source_sha256"],
        "source_shape_verified": state["source_shape_verified"],
        "python_payload_cache": False,
        "retained_response_verified": False,
    }
    if args.variant == "stock-rclpy":
        return {
            **common,
            "server_authority": "python",
            "client_authority": "python",
            "request_representation": "generated-python",
            "response_representation": "generated-python",
            "parameter_representation": "generated-python",
            "all_parameter_service_aliases_exact_cpp": False,
            "request_alias_exact_cpp": False,
            "response_exact_cpp": False,
            "parameter_value_exact_cpp": False,
            "parameter_message_alias_exact_cpp": False,
            "client_entities_exact_cpp": False,
            "set_result_exact_cpp": False,
            "server_node_exact_cpp": False,
            "client_node_exact_cpp": False,
            "parameter_owner_exact_cpp": False,
            "cpp_node_addresses": None,
        }

    import cppyy

    service_aliases = all(
        service.Request is getattr(
            cppyy.gbl.rcl_interfaces.srv, service.__name__).Request and
        service.Response is getattr(
            cppyy.gbl.rcl_interfaces.srv, service.__name__).Response
        for service in state["service_types"]
    )
    response = measured["last_response"]
    if args.workload == "get-one":
        response_exact = type(response) is state["get_service"].Response
        value_exact = type(response.values[0]) is state["parameter_value_class"]
        result_exact = False
    else:
        response_exact = type(response) is state["set_atomic_service"].Response
        value_exact = True
        result_exact = type(response.result) is state["set_result_class"]
    parameter_owner = measured["setters"][0]._rclcppyy_native_parameter.native
    request_service = (
        state["get_service"] if args.workload == "get-one"
        else state["set_atomic_service"])
    request_exact = type(request_service.Request()) is request_service.Request
    message_alias_exact = state["parameter_msg_class"] is (
        cppyy.gbl.rcl_interfaces.msg.Parameter)
    client_entities_exact = all(
        "rclcpp::Client<" in _cpp_name(client._native.raw_client)
        for client in state["clients"])
    return {
        **common,
        "server_authority": "cpp",
        "client_authority": "cpp",
        "request_representation": "actual-generated-cpp",
        "response_representation": "actual-generated-cpp",
        "parameter_representation": "actual-generated-cpp",
        "all_parameter_service_aliases_exact_cpp": bool(service_aliases),
        "request_alias_exact_cpp": bool(request_exact),
        "response_exact_cpp": bool(response_exact),
        "parameter_value_exact_cpp": bool(value_exact),
        "parameter_message_alias_exact_cpp": bool(message_alias_exact),
        "client_entities_exact_cpp": bool(client_entities_exact),
        "set_result_exact_cpp": bool(result_exact),
        "server_node_exact_cpp": isinstance(
            state["server"]._direct_cpp_node, cppyy.gbl.rclcpp.Node),
        "client_node_exact_cpp": isinstance(
            state["client_node"]._direct_cpp_node, cppyy.gbl.rclcpp.Node),
        "parameter_owner_exact_cpp": type(parameter_owner) is cppyy.gbl.rclcpp.Parameter,
        "cpp_node_addresses": [
            int(cppyy.addressof(state["server"]._direct_cpp_node)),
            int(cppyy.addressof(state["client_node"]._direct_cpp_node)),
        ],
    }


def _verify_retained_response(state, args, measured):
    response = measured["last_response"]
    if args.workload == "get-one":
        return _parameter_integer(response.values[0]) in (VALUE, VALUE + 1)
    return bool(response.result.successful)


def _cleanup(state):
    executor = state["executor"]
    client_node = state["client_node"]
    server = state["server"]
    executor.remove_node(client_node)
    executor.remove_node(server)
    for client in state["clients"]:
        client_node.destroy_client(client)
    executor.shutdown()
    client_node.destroy_node()
    server.destroy_node()
    state["rclpy"].shutdown()


def run(args):
    if args.warmup_operations <= 0 or args.operations <= 0:
        raise ValueError("warmup and measured operations must be positive")
    suffix = args.run_token[-12:]
    state = _setup(args.variant, suffix)
    loaded_rmw = _loaded_rmw()
    counters = _install_boundary_poison()
    time.sleep(POST_INIT_SETTLE_NS / 1_000_000_000)
    measured = None
    cleanup_error = None
    try:
        measured = _operation(state, args)
        topology = _topology(state)
        backend = _backend_evidence(state, args, measured)
    finally:
        try:
            _cleanup(state)
        except BaseException as exception:
            cleanup_error = "%s: %s" % (type(exception).__name__, exception)
    backend["retained_response_verified"] = _verify_retained_response(
        state, args, measured)
    cpu_time = measured["process_cpu_time_ns"]
    wall_time = measured["wall_time_ns"]
    latency = measured["latency_ns"]
    return {
        "schema": SAMPLE_SCHEMA,
        "variant": args.variant,
        "workload": args.workload,
        "repetition": args.repetition,
        "order_index": args.order_index,
        "run_token": args.run_token,
        "pid": os.getpid(),
        "warmup_operations": args.warmup_operations,
        "measured_operations": args.operations,
        "runtime": {
            "fresh_process": True,
            "setup_excluded": True,
            "jit_excluded": True,
            "warmup_completed": True,
            "post_init_settle_completed": True,
            "post_init_settle_ns": POST_INIT_SETTLE_NS,
            "fixed_work": True,
            "teardown_clean": cleanup_error is None,
            "cleanup_error": cleanup_error,
            "ros_distribution": os.environ.get("ROS_DISTRO"),
            "rmw_implementation": loaded_rmw,
            "domain_id": os.environ.get("ROS_DOMAIN_ID"),
        },
        "topology": topology,
        "correctness": {
            "completed_operations": measured["completed"],
            "expected_operations": args.operations,
            "exceptions": measured["exceptions"],
            "responses_verified": measured["responses_verified"],
            "pending_futures": measured["pending_futures"],
            "final_value": measured["final_value"],
            "verified": measured["verified"],
        },
        "timing": {
            "primary_metric": PRIMARY_METRIC,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "cpu_scope": "combined-server-client-process",
            "process_cpu_time_ns": cpu_time,
            PRIMARY_METRIC: cpu_time / args.operations,
            "wall_time_ns": wall_time,
            "wall_ns_per_operation": wall_time / args.operations,
            "operations_per_second": args.operations * 1e9 / wall_time,
            "latency_ns": latency,
            "latency": latency_summary(latency),
        },
        "boundary_evidence": {
            "conversion_poison_installed": True,
            "serialization_poison_installed": True,
            **counters,
            "exact_cpp_data_path_required": args.variant == "direct-rclcppyy",
        },
        "backend_evidence": backend,
    }


def main(argv=None):
    args = _arguments(argv)
    sample = run(args)
    print(PREFIX + json.dumps(sample, sort_keys=True, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
