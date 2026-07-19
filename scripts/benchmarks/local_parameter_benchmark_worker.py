#!/usr/bin/env python3
"""Run one fresh-process local parameter benchmark sample."""

from __future__ import annotations

import argparse
import importlib
import json
import os
import time

from _local_parameter_benchmark_protocol import (
    OPERATION_ROUTES,
    PRIMARY_METRIC,
    RMW,
    ROS_DISTRO,
    SAMPLE_SCHEMA,
    VARIANTS,
    WORKLOADS,
)


PREFIX = "@@RCLCPPYY_LOCAL_PARAMETER_V1@@"
VALUE = 73


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


def _cpp_name(value):
    return str(
        getattr(type(value), "__cpp_name__", "")
        or getattr(value, "__cpp_name__", "")
    )


def _loaded_rmw():
    from rclpy.utilities import get_rmw_implementation_identifier

    loaded = get_rmw_implementation_identifier()
    if os.environ.get("ROS_DISTRO") != ROS_DISTRO or loaded != RMW:
        raise RuntimeError("local parameter benchmark requires Jazzy/CycloneDDS")
    return loaded


def _install_boundary_poison():
    counters = {
        "application_message_conversions": 0,
        "serialization_calls": 0,
        "cdr_calls": 0,
    }

    def conversion(*_args, **_kwargs):
        counters["application_message_conversions"] += 1
        raise AssertionError("application-message conversion entered parameter benchmark")

    def serialization_call(*_args, **_kwargs):
        counters["serialization_calls"] += 1
        raise AssertionError("serialization entered parameter benchmark")

    def cdr_call(*_args, **_kwargs):
        counters["cdr_calls"] += 1
        raise AssertionError("CDR byte adaptation entered parameter benchmark")

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


def _setup(variant, node_name):
    if variant == "direct-rclcppyy":
        import rclcppyy

        rclcppyy.enable_cpp_acceleration(profile="direct_cpp")
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter

        rclpy.init(args=[])
        node = Node(node_name)
        return {
            "node": node,
            "parameter_class": Parameter,
            "native_parameters": importlib.import_module(
                "rclcpp_kit.native_parameters"),
            "native_node": node._direct_cpp_node,
            "cleanup": lambda: (node.destroy_node(), rclpy.shutdown()),
        }
    if variant == "stock-rclpy":
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter

        rclpy.init(args=[])
        node = Node(node_name)
        return {
            "node": node,
            "parameter_class": Parameter,
            "native_parameters": None,
            "native_node": None,
            "cleanup": lambda: (node.destroy_node(), rclpy.shutdown()),
        }

    from rclcpp_kit.native import native

    native_parameters = importlib.import_module("rclcpp_kit.native_parameters")
    session = native(["local-parameter-benchmark"])
    session.open()
    node = session.create_node(node_name)
    return {
        "node": node,
        "parameter_class": None,
        "native_parameters": native_parameters,
        "native_node": node,
        "cleanup": session.close,
        "session": session,
    }


def _declare(state, name, value):
    if state["parameter_class"] is not None:
        return state["node"].declare_parameter(name, value)
    native_parameters = state["native_parameters"]
    return native_parameters.declare_parameter(
        state["node"], native_parameters.parameter_integer(name, value))


def _get(state, name):
    if state["parameter_class"] is not None:
        return state["node"].get_parameter(name)
    return state["native_parameters"].get_parameter(state["node"], name)


def _snapshot(state, parameter):
    if state["parameter_class"] is not None:
        return parameter.value
    return parameter.value_snapshot()


def _has(state, name):
    if state["parameter_class"] is not None:
        return state["node"].has_parameter(name)
    return state["native_parameters"].has_parameter(state["node"], name)


def _setter_values(state, name):
    if state["parameter_class"] is not None:
        parameter_class = state["parameter_class"]
        return (
            parameter_class(name, value=VALUE + 1),
            parameter_class(name, value=VALUE),
        )
    native_parameters = state["native_parameters"]
    return (
        native_parameters.parameter_integer(name, VALUE + 1),
        native_parameters.parameter_integer(name, VALUE),
    )


def _set_atomically(state, parameter):
    if state["parameter_class"] is not None:
        return state["node"].set_parameters_atomically([parameter])
    return state["native_parameters"].set_parameters_atomically(
        state["node"], (parameter,))


def _is_success(result):
    return bool(result.successful)


def _operation(state, args):
    prefix = "bench_%s" % args.run_token[-12:]
    target_name = prefix + ".target"
    parameter_probe = None
    setters = None
    if args.workload != "declare":
        parameter_probe = _declare(state, target_name, VALUE)
    if args.workload == "set-atomically":
        setters = _setter_values(state, target_name)

    if args.workload == "declare":
        def operation(index):
            return _declare(state, "%s.item_%d" % (prefix, index), VALUE)

        measured_start = args.warmup_operations
    elif args.workload == "get-native":
        def operation(_index):
            return _get(state, target_name)

        measured_start = 0
    elif args.workload == "get-value-snapshot":
        def operation(_index):
            return _snapshot(state, _get(state, target_name))

        measured_start = 0
    else:
        def operation(index):
            return _set_atomically(state, setters[index % 2])

        measured_start = 0

    last = None
    for index in range(args.warmup_operations):
        last = operation(index)
        if args.workload == "set-atomically" and not _is_success(last):
            raise RuntimeError("atomic set warmup failed")
    if args.workload == "set-atomically":
        reset = state["parameter_class"](target_name, value=VALUE) \
            if state["parameter_class"] is not None else \
            state["native_parameters"].parameter_integer(target_name, VALUE)
        if not _is_success(_set_atomically(state, reset)):
            raise RuntimeError("atomic set reset failed")

    completed = 0
    cpu_started = time.process_time_ns()
    wall_started = time.perf_counter_ns()
    for offset in range(args.operations):
        last = operation(measured_start + offset)
        completed += 1
    wall_time_ns = time.perf_counter_ns() - wall_started
    process_cpu_time_ns = time.process_time_ns() - cpu_started

    exceptions = 0
    if args.workload == "declare":
        first_name = "%s.item_%d" % (prefix, measured_start)
        last_name = "%s.item_%d" % (
            prefix, measured_start + args.operations - 1)
        verified = _has(state, first_name) and _has(state, last_name)
        final_value = _snapshot(state, last)
        parameter_probe = last
    elif args.workload == "get-native":
        final_value = _snapshot(state, last)
        verified = final_value == VALUE
        parameter_probe = last
    elif args.workload == "get-value-snapshot":
        final_value = int(last)
        verified = final_value == VALUE
    else:
        final_value = _snapshot(state, _get(state, target_name))
        verified = _is_success(last) and final_value == (
            VALUE + 1 if args.operations % 2 else VALUE)
        parameter_probe = setters[0]
    return {
        "process_cpu_time_ns": process_cpu_time_ns,
        "wall_time_ns": wall_time_ns,
        "completed": completed,
        "exceptions": exceptions,
        "final_value": final_value,
        "verified": bool(verified),
        "parameter_probe": parameter_probe,
        "last_result": last,
    }


def _backend_evidence(state, args, measured):
    if args.variant == "stock-rclpy":
        return {
            "verified": True,
            "operation_route": OPERATION_ROUTES[args.variant][args.workload],
            "parameter_representation": "rclpy.parameter.Parameter",
            "exact_cpp_parameter": False,
            "exact_cpp_node": False,
            "exact_cpp_result": False,
            "measured_parameter_values_exact_cpp": False,
            "cpp_parameter_address": None,
            "node_implementation": (
                type(state["node"]).__module__ + "." +
                type(state["node"]).__qualname__),
        }

    import cppyy

    parameter = measured["parameter_probe"]
    if args.variant == "direct-rclcppyy":
        native_parameter = parameter._rclcppyy_native_parameter
    else:
        native_parameter = parameter
    exact_parameter = isinstance(
        native_parameter.native, cppyy.gbl.rclcpp.Parameter)
    exact_node = isinstance(state["native_node"], cppyy.gbl.rclcpp.Node)
    exact_result = (
        isinstance(
            measured["last_result"],
            cppyy.gbl.rcl_interfaces.msg.SetParametersResult,
        )
        if args.workload == "set-atomically" else False
    )
    return {
        "verified": bool(exact_parameter and exact_node),
        "operation_route": OPERATION_ROUTES[args.variant][args.workload],
        "parameter_representation": "rclcpp::Parameter",
        "exact_cpp_parameter": bool(exact_parameter),
        "exact_cpp_node": bool(exact_node),
        "exact_cpp_result": bool(exact_result),
        "measured_parameter_values_exact_cpp": True,
        "cpp_parameter_address": int(cppyy.addressof(native_parameter.native)),
        "parameter_implementation": _cpp_name(native_parameter.native),
        "node_implementation": _cpp_name(state["native_node"]),
    }


def run(args):
    if args.warmup_operations <= 0 or args.operations <= 0:
        raise ValueError("warmup and measured operations must be positive")
    node_name = "local_parameter_bench_%s" % args.run_token[-12:]
    state = _setup(args.variant, node_name)
    loaded_rmw = _loaded_rmw()
    counters = _install_boundary_poison()
    measured = None
    cleanup_error = None
    try:
        measured = _operation(state, args)
        backend = _backend_evidence(state, args, measured)
    finally:
        try:
            state["cleanup"]()
        except BaseException as exception:
            cleanup_error = "%s: %s" % (type(exception).__name__, exception)
    teardown_clean = cleanup_error is None
    cpu_time = measured["process_cpu_time_ns"]
    wall_time = measured["wall_time_ns"]
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
            "fixed_work": True,
            "teardown_clean": teardown_clean,
            "cleanup_error": cleanup_error,
            "ros_distribution": os.environ.get("ROS_DISTRO"),
            "rmw_implementation": loaded_rmw,
            "domain_id": os.environ.get("ROS_DOMAIN_ID"),
        },
        "correctness": {
            "completed_operations": measured["completed"],
            "expected_operations": args.operations,
            "exceptions": measured["exceptions"],
            "final_value": measured["final_value"],
            "verified": measured["verified"],
        },
        "timing": {
            "primary_metric": PRIMARY_METRIC,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "process_cpu_time_ns": cpu_time,
            PRIMARY_METRIC: cpu_time / args.operations,
            "wall_time_ns": wall_time,
            "wall_ns_per_operation": wall_time / args.operations,
            "operations_per_second": args.operations * 1e9 / wall_time,
        },
        "boundary_evidence": {
            "conversion_poison_installed": True,
            "serialization_poison_installed": True,
            **counters,
            "value_snapshot_in_cpu_window": (
                args.workload == "get-value-snapshot"),
        },
        "backend_evidence": backend,
    }


def main(argv=None):
    args = _arguments(argv)
    sample = run(args)
    print(PREFIX + json.dumps(sample, sort_keys=True, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
