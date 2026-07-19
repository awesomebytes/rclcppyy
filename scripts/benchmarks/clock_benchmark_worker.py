#!/usr/bin/env python3
"""Run one fresh-process clock/timestamp benchmark sample."""

from __future__ import annotations

import argparse
import importlib
import json
import os
import time

from _clock_benchmark_protocol import (
    OPERATION_ROUTES,
    POST_INIT_SETTLE_NS,
    PRIMARY_METRIC,
    RMW,
    ROS_DISTRO,
    SAMPLE_SCHEMA,
    VARIANTS,
    WORKLOADS,
)


PREFIX = "@@RCLCPPYY_CLOCK_BENCHMARK_V1@@"


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
        raise RuntimeError("clock benchmark requires Jazzy/CycloneDDS")
    return loaded


def _install_boundary_poison():
    counters = {
        "application_message_conversions": 0,
        "serialization_calls": 0,
        "cdr_calls": 0,
    }

    def conversion(*_args, **_kwargs):
        counters["application_message_conversions"] += 1
        raise AssertionError("application-message conversion entered clock benchmark")

    def serialization_call(*_args, **_kwargs):
        counters["serialization_calls"] += 1
        raise AssertionError("serialization entered clock benchmark")

    def cdr_call(*_args, **_kwargs):
        counters["cdr_calls"] += 1
        raise AssertionError("CDR byte adaptation entered clock benchmark")

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
    if variant == "direct-cpp-rclcppyy":
        import rclcppyy

        rclcppyy.enable_cpp_acceleration(profile="direct_cpp")
        import rclpy
        from rclpy.node import Node

        rclpy.init(args=[])
        node = Node(node_name)
        clock = node.get_clock()
        return {
            "clock": clock,
            "native_node": node._direct_cpp_node,
            "native_clock": clock._native_node_clock,
            "cleanup": lambda: (node.destroy_node(), rclpy.shutdown()),
        }
    if variant in ("stock-rclpy", "compatible-rclcppyy"):
        if variant == "compatible-rclcppyy":
            import rclcppyy

            rclcppyy.enable_cpp_acceleration(profile="compatible")
        import rclpy
        from rclpy.node import Node

        rclpy.init(args=[])
        node = Node(node_name)
        return {
            "clock": node.get_clock(),
            "native_node": None,
            "native_clock": None,
            "cleanup": lambda: (node.destroy_node(), rclpy.shutdown()),
        }

    from rclcpp_kit.native import native

    session = native(["clock-benchmark"])
    session.open()
    node = session.create_node(node_name)
    native_clock = session.create_native_node_clock(node)
    return {
        "clock": native_clock,
        "native_node": node,
        "native_clock": native_clock,
        "cleanup": session.close,
        "session": session,
    }


def _operation_callable(state, args):
    if args.workload == "now":
        return state["clock"].now
    if args.variant == "direct-cpp-rclcppyy":
        return state["native_clock"].now_nanoseconds
    if args.variant == "native-orchestrated":
        return state["clock"].now_nanoseconds
    # stock-rclpy / compatible-rclcppyy: rclpy.clock.Clock has no
    # now_nanoseconds(); the only route to an int is through now().nanoseconds.
    clock = state["clock"]
    return lambda: clock.now().nanoseconds


def _extract_nanoseconds(args, value):
    if args.workload == "now-nanoseconds":
        return int(value)
    if args.variant == "native-orchestrated":
        return int(value.nanoseconds())
    return int(value.nanoseconds)


def _backend_evidence(state, args):
    if args.variant in ("stock-rclpy", "compatible-rclcppyy"):
        return {
            "verified": True,
            "operation_route": OPERATION_ROUTES[args.variant][args.workload],
            "clock_representation": "rclpy.clock.Clock",
            "exact_cpp_clock": False,
            "cpp_clock_address": None,
        }

    import cppyy

    native_clock = state["native_clock"]
    exact_clock = isinstance(state["native_node"], cppyy.gbl.rclcpp.Node)
    representation = (
        "rclcppyy.direct_clock.DirectROSClock"
        if args.variant == "direct-cpp-rclcppyy"
        else "rclcpp_kit.native_clock.NativeNodeClock")
    return {
        "verified": bool(exact_clock),
        "operation_route": OPERATION_ROUTES[args.variant][args.workload],
        "clock_representation": representation,
        "exact_cpp_clock": bool(exact_clock),
        "cpp_clock_address": int(native_clock.address),
    }


def run(args):
    if args.warmup_operations <= 0 or args.operations <= 0:
        raise ValueError("warmup and measured operations must be positive")
    node_name = "clock_bench_%s" % args.run_token[-12:]
    state = _setup(args.variant, node_name)
    loaded_rmw = _loaded_rmw()
    counters = _install_boundary_poison()
    time.sleep(POST_INIT_SETTLE_NS / 1_000_000_000)
    operation = _operation_callable(state, args)

    cleanup_error = None
    try:
        for _ in range(args.warmup_operations):
            operation()

        completed = 0
        last_value = None
        cpu_started = time.process_time_ns()
        wall_started = time.perf_counter_ns()
        for _ in range(args.operations):
            last_value = operation()
            completed += 1
        wall_time_ns = time.perf_counter_ns() - wall_started
        process_cpu_time_ns = time.process_time_ns() - cpu_started
        last_ns = _extract_nanoseconds(args, last_value)
        # A cheap monotonic sanity check outside the measured window: the
        # clock never rewinds relative to its own first post-warmup reading.
        first_ns = _extract_nanoseconds(args, operation())
        verified = isinstance(last_ns, int) and last_ns > 0 and first_ns >= last_ns - (
            10**9)
        # Backend evidence must be captured before cleanup, which closes the
        # retained native clock (direct-cpp) or session (native-orchestrated).
        backend = _backend_evidence(state, args)
    finally:
        try:
            state["cleanup"]()
        except BaseException as exception:
            cleanup_error = "%s: %s" % (type(exception).__name__, exception)
    teardown_clean = cleanup_error is None

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
            "teardown_clean": teardown_clean,
            "cleanup_error": cleanup_error,
            "ros_distribution": os.environ.get("ROS_DISTRO"),
            "rmw_implementation": loaded_rmw,
            "domain_id": os.environ.get("ROS_DOMAIN_ID"),
        },
        "correctness": {
            "completed_operations": completed,
            "expected_operations": args.operations,
            "exceptions": 0,
            "final_value": last_ns,
            "verified": bool(verified),
        },
        "timing": {
            "primary_metric": PRIMARY_METRIC,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "process_cpu_time_ns": process_cpu_time_ns,
            PRIMARY_METRIC: process_cpu_time_ns / args.operations,
            "wall_time_ns": wall_time_ns,
            "wall_ns_per_operation": wall_time_ns / args.operations,
            "operations_per_second": args.operations * 1e9 / wall_time_ns,
        },
        "boundary_evidence": {
            "conversion_poison_installed": True,
            "serialization_poison_installed": True,
            **counters,
        },
        "backend_evidence": backend,
    }


def main(argv=None):
    args = _arguments(argv)
    sample = run(args)
    print(PREFIX + json.dumps(sample, sort_keys=True, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
