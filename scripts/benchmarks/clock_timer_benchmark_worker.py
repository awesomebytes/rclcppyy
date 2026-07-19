#!/usr/bin/env python3
"""Run one fresh-process clock-timer / rate-sleep benchmark sample."""

from __future__ import annotations

import argparse
import importlib
import json
import os
import threading
import time

from _clock_timer_benchmark_protocol import (
    OPERATION_ROUTES,
    PERIOD_NS,
    PRIMARY_METRIC,
    RMW,
    ROS_DISTRO,
    SAMPLE_SCHEMA,
    VARIANTS,
    WORKLOADS,
)


PREFIX = "@@RCLCPPYY_CLOCK_TIMER_BENCHMARK_V1@@"
SPIN_TIMEOUT_SEC = PERIOD_NS / 1e9 * 5
DEADLINE_MARGIN_S = 30.0


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
        raise RuntimeError("clock-timer benchmark requires Jazzy/CycloneDDS")
    return loaded


def _install_boundary_poison():
    counters = {
        "application_message_conversions": 0,
        "serialization_calls": 0,
        "cdr_calls": 0,
    }

    def conversion(*_args, **_kwargs):
        counters["application_message_conversions"] += 1
        raise AssertionError("application-message conversion entered clock-timer benchmark")

    def serialization_call(*_args, **_kwargs):
        counters["serialization_calls"] += 1
        raise AssertionError("serialization entered clock-timer benchmark")

    def cdr_call(*_args, **_kwargs):
        counters["cdr_calls"] += 1
        raise AssertionError("CDR byte adaptation entered clock-timer benchmark")

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


def _node_name(args):
    return "clock_timer_bench_%s" % args.run_token[-12:]


# ---------------------------------------------------------------------------
# Shared two-phase measurement: run warmup firings/sleeps first (excluded from
# timing), then time exactly `operations` more with CLOCK_PROCESS_CPUTIME_ID.
# ---------------------------------------------------------------------------

def _measure(args, step, cleanup, backend):
    deadline = time.monotonic() + max(
        DEADLINE_MARGIN_S,
        (args.warmup_operations + args.operations) * PERIOD_NS / 1e9 * 10)
    for _ in range(args.warmup_operations):
        step()
        if time.monotonic() > deadline:
            raise RuntimeError("clock-timer benchmark warmup timed out")

    cpu_started = time.process_time_ns()
    wall_started = time.perf_counter_ns()
    for _ in range(args.operations):
        step()
        if time.monotonic() > deadline:
            raise RuntimeError("clock-timer benchmark measurement timed out")
    wall_time_ns = time.perf_counter_ns() - wall_started
    process_cpu_time_ns = time.process_time_ns() - cpu_started

    cleanup_error = None
    try:
        cleanup()
    except BaseException as exception:
        cleanup_error = "%s: %s" % (type(exception).__name__, exception)

    return {
        "process_cpu_time_ns": process_cpu_time_ns,
        "wall_time_ns": wall_time_ns,
        "backend": backend,
        "cleanup_error": cleanup_error,
    }


# ---------------------------------------------------------------------------
# clock-timer workload: one firing per `step()`, driven by spinning an
# executor until a counter set from the timer callback advances.
# ---------------------------------------------------------------------------

def _spin_until_fired(spin_once, counter, deadline):
    target = counter["n"] + 1
    while counter["n"] < target:
        spin_once()
        if time.monotonic() > deadline:
            raise RuntimeError("clock-timer benchmark firing timed out")


def _setup_clock_timer(args):
    counter = {"n": 0}
    step_deadline = time.monotonic() + max(
        DEADLINE_MARGIN_S,
        (args.warmup_operations + args.operations) * PERIOD_NS / 1e9 * 10)

    if args.variant in ("stock-rclpy", "compatible-rclcppyy"):
        if args.variant == "compatible-rclcppyy":
            import rclcppyy

            rclcppyy.enable_cpp_acceleration(profile="compatible")
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node

        rclpy.init(args=[])
        node = Node(_node_name(args))
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        def callback():
            counter["n"] += 1

        timer = node.create_timer(PERIOD_NS / 1e9, callback)

        def step():
            _spin_until_fired(
                lambda: executor.spin_once(timeout_sec=SPIN_TIMEOUT_SEC),
                counter, step_deadline)

        def cleanup():
            node.destroy_timer(timer)
            executor.remove_node(node)
            executor.shutdown(timeout_sec=2.0)
            node.destroy_node()
            rclpy.shutdown()

        backend = {
            "verified": True,
            "operation_route": OPERATION_ROUTES[args.variant]["clock-timer"],
            "exact_cpp_entity": False,
            "native_type": None,
        }
        return step, cleanup, backend

    if args.variant == "direct-cpp-rclcppyy":
        import rclcppyy

        rclcppyy.enable_cpp_acceleration(profile="direct_cpp")
        import rclpy
        from rclpy.node import Node

        rclpy.init(args=[])
        node = Node(_node_name(args))

        def callback():
            counter["n"] += 1

        timer = node.create_timer(PERIOD_NS / 1e9, callback)

        def step():
            _spin_until_fired(
                lambda: rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_SEC),
                counter, step_deadline)

        def cleanup():
            node.destroy_timer(timer)
            node.destroy_node()
            rclpy.shutdown()

        backend = {
            "verified": True,
            "operation_route": OPERATION_ROUTES[args.variant]["clock-timer"],
            "exact_cpp_entity": True,
            "native_type": timer.native_type_name,
        }
        return step, cleanup, backend

    # native-orchestrated: the facade floor -- create_clock_timer driven
    # directly through a NativeSession executor, with no DirectNode at all.
    import cppyy
    from rclcpp_kit import direct_entities
    from rclcpp_kit.native import native

    session = native(["clock-timer-benchmark-native-timer"])
    session.open()
    node = session.create_node(_node_name(args))
    executor = session.create_executor("single_threaded")
    executor.add_node(node)

    def callback():
        counter["n"] += 1

    timer = direct_entities.create_clock_timer(node, PERIOD_NS, callback)
    duration = cppyy.gbl.std.chrono.nanoseconds(int(SPIN_TIMEOUT_SEC * 1e9))

    def step():
        _spin_until_fired(
            lambda: executor.spin_once(duration), counter, step_deadline)

    def cleanup():
        timer.destroy()
        session.close()

    backend = {
        "verified": True,
        "operation_route": OPERATION_ROUTES[args.variant]["clock-timer"],
        "exact_cpp_entity": True,
        "native_type": timer.native_type_name,
    }
    return step, cleanup, backend


# ---------------------------------------------------------------------------
# rate-sleep workload: one Rate.sleep()/sleeper.sleep_until() call per
# `step()`. Stock's Rate needs a background-thread-spun executor (its sleep()
# blocks on a threading.Event a Timer callback sets); DirectRate needs none.
# ---------------------------------------------------------------------------

def _setup_rate_sleep(args):
    if args.variant in ("stock-rclpy", "compatible-rclcppyy"):
        if args.variant == "compatible-rclcppyy":
            import rclcppyy

            rclcppyy.enable_cpp_acceleration(profile="compatible")
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node

        rclpy.init(args=[])
        node = Node(_node_name(args))
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        rate = node.create_rate(1e9 / PERIOD_NS)

        def step():
            rate.sleep()

        def cleanup():
            node.destroy_rate(rate)
            executor.remove_node(node)
            executor.shutdown(timeout_sec=2.0)
            node.destroy_node()
            rclpy.shutdown()
            spin_thread.join(timeout=2.0)

        backend = {
            "verified": True,
            "operation_route": OPERATION_ROUTES[args.variant]["rate-sleep"],
            "exact_cpp_entity": False,
            "native_type": None,
        }
        return step, cleanup, backend

    if args.variant == "direct-cpp-rclcppyy":
        import rclcppyy

        rclcppyy.enable_cpp_acceleration(profile="direct_cpp")
        import rclpy
        from rclpy.node import Node

        rclpy.init(args=[])
        node = Node(_node_name(args))
        rate = node.create_rate(1e9 / PERIOD_NS)

        def step():
            rate.sleep()

        def cleanup():
            node.destroy_rate(rate)
            node.destroy_node()
            rclpy.shutdown()

        backend = {
            "verified": True,
            "operation_route": OPERATION_ROUTES[args.variant]["rate-sleep"],
            "exact_cpp_entity": True,
            "native_type": None,
        }
        return step, cleanup, backend

    # native-orchestrated: the facade floor -- NativeClockSleeper driven
    # directly with the same fixed-rate bookkeeping DirectRate uses, no
    # DirectNode and no DirectRate Python wrapper at all.
    from rclcpp_kit.native import native

    session = native(["clock-timer-benchmark-native-rate"])
    session.open()
    node = session.create_node(_node_name(args))
    sleeper = session.create_native_clock_sleeper(node)
    clock = session.create_native_node_clock(node)
    schedule = {"next_ns": clock.now_nanoseconds() + PERIOD_NS}

    def step():
        sleeper.sleep_until(schedule["next_ns"])
        schedule["next_ns"] += PERIOD_NS
        # Same catch-up guard DirectRate applies (rclcpp::Rate's algorithm):
        # without it, a one-time startup delay before the first call would
        # permanently offset the schedule into the past, resolving every
        # later sleep instantly instead of pacing at PERIOD_NS.
        now_ns = clock.now_nanoseconds()
        if now_ns > schedule["next_ns"] + PERIOD_NS:
            schedule["next_ns"] = now_ns + PERIOD_NS

    def cleanup():
        session.close()

    backend = {
        "verified": True,
        "operation_route": OPERATION_ROUTES[args.variant]["rate-sleep"],
        "exact_cpp_entity": True,
        "native_type": None,
    }
    return step, cleanup, backend


def run(args):
    if args.warmup_operations <= 0 or args.operations <= 0:
        raise ValueError("warmup and measured operations must be positive")

    if args.workload == "clock-timer":
        step, cleanup, backend = _setup_clock_timer(args)
    else:
        step, cleanup, backend = _setup_rate_sleep(args)

    loaded_rmw = _loaded_rmw()
    counters = _install_boundary_poison()
    result = _measure(args, step, cleanup, backend)
    teardown_clean = result["cleanup_error"] is None

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
            "cleanup_error": result["cleanup_error"],
            "ros_distribution": os.environ.get("ROS_DISTRO"),
            "rmw_implementation": loaded_rmw,
            "domain_id": os.environ.get("ROS_DOMAIN_ID"),
            "period_ns": PERIOD_NS,
        },
        "correctness": {
            "completed_operations": args.operations,
            "expected_operations": args.operations,
            "exceptions": 0,
            "verified": True,
        },
        "timing": {
            "primary_metric": PRIMARY_METRIC,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "process_cpu_time_ns": result["process_cpu_time_ns"],
            PRIMARY_METRIC: result["process_cpu_time_ns"] / args.operations,
            "wall_time_ns": result["wall_time_ns"],
            "wall_ns_per_operation": result["wall_time_ns"] / args.operations,
            "operations_per_second": args.operations * 1e9 / result["wall_time_ns"],
        },
        "boundary_evidence": {
            "conversion_poison_installed": True,
            "serialization_poison_installed": True,
            **counters,
        },
        "backend_evidence": result["backend"],
    }


def main(argv=None):
    args = _arguments(argv)
    sample = run(args)
    print(PREFIX + json.dumps(sample, sort_keys=True, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
