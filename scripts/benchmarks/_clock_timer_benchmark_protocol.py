"""Strict evidence contract for the ROS-clock-timer + Rate-sleep CPU benchmark.

Re-measures the create_timer hot path now that its default routes through a
GenericTimer on the node's own ROS clock (rather than a steady WallTimer),
and adds a first CPU characterization of DirectRate.sleep() over the same
NativeClockSleeper. Mirrors the clock/timestamp benchmark's four-variant
taxonomy and CPU-first primary metric; mirrors the timer-executor benchmark's
warmup/measured firing counts and fixed period.
"""

from __future__ import annotations

import datetime
import json
import math
import statistics


SCHEMA_ID = "rclcppyy.clock-timer-benchmark/v1"
SAMPLE_SCHEMA = "rclcppyy.clock-timer-sample/v1"
VARIANTS = (
    "stock-rclpy", "compatible-rclcppyy", "direct-cpp-rclcppyy",
    "native-orchestrated")
WORKLOADS = ("clock-timer", "rate-sleep")
PRIMARY_METRIC = "process_cpu_ns_per_operation"
RMW = "rmw_cyclonedds_cpp"
ROS_DISTRO = "jazzy"
# 1 ms, matching the timer-executor benchmark's fixed period.
PERIOD_NS = 1_000_000
OPERATION_ROUTES = {
    "stock-rclpy": {
        "clock-timer": "rclpy.node.Node.create_timer (Timer callback via spin_once)",
        "rate-sleep": "rclpy.node.Node.create_rate (Rate.sleep)",
    },
    "compatible-rclcppyy": {
        "clock-timer": "rclpy.node.Node.create_timer (Timer callback via spin_once)",
        "rate-sleep": "rclpy.node.Node.create_rate (Rate.sleep)",
    },
    "direct-cpp-rclcppyy": {
        "clock-timer": (
            "rclcppyy.direct_cpp.DirectNode.create_timer "
            "(rclcpp_kit.direct_entities.create_clock_timer)"),
        "rate-sleep": (
            "rclcppyy.direct_cpp.DirectNode.create_rate "
            "(rclcppyy.direct_clock.DirectRate.sleep)"),
    },
    "native-orchestrated": {
        "clock-timer": (
            "rclcpp_kit.direct_entities.create_clock_timer "
            "(NativeSession executor, no DirectNode)"),
        "rate-sleep": "rclcpp_kit.native_clock_sleep.NativeClockSleeper.sleep_until",
    },
}


def _positive_integer(value):
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def _finite_positive(value):
    return (
        isinstance(value, (int, float)) and
        not isinstance(value, bool) and
        math.isfinite(value) and
        value > 0
    )


def execution_order(repetitions):
    order = []
    for repetition in range(repetitions):
        for workload_index, workload in enumerate(WORKLOADS):
            shift = (repetition + workload_index) % len(VARIANTS)
            variants = VARIANTS[shift:] + VARIANTS[:shift]
            for variant in variants:
                order.append({
                    "repetition": repetition,
                    "workload": workload,
                    "variant": variant,
                })
    return order


def validate_sample(sample, *, warmup, operations, repetition, order_index):
    if not isinstance(sample, dict) or sample.get("schema") != SAMPLE_SCHEMA:
        raise ValueError("unsupported clock-timer sample schema")
    variant = sample.get("variant")
    workload = sample.get("workload")
    if variant not in VARIANTS or workload not in WORKLOADS:
        raise ValueError("sample variant or workload is invalid")
    if sample.get("warmup_operations") != warmup or sample.get(
            "measured_operations") != operations:
        raise ValueError("sample work differs from the fixed request")
    if sample.get("repetition") != repetition or sample.get(
            "order_index") != order_index:
        raise ValueError("sample ordering evidence is invalid")
    if not _positive_integer(sample.get("pid")) or not isinstance(
            sample.get("run_token"), str) or not sample["run_token"]:
        raise ValueError("sample fresh-process identity is invalid")

    runtime = sample.get("runtime")
    required_true = (
        "fresh_process", "setup_excluded", "jit_excluded", "warmup_completed",
        "fixed_work", "teardown_clean",
    )
    if not isinstance(runtime, dict) or any(
            runtime.get(field) is not True for field in required_true):
        raise ValueError("sample runtime controls are incomplete")
    if runtime.get("ros_distribution") != ROS_DISTRO or runtime.get(
            "rmw_implementation") != RMW:
        raise ValueError("sample did not use Jazzy with CycloneDDS")
    if runtime.get("period_ns") != PERIOD_NS:
        raise ValueError("sample period control is invalid")

    correctness = sample.get("correctness")
    if not isinstance(correctness, dict) or correctness.get(
            "completed_operations") != operations:
        raise ValueError("sample did not complete the fixed work")
    if correctness.get("expected_operations") != operations or correctness.get(
            "exceptions") != 0 or correctness.get("verified") is not True:
        raise ValueError("sample correctness evidence is invalid")

    timing = sample.get("timing")
    if not isinstance(timing, dict) or timing.get(
            "cpu_clock") != "CLOCK_PROCESS_CPUTIME_ID":
        raise ValueError("sample CPU clock evidence is invalid")
    if timing.get("primary_metric") != PRIMARY_METRIC:
        raise ValueError("process CPU per operation must be primary")
    if not _positive_integer(timing.get("process_cpu_time_ns")) or not _positive_integer(
            timing.get("wall_time_ns")):
        raise ValueError("sample timing window is invalid")
    for field in (PRIMARY_METRIC, "wall_ns_per_operation", "operations_per_second"):
        if not _finite_positive(timing.get(field)):
            raise ValueError("sample metric %s is invalid" % field)
    if not math.isclose(
            timing[PRIMARY_METRIC],
            timing["process_cpu_time_ns"] / operations,
            rel_tol=1e-12):
        raise ValueError("sample CPU normalization is inconsistent")
    if not math.isclose(
            timing["operations_per_second"],
            operations * 1e9 / timing["wall_time_ns"],
            rel_tol=1e-12):
        raise ValueError("sample throughput normalization is inconsistent")

    boundary = sample.get("boundary_evidence")
    if not isinstance(boundary, dict) or boundary.get(
            "conversion_poison_installed") is not True:
        raise ValueError("sample conversion poison is missing")
    if boundary.get("serialization_poison_installed") is not True or boundary.get(
            "application_message_conversions") != 0 or boundary.get(
            "serialization_calls") != 0 or boundary.get("cdr_calls") != 0:
        raise ValueError("sample used a forbidden conversion or serialization path")

    backend = sample.get("backend_evidence")
    if not isinstance(backend, dict) or backend.get("verified") is not True:
        raise ValueError("sample backend evidence is missing")
    if backend.get("operation_route") != OPERATION_ROUTES[variant][workload]:
        raise ValueError("sample operation route is invalid")
    if variant in ("stock-rclpy", "compatible-rclcppyy"):
        if backend.get("exact_cpp_entity") is not False:
            raise ValueError("stock/compatible lane claimed exact C++ entity authority")
    else:
        if backend.get("exact_cpp_entity") is not True:
            raise ValueError("native-backed lane lacks exact C++ entity evidence")
        if workload == "clock-timer" and "GenericTimer" not in (
                backend.get("native_type") or ""):
            raise ValueError("clock-timer lane is not backed by a native GenericTimer")


def summarize(samples, repetitions):
    ratio_keys = (
        "compatible_over_stock_process_cpu",
        "direct_over_stock_process_cpu",
        "native_over_stock_process_cpu",
    )
    paired = []
    for repetition in range(repetitions):
        for workload in WORKLOADS:
            selected = {
                sample["variant"]: sample
                for sample in samples
                if sample["repetition"] == repetition and
                sample["workload"] == workload
            }
            if set(selected) != set(VARIANTS):
                raise ValueError("each workload requires a complete rotated set")
            stock_cpu = selected["stock-rclpy"]["timing"][PRIMARY_METRIC]
            paired.append({
                "repetition": repetition,
                "workload": workload,
                "compatible_over_stock_process_cpu": (
                    selected["compatible-rclcppyy"]["timing"][PRIMARY_METRIC] /
                    stock_cpu),
                "direct_over_stock_process_cpu": (
                    selected["direct-cpp-rclcppyy"]["timing"][PRIMARY_METRIC] /
                    stock_cpu),
                "native_over_stock_process_cpu": (
                    selected["native-orchestrated"]["timing"][PRIMARY_METRIC] /
                    stock_cpu),
            })
    medians = {
        workload: {
            key: statistics.median(
                row[key] for row in paired if row["workload"] == workload)
            for key in ratio_keys
        }
        for workload in WORKLOADS
    }
    return {
        "primary_metric": PRIMARY_METRIC,
        "paired_cpu_ratios": paired,
        "median_cpu_ratios": medians,
    }


def build_document(*, environment, command, warmup, operations, repetitions, samples):
    document = {
        "schema": SCHEMA_ID,
        "generated_at": datetime.datetime.now(
            datetime.timezone.utc).isoformat().replace("+00:00", "Z"),
        "command": list(command),
        "environment": environment,
        "benchmark": {
            "name": "clock-timer-rate-sleep-cpu-characterization",
            "performance_claims_allowed": False,
            "primary_metric": PRIMARY_METRIC,
            "warmup_operations": warmup,
            "measured_operations": operations,
            "repetitions": repetitions,
            "variants": list(VARIANTS),
            "workloads": list(WORKLOADS),
            "period_ns": PERIOD_NS,
            "execution_order_policy": "deterministic-rotating-variant",
        },
        "execution_order": execution_order(repetitions),
        "samples": samples,
        "summary": summarize(samples, repetitions),
    }
    validate_document(document)
    return document


def validate_document(document):
    if not isinstance(document, dict) or document.get("schema") != SCHEMA_ID:
        raise ValueError("unsupported clock-timer benchmark schema")
    if not isinstance(document.get("generated_at"), str) or not isinstance(
            document.get("environment"), dict):
        raise ValueError("benchmark time/environment evidence is missing")
    if not isinstance(document.get("command"), list) or not document["command"]:
        raise ValueError("benchmark command evidence is missing")
    benchmark = document.get("benchmark")
    if not isinstance(benchmark, dict) or benchmark.get(
            "performance_claims_allowed") is not False:
        raise ValueError("clock-timer benchmark claims must remain disabled")
    if benchmark.get("primary_metric") != PRIMARY_METRIC or benchmark.get(
            "execution_order_policy") != "deterministic-rotating-variant":
        raise ValueError("benchmark CPU/order policy is invalid")
    if benchmark.get("name") != "clock-timer-rate-sleep-cpu-characterization" or benchmark.get(
            "variants") != list(VARIANTS) or benchmark.get(
            "workloads") != list(WORKLOADS):
        raise ValueError("benchmark matrix declaration is invalid")
    if benchmark.get("period_ns") != PERIOD_NS:
        raise ValueError("benchmark period control is invalid")
    warmup = benchmark.get("warmup_operations")
    operations = benchmark.get("measured_operations")
    repetitions = benchmark.get("repetitions")
    if any(not _positive_integer(value) for value in (
            warmup, operations, repetitions)):
        raise ValueError("benchmark work controls are invalid")
    expected_order = execution_order(repetitions)
    if document.get("execution_order") != expected_order:
        raise ValueError("benchmark execution order is not deterministically rotating")
    samples = document.get("samples")
    if not isinstance(samples, list) or len(samples) != len(expected_order):
        raise ValueError("benchmark sample matrix is incomplete")
    for index, (sample, expected) in enumerate(zip(samples, expected_order)):
        if any(sample.get(field) != expected[field] for field in expected):
            raise ValueError("sample order differs from its declaration")
        validate_sample(
            sample,
            warmup=warmup,
            operations=operations,
            repetition=expected["repetition"],
            order_index=index,
        )
    if document.get("summary") != summarize(samples, repetitions):
        raise ValueError("benchmark summary is inconsistent")


def dumps(document):
    validate_document(document)
    return json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n"


__all__ = [
    "OPERATION_ROUTES",
    "PERIOD_NS",
    "PRIMARY_METRIC",
    "RMW",
    "ROS_DISTRO",
    "SAMPLE_SCHEMA",
    "SCHEMA_ID",
    "VARIANTS",
    "WORKLOADS",
    "build_document",
    "dumps",
    "execution_order",
    "summarize",
    "validate_document",
    "validate_sample",
]
