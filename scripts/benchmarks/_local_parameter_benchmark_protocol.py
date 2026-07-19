"""Strict evidence contract for local parameter CPU characterization."""

from __future__ import annotations

import datetime
import json
import math
import statistics


SCHEMA_ID = "rclcppyy.local-parameter-benchmark/v1"
SAMPLE_SCHEMA = "rclcppyy.local-parameter-sample/v1"
VARIANTS = (
    "stock-rclpy", "direct-rclcppyy", "native-python-orchestrated")
WORKLOADS = ("declare", "get-native", "get-value-snapshot", "set-atomically")
PRIMARY_METRIC = "process_cpu_ns_per_operation"
RMW = "rmw_cyclonedds_cpp"
ROS_DISTRO = "jazzy"
POST_INIT_SETTLE_NS = 250_000_000
OPERATION_ROUTES = {
    "stock-rclpy": {
        "declare": "rclpy.node.Node.declare_parameter",
        "get-native": "rclpy.node.Node.get_parameter",
        "get-value-snapshot": "rclpy.node.Node.get_parameter.value",
        "set-atomically": "rclpy.node.Node.set_parameters_atomically",
    },
    "direct-rclcppyy": {
        "declare": "rclcppyy.direct_cpp.DirectNode.declare_parameter",
        "get-native": "rclcppyy.direct_cpp.DirectNode.get_parameter",
        "get-value-snapshot": "rclcppyy.direct_cpp.DirectNode.get_parameter.value",
        "set-atomically": "rclcppyy.direct_cpp.DirectNode.set_parameters_atomically",
    },
    "native-python-orchestrated": {
        "declare": "rclcpp_kit.native_parameters.declare_parameter",
        "get-native": "rclcpp_kit.native_parameters.get_parameter",
        "get-value-snapshot": (
            "rclcpp_kit.native_parameters.get_parameter.value_snapshot"),
        "set-atomically": "rclcpp_kit.native_parameters.set_parameters_atomically",
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
        raise ValueError("unsupported local-parameter sample schema")
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
        "post_init_settle_completed", "fixed_work", "teardown_clean",
    )
    if not isinstance(runtime, dict) or any(
            runtime.get(field) is not True for field in required_true):
        raise ValueError("sample runtime controls are incomplete")
    if runtime.get("ros_distribution") != ROS_DISTRO or runtime.get(
            "rmw_implementation") != RMW:
        raise ValueError("sample did not use Jazzy with CycloneDDS")
    if runtime.get("post_init_settle_ns") != POST_INIT_SETTLE_NS:
        raise ValueError("sample post-init settling control is invalid")

    correctness = sample.get("correctness")
    if not isinstance(correctness, dict) or correctness.get(
            "completed_operations") != operations:
        raise ValueError("sample did not complete the fixed work")
    if correctness.get("expected_operations") != operations or correctness.get(
            "exceptions") != 0 or correctness.get("verified") is not True:
        raise ValueError("sample correctness evidence is invalid")
    if not isinstance(correctness.get("final_value"), int):
        raise ValueError("sample final value was not verified")

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
    if boundary.get("value_snapshot_in_cpu_window") is not (
            workload == "get-value-snapshot"):
        raise ValueError("sample value snapshot boundary is misreported")

    backend = sample.get("backend_evidence")
    if not isinstance(backend, dict) or backend.get("verified") is not True:
        raise ValueError("sample backend evidence is missing")
    if backend.get("operation_route") != OPERATION_ROUTES[variant][workload]:
        raise ValueError("sample operation route is invalid")
    if variant == "stock-rclpy":
        if backend.get("parameter_representation") != "rclpy.parameter.Parameter":
            raise ValueError("stock parameter representation is invalid")
        if backend.get("exact_cpp_parameter") is not False or backend.get(
                "cpp_parameter_address") is not None:
            raise ValueError("stock lane claimed exact C++ parameter storage")
        if backend.get("exact_cpp_node") is not False or backend.get(
                "measured_parameter_values_exact_cpp") is not False:
            raise ValueError("stock lane claimed native control storage")
    else:
        if backend.get("parameter_representation") != "rclcpp::Parameter":
            raise ValueError("native parameter representation is invalid")
        if backend.get("exact_cpp_parameter") is not True or not _positive_integer(
                backend.get("cpp_parameter_address")):
            raise ValueError("native lane lacks exact C++ storage evidence")
        if backend.get("exact_cpp_node") is not True:
            raise ValueError("native lane lacks exact C++ node evidence")
        if backend.get("measured_parameter_values_exact_cpp") is not True:
            raise ValueError("native lane lacks exact C++ control values")
        if backend.get("parameter_implementation") != "rclcpp::Parameter" or backend.get(
                "node_implementation") != "rclcpp::Node":
            raise ValueError("native lane implementation identities are invalid")
        if workload == "set-atomically" and backend.get(
                "exact_cpp_result") is not True:
            raise ValueError("native set lane lacks exact C++ result evidence")


def summarize(samples, repetitions):
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
                raise ValueError("each workload requires a complete rotated triple")
            stock_cpu = selected["stock-rclpy"]["timing"][PRIMARY_METRIC]
            paired.append({
                "repetition": repetition,
                "workload": workload,
                "direct_over_stock_process_cpu": (
                    selected["direct-rclcppyy"]["timing"][PRIMARY_METRIC] /
                    stock_cpu),
                "native_python_over_stock_process_cpu": (
                    selected["native-python-orchestrated"]["timing"][PRIMARY_METRIC] /
                    stock_cpu),
            })
    medians = {
        workload: {
            "direct_over_stock_process_cpu": statistics.median(
                row["direct_over_stock_process_cpu"]
                for row in paired if row["workload"] == workload),
            "native_python_over_stock_process_cpu": statistics.median(
                row["native_python_over_stock_process_cpu"]
                for row in paired if row["workload"] == workload),
        }
        for workload in WORKLOADS
    }
    snapshot_cost = []
    for repetition in range(repetitions):
        for variant in VARIANTS:
            selected = {
                sample["workload"]: sample["timing"][PRIMARY_METRIC]
                for sample in samples
                if sample["repetition"] == repetition and
                sample["variant"] == variant and
                sample["workload"] in ("get-native", "get-value-snapshot")
            }
            snapshot_cost.append({
                "repetition": repetition,
                "variant": variant,
                "snapshot_over_native_get_process_cpu": (
                    selected["get-value-snapshot"] / selected["get-native"]),
            })
    return {
        "primary_metric": PRIMARY_METRIC,
        "paired_cpu_ratios": paired,
        "median_cpu_ratios": medians,
        "snapshot_materialization_ratios": snapshot_cost,
    }


def build_document(*, environment, command, warmup, operations, repetitions, samples):
    document = {
        "schema": SCHEMA_ID,
        "generated_at": datetime.datetime.now(
            datetime.timezone.utc).isoformat().replace("+00:00", "Z"),
        "command": list(command),
        "environment": environment,
        "benchmark": {
            "name": "local-parameter-cpu-characterization",
            "performance_claims_allowed": False,
            "primary_metric": PRIMARY_METRIC,
            "warmup_operations": warmup,
            "measured_operations": operations,
            "repetitions": repetitions,
            "variants": list(VARIANTS),
            "workloads": list(WORKLOADS),
            "execution_order_policy": "deterministic-rotating-variant",
            "setup_init_jit_excluded": True,
            "post_init_settle_ns": POST_INIT_SETTLE_NS,
        },
        "execution_order": execution_order(repetitions),
        "samples": samples,
        "summary": summarize(samples, repetitions),
    }
    validate_document(document)
    return document


def validate_document(document):
    if not isinstance(document, dict) or document.get("schema") != SCHEMA_ID:
        raise ValueError("unsupported local-parameter benchmark schema")
    if not isinstance(document.get("generated_at"), str) or not isinstance(
            document.get("environment"), dict):
        raise ValueError("benchmark time/environment evidence is missing")
    if not isinstance(document.get("command"), list) or not document["command"]:
        raise ValueError("benchmark command evidence is missing")
    benchmark = document.get("benchmark")
    if not isinstance(benchmark, dict) or benchmark.get(
            "performance_claims_allowed") is not False:
        raise ValueError("local parameter benchmark claims must remain disabled")
    if benchmark.get("primary_metric") != PRIMARY_METRIC or benchmark.get(
            "execution_order_policy") != "deterministic-rotating-variant":
        raise ValueError("benchmark CPU/order policy is invalid")
    if benchmark.get("setup_init_jit_excluded") is not True:
        raise ValueError("benchmark setup exclusion is invalid")
    if benchmark.get("post_init_settle_ns") != POST_INIT_SETTLE_NS:
        raise ValueError("benchmark post-init settling control is invalid")
    if benchmark.get("name") != "local-parameter-cpu-characterization" or benchmark.get(
            "variants") != list(VARIANTS) or benchmark.get(
            "workloads") != list(WORKLOADS):
        raise ValueError("benchmark matrix declaration is invalid")
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
    "PRIMARY_METRIC",
    "OPERATION_ROUTES",
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
