"""Strict evidence contract for remote parameter CPU characterization."""

from __future__ import annotations

import datetime
import json
import math
import re
import statistics


SCHEMA_ID = "rclcppyy.remote-parameter-benchmark/v1"
SAMPLE_SCHEMA = "rclcppyy.remote-parameter-sample/v1"
VARIANTS = ("stock-rclpy", "direct-rclcppyy")
WORKLOADS = ("get-one", "set-atomically-one")
PRIMARY_METRIC = "process_cpu_ns_per_operation"
RMW = "rmw_cyclonedds_cpp"
ROS_DISTRO = "jazzy"
POST_INIT_SETTLE_NS = 250_000_000
SERVICE_COUNT = 6
QOS = {
    "history": "keep_last",
    "depth": 10,
    "reliability": "reliable",
    "durability": "volatile",
}
OPERATION_ROUTES = {
    workload: "rclpy.parameter_client.AsyncParameterClient.%s" % method
    for workload, method in (
        ("get-one", "get_parameters"),
        ("set-atomically-one", "set_parameters_atomically"),
    )
}
SHA256 = re.compile(r"^[0-9a-f]{64}$")


def _positive_integer(value):
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def _finite_positive(value):
    return (
        isinstance(value, (int, float)) and
        not isinstance(value, bool) and
        math.isfinite(value) and
        value > 0
    )


def nearest_rank(values, percentile):
    if not values:
        raise ValueError("latency observations cannot be empty")
    ordered = sorted(values)
    index = max(1, math.ceil(percentile / 100 * len(ordered))) - 1
    return ordered[index]


def latency_summary(values):
    return {
        "p50_ns": nearest_rank(values, 50),
        "p95_ns": nearest_rank(values, 95),
        "p99_ns": nearest_rank(values, 99),
        "max_ns": max(values),
    }


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


def _validate_timing(timing, operations):
    if not isinstance(timing, dict) or timing.get(
            "primary_metric") != PRIMARY_METRIC:
        raise ValueError("remote parameter primary metric is invalid")
    if timing.get("cpu_clock") != "CLOCK_PROCESS_CPUTIME_ID" or timing.get(
            "cpu_scope") != "combined-server-client-process":
        raise ValueError("remote parameter CPU scope is invalid")
    cpu_time = timing.get("process_cpu_time_ns")
    wall_time = timing.get("wall_time_ns")
    if not _positive_integer(cpu_time) or not _positive_integer(wall_time):
        raise ValueError("remote parameter timing window is invalid")
    for field in (
        PRIMARY_METRIC, "wall_ns_per_operation", "operations_per_second",
    ):
        if not _finite_positive(timing.get(field)):
            raise ValueError("remote parameter metric %s is invalid" % field)
    if not math.isclose(
            timing[PRIMARY_METRIC], cpu_time / operations, rel_tol=1e-12):
        raise ValueError("remote parameter CPU normalization is inconsistent")
    if not math.isclose(
            timing["operations_per_second"], operations * 1e9 / wall_time,
            rel_tol=1e-12):
        raise ValueError("remote parameter throughput normalization is inconsistent")
    latency = timing.get("latency_ns")
    if not isinstance(latency, list) or len(latency) != operations or any(
            not _positive_integer(value) for value in latency):
        raise ValueError("remote parameter raw latency evidence is invalid")
    if timing.get("latency") != latency_summary(latency):
        raise ValueError("remote parameter latency summary is inconsistent")


def _validate_topology(topology):
    expected = {
        "process_count": 1,
        "node_count": 2,
        "executor": "rclpy.executors.SingleThreadedExecutor",
        "one_outstanding_request": True,
        "server_parameter_service_count": SERVICE_COUNT,
        "client_parameter_client_count": SERVICE_COUNT,
        "same_context": True,
        "graph_verified": True,
        "service_qos": QOS,
    }
    if not isinstance(topology, dict) or any(
            topology.get(key) != value for key, value in expected.items()):
        raise ValueError("remote parameter topology/QoS evidence is invalid")
    for name in ("server_node", "client_node", "remote_node_name"):
        if not isinstance(topology.get(name), str) or not topology[name]:
            raise ValueError("remote parameter topology names are invalid")


def _validate_backend(backend, variant, workload):
    if not isinstance(backend, dict) or backend.get("verified") is not True:
        raise ValueError("remote parameter backend evidence is missing")
    if backend.get("operation_route") != OPERATION_ROUTES[workload]:
        raise ValueError("remote parameter operation route is invalid")
    if backend.get(
            "async_parameter_client_source") != (
                "rclpy.parameter_client.AsyncParameterClient"):
        raise ValueError("remote parameter client source is not unchanged rclpy")
    if backend.get("source_shape_verified") is not True:
        raise ValueError("remote parameter client source shape is invalid")
    if not isinstance(backend.get("source_path"), str) or not backend[
            "source_path"].endswith("rclpy/parameter_client.py"):
        raise ValueError("remote parameter client source path is invalid")
    if not isinstance(backend.get("source_sha256"), str) or SHA256.fullmatch(
            backend["source_sha256"]) is None:
        raise ValueError("remote parameter client source digest is invalid")
    if backend.get("python_payload_cache") is not False:
        raise ValueError("remote parameter lane cached a Python payload")
    direct = variant == "direct-rclcppyy"
    expected_authority = "cpp" if direct else "python"
    expected_representation = "actual-generated-cpp" if direct else "generated-python"
    for field in ("server_authority", "client_authority"):
        if backend.get(field) != expected_authority:
            raise ValueError("remote parameter authority evidence is invalid")
    for field in (
        "request_representation", "response_representation",
        "parameter_representation",
    ):
        if backend.get(field) != expected_representation:
            raise ValueError("remote parameter representation evidence is invalid")
    exact_fields = (
        "all_parameter_service_aliases_exact_cpp",
        "request_alias_exact_cpp",
        "response_exact_cpp",
        "parameter_value_exact_cpp",
        "parameter_message_alias_exact_cpp",
        "client_entities_exact_cpp",
        "server_node_exact_cpp",
        "client_node_exact_cpp",
        "parameter_owner_exact_cpp",
    )
    if any(backend.get(field) is not direct for field in exact_fields):
        raise ValueError("remote parameter exact-C++ evidence is invalid")
    result_exact = backend.get("set_result_exact_cpp")
    if result_exact is not (direct and workload == "set-atomically-one"):
        raise ValueError("remote parameter set-result evidence is invalid")
    addresses = backend.get("cpp_node_addresses")
    if direct:
        if not isinstance(addresses, list) or len(addresses) != 2 or any(
                not _positive_integer(value) for value in addresses):
            raise ValueError("remote parameter C++ node identity is invalid")
    elif addresses is not None:
        raise ValueError("stock remote parameter lane claimed C++ node identity")
    if backend.get("retained_response_verified") is not True:
        raise ValueError("remote parameter response lifetime was not verified")


def validate_sample(sample, *, warmup, operations, repetition, order_index):
    if not isinstance(sample, dict) or sample.get("schema") != SAMPLE_SCHEMA:
        raise ValueError("unsupported remote-parameter sample schema")
    variant = sample.get("variant")
    workload = sample.get("workload")
    if variant not in VARIANTS or workload not in WORKLOADS:
        raise ValueError("remote parameter sample variant/workload is invalid")
    if sample.get("warmup_operations") != warmup or sample.get(
            "measured_operations") != operations:
        raise ValueError("remote parameter sample work differs from request")
    if sample.get("repetition") != repetition or sample.get(
            "order_index") != order_index:
        raise ValueError("remote parameter sample ordering is invalid")
    if not _positive_integer(sample.get("pid")) or not isinstance(
            sample.get("run_token"), str) or not sample["run_token"]:
        raise ValueError("remote parameter fresh-process identity is invalid")

    runtime = sample.get("runtime")
    required_true = (
        "fresh_process", "setup_excluded", "jit_excluded", "warmup_completed",
        "post_init_settle_completed", "fixed_work", "teardown_clean",
    )
    if not isinstance(runtime, dict) or any(
            runtime.get(field) is not True for field in required_true):
        raise ValueError("remote parameter runtime controls are incomplete")
    if runtime.get("ros_distribution") != ROS_DISTRO or runtime.get(
            "rmw_implementation") != RMW or runtime.get(
                "post_init_settle_ns") != POST_INIT_SETTLE_NS:
        raise ValueError("remote parameter Jazzy/Cyclone controls are invalid")

    correctness = sample.get("correctness")
    if not isinstance(correctness, dict) or correctness.get(
            "completed_operations") != operations or correctness.get(
                "expected_operations") != operations:
        raise ValueError("remote parameter fixed work is incomplete")
    if correctness.get("exceptions") != 0 or correctness.get(
            "responses_verified") != operations or correctness.get(
                "pending_futures") != 0 or correctness.get("verified") is not True:
        raise ValueError("remote parameter correctness evidence is invalid")
    if not isinstance(correctness.get("final_value"), int):
        raise ValueError("remote parameter final value is invalid")

    _validate_timing(sample.get("timing"), operations)
    _validate_topology(sample.get("topology"))
    boundary = sample.get("boundary_evidence")
    if not isinstance(boundary, dict) or boundary.get(
            "conversion_poison_installed") is not True or boundary.get(
                "serialization_poison_installed") is not True:
        raise ValueError("remote parameter boundary poison is missing")
    if any(boundary.get(field) != 0 for field in (
            "application_message_conversions", "serialization_calls", "cdr_calls")):
        raise ValueError("remote parameter used conversion, serialization, or CDR")
    if boundary.get("exact_cpp_data_path_required") is not (
            variant == "direct-rclcppyy"):
        raise ValueError("remote parameter data-path requirement is invalid")
    _validate_backend(sample.get("backend_evidence"), variant, workload)


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
                raise ValueError("remote parameter paired sample is incomplete")
            stock = selected["stock-rclpy"]["timing"]
            direct = selected["direct-rclcppyy"]["timing"]
            paired.append({
                "repetition": repetition,
                "workload": workload,
                "direct_over_stock_process_cpu": (
                    direct[PRIMARY_METRIC] / stock[PRIMARY_METRIC]),
                "direct_over_stock_latency_p50": (
                    direct["latency"]["p50_ns"] / stock["latency"]["p50_ns"]),
                "direct_over_stock_latency_p99": (
                    direct["latency"]["p99_ns"] / stock["latency"]["p99_ns"]),
                "direct_over_stock_throughput": (
                    direct["operations_per_second"] / stock["operations_per_second"]),
            })
    fields = (
        "direct_over_stock_process_cpu", "direct_over_stock_latency_p50",
        "direct_over_stock_latency_p99", "direct_over_stock_throughput",
    )
    medians = {
        workload: {
            field: statistics.median(
                row[field] for row in paired if row["workload"] == workload)
            for field in fields
        }
        for workload in WORKLOADS
    }
    return {
        "primary_metric": PRIMARY_METRIC,
        "paired_ratios": paired,
        "median_ratios": medians,
    }


def build_document(*, environment, command, warmup, operations, repetitions, samples):
    document = {
        "schema": SCHEMA_ID,
        "generated_at": datetime.datetime.now(
            datetime.timezone.utc).isoformat().replace("+00:00", "Z"),
        "command": list(command),
        "environment": environment,
        "benchmark": {
            "name": "remote-parameter-cpu-characterization",
            "performance_claims_allowed": False,
            "primary_metric": PRIMARY_METRIC,
            "warmup_operations": warmup,
            "measured_operations": operations,
            "repetitions": repetitions,
            "variants": list(VARIANTS),
            "workloads": list(WORKLOADS),
            "execution_order_policy": "deterministic-rotating-variant",
            "setup_init_jit_graph_excluded": True,
            "post_init_settle_ns": POST_INIT_SETTLE_NS,
            "cpu_scope": "combined-server-client-process",
        },
        "execution_order": execution_order(repetitions),
        "samples": samples,
        "summary": summarize(samples, repetitions),
    }
    validate_document(document)
    return document


def validate_document(document):
    if not isinstance(document, dict) or document.get("schema") != SCHEMA_ID:
        raise ValueError("unsupported remote-parameter benchmark schema")
    if not isinstance(document.get("generated_at"), str) or not isinstance(
            document.get("environment"), dict) or not isinstance(
                document.get("command"), list) or not document["command"]:
        raise ValueError("remote parameter document provenance is incomplete")
    benchmark = document.get("benchmark")
    expected = {
        "name": "remote-parameter-cpu-characterization",
        "performance_claims_allowed": False,
        "primary_metric": PRIMARY_METRIC,
        "variants": list(VARIANTS),
        "workloads": list(WORKLOADS),
        "execution_order_policy": "deterministic-rotating-variant",
        "setup_init_jit_graph_excluded": True,
        "post_init_settle_ns": POST_INIT_SETTLE_NS,
        "cpu_scope": "combined-server-client-process",
    }
    if not isinstance(benchmark, dict) or any(
            benchmark.get(key) != value for key, value in expected.items()):
        raise ValueError("remote parameter benchmark controls are invalid")
    warmup = benchmark.get("warmup_operations")
    operations = benchmark.get("measured_operations")
    repetitions = benchmark.get("repetitions")
    if any(not _positive_integer(value) for value in (
            warmup, operations, repetitions)):
        raise ValueError("remote parameter work controls are invalid")
    if warmup > 20_000 or operations > 20_000 or repetitions > 30:
        raise ValueError("remote parameter work controls exceed schema bounds")
    order = execution_order(repetitions)
    if document.get("execution_order") != order:
        raise ValueError("remote parameter order is not deterministic")
    samples = document.get("samples")
    if not isinstance(samples, list) or len(samples) != len(order):
        raise ValueError("remote parameter sample matrix is incomplete")
    source_digests = set()
    for index, (sample, expected_item) in enumerate(zip(samples, order)):
        if any(sample.get(field) != expected_item[field] for field in expected_item):
            raise ValueError("remote parameter sample order is inconsistent")
        validate_sample(
            sample, warmup=warmup, operations=operations,
            repetition=expected_item["repetition"], order_index=index)
        source_digests.add(sample["backend_evidence"]["source_sha256"])
    if len(source_digests) != 1:
        raise ValueError("remote parameter lanes used different client source")
    if document.get("summary") != summarize(samples, repetitions):
        raise ValueError("remote parameter summary is inconsistent")


def dumps(document):
    validate_document(document)
    return json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n"


__all__ = [
    "OPERATION_ROUTES", "POST_INIT_SETTLE_NS", "PRIMARY_METRIC", "QOS", "RMW",
    "ROS_DISTRO", "SAMPLE_SCHEMA", "SCHEMA_ID", "SERVICE_COUNT", "VARIANTS",
    "WORKLOADS", "build_document", "dumps", "execution_order", "latency_summary",
    "nearest_rank", "summarize", "validate_document", "validate_sample",
]
