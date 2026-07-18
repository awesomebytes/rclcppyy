"""Evidence contract for C++ message-facade characterization."""

from __future__ import annotations

import datetime
import json
import math
import statistics


SCHEMA_ID = "rclcppyy.message-facade-benchmark/v1"
SAMPLE_SCHEMA = "rclcppyy.message-facade-sample/v1"
VARIANTS = ("stock-rclpy", "message-facade-rclcppyy")
MESSAGE_TYPES = ("UInt64", "String")
RSS_GUARD_LIMIT_BYTES = 64 * 1024 * 1024
METRICS = (
    "cpu_ns_per_message",
    "latency_p50_ns",
    "latency_p99_ns",
    "throughput_messages_per_second",
)


def _positive_integer(value):
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def _nonnegative_integer(value):
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _finite_number(value):
    return (
        isinstance(value, (int, float)) and
        not isinstance(value, bool) and
        math.isfinite(value)
    )


def nearest_rank(values, percentile):
    if not values:
        raise ValueError("latency observations cannot be empty")
    ordered = sorted(values)
    rank = max(1, math.ceil(percentile / 100.0 * len(ordered)))
    return ordered[rank - 1]


def latency_summary(values):
    return {
        "p50_ns": nearest_rank(values, 50),
        "p99_ns": nearest_rank(values, 99),
        "max_ns": max(values),
    }


def expected_checksum(message_type, first_sequence, count):
    if message_type not in MESSAGE_TYPES:
        raise ValueError("unknown message type")
    return sum(range(first_sequence, first_sequence + count))


def validate_sample(sample, *, warmup, messages, repetition, order_index):
    if not isinstance(sample, dict) or sample.get("schema") != SAMPLE_SCHEMA:
        raise ValueError("unsupported message-facade sample schema")
    variant = sample.get("variant")
    message_type = sample.get("message_type")
    if variant not in VARIANTS or message_type not in MESSAGE_TYPES:
        raise ValueError("sample variant or message type is invalid")
    if sample.get("warmup_messages") != warmup or sample.get(
            "measured_messages") != messages:
        raise ValueError("sample work does not match the fixed benchmark request")
    if sample.get("repetition") != repetition or sample.get(
            "order_index") != order_index:
        raise ValueError("sample order evidence is invalid")
    runtime = sample.get("runtime")
    required_true = (
        "fresh_process", "setup_excluded", "discovery_complete",
        "single_threaded_executor", "fixed_work", "teardown_clean",
    )
    if not isinstance(runtime, dict) or any(
            runtime.get(field) is not True for field in required_true):
        raise ValueError("sample runtime controls are incomplete")
    if runtime.get("rmw_implementation") != "rmw_cyclonedds_cpp" or runtime.get(
            "ros_distribution") != "jazzy":
        raise ValueError("sample did not use the reviewed Jazzy/Cyclone stack")

    correctness = sample.get("correctness")
    if not isinstance(correctness, dict) or correctness.get("received") != messages:
        raise ValueError("sample did not receive the complete measured work")
    if correctness.get("dropped") != 0 or correctness.get("exceptions") != 0:
        raise ValueError("sample reported drops or callback exceptions")
    if correctness.get("checksum") != expected_checksum(
            message_type, warmup + 1, messages):
        raise ValueError("sample checksum is invalid")
    if correctness.get("last_sequence") != warmup + messages:
        raise ValueError("sample last sequence is invalid")
    if correctness.get("value_contract_verified") is not True:
        raise ValueError("sample value contract is unverified")

    timing = sample.get("timing")
    if not isinstance(timing, dict) or timing.get(
            "cpu_clock") != "CLOCK_PROCESS_CPUTIME_ID":
        raise ValueError("sample CPU clock evidence is invalid")
    for field in ("cpu_time_ns", "elapsed_ns"):
        if not _positive_integer(timing.get(field)):
            raise ValueError("sample %s is invalid" % field)
    for field in METRICS:
        if not _finite_number(timing.get(field)) or timing[field] <= 0:
            raise ValueError("sample metric %s is invalid" % field)
    if not math.isclose(
            timing["cpu_ns_per_message"],
            timing["cpu_time_ns"] / messages,
            rel_tol=1e-12):
        raise ValueError("sample CPU normalization is inconsistent")
    if not math.isclose(
            timing["throughput_messages_per_second"],
            messages * 1e9 / timing["elapsed_ns"],
            rel_tol=1e-12):
        raise ValueError("sample throughput normalization is inconsistent")
    latency = timing.get("latency")
    if not isinstance(latency, dict) or latency.get("count") != messages:
        raise ValueError("sample latency count is invalid")
    if latency.get("p50_ns") != timing["latency_p50_ns"] or latency.get(
            "p99_ns") != timing["latency_p99_ns"]:
        raise ValueError("sample latency summary is inconsistent")
    if any(not _positive_integer(latency.get(field)) for field in (
            "p50_ns", "p99_ns", "max_ns")):
        raise ValueError("sample latency percentile is invalid")
    if not latency["p50_ns"] <= latency["p99_ns"] <= latency["max_ns"]:
        raise ValueError("sample latency percentiles are not monotonic")

    rss = sample.get("rss_guard")
    if not isinstance(rss, dict) or rss.get("limit_bytes") != RSS_GUARD_LIMIT_BYTES:
        raise ValueError("sample RSS guard is invalid")
    if any(not _nonnegative_integer(rss.get(field)) for field in (
            "baseline_bytes", "peak_bytes", "growth_bytes")):
        raise ValueError("sample RSS evidence is invalid")
    if rss["growth_bytes"] > rss["limit_bytes"] or rss.get("passed") is not True:
        raise ValueError("sample exceeded the RSS guard")
    if rss["peak_bytes"] < rss["baseline_bytes"] or rss["growth_bytes"] != (
            rss["peak_bytes"] - rss["baseline_bytes"]):
        raise ValueError("sample RSS growth is inconsistent")

    backend = sample.get("backend_evidence")
    if not isinstance(backend, dict) or backend.get("verified") is not True:
        raise ValueError("sample backend evidence is missing")
    if backend.get("entity_types") != {
            "node": "rclpy.node.Node",
            "publisher": "rclpy.publisher.Publisher",
            "subscription": "rclpy.subscription.Subscription",
            "executor": "rclpy.executors.SingleThreadedExecutor",
    }:
        raise ValueError("sample changed the stock entity identities")
    expected_callback_type = {
        "UInt64": "std_msgs.msg._u_int64.UInt64",
        "String": "std_msgs.msg._string.String",
    }[message_type]
    if backend.get("callback_message_type") != expected_callback_type or backend.get(
            "graph_node_identity_count") != 1:
        raise ValueError("sample public message or graph identity is invalid")
    if variant == "stock-rclpy":
        if backend.get("publisher") != "python" or backend.get(
                "subscription_take") != "python":
            raise ValueError("stock sample backend is invalid")
        if backend.get("representation") != "generated_python_message":
            raise ValueError("stock representation evidence is invalid")
        if backend.get("converter_forbidden") is not False:
            raise ValueError("stock sample claimed a converter guard")
    else:
        expected_representation = "std_msgs::msg::" + message_type
        if backend.get("publisher") != "cpp" or backend.get(
                "subscription_take") != "cpp":
            raise ValueError("facade sample did not use both direct C++ routes")
        if backend.get("representation") != expected_representation:
            raise ValueError("facade C++ representation evidence is invalid")
        if backend.get("converter_forbidden") is not True or backend.get(
                "python_to_cpp_whole_message_conversions") != 0:
            raise ValueError("facade sample lacks converter-forbidden evidence")
        if backend.get("publish_route_messages") != warmup + messages or backend.get(
                "take_route_messages") != warmup + messages:
            raise ValueError("facade direct route counters are invalid")
        if backend.get("total_route_messages") != warmup + messages:
            raise ValueError("facade route work evidence is invalid")
        if backend.get("fallback_operations") != 0:
            raise ValueError("facade sample used a fallback operation")
        if backend.get("hidden_original_entity_mapping") is not True:
            raise ValueError("facade sample lacks hidden-original entity mapping")
        if not _positive_integer(backend.get("facade_storage_address")):
            raise ValueError("facade sample lacks existing C++ storage evidence")


def execution_order(repetitions):
    order = []
    for repetition in range(repetitions):
        types = MESSAGE_TYPES if repetition % 2 == 0 else tuple(reversed(MESSAGE_TYPES))
        for type_index, message_type in enumerate(types):
            variants = (
                VARIANTS if (repetition + type_index) % 2 == 0
                else tuple(reversed(VARIANTS))
            )
            for variant in variants:
                order.append({
                    "repetition": repetition,
                    "message_type": message_type,
                    "variant": variant,
                })
    return order


def summarize(samples, repetitions):
    pairs = []
    for message_type in MESSAGE_TYPES:
        for repetition in range(repetitions):
            selected = {
                sample["variant"]: sample
                for sample in samples
                if sample["message_type"] == message_type and
                sample["repetition"] == repetition
            }
            if set(selected) != set(VARIANTS):
                raise ValueError("each repetition requires one rotating stock/facade pair")
            stock = selected["stock-rclpy"]["timing"]
            facade = selected["message-facade-rclcppyy"]["timing"]
            ratios = {
                metric: facade[metric] / stock[metric]
                for metric in METRICS
            }
            pairs.append({
                "message_type": message_type,
                "repetition": repetition,
                "facade_over_stock": ratios,
            })
    medians = {}
    for message_type in MESSAGE_TYPES:
        typed = [row for row in pairs if row["message_type"] == message_type]
        medians[message_type] = {
            metric: statistics.median(
                row["facade_over_stock"][metric] for row in typed)
            for metric in METRICS
        }
    return {"paired_ratios": pairs, "median_facade_over_stock": medians}


def build_document(*, environment, command, warmup, messages, repetitions, samples):
    order = execution_order(repetitions)
    document = {
        "schema": SCHEMA_ID,
        "generated_at": datetime.datetime.now(
            datetime.timezone.utc).isoformat().replace("+00:00", "Z"),
        "command": list(command),
        "environment": environment,
        "benchmark": {
            "name": "message-facade-characterization",
            "performance_claims_allowed": False,
            "promotion_blockers": [
                "native ARM64 correctness",
                "repeatable controlled-host CPU wins",
            ],
            "executor": "SingleThreadedExecutor",
            "warmup_messages": warmup,
            "measured_messages": messages,
            "repetitions": repetitions,
            "message_types": list(MESSAGE_TYPES),
            "variants": list(VARIANTS),
            "setup_excluded": True,
            "rss_is_guard_only": True,
        },
        "execution_order": order,
        "samples": samples,
        "summary": summarize(samples, repetitions),
    }
    validate_document(document)
    return document


def validate_document(document):
    if not isinstance(document, dict) or document.get("schema") != SCHEMA_ID:
        raise ValueError("unsupported message-facade benchmark schema")
    if not isinstance(document.get("generated_at"), str) or not isinstance(
            document.get("environment"), dict):
        raise ValueError("benchmark time/environment evidence is missing")
    benchmark = document.get("benchmark")
    if not isinstance(benchmark, dict) or benchmark.get(
            "performance_claims_allowed") is not False:
        raise ValueError("raw facade benchmark claims must remain disabled")
    if benchmark.get("setup_excluded") is not True or benchmark.get(
            "rss_is_guard_only") is not True:
        raise ValueError("benchmark setup/RSS policy is invalid")
    if benchmark.get("promotion_blockers") != [
            "native ARM64 correctness", "repeatable controlled-host CPU wins"]:
        raise ValueError("benchmark promotion blockers are incomplete")
    warmup = benchmark.get("warmup_messages")
    messages = benchmark.get("measured_messages")
    repetitions = benchmark.get("repetitions")
    if any(not _positive_integer(value) for value in (
            warmup, messages, repetitions)):
        raise ValueError("benchmark work controls are invalid")
    expected_order = execution_order(repetitions)
    if document.get("execution_order") != expected_order:
        raise ValueError("benchmark execution order is not rotating and paired")
    samples = document.get("samples")
    if not isinstance(samples, list) or len(samples) != len(expected_order):
        raise ValueError("benchmark sample matrix is incomplete")
    for index, (sample, expected) in enumerate(zip(samples, expected_order)):
        if any(sample.get(field) != expected[field] for field in expected):
            raise ValueError("benchmark sample order differs from its declaration")
        validate_sample(
            sample,
            warmup=warmup,
            messages=messages,
            repetition=expected["repetition"],
            order_index=index,
        )
    if document.get("summary") != summarize(samples, repetitions):
        raise ValueError("benchmark paired summary is inconsistent")


def dumps(document):
    validate_document(document)
    return json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n"


__all__ = [
    "MESSAGE_TYPES",
    "METRICS",
    "RSS_GUARD_LIMIT_BYTES",
    "SAMPLE_SCHEMA",
    "SCHEMA_ID",
    "VARIANTS",
    "build_document",
    "dumps",
    "execution_order",
    "expected_checksum",
    "latency_summary",
    "summarize",
    "validate_document",
    "validate_sample",
]
