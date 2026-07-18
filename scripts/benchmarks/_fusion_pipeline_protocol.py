"""Strict evidence contract for the controlled C++ topology-fusion benchmark."""

from __future__ import annotations

import datetime
import json
import math
from pathlib import Path
import re
import statistics
import sys

from _result_schema import environment_metadata


SCHEMA_ID = "rclcppyy.fusion-pipeline-benchmark/v1"
SAMPLE_SCHEMA = "rclcppyy.fusion-pipeline-sample/v1"
PREWARM_SCHEMA = "rclcppyy.fusion-pipeline-prewarm/v1"
RELAY_SCHEMA = "rclcppyy.fusion-pipeline-relay/v1"
DRIVER_SCHEMA = "rclcppyy.fusion-pipeline-driver/v1"
VARIANTS = {
    "aot-staged": {
        "model": "release-aot-four-stage-multi-entity",
        "entities": 8,
        "topics": 5,
        "cache": "aot-binary",
        "fused": False,
    },
    "cppyy-fused": {
        "model": "cppyy-loaded-single-callback-fused-cpp",
        "entities": 2,
        "topics": 2,
        "cache": "fused-shared-library",
        "fused": True,
    },
    "aot-fused": {
        "model": "release-aot-single-callback-fused-ceiling",
        "entities": 2,
        "topics": 2,
        "cache": "aot-binary",
        "fused": True,
    },
}
QOS = {
    "history": "keep_last",
    "depth": 1,
    "reliability": "reliable",
    "durability": "volatile",
}
RSS_LIMIT = 64 * 1024 * 1024
METRICS = (
    "relay_cpu_ns_per_message",
    "latency_p50_ns",
    "latency_p99_ns",
    "throughput_messages_per_second",
)
SHA256 = re.compile(r"^[a-f0-9]{64}$")
COMMIT = re.compile(r"^[a-f0-9]{40}$")


def _int(value) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _positive(value) -> bool:
    return _int(value) and value > 0


def _nonnegative(value) -> bool:
    return _int(value) and value >= 0


def _digest(value) -> bool:
    return isinstance(value, str) and SHA256.fullmatch(value) is not None


def stage(index: int, value: int) -> int:
    if index == 0:
        return (value + 1) & ((1 << 64) - 1)
    if index == 1:
        return (value * 3) & ((1 << 64) - 1)
    if index == 2:
        return (value + 5) & ((1 << 64) - 1)
    if index == 3:
        return value ^ 0x5A
    raise ValueError("stage index must be between zero and three")


def fused(value: int) -> int:
    for index in range(4):
        value = stage(index, value)
    return value


def expected_checksum(warmup: int, messages: int) -> int:
    return sum(fused(value) for value in range(warmup + 1, warmup + messages + 1))


def nearest_rank(values: list[int], percentile: int) -> int:
    if not values:
        raise ValueError("latency list cannot be empty")
    ordered = sorted(values)
    return ordered[max(1, math.ceil(len(ordered) * percentile / 100)) - 1]


def latency_summary(values: list[int]) -> dict:
    return {
        "p50": nearest_rank(values, 50),
        "p95": nearest_rank(values, 95),
        "p99": nearest_rank(values, 99),
        "max": max(values),
    }


def _validate_rss(guard: dict) -> None:
    if not isinstance(guard, dict) or guard.get(
            "kind") != "post-warmup-peak-rss-growth" or guard.get("unit") != "bytes":
        raise ValueError("RSS guard schema is invalid")
    baseline = guard.get("baseline_peak_bytes")
    final = guard.get("final_peak_bytes")
    if not _positive(baseline) or not _positive(final) or final < baseline:
        raise ValueError("RSS observations are invalid")
    if guard.get("growth_bytes") != final - baseline or guard.get(
            "limit_bytes") != RSS_LIMIT or guard.get("within_limit") is not True:
        raise ValueError("RSS guard calculation or bound is invalid")


def validate_prewarm(value: dict, *, cached: bool, requested_rmw: str) -> None:
    if not isinstance(value, dict) or value.get("schema") != PREWARM_SCHEMA:
        raise ValueError("prewarm schema is invalid")
    if not _positive(value.get("pid")) or value.get("loaded_rmw") != requested_rmw:
        raise ValueError("prewarm process or RMW evidence is invalid")
    artifact = value.get("artifact")
    if not isinstance(artifact, dict) or artifact.get("cached") is not cached:
        raise ValueError("prewarm cache state is invalid")
    if not isinstance(artifact.get("path"), str) or not _digest(
            artifact.get("sha256")) or not _positive(artifact.get("size_bytes")):
        raise ValueError("prewarm artifact identity is invalid")
    if not isinstance(artifact.get("source_id"), str) or not re.fullmatch(
            r"[a-f0-9]{16}", artifact["source_id"]):
        raise ValueError("prewarm source identity is invalid")
    diagnostics = value.get("stdout_diagnostics")
    if not isinstance(diagnostics, list) or any(not isinstance(line, str) for line in diagnostics):
        raise ValueError("prewarm diagnostics are invalid")


def validate_cache(cache: dict, requested_rmw: str) -> None:
    required = (
        "isolated_root", "autopch_disabled", "fresh_process_per_phase",
        "compilation_excluded_from_samples", "warm_hits_verified")
    if not isinstance(cache, dict) or any(cache.get(name) is not True for name in required):
        raise ValueError("cache isolation policy is incomplete")
    if cache.get("persisted_after_run") is not False:
        raise ValueError("private benchmark cache must be removed")
    phases = cache.get("phases")
    if not isinstance(phases, dict) or set(phases) != {"cold", "warm"}:
        raise ValueError("cache phase matrix is invalid")
    validate_prewarm(phases["cold"], cached=False, requested_rmw=requested_rmw)
    validate_prewarm(phases["warm"], cached=True, requested_rmw=requested_rmw)
    cold = phases["cold"]["artifact"]
    warm = phases["warm"]["artifact"]
    for field in ("path", "sha256", "size_bytes", "source_id"):
        if cold[field] != warm[field]:
            raise ValueError("cold and warm prewarm selected different artifacts")


def _validate_build(build: dict) -> None:
    if not isinstance(build, dict) or build.get("build_type") != "Release" or build.get(
            "private_build_directory") is not True or build.get(
            "build_directory_persisted") is not False:
        raise ValueError("AOT build isolation is invalid")
    command = build.get("compile_command")
    if not isinstance(command, str) or "-O3" not in command or "-DNDEBUG" not in command:
        raise ValueError("AOT compile command does not prove Release optimization")
    if build.get("executable_format") != "ELF" or not isinstance(
            build.get("compiler_version"), str):
        raise ValueError("AOT compiler or executable evidence is invalid")
    for field in (
            "source_sha256", "kernel_sha256", "cmake_sha256",
            "compile_commands_sha256", "executable_sha256"):
        if not _digest(build.get(field)):
            raise ValueError("AOT build digest is invalid")
    if not _positive(build.get("build_elapsed_ns")):
        raise ValueError("AOT build elapsed time is invalid")


def _validate_ready(ready: dict, sample: dict, build: dict, cache: dict) -> None:
    variant = sample["variant"]
    spec = VARIANTS[variant]
    expected = {
        "schema": RELAY_SCHEMA,
        "event": "ready",
        "variant": variant,
        "run_token": sample["run_token"],
        "pid": sample["relay_pid"],
        "process_group_id": sample["relay_pid"],
        "node_name": sample["topology"]["relay_node"],
        "loaded_rmw": sample["requested_rmw"],
        "execution_model": spec["model"],
        "representation": "std_msgs::msg::UInt64",
        "python_message_conversions": 0,
        "ros_entity_count": spec["entities"],
        "observable_topic_count": spec["topics"],
        "composition": {
            "relay_processes": 1,
            "relay_nodes": 1,
            "executor": "single_threaded",
            "executor_threads": 1,
            "use_intra_process_comms": True,
        },
    }
    if not isinstance(ready, dict) or any(ready.get(key) != value for key, value in expected.items()):
        raise ValueError("relay ready evidence is invalid")
    artifact = ready.get("cache")
    if not isinstance(artifact, dict) or artifact.get("state") != "prebuilt" or artifact.get(
            "kind") != spec["cache"]:
        raise ValueError("relay cache authority is invalid")
    if variant == "cppyy-fused":
        warm = cache["phases"]["warm"]["artifact"]
        for field in ("path", "sha256", "size_bytes", "source_id"):
            if artifact.get(field) != warm[field]:
                raise ValueError("cppyy relay did not use the prewarmed artifact")
        if artifact.get("cached") is not True:
            raise ValueError("cppyy relay cache hit evidence is invalid")
    elif artifact != {"state": "prebuilt", "kind": "aot-binary"}:
        raise ValueError("AOT relay cache evidence is invalid")
    _validate_build(build)


def _validate_graph(warmed: dict, sample: dict) -> None:
    variant = sample["variant"]
    topics = sample["topology"]["all_topics"]
    selected = topics if variant == "aot-staged" else [topics[0], topics[-1]]
    graph = warmed.get("graph")
    if not isinstance(graph, list) or [row.get("topic") for row in graph] != selected:
        raise ValueError("driver observed the wrong topic graph")
    driver = sample["topology"]["driver_node"]
    relay = sample["topology"]["relay_node"]
    for index, row in enumerate(graph):
        if row.get("publisher_count") != 1 or row.get("subscription_count") != 1:
            raise ValueError("driver endpoint cardinality is invalid")
        publishers = row.get("publishers")
        subscriptions = row.get("subscriptions")
        if not isinstance(publishers, list) or len(publishers) != 1 or not isinstance(
                subscriptions, list) or len(subscriptions) != 1:
            raise ValueError("driver endpoint arrays are invalid")
        expected_publisher = driver if index == 0 else relay
        expected_subscription = driver if index == len(graph) - 1 else relay
        for endpoint, expected in (
                (publishers[0], expected_publisher),
                (subscriptions[0], expected_subscription)):
            observed = endpoint.get("node_name") if isinstance(endpoint, dict) else None
            if observed != expected and not (
                    expected == relay
                    and sample["requested_rmw"] == "rmw_cyclonedds_cpp"
                    and observed == "_NODE_NAME_UNKNOWN_"):
                raise ValueError("endpoint owner authority is invalid")
            namespace = endpoint.get("node_namespace")
            if namespace not in ("/", "_NODE_NAMESPACE_UNKNOWN_"):
                raise ValueError("endpoint namespace authority is invalid")


def validate_sample(sample: dict, parameters: dict, build: dict, cache: dict) -> None:
    if not isinstance(sample, dict) or sample.get("schema") != SAMPLE_SCHEMA:
        raise ValueError("sample schema is invalid")
    variant = sample.get("variant")
    if variant not in VARIANTS or not _positive(sample.get("repetition")):
        raise ValueError("sample variant or repetition is invalid")
    if not isinstance(sample.get("run_token"), str) or not re.fullmatch(
            r"run_[a-f0-9]{32}", sample["run_token"]):
        raise ValueError("sample run token is invalid")
    if sample.get("requested_rmw") != parameters["requested_rmw"] or sample.get("qos") != QOS:
        raise ValueError("sample RMW or QoS evidence is invalid")
    if not _positive(sample.get("relay_pid")) or not _positive(
            sample.get("driver_pid")) or sample["relay_pid"] == sample["driver_pid"]:
        raise ValueError("sample requires two distinct process groups")
    topology = sample.get("topology")
    if not isinstance(topology, dict) or topology.get("process_count") != 2 or topology.get(
            "fresh_process_groups") is not True:
        raise ValueError("sample process topology is invalid")
    topics = topology.get("all_topics")
    if not isinstance(topics, list) or len(topics) != 5 or len(set(topics)) != 5:
        raise ValueError("sample topic topology is invalid")
    contract = topology.get("contract_change")
    if not isinstance(contract, dict) or contract.get(
            "external_input_output_unchanged") is not True or contract.get(
            "intermediate_topics_observable") is VARIANTS[variant]["fused"]:
        raise ValueError("topology contract disclosure is invalid")
    _validate_ready(sample.get("relay_ready"), sample, build, cache)
    armed = sample.get("relay_armed")
    if not isinstance(armed, dict) or armed != {
        "schema": RELAY_SCHEMA,
        "event": "armed",
        "variant": variant,
        "run_token": sample["run_token"],
        "pid": sample["relay_pid"],
        "process_group_id": sample["relay_pid"],
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
    }:
        raise ValueError("relay armed barrier evidence is invalid")
    warmed = sample.get("driver_warmed")
    if not isinstance(warmed, dict) or warmed.get("schema") != DRIVER_SCHEMA or warmed.get(
            "event") != "warmed" or warmed.get("variant") != variant or warmed.get(
            "run_token") != sample["run_token"] or warmed.get("pid") != sample["driver_pid"]:
        raise ValueError("driver warmup barrier evidence is invalid")
    if warmed.get("process_group_id") != sample["driver_pid"] or warmed.get(
            "warmup_messages") != parameters["warmup_messages"] or warmed.get(
            "loaded_rmw") != parameters["requested_rmw"]:
        raise ValueError("driver warmup identity is invalid")
    if any(warmed.get(field) is not True for field in (
            "topology_verified", "qos_verified", "authority_verified")) or warmed.get(
            "observable_topic_count") != VARIANTS[variant]["topics"]:
        raise ValueError("driver graph verification evidence is invalid")
    _validate_graph(warmed, sample)
    total = parameters["warmup_messages"] + parameters["messages"]
    report = sample.get("relay_report")
    if not isinstance(report, dict) or report.get("schema") != RELAY_SCHEMA or report.get(
            "event") != "report" or report.get("variant") != variant or report.get(
            "run_token") != sample["run_token"]:
        raise ValueError("relay report identity is invalid")
    if report.get("received") != total or report.get("published") != total or report.get(
            "logical_stage_events") != total * 4:
        raise ValueError("relay work counters are invalid")
    if any(report.get(field) != 0 for field in (
            "python_callback_count", "python_boundary_crossings",
            "python_message_conversions", "dropped", "exceptions")):
        raise ValueError("relay crossed Python or reported runtime errors")
    if report.get("correct") is not True or report.get("teardown_clean") is not True or not _nonnegative(
            report.get("cpu_time_ns")) or report.get("cpu_clock") != "CLOCK_PROCESS_CPUTIME_ID":
        raise ValueError("relay correctness, teardown, or CPU evidence is invalid")
    _validate_rss(report.get("rss_guard"))
    if variant == "cppyy-fused" and (
            report.get("compile_cache_hits") != 1 or report.get("compile_cache_misses") != 0):
        raise ValueError("cppyy compile-cache counters are invalid")
    measured = sample.get("driver_measured")
    messages = parameters["messages"]
    if not isinstance(measured, dict) or measured.get("schema") != DRIVER_SCHEMA or measured.get(
            "event") != "measured" or measured.get("variant") != variant or measured.get(
            "run_token") != sample["run_token"] or measured.get("pid") != sample["driver_pid"]:
        raise ValueError("driver measured identity is invalid")
    if measured.get("messages") != messages or measured.get(
            "checksum") != expected_checksum(parameters["warmup_messages"], messages) or measured.get(
            "last") != fused(total):
        raise ValueError("driver output contract is invalid")
    latencies = measured.get("latency_ns")
    if not isinstance(latencies, list) or len(latencies) != messages or any(
            not _positive(value) for value in latencies):
        raise ValueError("driver latency evidence is invalid")
    if not _positive(measured.get("elapsed_ns")) or not _nonnegative(
            measured.get("cpu_time_ns")) or measured.get(
            "cpu_clock") != "CLOCK_PROCESS_CPUTIME_ID":
        raise ValueError("driver timing evidence is invalid")
    _validate_rss(measured.get("rss_guard"))
    teardown = sample.get("driver_teardown")
    if not isinstance(teardown, dict) or teardown.get("schema") != DRIVER_SCHEMA or teardown.get(
            "event") != "teardown" or teardown.get("variant") != variant or teardown.get(
            "run_token") != sample["run_token"] or teardown.get(
            "endpoint_disappeared") is not True or teardown.get("teardown_clean") is not True:
        raise ValueError("driver teardown evidence is invalid")
    timing = sample.get("timing")
    relay_cpu = report["cpu_time_ns"]
    if not isinstance(timing, dict) or timing.get("relay_cpu_time_ns") != relay_cpu or timing.get(
            "relay_cpu_ns_per_message") != relay_cpu / messages:
        raise ValueError("relay CPU summary is invalid")
    if timing.get("latency_ns") != latency_summary(latencies) or timing.get(
            "elapsed_ns") != measured["elapsed_ns"]:
        raise ValueError("driver latency or elapsed summary is invalid")
    expected_rate = messages * 1e9 / measured["elapsed_ns"]
    if not math.isclose(timing.get("throughput_messages_per_second", -1), expected_rate):
        raise ValueError("throughput summary is invalid")
    if any(sample.get(field) is not True for field in (
            "backend_verified", "correctness_verified", "teardown_verified")):
        raise ValueError("sample acceptance flags are incomplete")


def _metric(sample: dict, name: str):
    timing = sample["timing"]
    return {
        "relay_cpu_ns_per_message": timing["relay_cpu_ns_per_message"],
        "latency_p50_ns": timing["latency_ns"]["p50"],
        "latency_p99_ns": timing["latency_ns"]["p99"],
        "throughput_messages_per_second": timing["throughput_messages_per_second"],
    }[name]


def summarize(results: list[dict], variants: list[str]) -> dict:
    grouped = {
        variant: {row["repetition"]: row for row in results if row["variant"] == variant}
        for variant in variants
    }
    medians = {
        variant: {
            metric: statistics.median([_metric(row, metric) for row in rows.values()])
            for metric in METRICS
        } if rows else {}
        for variant, rows in grouped.items()
    }

    def paired(reference: str) -> dict:
        output = {}
        for variant, rows in grouped.items():
            repetitions = sorted(set(rows) & set(grouped.get(reference, {})))
            output[variant] = {
                metric: [
                    _metric(rows[rep], metric) / _metric(grouped[reference][rep], metric)
                    for rep in repetitions
                ]
                for metric in METRICS
            }
        return output

    return {
        "raw_medians": medians,
        "paired_ratios_to_aot_staged": paired("aot-staged"),
        "paired_ratios_to_aot_fused": paired("aot-fused"),
        "interpretation_allowed": False,
        "note": (
            "Ratios are descriptive only. Fused lanes intentionally remove three observable "
            "intermediate ROS topics while preserving external input, final output, and the four "
            "logical transforms. Lower is better for CPU and latency; higher is better for rate."
        ),
    }


def build_document(
        *, repo_root: Path, mode: str, parameters: dict, isolation: dict,
        aot_build: dict, cache: dict, source_files: dict, results: list,
        failures: list, command: list[str] | None = None) -> dict:
    document = {
        "schema": SCHEMA_ID,
        "generated_at": datetime.datetime.now(
            datetime.timezone.utc).isoformat().replace("+00:00", "Z"),
        "command": list(command if command is not None else sys.argv),
        "environment": environment_metadata(repo_root),
        "benchmark": {
            "name": "controlled_cpp_topology_fusion",
            "mode": mode,
            "performance_claims_allowed": False,
            "parameters": parameters,
            "isolation": isolation,
            "aot_build": aot_build,
            "cache": cache,
            "source_files": source_files,
            "statistics": {
                "primary": "relay-owned CLOCK_PROCESS_CPUTIME_ID per external message",
                "secondary": "closed-loop end-to-end latency and throughput",
                "memory": "post-warmup peak-RSS growth is a runaway guard only",
                "comparison": "paired by repetition with rotating variant order",
            },
            "contract_change": {
                "opt_in": True,
                "external_input_output_unchanged": True,
                "logical_transform_sequence_unchanged": True,
                "removed_intermediate_ros_topics": 3,
                "removed_intermediate_entity_pairs": 3,
                "intermediate_graph_observability_preserved": False,
            },
        },
        "results": list(results),
        "comparison": summarize(results, parameters["variants"]),
        "failures": list(failures),
    }
    validate_document(document)
    return document


def validate_document(document: dict) -> None:
    if not isinstance(document, dict) or document.get("schema") != SCHEMA_ID:
        raise ValueError("document schema is invalid")
    environment = document.get("environment")
    source = environment.get("source") if isinstance(environment, dict) else None
    suite = environment.get("source_dependencies", {}).get(
        "rclcpp_kit") if isinstance(environment, dict) else None
    if not isinstance(source, dict) or COMMIT.fullmatch(source.get("commit", "")) is None or not isinstance(
            source.get("dirty"), bool):
        raise ValueError("product source identity is invalid")
    if not isinstance(suite, dict) or COMMIT.fullmatch(suite.get("commit", "")) is None or not isinstance(
            suite.get("dirty"), bool):
        raise ValueError("suite source identity is invalid")
    benchmark = document.get("benchmark")
    if not isinstance(benchmark, dict) or benchmark.get(
            "name") != "controlled_cpp_topology_fusion" or benchmark.get(
            "mode") not in ("smoke", "measurement") or benchmark.get(
            "performance_claims_allowed") is not False:
        raise ValueError("benchmark metadata is invalid")
    parameters = benchmark.get("parameters")
    if not isinstance(parameters, dict) or not isinstance(parameters.get("variants"), list) or not parameters[
            "variants"] or any(name not in VARIANTS for name in parameters["variants"]):
        raise ValueError("benchmark variants are invalid")
    if len(set(parameters["variants"])) != len(parameters["variants"]) or any(
            not _positive(parameters.get(name)) for name in (
                "messages", "warmup_messages", "repetitions")):
        raise ValueError("benchmark dimensions are invalid")
    requested_rmw = parameters.get("requested_rmw")
    if requested_rmw != "rmw_cyclonedds_cpp" or parameters.get(
            "ros_distro") != "jazzy" or parameters.get("qos") != QOS:
        raise ValueError("benchmark RMW or QoS is invalid")
    _validate_build(benchmark.get("aot_build"))
    validate_cache(benchmark.get("cache"), requested_rmw)
    isolation = benchmark.get("isolation")
    if not isinstance(isolation, dict) or any(isolation.get(name) is not True for name in (
            "fresh_process_pair_per_sample", "two_process_groups_per_sample",
            "unique_topics_per_sample", "one_leased_domain_per_run",
            "rotating_variant_order", "exact_warmup_measurement_barriers")):
        raise ValueError("benchmark isolation evidence is incomplete")
    source_files = benchmark.get("source_files")
    if not isinstance(source_files, dict) or not source_files or any(
            not _digest(value) for value in source_files.values()):
        raise ValueError("benchmark source identities are invalid")
    results = document.get("results")
    failures = document.get("failures")
    if not isinstance(results, list) or not isinstance(failures, list):
        raise ValueError("results and failures must be lists")
    identities = set()
    for sample in results:
        validate_sample(sample, parameters, benchmark["aot_build"], benchmark["cache"])
        identity = (sample["variant"], sample["repetition"])
        if identity in identities:
            raise ValueError("duplicate sample identity")
        identities.add(identity)
    comparison = document.get("comparison")
    if comparison != summarize(results, parameters["variants"]):
        raise ValueError("comparison does not derive from raw samples")


def dumps(document: dict) -> str:
    validate_document(document)
    return json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n"


def write(document: dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(dumps(document), encoding="utf-8")
