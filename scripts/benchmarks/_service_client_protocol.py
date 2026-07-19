"""Strict evidence contract for the controlled service client benchmark."""

from __future__ import annotations

import datetime
import json
import math
from pathlib import Path
import re
import statistics
import sys

from _result_schema import environment_metadata


SCHEMA_ID = "rclcppyy.service-client-benchmark/v1"
SAMPLE_SCHEMA = "rclcppyy.service-client-sample/v1"
PREWARM_SCHEMA = "rclcppyy.service-client-prewarm/v1"
SERVER_SCHEMA = "rclcppyy.service-client-server-event/v1"
CLIENT_SCHEMA = "rclcppyy.service-client-client-event/v1"
DEFAULT_SERVICE_TYPE = "std_srvs/srv/SetBool"
SERVICE_TYPES = (DEFAULT_SERVICE_TYPE, "std_srvs/srv/Trigger")
SERVICE_SPECS = {
    DEFAULT_SERVICE_TYPE: {
        "benchmark_name": "jazzy_cyclone_setbool_client_cpu",
        "cpp_type": "std_srvs::srv::SetBool",
        "server_model": "common-release-aot-setbool-server",
        "workload": "one-outstanding alternating SetBool requests",
    },
    "std_srvs/srv/Trigger": {
        "benchmark_name": "jazzy_cyclone_trigger_client_cpu",
        "cpp_type": "std_srvs::srv::Trigger",
        "server_model": "common-release-aot-trigger-server",
        "workload": "one-outstanding empty Trigger requests",
    },
}
VARIANTS = {
    "stock-rclpy": {
        "model": "same-python-client-stock-rclpy",
        "cache": "stock-rclpy",
        "authority": "python",
        "orchestration": 1,
        "request_crossing": 1,
        "response_crossing": 1,
        "message_conversions": 2,
    },
    "compatible-rclcppyy": {
        "model": "same-python-client-compatible-activation",
        "cache": "compatible-python-client",
        "authority": "python",
        "orchestration": 1,
        "request_crossing": 1,
        "response_crossing": 1,
        "message_conversions": 2,
    },
    "direct-cpp-rclcppyy": {
        "model": "direct-cpp-rclpy-call-shape-client",
        "cache": "native-client",
        "authority": "cpp",
        "orchestration": 1,
        "request_crossing": 1,
        "response_crossing": 1,
        "message_conversions": 0,
        "cpp_request_copies": 1,
    },
    "native-python-orchestrated": {
        "model": "native-session-python-orchestrated-client",
        "cache": "native-client",
        "authority": "cpp",
        "orchestration": 1,
        "request_crossing": 1,
        "response_crossing": 1,
        "message_conversions": 0,
    },
    "native-cpp-state-machine": {
        "model": "native-content-addressed-cpp-client-state-machine",
        "cache": "cpp-client-state-machine",
        "authority": "cpp",
        "orchestration": 0,
        "request_crossing": 0,
        "response_crossing": 0,
        "message_conversions": 0,
    },
    "aot-staged": {
        "model": "conventional-release-aot-client",
        "cache": "aot-binary",
        "authority": "cpp",
        "orchestration": 0,
        "request_crossing": 0,
        "response_crossing": 0,
        "message_conversions": 0,
    },
}
QOS = {
    "history": "keep_last",
    "depth": 10,
    "reliability": "reliable",
    "durability": "volatile",
}
METRICS = (
    "client_cpu_ns_per_response",
    "latency_p50_ns",
    "latency_p95_ns",
    "latency_p99_ns",
    "latency_max_ns",
    "requests_per_second",
)
RSS_LIMIT_BYTES = 64 * 1024 * 1024
SHA = re.compile(r"^[a-f0-9]{64}$")
COMMIT = re.compile(r"^[a-f0-9]{40}$")


def _int(value) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _positive(value) -> bool:
    return _int(value) and value > 0


def _nonnegative(value) -> bool:
    return _int(value) and value >= 0


def _sha(value) -> bool:
    return isinstance(value, str) and SHA.fullmatch(value) is not None


def nearest_rank(values: list[int], percentile: int) -> int:
    if not values:
        raise ValueError("latency observations cannot be empty")
    ordered = sorted(values)
    return ordered[max(1, math.ceil(percentile / 100 * len(ordered))) - 1]


def latency_summary(values: list[int]) -> dict[str, int]:
    return {
        "p50": nearest_rank(values, 50),
        "p95": nearest_rank(values, 95),
        "p99": nearest_rank(values, 99),
        "max": max(values),
    }


def true_requests(
        start: int, count: int, service_type: str = DEFAULT_SERVICE_TYPE) -> int:
    if service_type == "std_srvs/srv/Trigger":
        return count
    return sum(sequence % 2 == 1 for sequence in range(start, start + count))


def response_checksum(
        start: int, count: int, service_type: str = DEFAULT_SERVICE_TYPE) -> int:
    if service_type == "std_srvs/srv/Trigger":
        return 109 * count
    total = 0
    for sequence in range(start, start + count):
        total += 117 if sequence % 2 == 1 else 8
    return total


def _artifact(value: dict, cached: bool | None = None) -> None:
    if not isinstance(value, dict):
        raise ValueError("client cache artifact is required")
    if cached is not None and value.get("cached") is not cached:
        raise ValueError("client cache hit state is invalid")
    if not isinstance(value.get("path"), str) or not value["path"]:
        raise ValueError("client cache artifact path is required")
    if not _sha(value.get("sha256")) or not _positive(value.get("size_bytes")):
        raise ValueError("client cache artifact identity is invalid")


def validate_prewarm(
        value: dict, expect_hits: bool,
        service_type: str = DEFAULT_SERVICE_TYPE) -> None:
    if not isinstance(value, dict) or value.get("schema") != PREWARM_SCHEMA:
        raise ValueError("unsupported client prewarm schema")
    if not _positive(value.get("pid")) or not isinstance(value.get("loaded_rmw"), str):
        raise ValueError("client prewarm process/RMW evidence is invalid")
    if value.get("service_type", DEFAULT_SERVICE_TYPE) != service_type:
        raise ValueError("client prewarm service type is invalid")
    source_id = value.get("native_client_source_id")
    if not isinstance(source_id, str) or not re.fullmatch(r"[a-f0-9]{16}", source_id):
        raise ValueError("native client source id is invalid")
    artifacts = value.get("artifacts")
    if not isinstance(artifacts, dict) or set(artifacts) != {
            "native_client", "cpp_state_machine"}:
        raise ValueError("client prewarm artifact matrix is incomplete")
    for artifact in artifacts.values():
        _artifact(artifact, expect_hits)
    diagnostics = value.get("stdout_diagnostics", [])
    if not isinstance(diagnostics, list) or any(
            not isinstance(line, str) for line in diagnostics):
        raise ValueError("client prewarm diagnostics are invalid")


def validate_cache(
        value: dict, rmw: str,
        service_type: str = DEFAULT_SERVICE_TYPE) -> None:
    required = (
        "isolated_root", "autopch_disabled", "fresh_process_per_phase",
        "compilation_excluded_from_samples", "warm_hits_verified")
    if not isinstance(value, dict) or any(value.get(key) is not True for key in required):
        raise ValueError("client cache policy is incomplete")
    if value.get("persisted_after_run") is not False:
        raise ValueError("client cache must be temporary")
    phases = value.get("phases")
    if not isinstance(phases, dict) or set(phases) != {"cold", "warm"}:
        raise ValueError("client cache phases are incomplete")
    validate_prewarm(phases["cold"], False, service_type)
    validate_prewarm(phases["warm"], True, service_type)
    if phases["cold"]["loaded_rmw"] != rmw or phases["warm"]["loaded_rmw"] != rmw:
        raise ValueError("client prewarm used the wrong RMW")
    for name in ("native_client", "cpp_state_machine"):
        cold = phases["cold"]["artifacts"][name]
        warm = phases["warm"]["artifacts"][name]
        if (cold["path"], cold["sha256"], cold["size_bytes"]) != (
                warm["path"], warm["sha256"], warm["size_bytes"]):
            raise ValueError("client cold/warm artifact identity changed")
    if phases["cold"]["native_client_source_id"] != phases["warm"][
            "native_client_source_id"]:
        raise ValueError("native client source identity changed")


def _rss_guard(value: dict, owner: str) -> None:
    if not isinstance(value, dict) or value.get(
            "kind") != "post-warmup-peak-rss-growth" or value.get("unit") != "bytes":
        raise ValueError("%s RSS guard is invalid" % owner)
    baseline = value.get("baseline_peak_bytes")
    final = value.get("final_peak_bytes")
    growth = value.get("growth_bytes")
    if not _positive(baseline) or not _positive(final) or final < baseline:
        raise ValueError("%s RSS observations are invalid" % owner)
    if growth != final - baseline or value.get("limit_bytes") != RSS_LIMIT_BYTES:
        raise ValueError("%s RSS calculation is invalid" % owner)
    if value.get("within_limit") is not True or growth > RSS_LIMIT_BYTES:
        raise ValueError("%s exceeded the RSS guard" % owner)


def _validate_build(value: dict) -> None:
    if not isinstance(value, dict) or value.get("build_type") != "Release" or value.get(
            "private_build_directory") is not True or value.get(
                "build_directory_persisted") is not False:
        raise ValueError("client AOT build isolation is invalid")
    command = value.get("compile_command")
    if value.get("executable_format") != "ELF" or not isinstance(
            command, str) or "-O3" not in command or "-DNDEBUG" not in command:
        raise ValueError("client AOT build is not proven Release ELF")
    for name in (
            "source_sha256", "cmake_sha256", "compile_commands_sha256",
            "executable_sha256"):
        if not _sha(value.get(name)):
            raise ValueError("client AOT %s is invalid" % name)
    if not _positive(value.get("build_elapsed_ns")):
        raise ValueError("client AOT build time is invalid")


def _validate_server_ready(value: dict, sample: dict, rmw: str) -> None:
    service_type = sample.get("service_type", DEFAULT_SERVICE_TYPE)
    spec = SERVICE_SPECS[service_type]
    expected = {
        "schema": SERVER_SCHEMA,
        "event": "ready",
        "run_token": sample["run_token"],
        "pid": sample["server_pid"],
        "process_group_id": sample["server_pid"],
        "node_name": sample["topology"]["server_node"],
        "loaded_rmw": rmw,
        "execution_model": spec["server_model"],
        "service_name": sample["topology"]["service_name"],
        "service_type": service_type,
    }
    if value != expected:
        raise ValueError("common service server ready evidence is invalid")


def _validate_server_armed(value: dict, sample: dict, warmup: int) -> None:
    service_type = sample.get("service_type", DEFAULT_SERVICE_TYPE)
    expected = {
        "schema": SERVER_SCHEMA,
        "event": "armed",
        "run_token": sample["run_token"],
        "pid": sample["server_pid"],
        "process_group_id": sample["server_pid"],
        "warmup_requests": warmup,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "service_type": service_type,
    }
    actual = dict(value) if isinstance(value, dict) else value
    if isinstance(actual, dict):
        actual.setdefault("service_type", DEFAULT_SERVICE_TYPE)
    if actual != expected:
        raise ValueError("common service server armed evidence is invalid")


def _validate_server_report(value: dict, sample: dict, warmup: int, messages: int) -> None:
    total = warmup + messages
    service_type = sample.get("service_type", DEFAULT_SERVICE_TYPE)
    if not isinstance(value, dict) or value.get("schema") != SERVER_SCHEMA or value.get(
            "event") != "report" or value.get("run_token") != sample["run_token"]:
        raise ValueError("common service server report identity is invalid")
    if value.get("service_type", DEFAULT_SERVICE_TYPE) != service_type:
        raise ValueError("common service server type evidence is invalid")
    expected = {
        "warmup_requests": warmup,
        "total_requests": total,
        "measured_requests": messages,
        "true_total": true_requests(1, total, service_type),
        "true_measured": true_requests(warmup + 1, messages, service_type),
        "response_checksum": response_checksum(warmup + 1, messages, service_type),
        "exceptions": 0,
        "pending_requests": 0,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "correct": True,
        "teardown_clean": True,
    }
    if any(value.get(name) != expected_value for name, expected_value in expected.items()):
        raise ValueError("common service server count/parity evidence is invalid")
    if not _nonnegative(value.get("cpu_time_ns")):
        raise ValueError("common service server CPU evidence is invalid")
    _rss_guard(value.get("rss_guard"), "server")


def _validate_client_warmed(value: dict, sample: dict, cache: dict, rmw: str,
                            warmup: int) -> None:
    variant = sample["variant"]
    spec = VARIANTS[variant]
    service_type = sample.get("service_type", DEFAULT_SERVICE_TYPE)
    service_spec = SERVICE_SPECS[service_type]
    if not isinstance(value, dict) or value.get("schema") != CLIENT_SCHEMA or value.get(
            "event") != "warmed":
        raise ValueError("client warmed evidence is invalid")
    exact = {
        "variant": variant,
        "run_token": sample["run_token"],
        "pid": sample["client_pid"],
        "process_group_id": sample["client_pid"],
        "node_name": sample["topology"]["client_node"],
        "loaded_rmw": rmw,
        "execution_model": spec["model"],
        "client_authority": spec["authority"],
        "warmup_requests": warmup,
        "topology_verified": True,
        "endpoint_count": 1,
        "server_node": sample["topology"]["server_node"],
        "service_name": sample["topology"]["service_name"],
        "service_type": service_type,
        "qos_verified": True,
    }
    if any(value.get(name) != expected for name, expected in exact.items()):
        raise ValueError("client warmed identity/topology evidence is invalid")
    artifact = value.get("cache")
    if not isinstance(artifact, dict) or artifact.get("kind") != spec["cache"]:
        raise ValueError("client cache route is invalid")
    if variant == "aot-staged":
        if artifact != {"state": "prebuilt", "kind": "aot-binary"}:
            raise ValueError("AOT client cache evidence is invalid")
    elif variant in ("stock-rclpy", "compatible-rclcppyy"):
        if artifact != {"state": "not_applicable", "kind": spec["cache"]}:
            raise ValueError("Python client cache evidence is invalid")
        if value.get("entity_type") != "rclpy.client.Client":
            raise ValueError("Python client entity type is invalid")
        marker = value.get("backend_marker")
        evidence = (
            "stock_rclpy_entity" if variant == "stock-rclpy"
            else "rclcppyy_status_entity")
        if not isinstance(marker, dict) or marker.get("role") != "client" or marker.get(
                "backend") != "python" or marker.get("evidence") != evidence:
            raise ValueError("Python client authority marker is invalid")
    else:
        if artifact.get("state") != "prebuilt" or artifact.get("cached") is not True:
            raise ValueError("native client must use a warm artifact")
        _artifact(artifact, True)
        name = (
            "native_client"
            if variant in ("direct-cpp-rclcppyy", "native-python-orchestrated")
            else "cpp_state_machine")
        expected = cache["phases"]["warm"]["artifacts"][name]
        if (artifact["path"], artifact["sha256"], artifact["size_bytes"]) != (
                expected["path"], expected["sha256"], expected["size_bytes"]):
            raise ValueError("native client artifact differs from warm manifest")
        entity_type = "rclcpp::Client<%s>" % service_spec["cpp_type"]
        if value.get("entity_type") != entity_type:
            raise ValueError("native client entity type is invalid")
        if variant == "direct-cpp-rclcppyy":
            if artifact.get("source_id") != cache["phases"]["warm"][
                    "native_client_source_id"]:
                raise ValueError("direct_cpp client source identity is invalid")
            proof = value.get("direct_cpp_proof")
            expected_proof = {
                "profile": "direct_cpp",
                "node_authority": "cpp",
                "client_authority": "cpp",
                "client_entity_type": entity_type,
                "runtime_facade_node_count": 1,
                "native_session_node_count": 1,
                "native_node_identity_verified": True,
                "request_representation": "actual_cpp",
                "response_representation": "actual_cpp",
                "future_type": "rclpy.task.Future",
                "future_control": "per_operation_rclpy_task_future",
                "request_handoff": "one_native_cpp_value_copy",
                "response_handoff": "shared_cpp_response",
                "python_request_crossings_per_call": 1,
                "python_response_crossings_per_call": 1,
                "python_message_conversions_per_call": 0,
                "cpp_request_copies_per_call": 1,
                "python_conversion_guard_installed": True,
                "serialization_guards_installed": True,
            }
            if not isinstance(proof, dict) or set(proof) != {
                    *expected_proof, "status_decision"} or any(
                    proof.get(name) != expected_value
                    for name, expected_value in expected_proof.items()):
                raise ValueError("direct_cpp representation or authority proof is invalid")
            decision = proof.get("status_decision")
            expected_decision = {
                "backend": "cpp",
                "reason": "direct typed rclcpp client with C++ service messages",
                "policies": [
                    "direct_cpp", "direct_cpp_service", "no_conversion",
                    "per_operation_future", "cpp_pending_state",
                ],
                "metadata": {
                    "entity_type": "client",
                    "service_name": sample["topology"]["service_name"],
                    "service_type": service_spec["cpp_type"],
                    "service_interface": service_type,
                    "request_representation": "actual_cpp",
                    "response_representation": "actual_cpp",
                    "python_message_conversions": 0,
                    "source_id": artifact["source_id"],
                    "request_handoff": "one_native_cpp_value_copy",
                    "response_handoff": "shared_cpp_response",
                    "future_control": "per_operation_rclpy_task_future",
                    "python_request_crossings_per_call": 1,
                    "python_response_crossings_per_call": 1,
                    "cpp_request_copies_per_call": 1,
                },
            }
            if not isinstance(decision, dict) or set(decision) != {
                    "id", *expected_decision} or not re.fullmatch(
                    r"entity-[0-9]{8}", str(decision.get("id", ""))) or any(
                        decision.get(name) != expected_value
                        for name, expected_value in expected_decision.items()):
                raise ValueError("direct_cpp status authority evidence is invalid")


def _validate_client_report(value: dict, sample: dict, warmup: int, messages: int) -> None:
    variant = sample["variant"]
    spec = VARIANTS[variant]
    service_type = sample.get("service_type", DEFAULT_SERVICE_TYPE)
    if not isinstance(value, dict) or value.get("schema") != CLIENT_SCHEMA or value.get(
            "event") != "measured" or value.get("variant") != variant or value.get(
                "run_token") != sample["run_token"]:
        raise ValueError("client measured identity is invalid")
    if value.get("service_type", DEFAULT_SERVICE_TYPE) != service_type:
        raise ValueError("client measured service type is invalid")
    exact = {
        "pid": sample["client_pid"],
        "process_group_id": sample["client_pid"],
        "messages": messages,
        "total_requests": warmup + messages,
        "true_measured": true_requests(warmup + 1, messages, service_type),
        "response_checksum": response_checksum(warmup + 1, messages, service_type),
        "python_orchestration_requests_measured": spec["orchestration"] * messages,
        "python_request_crossings_measured": spec["request_crossing"] * messages,
        "python_response_crossings_measured": spec["response_crossing"] * messages,
        "python_message_conversions_measured": spec["message_conversions"] * messages,
        "exceptions": 0,
        "pending_requests": 0,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
    }
    if variant == "direct-cpp-rclcppyy":
        exact["cpp_request_copies_measured"] = (
            spec["cpp_request_copies"] * messages)
    if any(value.get(name) != expected for name, expected in exact.items()):
        raise ValueError("client count/parity/crossing evidence is invalid")
    latencies = value.get("latency_ns")
    if not isinstance(latencies, list) or len(latencies) != messages or any(
            not _positive(item) for item in latencies):
        raise ValueError("client raw RTT evidence is invalid")
    if not _positive(value.get("elapsed_ns")) or not _nonnegative(value.get("cpu_time_ns")):
        raise ValueError("client timing evidence is invalid")
    _rss_guard(value.get("rss_guard"), "client")


def _validate_teardown(value: dict, sample: dict) -> None:
    service_type = sample.get("service_type", DEFAULT_SERVICE_TYPE)
    expected = {
        "schema": CLIENT_SCHEMA,
        "event": "teardown",
        "variant": sample["variant"],
        "run_token": sample["run_token"],
        "pid": sample["client_pid"],
        "process_group_id": sample["client_pid"],
        "service_type": service_type,
        "endpoint_disappeared": True,
        "teardown_clean": True,
    }
    if sample["variant"] == "direct-cpp-rclcppyy":
        expected["direct_cpp_teardown"] = {
            "endpoint_disappeared": True,
            "client_closed": True,
            "node_destroyed": True,
            "context_shutdown": True,
            "native_session_closed": True,
            "native_session_released": True,
            "native_executor_released": True,
            "runtime_nodes_released": True,
        }
    actual = dict(value) if isinstance(value, dict) else value
    if isinstance(actual, dict):
        actual.setdefault("service_type", DEFAULT_SERVICE_TYPE)
    if actual != expected:
        raise ValueError("client endpoint disappearance/teardown evidence is invalid")


def validate_sample(sample: dict, parameters: dict, build: dict, cache: dict) -> None:
    if not isinstance(sample, dict) or sample.get("schema") != SAMPLE_SCHEMA:
        raise ValueError("unsupported service client sample schema")
    variant = sample.get("variant")
    if variant not in VARIANTS or not _positive(sample.get("repetition")):
        raise ValueError("service client variant/repetition is invalid")
    service_type = parameters.get("service_type", DEFAULT_SERVICE_TYPE)
    if service_type not in SERVICE_TYPES or sample.get(
            "service_type", DEFAULT_SERVICE_TYPE) != service_type:
        raise ValueError("service client interface is invalid")
    token = sample.get("run_token")
    if not isinstance(token, str) or not re.fullmatch(r"run_[a-f0-9]{32}", token):
        raise ValueError("service client run token is invalid")
    server_pid = sample.get("server_pid")
    client_pid = sample.get("client_pid")
    if not _positive(server_pid) or not _positive(client_pid) or server_pid == client_pid:
        raise ValueError("service client requires two distinct process identities")
    topology = sample.get("topology")
    if not isinstance(topology, dict) or topology.get("process_count") != 2 or topology.get(
            "fresh_process_groups") is not True or not all(
                isinstance(topology.get(name), str) and topology[name]
                for name in ("server_node", "client_node", "service_name")):
        raise ValueError("service client process/graph topology is invalid")
    rmw = parameters["requested_rmw"]
    if rmw != "rmw_cyclonedds_cpp" or sample.get("requested_rmw") != rmw or sample.get(
            "qos") != QOS:
        raise ValueError("service client benchmark must use explicit Cyclone service QoS")
    warmup = parameters["warmup_requests"]
    messages = parameters["messages"]
    _validate_server_ready(sample.get("server_ready"), sample, rmw)
    _validate_client_warmed(sample.get("client_warmed"), sample, cache, rmw, warmup)
    _validate_server_armed(sample.get("server_armed"), sample, warmup)
    _validate_client_report(sample.get("client_report"), sample, warmup, messages)
    _validate_server_report(sample.get("server_report"), sample, warmup, messages)
    _validate_teardown(sample.get("client_teardown"), sample)
    timing = sample.get("timing")
    client = sample["client_report"]
    server = sample["server_report"]
    if not isinstance(timing, dict) or timing.get("rtt_ns") != latency_summary(
            client["latency_ns"]):
        raise ValueError("service client RTT summary contradicts raw evidence")
    if timing.get("client_cpu_time_ns") != client["cpu_time_ns"] or timing.get(
            "client_cpu_ns_per_response") != client["cpu_time_ns"] / messages:
        raise ValueError("service client CPU summary is invalid")
    if timing.get("server_cpu_time_ns") != server["cpu_time_ns"] or timing.get(
            "server_cpu_ns_per_request_drift") != server["cpu_time_ns"] / messages:
        raise ValueError("common server CPU drift summary is invalid")
    expected_rate = messages * 1e9 / client["elapsed_ns"]
    if not math.isclose(timing.get("requests_per_second", -1), expected_rate, rel_tol=1e-12):
        raise ValueError("service client request rate is invalid")
    if sample.get("backend_verified") is not True or sample.get(
            "correctness_verified") is not True or sample.get("teardown_verified") is not True:
        raise ValueError("service client acceptance flags are incomplete")


def _metric(sample: dict, name: str):
    timing = sample["timing"]
    return {
        "client_cpu_ns_per_response": timing["client_cpu_ns_per_response"],
        "latency_p50_ns": timing["rtt_ns"]["p50"],
        "latency_p95_ns": timing["rtt_ns"]["p95"],
        "latency_p99_ns": timing["rtt_ns"]["p99"],
        "latency_max_ns": timing["rtt_ns"]["max"],
        "requests_per_second": timing["requests_per_second"],
    }[name]


def summarize(results: list[dict], variants: list[str]) -> dict:
    rows = {}
    medians = {}
    for variant in variants:
        selected = sorted(
            (row for row in results if row.get("variant") == variant),
            key=lambda row: row["repetition"])
        rows[variant] = {row["repetition"]: row for row in selected}
        medians[variant] = {
            metric: statistics.median([_metric(row, metric) for row in selected])
            for metric in METRICS
        } if selected else {}

    def paired(reference):
        output = {}
        for variant in variants:
            repetitions = sorted(set(rows[variant]) & set(rows.get(reference, {})))
            output[variant] = {
                metric: [
                    (_metric(rows[variant][rep], metric) /
                     _metric(rows[reference][rep], metric))
                    if _metric(rows[reference][rep], metric) else None
                    for rep in repetitions
                ]
                for metric in METRICS
            }
        return output

    return {
        "raw_medians": medians,
        "paired_ratios_to_stock": paired("stock-rclpy"),
        "paired_ratios_to_aot": paired("aot-staged"),
        "interpretation_allowed": False,
        "note": (
            "Raw paired descriptions only. Common server CPU and RSS are drift/guard evidence "
            "and are never ranked; no threshold or winner is selected."),
    }


def _source_environment(value: dict) -> None:
    source = value.get("source") if isinstance(value, dict) else None
    if not isinstance(source, dict) or not isinstance(source.get("commit"), str) or not COMMIT.fullmatch(
            source["commit"]) or not isinstance(source.get("dirty"), bool):
        raise ValueError("service client source identity is invalid")
    dependencies = value.get("source_dependencies")
    suite = dependencies.get("rclcpp_kit") if isinstance(dependencies, dict) else None
    if not isinstance(suite, dict) or not isinstance(suite.get("commit"), str) or not COMMIT.fullmatch(
            suite["commit"]) or not isinstance(suite.get("dirty"), bool):
        raise ValueError("service client suite identity is invalid")


def build_document(
        *, repo_root: Path, mode: str, parameters: dict, isolation: dict,
        aot_build: dict, cache: dict, source_files: dict, results: list,
        failures: list, command: list[str] | None = None) -> dict:
    service_type = parameters.get("service_type", DEFAULT_SERVICE_TYPE)
    service_spec = SERVICE_SPECS[service_type]
    document = {
        "schema": SCHEMA_ID,
        "generated_at": datetime.datetime.now(
            datetime.timezone.utc).isoformat().replace("+00:00", "Z"),
        "command": list(command if command is not None else sys.argv),
        "environment": environment_metadata(repo_root),
        "benchmark": {
            "name": service_spec["benchmark_name"],
            "mode": mode,
            "performance_claims_allowed": False,
            "parameters": parameters,
            "isolation": isolation,
            "aot_build": aot_build,
            "cache": cache,
            "source_files": source_files,
            "statistics": {
                "workload": service_spec["workload"],
                "client_cpu": "client-owned CLOCK_PROCESS_CPUTIME_ID delta",
                "server_cpu": "common server drift diagnostic only",
                "rtt": "client steady-clock observations; nearest-rank percentiles",
                "memory": "post-warmup peak-RSS growth is a guard only and is never ranked",
            },
            "scope": {
                "ros_distribution": "jazzy",
                "rmw": "rmw_cyclonedds_cpp",
                "steady_state_only": True,
                "maximum_rate_claim": False,
                "cross_machine_claim": False,
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
        raise ValueError("unsupported service client benchmark schema")
    _source_environment(document.get("environment"))
    benchmark = document.get("benchmark")
    if not isinstance(benchmark, dict) or benchmark.get(
            "mode") not in ("smoke", "measurement"):
        raise ValueError("service client benchmark metadata is invalid")
    if benchmark.get("performance_claims_allowed") is not False:
        raise ValueError("raw service client benchmark cannot allow claims")
    parameters = benchmark.get("parameters")
    service_type = parameters.get(
        "service_type", DEFAULT_SERVICE_TYPE) if isinstance(parameters, dict) else None
    if service_type not in SERVICE_TYPES or benchmark.get(
            "name") != SERVICE_SPECS[service_type]["benchmark_name"]:
        raise ValueError("service client benchmark interface metadata is invalid")
    variants = parameters.get("variants") if isinstance(parameters, dict) else None
    if not isinstance(variants, list) or not variants or len(set(variants)) != len(variants) or any(
            variant not in VARIANTS for variant in variants):
        raise ValueError("service client benchmark variant matrix is invalid")
    if parameters.get("requested_rmw") != "rmw_cyclonedds_cpp" or parameters.get(
            "qos") != QOS or not _positive(parameters.get("messages")) or not _positive(
                parameters.get("warmup_requests")) or not _positive(
                    parameters.get("repetitions")):
        raise ValueError("service client benchmark parameters are invalid")
    _validate_build(benchmark.get("aot_build"))
    service_type = parameters.get("service_type", DEFAULT_SERVICE_TYPE)
    if service_type not in SERVICE_TYPES:
        raise ValueError("service client benchmark interface is invalid")
    validate_cache(benchmark.get("cache"), parameters["requested_rmw"], service_type)
    source_files = benchmark.get("source_files")
    if not isinstance(source_files, dict) or not source_files or any(
            not _sha(value) for value in source_files.values()):
        raise ValueError("service client source hashes are invalid")
    isolation = benchmark.get("isolation")
    required = (
        "fresh_process_pair_per_sample", "two_process_groups_per_sample",
        "unique_service_per_sample", "one_leased_domain_per_run",
        "rotating_variant_order")
    if not isinstance(isolation, dict) or any(isolation.get(name) is not True for name in required):
        raise ValueError("service client isolation evidence is invalid")
    domain = isolation.get("ros_domain_id")
    if not _int(domain) or not 0 <= domain <= 232:
        raise ValueError("service client domain is invalid")
    results = document.get("results")
    failures = document.get("failures")
    if not isinstance(results, list) or not isinstance(failures, list):
        raise ValueError("service client results/failures must be arrays")
    identities = set()
    pids = []
    tokens = []
    for sample in results:
        validate_sample(sample, parameters, benchmark["aot_build"], benchmark["cache"])
        identity = (sample["variant"], sample["repetition"])
        if identity in identities:
            raise ValueError("duplicate service client benchmark sample")
        identities.add(identity)
        pids.extend((sample["server_pid"], sample["client_pid"]))
        tokens.append(sample["run_token"])
        if sample["ros_domain_id"] != domain:
            raise ValueError("service client sample used an unleased domain")
    if len(pids) != len(set(pids)) or len(tokens) != len(set(tokens)):
        raise ValueError("service client samples require fresh processes/services")
    if not failures:
        expected = {
            (variant, repetition)
            for repetition in range(1, parameters["repetitions"] + 1)
            for variant in variants
        }
        if identities != expected:
            raise ValueError("successful service client matrix is incomplete")
    comparison = document.get("comparison")
    if comparison != summarize(results, variants) or comparison.get(
            "interpretation_allowed") is not False:
        raise ValueError("service client descriptive comparison is invalid")


def dumps(document: dict) -> str:
    validate_document(document)
    return json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n"


def write(document: dict, path: Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(dumps(document), encoding="utf-8")
    temporary.replace(path)


__all__ = [
    "CLIENT_SCHEMA", "PREWARM_SCHEMA", "QOS", "SAMPLE_SCHEMA", "SCHEMA_ID",
    "SERVER_SCHEMA", "VARIANTS", "build_document", "dumps", "latency_summary",
    "nearest_rank", "response_checksum", "summarize", "true_requests",
    "validate_document", "validate_prewarm", "validate_sample", "write",
]
