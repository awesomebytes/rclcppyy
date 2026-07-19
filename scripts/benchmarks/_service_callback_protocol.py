"""Strict evidence contract for the controlled SetBool callback benchmark."""

from __future__ import annotations

import datetime
import json
import math
from pathlib import Path
import re
import statistics
import sys

from _result_schema import environment_metadata


SCHEMA_ID = "rclcppyy.service-callback-benchmark/v1"
SAMPLE_SCHEMA = "rclcppyy.service-callback-sample/v1"
PREWARM_SCHEMA = "rclcppyy.service-callback-prewarm/v1"
SERVER_SCHEMA = "rclcppyy.service-callback-server-event/v1"
CLIENT_SCHEMA = "rclcppyy.service-callback-client-event/v1"
VARIANTS = {
    "stock-rclpy": {
        "model": "same-python-service-stock-rclpy",
        "cache": "stock-rclpy",
        "python": True,
        "authority": "python",
    },
    "compatible-rclcppyy": {
        "model": "same-python-service-compatible-activation",
        "cache": "compatible-python-service",
        "python": True,
        "authority": "python",
    },
    "native-python-callback": {
        "model": "native-rclcpp-service-python-callback",
        "cache": "python-service-bridge",
        "python": True,
        "authority": "cpp",
    },
    "direct-cpp-rclcppyy": {
        "model": "source-compatible-direct-cpp-service-python-callback",
        "cache": "direct-cpp-python-service",
        "python": True,
        "authority": "cpp",
    },
    "native-cpp-callback": {
        "model": "native-session-cpp-service-callback",
        "cache": "native-cpp-service",
        "python": False,
        "authority": "cpp",
    },
    "aot-staged": {
        "model": "conventional-release-aot-service",
        "cache": "aot-binary",
        "python": False,
        "authority": "cpp",
    },
}
QOS = {
    "history": "keep_last",
    "depth": 10,
    "reliability": "reliable",
    "durability": "volatile",
}
METRICS = (
    "server_cpu_ns_per_request",
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


def true_requests(start: int, count: int) -> int:
    return sum(sequence % 2 == 1 for sequence in range(start, start + count))


def response_checksum(start: int, count: int) -> int:
    total = 0
    for sequence in range(start, start + count):
        value = sequence % 2 == 1
        total += 117 if value else 8
    return total


def _artifact(value: dict, cached: bool | None = None) -> None:
    if not isinstance(value, dict):
        raise ValueError("service cache artifact is required")
    if cached is not None and value.get("cached") is not cached:
        raise ValueError("service cache hit state is invalid")
    if not isinstance(value.get("path"), str) or not value["path"]:
        raise ValueError("service cache artifact path is required")
    if not _sha(value.get("sha256")) or not _positive(value.get("size_bytes")):
        raise ValueError("service cache artifact identity is invalid")


def validate_prewarm(value: dict, expect_hits: bool) -> None:
    if not isinstance(value, dict) or value.get("schema") != PREWARM_SCHEMA:
        raise ValueError("unsupported service prewarm schema")
    if not _positive(value.get("pid")) or not isinstance(value.get("loaded_rmw"), str):
        raise ValueError("service prewarm process/RMW evidence is invalid")
    source_id = value.get("native_cpp_source_id")
    if not isinstance(source_id, str) or not re.fullmatch(r"[a-f0-9]{16}", source_id):
        raise ValueError("native C++ service source id is invalid")
    artifacts = value.get("artifacts")
    if not isinstance(artifacts, dict) or set(artifacts) != {
            "python_bridge", "direct_cpp_python_service", "native_cpp_service"}:
        raise ValueError("service prewarm artifact matrix is incomplete")
    for artifact in artifacts.values():
        _artifact(artifact, expect_hits)
    diagnostics = value.get("stdout_diagnostics", [])
    if not isinstance(diagnostics, list) or any(
            not isinstance(line, str) for line in diagnostics):
        raise ValueError("service prewarm diagnostics are invalid")


def validate_cache(value: dict, rmw: str) -> None:
    required = (
        "isolated_root", "autopch_disabled", "fresh_process_per_phase",
        "compilation_excluded_from_samples", "warm_hits_verified")
    if not isinstance(value, dict) or any(value.get(key) is not True for key in required):
        raise ValueError("service cache policy is incomplete")
    if value.get("persisted_after_run") is not False:
        raise ValueError("service cache must be temporary")
    phases = value.get("phases")
    if not isinstance(phases, dict) or set(phases) != {"cold", "warm"}:
        raise ValueError("service cache phases are incomplete")
    validate_prewarm(phases["cold"], False)
    validate_prewarm(phases["warm"], True)
    if phases["cold"]["loaded_rmw"] != rmw or phases["warm"]["loaded_rmw"] != rmw:
        raise ValueError("service prewarm used the wrong RMW")
    for name in ("python_bridge", "direct_cpp_python_service", "native_cpp_service"):
        cold = phases["cold"]["artifacts"][name]
        warm = phases["warm"]["artifacts"][name]
        if (cold["path"], cold["sha256"], cold["size_bytes"]) != (
                warm["path"], warm["sha256"], warm["size_bytes"]):
            raise ValueError("service cold/warm artifact identity changed")
    if phases["cold"]["native_cpp_source_id"] != phases["warm"][
            "native_cpp_source_id"]:
        raise ValueError("native C++ service source identity changed")


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
        raise ValueError("service AOT build isolation is invalid")
    if value.get("executable_format") != "ELF":
        raise ValueError("service AOT executable must be ELF")
    command = value.get("compile_command")
    if not isinstance(command, str) or "-O3" not in command or "-DNDEBUG" not in command:
        raise ValueError("service AOT build is not proven Release")
    for name in (
            "source_sha256", "cmake_sha256", "compile_commands_sha256",
            "executable_sha256"):
        if not _sha(value.get(name)):
            raise ValueError("service AOT %s is invalid" % name)
    if not _positive(value.get("build_elapsed_ns")):
        raise ValueError("service AOT build time is invalid")


def _validate_ready(ready: dict, sample: dict, cache: dict, rmw: str) -> None:
    variant = sample["variant"]
    spec = VARIANTS[variant]
    if not isinstance(ready, dict) or ready.get("schema") != SERVER_SCHEMA or ready.get(
            "event") != "ready":
        raise ValueError("service server ready evidence is invalid")
    exact = {
        "variant": variant,
        "run_token": sample["run_token"],
        "pid": sample["server_pid"],
        "process_group_id": sample["server_pid"],
        "node_name": sample["topology"]["server_node"],
        "loaded_rmw": rmw,
        "execution_model": spec["model"],
        "service_authority": spec["authority"],
    }
    if any(ready.get(key) != expected for key, expected in exact.items()):
        raise ValueError("service server ready identity is invalid")
    artifact = ready.get("cache")
    if not isinstance(artifact, dict) or artifact.get("kind") != spec["cache"]:
        raise ValueError("service server cache route is invalid")
    if variant == "aot-staged":
        if artifact != {"state": "prebuilt", "kind": "aot-binary"}:
            raise ValueError("AOT service cache evidence is invalid")
    elif variant in ("stock-rclpy", "compatible-rclcppyy"):
        if artifact.get("state") != "not_applicable" or set(artifact) != {"state", "kind"}:
            raise ValueError("Python service cache evidence is invalid")
        if ready.get("entity_type") != "rclpy.service.Service":
            raise ValueError("Python service entity type is invalid")
        marker = ready.get("backend_marker")
        expected_evidence = (
            "stock_rclpy_entity" if variant == "stock-rclpy"
            else "rclcppyy_status_entity")
        if not isinstance(marker, dict) or marker.get("backend") != "python" or marker.get(
                "role") != "server" or marker.get("evidence") != expected_evidence:
            raise ValueError("Python service authority marker is invalid")
    else:
        if artifact.get("state") != "prebuilt" or artifact.get("cached") is not True:
            raise ValueError("native service must use a warm artifact")
        _artifact(artifact, True)
        name = {
            "native-python-callback": "python_bridge",
            "direct-cpp-rclcppyy": "direct_cpp_python_service",
        }.get(variant, "native_cpp_service")
        expected = cache["phases"]["warm"]["artifacts"][name]
        if (artifact["path"], artifact["sha256"], artifact["size_bytes"]) != (
                expected["path"], expected["sha256"], expected["size_bytes"]):
            raise ValueError("native service artifact differs from warm manifest")
        if ready.get("entity_type") != "rclcpp::Service<std_srvs::srv::SetBool>":
            raise ValueError("native service entity type is invalid")
        if variant == "direct-cpp-rclcppyy":
            marker = ready.get("backend_marker")
            if not isinstance(marker, dict) or marker.get("backend") != "cpp" or marker.get(
                    "role") != "server" or marker.get(
                        "evidence") != "rclcppyy_status_entity":
                raise ValueError("direct C++ service authority marker is invalid")
            expected_path = {
                "request_representation": "actual_cpp",
                "response_representation": "actual_cpp",
                "python_message_conversions": 0,
                "serialization_bridges": 0,
                "python_callback_crossings_per_request": 1,
                "request_cpp_copies_per_request": 1,
                "response_cpp_copies_per_request": 1,
            }
            if ready.get("data_path") != expected_path:
                raise ValueError("direct C++ service data-path evidence is invalid")


def _validate_armed(value: dict, sample: dict, warmup: int) -> None:
    expected = {
        "schema": SERVER_SCHEMA,
        "event": "armed",
        "variant": sample["variant"],
        "run_token": sample["run_token"],
        "pid": sample["server_pid"],
        "process_group_id": sample["server_pid"],
        "warmup_requests": warmup,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
    }
    if value != expected:
        raise ValueError("service server armed evidence is invalid")


def _validate_report(value: dict, sample: dict, warmup: int, messages: int) -> None:
    variant = sample["variant"]
    total = warmup + messages
    if not isinstance(value, dict) or value.get("schema") != SERVER_SCHEMA or value.get(
            "event") != "report" or value.get("variant") != variant or value.get(
                "run_token") != sample["run_token"]:
        raise ValueError("service server report identity is invalid")
    if value.get("warmup_requests") != warmup or value.get("total_requests") != total or value.get(
            "measured_requests") != messages:
        raise ValueError("service server request counts are invalid")
    if value.get("exceptions") != 0 or value.get("pending_requests") != 0 or value.get(
            "correct") is not True or value.get("teardown_clean") is not True:
        raise ValueError("service server correctness/teardown evidence is invalid")
    python = VARIANTS[variant]["python"]
    if value.get("python_callback_count_total") != (total if python else 0) or value.get(
            "python_boundary_crossings_measured") != (messages if python else 0):
        raise ValueError("service Python callback crossing counts are invalid")
    if variant == "direct-cpp-rclcppyy":
        exact = {
            "request_cpp_copies_measured": messages,
            "response_cpp_copies_measured": messages,
            "python_message_conversions_measured": 0,
            "serialization_bridges_measured": 0,
        }
        if any(value.get(key) != expected for key, expected in exact.items()):
            raise ValueError("direct C++ service copy/conversion evidence is invalid")
    if variant != "native-cpp-callback":
        if value.get("true_total") != true_requests(1, total) or value.get(
                "true_measured") != true_requests(warmup + 1, messages):
            raise ValueError("service alternating request counts are invalid")
        if value.get("response_checksum") != response_checksum(warmup + 1, messages):
            raise ValueError("service response checksum is invalid")
    elif any(value.get(name) is not None for name in (
            "true_total", "true_measured", "response_checksum")):
        raise ValueError("native C++ service exposed unsupported internal counters")
    if not _nonnegative(value.get("cpu_time_ns")) or value.get(
            "cpu_clock") != "CLOCK_PROCESS_CPUTIME_ID":
        raise ValueError("service server CPU evidence is invalid")
    _rss_guard(value.get("rss_guard"), "server")


def _validate_client(value: dict, sample: dict, rmw: str, messages: int, warmup: int) -> None:
    if not isinstance(value, dict) or value.get("schema") != CLIENT_SCHEMA or value.get(
            "event") != "measured":
        raise ValueError("service AOT client result is invalid")
    exact = {
        "run_token": sample["run_token"],
        "pid": sample["client_pid"],
        "process_group_id": sample["client_pid"],
        "loaded_rmw": rmw,
        "execution_model": "identical-release-aot-one-outstanding-client",
        "messages": messages,
        "response_checksum": response_checksum(warmup + 1, messages),
        "topology_verified": True,
        "qos_verified": True,
        "qos": QOS,
        "server_node": sample["topology"]["server_node"],
        "service_name": sample["topology"]["service_name"],
        "service_type": "std_srvs/srv/SetBool",
        "pending_requests": 0,
    }
    if any(value.get(key) != expected for key, expected in exact.items()):
        raise ValueError("service AOT client parity/topology evidence is invalid")
    latency = value.get("latency_ns")
    if not isinstance(latency, list) or len(latency) != messages or any(
            not _positive(item) for item in latency):
        raise ValueError("service raw RTT evidence is invalid")
    if not _positive(value.get("elapsed_ns")) or not _nonnegative(value.get("cpu_time_ns")) or value.get(
            "cpu_clock") != "CLOCK_PROCESS_CPUTIME_ID":
        raise ValueError("service AOT client timing evidence is invalid")
    _rss_guard(value.get("rss_guard"), "client")


def _validate_client_teardown(value: dict, sample: dict) -> None:
    if not isinstance(value, dict) or value.get("schema") != CLIENT_SCHEMA or value.get(
            "event") != "teardown":
        raise ValueError("service AOT client teardown evidence is invalid")
    if value.get("run_token") != sample["run_token"] or value.get("pid") != sample[
            "client_pid"] or value.get("process_group_id") != sample["client_pid"]:
        raise ValueError("service AOT client teardown identity is invalid")
    if value.get("endpoint_disappeared") is not True or value.get("teardown_clean") is not True:
        raise ValueError("service endpoint disappearance/teardown proof is invalid")


def validate_sample(sample: dict, parameters: dict, build: dict, cache: dict) -> None:
    if not isinstance(sample, dict) or sample.get("schema") != SAMPLE_SCHEMA:
        raise ValueError("unsupported service callback sample schema")
    variant = sample.get("variant")
    if variant not in VARIANTS or not _positive(sample.get("repetition")):
        raise ValueError("service callback variant/repetition is invalid")
    token = sample.get("run_token")
    if not isinstance(token, str) or not re.fullmatch(r"run_[a-f0-9]{32}", token):
        raise ValueError("service callback run token is invalid")
    server_pid = sample.get("server_pid")
    client_pid = sample.get("client_pid")
    if not _positive(server_pid) or not _positive(client_pid) or server_pid == client_pid:
        raise ValueError("service callback requires two distinct process identities")
    topology = sample.get("topology")
    if not isinstance(topology, dict) or topology.get("process_count") != 2 or topology.get(
            "fresh_process_groups") is not True:
        raise ValueError("service callback process topology is invalid")
    if not all(isinstance(topology.get(name), str) and topology[name] for name in (
            "server_node", "client_node", "service_name")):
        raise ValueError("service graph identity is incomplete")
    rmw = parameters["requested_rmw"]
    if rmw != "rmw_cyclonedds_cpp" or sample.get("requested_rmw") != rmw or sample.get(
            "qos") != QOS:
        raise ValueError("service benchmark must use explicit Cyclone service QoS")
    warmup = parameters["warmup_requests"]
    messages = parameters["messages"]
    _validate_ready(sample.get("server_ready"), sample, cache, rmw)
    _validate_armed(sample.get("server_armed"), sample, warmup)
    _validate_report(sample.get("server_report"), sample, warmup, messages)
    _validate_client(sample.get("client_result"), sample, rmw, messages, warmup)
    _validate_client_teardown(sample.get("client_teardown"), sample)
    timing = sample.get("timing")
    client = sample["client_result"]
    server = sample["server_report"]
    if not isinstance(timing, dict) or timing.get("rtt_ns") != latency_summary(client["latency_ns"]):
        raise ValueError("service RTT summary contradicts raw evidence")
    if timing.get("server_cpu_time_ns") != server["cpu_time_ns"] or timing.get(
            "server_cpu_ns_per_request") != server["cpu_time_ns"] / messages:
        raise ValueError("service server CPU summary is invalid")
    if timing.get("client_cpu_time_ns") != client["cpu_time_ns"] or timing.get(
            "client_cpu_ns_per_request") != client["cpu_time_ns"] / messages:
        raise ValueError("service client CPU summary is invalid")
    expected_rate = messages * 1e9 / client["elapsed_ns"]
    if not math.isclose(timing.get("requests_per_second", -1), expected_rate, rel_tol=1e-12):
        raise ValueError("service request rate is invalid")
    if sample.get("backend_verified") is not True or sample.get(
            "correctness_verified") is not True or sample.get("teardown_verified") is not True:
        raise ValueError("service acceptance flags are incomplete")


def _metric(sample: dict, name: str):
    timing = sample["timing"]
    return {
        "server_cpu_ns_per_request": timing["server_cpu_ns_per_request"],
        "latency_p50_ns": timing["rtt_ns"]["p50"],
        "latency_p95_ns": timing["rtt_ns"]["p95"],
        "latency_p99_ns": timing["rtt_ns"]["p99"],
        "latency_max_ns": timing["rtt_ns"]["max"],
        "requests_per_second": timing["requests_per_second"],
    }[name]


def summarize(results: list[dict], variants: list[str]) -> dict:
    medians = {}
    rows = {}
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
            "Raw paired descriptions only. Lower is better for CPU/RTT and higher for request "
            "rate; no threshold, ranking, or winner is selected."),
    }


def _source_environment(value: dict) -> None:
    source = value.get("source") if isinstance(value, dict) else None
    if not isinstance(source, dict) or not isinstance(source.get("commit"), str) or not COMMIT.fullmatch(
            source["commit"]) or not isinstance(source.get("dirty"), bool):
        raise ValueError("service benchmark source identity is invalid")
    dependencies = value.get("source_dependencies")
    suite = dependencies.get("rclcpp_kit") if isinstance(dependencies, dict) else None
    if not isinstance(suite, dict) or not isinstance(suite.get("commit"), str) or not COMMIT.fullmatch(
            suite["commit"]) or not isinstance(suite.get("dirty"), bool):
        raise ValueError("service benchmark suite identity is invalid")


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
            "name": "jazzy_cyclone_setbool_callback_cpu",
            "mode": mode,
            "performance_claims_allowed": False,
            "parameters": parameters,
            "isolation": isolation,
            "aot_build": aot_build,
            "cache": cache,
            "source_files": source_files,
            "statistics": {
                "workload": "one-outstanding alternating SetBool requests",
                "timed_region": "post-discovery and post-100-request warmup",
                "server_cpu": "server-owned CLOCK_PROCESS_CPUTIME_ID delta",
                "client_cpu": "client-owned CLOCK_PROCESS_CPUTIME_ID delta",
                "rtt": "AOT client steady-clock observations; nearest-rank percentiles",
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
        raise ValueError("unsupported service callback benchmark schema")
    _source_environment(document.get("environment"))
    benchmark = document.get("benchmark")
    if not isinstance(benchmark, dict) or benchmark.get(
            "name") != "jazzy_cyclone_setbool_callback_cpu" or benchmark.get(
                "mode") not in ("smoke", "measurement"):
        raise ValueError("service benchmark metadata is invalid")
    if benchmark.get("performance_claims_allowed") is not False:
        raise ValueError("raw service benchmark cannot allow performance claims")
    parameters = benchmark.get("parameters")
    variants = parameters.get("variants") if isinstance(parameters, dict) else None
    if not isinstance(variants, list) or not variants or len(set(variants)) != len(variants) or any(
            variant not in VARIANTS for variant in variants):
        raise ValueError("service benchmark variant matrix is invalid")
    if parameters.get("requested_rmw") != "rmw_cyclonedds_cpp" or parameters.get(
            "qos") != QOS or not _positive(parameters.get("messages")) or not _positive(
                parameters.get("warmup_requests")) or not _positive(parameters.get("repetitions")):
        raise ValueError("service benchmark parameters are invalid")
    _validate_build(benchmark.get("aot_build"))
    validate_cache(benchmark.get("cache"), parameters["requested_rmw"])
    source_files = benchmark.get("source_files")
    if not isinstance(source_files, dict) or not source_files or any(
            not _sha(value) for value in source_files.values()):
        raise ValueError("service benchmark source hashes are invalid")
    isolation = benchmark.get("isolation")
    required = (
        "fresh_process_pair_per_sample", "two_process_groups_per_sample",
        "unique_service_per_sample", "one_leased_domain_per_run",
        "rotating_variant_order")
    if not isinstance(isolation, dict) or any(isolation.get(name) is not True for name in required):
        raise ValueError("service benchmark isolation evidence is invalid")
    domain = isolation.get("ros_domain_id")
    if not _int(domain) or not 0 <= domain <= 232:
        raise ValueError("service benchmark domain is invalid")
    results = document.get("results")
    failures = document.get("failures")
    if not isinstance(results, list) or not isinstance(failures, list):
        raise ValueError("service results/failures must be arrays")
    identities = set()
    pids = []
    tokens = []
    for sample in results:
        validate_sample(sample, parameters, benchmark["aot_build"], benchmark["cache"])
        identity = (sample["variant"], sample["repetition"])
        if identity in identities:
            raise ValueError("duplicate service benchmark sample")
        identities.add(identity)
        pids.extend((sample["server_pid"], sample["client_pid"]))
        tokens.append(sample["run_token"])
        if sample["ros_domain_id"] != domain:
            raise ValueError("service sample used an unleased domain")
    if len(pids) != len(set(pids)) or len(tokens) != len(set(tokens)):
        raise ValueError("service samples require fresh processes and unique services")
    if not failures:
        expected = {
            (variant, repetition)
            for repetition in range(1, parameters["repetitions"] + 1)
            for variant in variants
        }
        if identities != expected:
            raise ValueError("successful service result matrix is incomplete")
    comparison = document.get("comparison")
    if comparison != summarize(results, variants) or comparison.get(
            "interpretation_allowed") is not False:
        raise ValueError("service descriptive comparison is invalid")


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
