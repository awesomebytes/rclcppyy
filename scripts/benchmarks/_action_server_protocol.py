"""Strict CPU-first evidence contract for the Jazzy/Cyclone action server."""

from __future__ import annotations

import datetime
import json
import math
from pathlib import Path
import re

from _action_client_protocol import (
    ACTION_TYPE,
    BOUNDARY_TRIPWIRE_SURFACES,
    CLIENT_SCHEMA,
    FEEDBACK_PER_GOAL,
    MEASURED_GOALS,
    QOS,
    REPETITIONS,
    RMW,
    ROS_DISTRO,
    RSS_LIMIT_BYTES,
    SERVER_SCHEMA,
    WARMUP_GOALS,
    endpoint_names,
    expected_checksum,
    validate_build,
)
from _result_schema import environment_metadata


SCHEMA_ID = "rclcppyy.action-server-benchmark/v1"
SAMPLE_SCHEMA = "rclcppyy.action-server-sample/v1"
PREWARM_SCHEMA = "rclcppyy.action-server-prewarm/v1"
SHA256_PATTERN = re.compile(r"^[a-f0-9]{64}$")
TOKEN_PATTERN = re.compile(r"^action_server_[a-f0-9]{32}$")

VARIANTS = {
    "stock-rclpy": {
        "execution_model": "stock-rclpy-action-server-python-callbacks",
        "implementation": "rclpy.action.server.ActionServer",
        "authority": "python",
        "representation": "python-message",
        "exact_cpp": False,
        "cache_kind": "stock-rclpy",
        "python_crossings_per_goal": {
            "goal_decision": 1, "accepted_goal": 1, "execute": 1,
        },
    },
    "direct-source-compatible": {
        "execution_model": "product-direct-rclcpp-action-server-python-callbacks",
        "implementation": "rclcppyy.direct_actions.DirectActionServer",
        "authority": "cpp",
        "representation": "generated-cpp",
        "exact_cpp": True,
        "cache_kind": "native-action-server-shared-library",
        "python_crossings_per_goal": {
            "goal_decision": 1, "accepted_goal": 1, "execute": 1,
        },
    },
    "native-python-orchestrated": {
        "execution_model": "raw-native-action-server-python-orchestration",
        "implementation": "rclcpp_kit.native_action_server.NativeActionServer",
        "authority": "cpp",
        "representation": "generated-cpp",
        "exact_cpp": True,
        "cache_kind": "native-action-server-shared-library",
        "python_crossings_per_goal": {
            "goal_decision": 1, "accepted_goal": 1, "execute": 0,
        },
    },
    "native-cpp-state-machine": {
        "execution_model": "cached-cpp-action-server-state-machine",
        "implementation": "rclcppyy_action_server_benchmark::StateMachine",
        "authority": "cpp",
        "representation": "generated-cpp",
        "exact_cpp": True,
        "cache_kind": "action-server-state-machine-shared-library",
        "python_crossings_per_goal": {
            "goal_decision": 0, "accepted_goal": 0, "execute": 0,
        },
    },
    "aot-staged": {
        "execution_model": "conventional-release-aot-rclcpp-action-server",
        "implementation": "rclcpp_action::Server<LookupTransform>",
        "authority": "cpp",
        "representation": "generated-cpp",
        "exact_cpp": True,
        "cache_kind": "aot-binary",
        "python_crossings_per_goal": {
            "goal_decision": 0, "accepted_goal": 0, "execute": 0,
        },
    },
}


def _is_int(value) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _positive_int(value) -> bool:
    return _is_int(value) and value > 0


def _nonnegative_int(value) -> bool:
    return _is_int(value) and value >= 0


def _validate_percentiles(value: dict) -> None:
    if not isinstance(value, dict) or set(value) != {"p50", "p95", "p99", "max"}:
        raise ValueError("action-server latency percentile fields are invalid")
    if any(not _nonnegative_int(item) for item in value.values()):
        raise ValueError("action-server latency percentiles are invalid")
    if not value["p50"] <= value["p95"] <= value["p99"] <= value["max"]:
        raise ValueError("action-server latency percentiles are not ordered")


def expected_python_crossings(variant: str, goals: int) -> dict[str, int]:
    result = {
        key: value * goals
        for key, value in VARIANTS[variant]["python_crossings_per_goal"].items()
    }
    result["total"] = sum(result.values())
    return result


def expected_cpp_operations(variant: str, goals: int) -> dict:
    native_value_route = variant in (
        "direct-source-compatible", "native-python-orchestrated")
    return {
        "known": variant != "stock-rclpy",
        "goal_shared_handoffs": goals if native_value_route else 0,
        "goal_id_materializations": goals if native_value_route else 0,
        "feedback_value_submissions": (
            goals * FEEDBACK_PER_GOAL if native_value_route else 0),
        "result_value_submissions": goals if native_value_route else 0,
        "adapter_message_deep_copies": (
            goals * (FEEDBACK_PER_GOAL + 1) if native_value_route else 0),
    }


def rotating_order(variants: list[str], repetition: int) -> list[str]:
    offset = (repetition - 1) % len(variants)
    return variants[offset:] + variants[:offset]


def _validate_rss(value: dict) -> None:
    required = {
        "kind": "post-warmup-peak-rss-growth",
        "unit": "bytes",
        "limit_bytes": RSS_LIMIT_BYTES,
        "within_limit": True,
    }
    if not isinstance(value, dict) or any(
            value.get(key) != expected for key, expected in required.items()):
        raise ValueError("action-server RSS guard policy is invalid")
    for key in ("baseline_peak_bytes", "final_peak_bytes", "growth_bytes"):
        if not _nonnegative_int(value.get(key)):
            raise ValueError("action-server RSS guard counter is invalid")
    growth = max(0, value["final_peak_bytes"] - value["baseline_peak_bytes"])
    if value["growth_bytes"] != growth or growth > RSS_LIMIT_BYTES:
        raise ValueError("action-server RSS guard growth is invalid")


def _validate_artifact(value: dict, *, cached: bool) -> None:
    if not isinstance(value, dict) or value.get("cached") is not cached:
        raise ValueError("action-server cache state is invalid")
    if not isinstance(value.get("path"), str) or not value["path"]:
        raise ValueError("action-server cache path is missing")
    if not SHA256_PATTERN.fullmatch(value.get("sha256", "")):
        raise ValueError("action-server cache digest is invalid")
    if not _positive_int(value.get("size_bytes")):
        raise ValueError("action-server cache size is invalid")


def validate_prewarm(value: dict, *, expect_hit: bool) -> None:
    if not isinstance(value, dict) or value.get("schema") != PREWARM_SCHEMA:
        raise ValueError("unsupported action-server prewarm schema")
    if not _positive_int(value.get("pid")) or value.get("loaded_rmw") != RMW:
        raise ValueError("action-server prewarm runtime identity is invalid")
    source_id = value.get("state_machine_source_id")
    if not isinstance(source_id, str) or re.fullmatch(r"[a-f0-9]{16}", source_id) is None:
        raise ValueError("action-server state-machine source identity is invalid")
    artifacts = value.get("artifacts")
    if not isinstance(artifacts, dict) or set(artifacts) != {
            "native_action_server", "state_machine"}:
        raise ValueError("action-server prewarm artifacts are incomplete")
    for artifact in artifacts.values():
        _validate_artifact(artifact, cached=expect_hit)
    diagnostics = value.get("stdout_diagnostics")
    if not isinstance(diagnostics, list) or any(
            not isinstance(item, str) for item in diagnostics):
        raise ValueError("action-server prewarm diagnostics are invalid")


def validate_cache(value: dict) -> None:
    required = (
        "isolated_root", "autopch_disabled", "fresh_process_per_phase",
        "compilation_excluded_from_samples", "warm_hits_verified",
    )
    if not isinstance(value, dict) or any(value.get(key) is not True for key in required):
        raise ValueError("action-server cache policy is incomplete")
    if value.get("persisted_after_run") is not False:
        raise ValueError("action-server cache must be temporary")
    phases = value.get("phases")
    if not isinstance(phases, dict) or set(phases) != {"cold", "warm"}:
        raise ValueError("action-server cache phases are invalid")
    validate_prewarm(phases["cold"], expect_hit=False)
    validate_prewarm(phases["warm"], expect_hit=True)
    if phases["cold"]["state_machine_source_id"] != phases["warm"][
            "state_machine_source_id"]:
        raise ValueError("action-server state-machine source identity changed")
    for name in phases["cold"]["artifacts"]:
        cold = phases["cold"]["artifacts"][name]
        warm = phases["warm"]["artifacts"][name]
        if tuple(cold[key] for key in ("path", "sha256", "size_bytes")) != tuple(
                warm[key] for key in ("path", "sha256", "size_bytes")):
            raise ValueError("action-server prewarm selected different artifacts")


def _identity(event: dict, sample: dict, event_name: str, role: str) -> None:
    schema = SERVER_SCHEMA if role == "server" else CLIENT_SCHEMA
    if not isinstance(event, dict) or event.get("schema") != schema or event.get(
            "event") != event_name:
        raise ValueError("action-server %s %s event is invalid" % (role, event_name))
    if event.get("variant") != sample["variant"] or event.get(
            "run_token") != sample["run_token"]:
        raise ValueError("action-server event identity changed")
    pid = sample["%s_pid" % role]
    if event.get("pid") != pid or event.get("process_group_id") != pid:
        raise ValueError("action-server process-group evidence is invalid")


def _validate_server_ready(event: dict, sample: dict, cache: dict) -> None:
    _identity(event, sample, "ready", "server")
    spec = VARIANTS[sample["variant"]]
    exact = {
        "node_name": sample["server_node"],
        "action_name": sample["action_name"],
        "loaded_rmw": RMW,
        "action_type": ACTION_TYPE,
        "execution_model": spec["execution_model"],
        "action_authority": spec["authority"],
        "action_implementation": spec["implementation"],
        "goal_representation": spec["representation"],
        "feedback_representation": spec["representation"],
        "result_representation": spec["representation"],
        "goal_id_representation": spec["representation"],
        "envelope_representation": spec["representation"],
        "qos": QOS,
        "endpoints": endpoint_names(sample["action_name"]),
    }
    if any(event.get(key) != expected for key, expected in exact.items()):
        raise ValueError("action-server READY contract is invalid")
    if event.get("executor") != {
            "authority": spec["authority"], "kind": "single_threaded", "threads": 1}:
        raise ValueError("action-server executor evidence is invalid")
    route = event.get("cache")
    if sample["variant"] == "stock-rclpy":
        if route != {"kind": "stock-rclpy", "state": "not_applicable"}:
            raise ValueError("stock action-server cache evidence is invalid")
    elif sample["variant"] == "aot-staged":
        if route != {"kind": "aot-binary", "state": "prebuilt"}:
            raise ValueError("AOT action-server cache evidence is invalid")
    else:
        name = (
            "state_machine" if sample["variant"] == "native-cpp-state-machine"
            else "native_action_server")
        expected = cache["phases"]["warm"]["artifacts"][name]
        if not isinstance(route, dict) or route.get("kind") != spec["cache_kind"] or route.get(
                "state") != "prebuilt" or route.get("hit") is not True:
            raise ValueError("native action-server cache route is invalid")
        if tuple(route.get(key) for key in ("path", "sha256", "size_bytes")) != tuple(
                expected[key] for key in ("path", "sha256", "size_bytes")):
            raise ValueError("native action-server selected the wrong artifact")


def _validate_server_report(event: dict, sample: dict) -> None:
    _identity(event, sample, "report", "server")
    total = WARMUP_GOALS + MEASURED_GOALS
    exact = {
        "warmup_goals": WARMUP_GOALS,
        "measured_goals": MEASURED_GOALS,
        "goals_received": total,
        "goals_accepted": total,
        "goals_rejected": 0,
        "feedback_sent": total * FEEDBACK_PER_GOAL,
        "results_sent": total,
        "terminal_succeeded": total,
        "warmup_checksum": expected_checksum(WARMUP_GOALS),
        "measured_checksum": expected_checksum(MEASURED_GOALS),
        "active_goals": 0,
        "pending_operations": 0,
        "exceptions": 0,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "cpu_role": "server_under_test",
        "teardown_clean": True,
        "python_crossings": expected_python_crossings(sample["variant"], total),
        "python_crossing_semantics": "callback_entries_only",
        "cpp_value_operations": expected_cpp_operations(sample["variant"], total),
    }
    if any(event.get(key) != expected for key, expected in exact.items()):
        raise ValueError("action-server REPORT contract is invalid")
    if not _positive_int(event.get("cpu_time_ns")):
        raise ValueError("action-server CPU evidence is invalid")
    _validate_rss(event.get("rss_guard"))
    boundary = event.get("boundary_evidence")
    spec = VARIANTS[sample["variant"]]
    if sample["variant"] in (
            "direct-source-compatible", "native-python-orchestrated",
            "native-cpp-state-machine"):
        expected_boundary = {
            "proof": "counter-backed-poison",
            "exact_generated_cpp": True,
            "python_message_conversions": 0,
            "python_serialization_calls": 0,
            "adapter_cdr_roundtrips": 0,
            "tripwires_armed": True,
            "tripwire_surfaces": list(BOUNDARY_TRIPWIRE_SURFACES),
        }
    elif sample["variant"] == "aot-staged":
        expected_boundary = {
            "proof": "cpp-only-process",
            "exact_generated_cpp": True,
            "python_message_conversions": 0,
            "python_serialization_calls": 0,
            "adapter_cdr_roundtrips": 0,
            "tripwires_armed": False,
            "tripwire_surfaces": [],
        }
    else:
        expected_boundary = {
            "proof": "python-message-lane",
            "exact_generated_cpp": spec["exact_cpp"],
            "python_message_conversions": None,
            "python_serialization_calls": None,
            "adapter_cdr_roundtrips": None,
            "tripwires_armed": False,
            "tripwire_surfaces": [],
        }
    if boundary != expected_boundary:
        raise ValueError("action-server representation boundary evidence is invalid")


def _validate_client(event: dict, sample: dict, report: bool) -> None:
    _identity(event, sample, "report" if report else "ready", "client")
    if not report:
        exact = {
            "node_name": sample["client_node"],
            "action_name": sample["action_name"],
            "loaded_rmw": RMW,
            "action_type": ACTION_TYPE,
            "execution_model": "conventional-release-aot-rclcpp-action-client",
            "action_authority": "cpp",
            "goal_representation": "cpp-message",
            "action_implementation": "rclcpp_action::Client<LookupTransform>",
            "qos": QOS,
            "endpoints": endpoint_names(sample["action_name"]),
            "executor": {"authority": "cpp", "kind": "single_threaded", "threads": 1},
            "cache": {"kind": "aot-binary", "state": "prebuilt"},
            "warmup_goals": WARMUP_GOALS,
            "warmup_checksum": expected_checksum(WARMUP_GOALS),
            "warmup_feedback": WARMUP_GOALS * FEEDBACK_PER_GOAL,
            "warmup_results": WARMUP_GOALS,
            "warmup_terminal_success": WARMUP_GOALS,
            "active_goals": 0,
            "pending_operations": 0,
        }
        if any(event.get(key) != expected for key, expected in exact.items()):
            raise ValueError("common AOT client READY contract is invalid")
        return
    total = WARMUP_GOALS + MEASURED_GOALS
    exact = {
        "warmup_goals": WARMUP_GOALS,
        "measured_goals": MEASURED_GOALS,
        "goals_sent": total,
        "goals_accepted": total,
        "goals_rejected": 0,
        "feedback_received": total * FEEDBACK_PER_GOAL,
        "feedback_dropped": 0,
        "results_received": total,
        "terminal_succeeded": total,
        "sequence_checksum": expected_checksum(MEASURED_GOALS),
        "last_sequence": MEASURED_GOALS,
        "active_goals": 0,
        "pending_operations": 0,
        "exceptions": 0,
        "python_crossings": {"goal": 0, "feedback": 0, "result": 0, "total": 0},
        "no_python_message_conversion": True,
        "boundary_evidence": {
            "proof": "cpp-only-process",
            "exact_generated_cpp": True,
            "tripwires_armed": False,
            "tripwire_surfaces": [],
            "python_message_conversions": 0,
            "python_serialization_calls": 0,
            "adapter_cdr_roundtrips": 0,
        },
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "orchestration_poll_count": 0,
        "teardown_clean": True,
        "executor_thread_joined": True,
    }
    if any(event.get(key) != expected for key, expected in exact.items()):
        raise ValueError("common AOT client correctness evidence is invalid")
    if not _positive_int(event.get("cpu_time_ns")) or not _positive_int(
            event.get("wall_duration_ns")):
        raise ValueError("common AOT client timing evidence is invalid")
    latency = event.get("latency_ns")
    if not isinstance(latency, dict) or set(latency) != {
            "send_to_accept", "send_to_first_feedback", "send_to_result"}:
        raise ValueError("common AOT client latency evidence is incomplete")
    for value in latency.values():
        _validate_percentiles(value)
    _validate_rss(event.get("rss_guard"))


def _validate_graph(value: dict) -> None:
    flags = (
        "server_present_after_ready", "client_present_after_ready",
        "exact_endpoints_present", "client_absent_after_exit",
        "server_absent_after_exit", "endpoints_absent_after_exit",
    )
    if not isinstance(value, dict) or any(value.get(key) is not True for key in flags):
        raise ValueError("action-server graph lifecycle evidence is incomplete")
    for key in ("ready_observation", "client_exit_observation", "final_observation"):
        observation = value.get(key)
        if not isinstance(observation, dict) or observation.get(
                "observed") is not True or not _positive_int(
                    observation.get("observations")) or not _nonnegative_int(
                        observation.get("elapsed_ns")):
            raise ValueError("action-server graph observation is invalid")


def validate_sample(sample: dict, cache: dict) -> None:
    if not isinstance(sample, dict) or sample.get("schema") != SAMPLE_SCHEMA:
        raise ValueError("unsupported action-server sample schema")
    variant = sample.get("variant")
    repetition = sample.get("repetition")
    if variant not in VARIANTS or not _is_int(repetition) or not 1 <= repetition <= REPETITIONS:
        raise ValueError("action-server sample identity is invalid")
    if sample.get("case_id") != "%s__rep_%d" % (variant, repetition):
        raise ValueError("action-server case identity is invalid")
    token = sample.get("run_token")
    if not isinstance(token, str) or TOKEN_PATTERN.fullmatch(token) is None:
        raise ValueError("action-server run token is invalid")
    expected_action = "/rclcppyy/action_server_benchmark/run_%s" % token[14:]
    if sample.get("action_name") != expected_action:
        raise ValueError("action-server name is invalid")
    suffix = token[14:26]
    if sample.get("server_node") != "action_server_%s" % suffix or sample.get(
            "client_node") != "action_client_%s" % suffix:
        raise ValueError("action-server node names are invalid")
    if sample.get("requested_rmw") != RMW or sample.get("ros_distro") != ROS_DISTRO:
        raise ValueError("action-server sample is not Jazzy/Cyclone")
    domain = sample.get("ros_domain_id")
    if not _is_int(domain) or not 0 <= domain <= 232:
        raise ValueError("action-server ROS domain is invalid")
    if not _positive_int(sample.get("server_pid")) or not _positive_int(
            sample.get("client_pid")) or sample["server_pid"] == sample["client_pid"]:
        raise ValueError("action-server process identities are invalid")
    topology = sample.get("topology")
    expected_topology = {
        "process_count": 2,
        "fresh_process_groups": True,
        "one_active_goal": True,
        "common_aot_client": True,
        "action_type": ACTION_TYPE,
        "qos": QOS,
        "endpoints": endpoint_names(sample["action_name"]),
    }
    if not isinstance(topology, dict) or topology != expected_topology:
        raise ValueError("action-server topology is invalid")
    _validate_graph(sample.get("graph"))
    _validate_server_ready(sample.get("server_ready"), sample, cache)
    _validate_client(sample.get("client_ready"), sample, False)
    _identity(sample.get("client_armed"), sample, "armed", "client")
    if sample["client_armed"] != {
        "schema": CLIENT_SCHEMA,
        "event": "armed",
        "variant": variant,
        "run_token": token,
        "pid": sample["client_pid"],
        "process_group_id": sample["client_pid"],
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "measurement_reset": True,
        "measurement_window_started": False,
        "protocol_emission_excluded": True,
    }:
        raise ValueError("common AOT client ARMED evidence is invalid")
    _validate_client(sample.get("client_report"), sample, True)
    _validate_server_report(sample.get("server_report"), sample)
    timing = sample.get("timing")
    cpu = sample["server_report"]["cpu_time_ns"] / MEASURED_GOALS
    wall = sample["client_report"]["wall_duration_ns"]
    rate = MEASURED_GOALS * 1e9 / wall
    if not isinstance(timing, dict) or timing.get("primary_metric") != (
            "server_cpu_ns_per_completed_goal") or not math.isclose(
                timing.get("server_cpu_ns_per_completed_goal", -1), cpu, rel_tol=1e-12):
        raise ValueError("action-server primary CPU metric is inconsistent")
    if not math.isclose(timing.get("completed_goals_per_second", -1), rate, rel_tol=1e-12):
        raise ValueError("action-server completion rate is inconsistent")
    if timing.get("latency_ns") != sample["client_report"].get("latency_ns"):
        raise ValueError("action-server client latency evidence changed")
    client_cpu = sample["client_report"]["cpu_time_ns"] / MEASURED_GOALS
    if not math.isclose(
            timing.get("client_cpu_diagnostic_ns_per_goal", -1),
            client_cpu,
            rel_tol=1e-12):
        raise ValueError("action-server client CPU diagnostic is inconsistent")
    if sample.get("correctness_verified") is not True or sample.get(
            "teardown_verified") is not True:
        raise ValueError("action-server verification flags are incomplete")


def validate_document(document: dict) -> None:
    if not isinstance(document, dict) or document.get("schema") != SCHEMA_ID:
        raise ValueError("unsupported action-server benchmark schema")
    if document.get("claims") != {"enabled": False, "reason": "characterization_only"}:
        raise ValueError("action-server performance claims must remain disabled")
    if document.get("interpretation") != {"enabled": False, "reason": "raw_evidence_only"}:
        raise ValueError("action-server interpretation must remain disabled")
    if document.get("mode") != "measurement" or not isinstance(
            document.get("environment"), dict):
        raise ValueError("action-server mode or environment is invalid")
    try:
        datetime.datetime.fromisoformat(document.get("generated_at"))
    except (TypeError, ValueError) as exc:
        raise ValueError("action-server timestamp is invalid") from exc
    command = document.get("command")
    if not isinstance(command, list) or not command or any(
            not isinstance(item, str) for item in command):
        raise ValueError("action-server command evidence is invalid")
    parameters = document.get("parameters")
    fixed = {
        "variants": list(VARIANTS),
        "warmup_goals": WARMUP_GOALS,
        "measured_goals": MEASURED_GOALS,
        "feedback_per_goal": FEEDBACK_PER_GOAL,
        "repetitions": REPETITIONS,
        "requested_rmw": RMW,
        "ros_distro": ROS_DISTRO,
        "action_type": ACTION_TYPE,
        "qos": QOS,
        "primary_metric": "server_cpu_ns_per_completed_goal",
        "common_driver": "conventional-release-aot-rclcpp-action-client",
    }
    if not isinstance(parameters, dict) or any(
            parameters.get(key) != expected for key, expected in fixed.items()):
        raise ValueError("action-server fixed parameters changed")
    expected_order = [
        "%s__rep_%d" % (variant, repetition)
        for repetition in range(1, REPETITIONS + 1)
        for variant in rotating_order(list(VARIANTS), repetition)
    ]
    if parameters.get("execution_order") != expected_order:
        raise ValueError("action-server execution order is invalid")
    isolation = document.get("isolation")
    required_isolation = (
        "fresh_process_pair_per_sample", "fresh_process_groups_per_sample",
        "unique_action_and_nodes_per_sample", "one_leased_domain_per_run",
        "rotating_variant_order",
    )
    if not isinstance(isolation, dict) or any(
            isolation.get(key) is not True for key in required_isolation):
        raise ValueError("action-server isolation evidence is incomplete")
    domain = isolation.get("ros_domain_id")
    if not _is_int(domain) or not 0 <= domain <= 232:
        raise ValueError("action-server leased domain is invalid")
    validate_cache(document.get("cache"))
    validate_build(document.get("aot_build"))
    sources = document.get("source_files")
    if not isinstance(sources, dict) or set(sources) != {
            "runner", "protocol", "worker", "aot_server", "aot_client", "aot_cmake"}:
        raise ValueError("action-server source evidence is incomplete")
    if any(not SHA256_PATTERN.fullmatch(value) for value in sources.values()):
        raise ValueError("action-server source digest is invalid")
    results = document.get("results")
    failures = document.get("failures")
    if not isinstance(results, list) or not isinstance(failures, list):
        raise ValueError("action-server result collections are invalid")
    cases = {}
    for sample in results:
        validate_sample(sample, document["cache"])
        if sample["case_id"] in cases:
            raise ValueError("duplicate action-server sample")
        cases[sample["case_id"]] = True
    for failure in failures:
        if not isinstance(failure, dict) or set(failure) != {
                "case_id", "variant", "repetition", "error"}:
            raise ValueError("action-server failure evidence is invalid")
        case_id = "%s__rep_%d" % (failure["variant"], failure["repetition"])
        if failure["case_id"] != case_id or case_id in cases or not failure["error"]:
            raise ValueError("action-server failure identity is invalid")
        cases[case_id] = True
    if set(cases) != set(expected_order):
        raise ValueError("action-server document does not cover the fixed matrix")


def build_document(
    *, repo_root: Path, parameters: dict, isolation: dict, aot_build: dict,
    cache: dict, source_files: dict, results: list, failures: list, command: list,
) -> dict:
    document = {
        "schema": SCHEMA_ID,
        "generated_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "mode": "measurement",
        "command": command,
        "environment": environment_metadata(repo_root),
        "parameters": parameters,
        "isolation": isolation,
        "aot_build": aot_build,
        "cache": cache,
        "source_files": source_files,
        "results": results,
        "failures": failures,
        "claims": {"enabled": False, "reason": "characterization_only"},
        "interpretation": {"enabled": False, "reason": "raw_evidence_only"},
    }
    validate_document(document)
    return document


def dumps(document: dict) -> str:
    validate_document(document)
    return json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n"


def write(document: dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(dumps(document), encoding="utf-8")
