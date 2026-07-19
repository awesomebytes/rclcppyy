"""Strict evidence contract for the Jazzy/Cyclone action-client benchmark."""

from __future__ import annotations

import datetime
import json
import math
from pathlib import Path
import re

from _result_schema import environment_metadata


SCHEMA_ID = "rclcppyy.action-client-benchmark/v1"
SAMPLE_SCHEMA = "rclcppyy.action-client-sample/v1"
CLIENT_SCHEMA = "rclcppyy.action-client-event/v1"
SERVER_SCHEMA = "rclcppyy.action-server-event/v1"
PREWARM_SCHEMA = "rclcppyy.action-client-prewarm/v1"
RMW = "rmw_cyclonedds_cpp"
ROS_DISTRO = "jazzy"
ACTION_TYPE = "tf2_msgs/action/LookupTransform"
WARMUP_GOALS = 20
MEASURED_GOALS = 500
FEEDBACK_PER_GOAL = 3
REPETITIONS = 5
RSS_LIMIT_BYTES = 128 * 1024 * 1024
SHA256_PATTERN = re.compile(r"^[a-f0-9]{64}$")
TOKEN_PATTERN = re.compile(r"^action_[a-f0-9]{32}$")

QOS = {
    "goal_service": {
        "history": "keep_last", "depth": 10,
        "reliability": "reliable", "durability": "volatile",
    },
    "result_service": {
        "history": "keep_last", "depth": 10,
        "reliability": "reliable", "durability": "volatile",
    },
    "cancel_service": {
        "history": "keep_last", "depth": 10,
        "reliability": "reliable", "durability": "volatile",
    },
    "feedback_topic": {
        "history": "system_default", "depth": 0,
        "reliability": "system_default", "durability": "system_default",
    },
    "status_topic": {
        "history": "keep_last", "depth": 1,
        "reliability": "reliable", "durability": "transient_local",
    },
}

VARIANTS = {
    "stock-rclpy": {
        "execution_model": "stock-rclpy-action-client-python-goals",
        "action_authority": "python",
        "action_implementation": "rclpy.action.client.ActionClient",
        "goal_representation": "python-message",
        "cache_kind": "stock-rclpy",
        "python_goal_crossings_per_goal": 1,
        "python_feedback_crossings_per_goal": FEEDBACK_PER_GOAL,
        "python_result_crossings_per_goal": 1,
        "no_python_message_conversion": False,
    },
    "compatible-rclcppyy": {
        "execution_model": "compatible-activation-stock-python-action-client",
        "action_authority": "python",
        "action_implementation": "rclpy.action.client.ActionClient",
        "goal_representation": "python-message",
        "cache_kind": "activation-only",
        "python_goal_crossings_per_goal": 1,
        "python_feedback_crossings_per_goal": FEEDBACK_PER_GOAL,
        "python_result_crossings_per_goal": 1,
        "no_python_message_conversion": False,
    },
    "direct-source-compatible": {
        "execution_model": "direct-rclcpp-action-client-source-compatible-control",
        "action_authority": "cpp",
        "action_implementation": "rclcppyy.direct_actions.DirectActionClient",
        "goal_representation": "cpp-message",
        "cache_kind": "native-action-client-shared-library",
        "python_goal_crossings_per_goal": 1,
        "python_feedback_crossings_per_goal": FEEDBACK_PER_GOAL,
        "python_result_crossings_per_goal": 1,
        "no_python_message_conversion": True,
    },
    "native-python-orchestrated": {
        "execution_model": "managed-rclcpp-action-client-python-orchestration",
        "action_authority": "cpp",
        "action_implementation": "rclcpp_kit.native_action.NativeActionClient",
        "goal_representation": "cpp-message",
        "cache_kind": "native-action-client-shared-library",
        "python_goal_crossings_per_goal": 1,
        "python_feedback_crossings_per_goal": FEEDBACK_PER_GOAL,
        "python_result_crossings_per_goal": 1,
        "no_python_message_conversion": True,
    },
    "native-cpp-state-machine": {
        "execution_model": "content-addressed-cpp-action-state-machine",
        "action_authority": "cpp",
        "action_implementation": "rclcppyy_action_benchmark::StateMachine",
        "goal_representation": "cpp-message",
        "cache_kind": "action-state-machine-shared-library",
        "python_goal_crossings_per_goal": 0,
        "python_feedback_crossings_per_goal": 0,
        "python_result_crossings_per_goal": 0,
        "no_python_message_conversion": True,
    },
    "aot-staged": {
        "execution_model": "conventional-release-aot-rclcpp-action-client",
        "action_authority": "cpp",
        "action_implementation": "rclcpp_action::Client<LookupTransform>",
        "goal_representation": "cpp-message",
        "cache_kind": "aot-binary",
        "python_goal_crossings_per_goal": 0,
        "python_feedback_crossings_per_goal": 0,
        "python_result_crossings_per_goal": 0,
        "no_python_message_conversion": True,
    },
}


def _is_int(value) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _positive_int(value) -> bool:
    return _is_int(value) and value > 0


def _nonnegative_int(value) -> bool:
    return _is_int(value) and value >= 0


def _finite_number(value) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def goal_strings(phase: str, sequence: int) -> tuple[str, str]:
    if phase not in ("warmup", "measured") or not _positive_int(sequence):
        raise ValueError("invalid action goal identity")
    base = "rclcppyy/action-benchmark/%s/%d" % (phase, sequence)
    return base + "/target", base + "/source"


def expected_checksum(goals: int) -> int:
    if not _nonnegative_int(goals):
        raise ValueError("goal count must be non-negative")
    return goals * (goals + 1) // 2


def nearest_rank(values: list[int], percentile: int) -> int:
    if not values:
        raise ValueError("action latency observations cannot be empty")
    ordered = sorted(values)
    rank = max(1, math.ceil(percentile / 100.0 * len(ordered)))
    return ordered[rank - 1]


def latency_summary(values: list[int]) -> dict[str, int]:
    if any(not _nonnegative_int(value) for value in values):
        raise ValueError("action latency observations must be non-negative integers")
    return {
        "p50": nearest_rank(values, 50),
        "p95": nearest_rank(values, 95),
        "p99": nearest_rank(values, 99),
        "max": max(values),
    }


def endpoint_names(action_name: str) -> dict[str, str]:
    return {
        "send_goal": action_name + "/_action/send_goal",
        "get_result": action_name + "/_action/get_result",
        "cancel_goal": action_name + "/_action/cancel_goal",
        "feedback": action_name + "/_action/feedback",
        "status": action_name + "/_action/status",
    }


def expected_crossings(variant: str, goals: int) -> dict[str, int]:
    spec = VARIANTS[variant]
    result = {
        "goal": goals * spec["python_goal_crossings_per_goal"],
        "feedback": goals * spec["python_feedback_crossings_per_goal"],
        "result": goals * spec["python_result_crossings_per_goal"],
    }
    result["total"] = sum(result.values())
    return result


def _validate_percentiles(value: dict) -> None:
    if not isinstance(value, dict) or set(value) != {"p50", "p95", "p99", "max"}:
        raise ValueError("action latency percentile fields are invalid")
    if any(not _nonnegative_int(item) for item in value.values()):
        raise ValueError("action latency percentiles must be non-negative integers")
    if not value["p50"] <= value["p95"] <= value["p99"] <= value["max"]:
        raise ValueError("action latency percentiles are not ordered")


def _validate_rss(value: dict) -> None:
    required = {
        "kind": "post-warmup-peak-rss-growth",
        "unit": "bytes",
        "limit_bytes": RSS_LIMIT_BYTES,
        "within_limit": True,
    }
    if not isinstance(value, dict) or any(value.get(key) != item for key, item in required.items()):
        raise ValueError("action RSS guard policy is invalid")
    for key in ("baseline_peak_bytes", "final_peak_bytes", "growth_bytes"):
        if not _nonnegative_int(value.get(key)):
            raise ValueError("action RSS guard counter is invalid")
    expected = max(0, value["final_peak_bytes"] - value["baseline_peak_bytes"])
    if value["growth_bytes"] != expected or expected > RSS_LIMIT_BYTES:
        raise ValueError("action RSS guard growth is invalid")


def _validate_artifact(value: dict, *, cached: bool) -> None:
    if not isinstance(value, dict) or value.get("cached") is not cached:
        raise ValueError("action cache hit state is invalid")
    if not isinstance(value.get("path"), str) or not value["path"]:
        raise ValueError("action cache artifact path is missing")
    if not isinstance(value.get("sha256"), str) or not SHA256_PATTERN.fullmatch(
            value["sha256"]):
        raise ValueError("action cache artifact digest is invalid")
    if not _positive_int(value.get("size_bytes")):
        raise ValueError("action cache artifact size is invalid")


def validate_prewarm(value: dict, *, expect_hit: bool) -> None:
    if not isinstance(value, dict) or value.get("schema") != PREWARM_SCHEMA:
        raise ValueError("unsupported action prewarm schema")
    if not _positive_int(value.get("pid")) or value.get("loaded_rmw") != RMW:
        raise ValueError("action prewarm runtime identity is invalid")
    if not isinstance(value.get("state_machine_source_id"), str) or re.fullmatch(
            r"[a-f0-9]{16}", value["state_machine_source_id"]) is None:
        raise ValueError("action state-machine source identity is invalid")
    artifacts = value.get("artifacts")
    if not isinstance(artifacts, dict) or set(artifacts) != {
            "native_action_client", "state_machine"}:
        raise ValueError("action prewarm artifact set is invalid")
    for artifact in artifacts.values():
        _validate_artifact(artifact, cached=expect_hit)


def validate_cache(value: dict) -> None:
    required_true = (
        "isolated_root", "autopch_disabled", "fresh_process_per_phase",
        "compilation_excluded_from_samples", "warm_hits_verified",
    )
    if not isinstance(value, dict) or any(value.get(key) is not True for key in required_true):
        raise ValueError("action cache policy evidence is incomplete")
    if value.get("persisted_after_run") is not False:
        raise ValueError("action cache must be temporary")
    phases = value.get("phases")
    if not isinstance(phases, dict) or set(phases) != {"cold", "warm"}:
        raise ValueError("action cache requires exact cold and warm phases")
    validate_prewarm(phases["cold"], expect_hit=False)
    validate_prewarm(phases["warm"], expect_hit=True)
    cold = phases["cold"]
    warm = phases["warm"]
    if cold["state_machine_source_id"] != warm["state_machine_source_id"]:
        raise ValueError("action state-machine source identity changed")
    for name in cold["artifacts"]:
        left = cold["artifacts"][name]
        right = warm["artifacts"][name]
        if tuple(left[key] for key in ("path", "sha256", "size_bytes")) != tuple(
                right[key] for key in ("path", "sha256", "size_bytes")):
            raise ValueError("action cold and warm phases selected different artifacts")


def validate_build(value: dict) -> None:
    if not isinstance(value, dict) or value.get("build_type") != "Release":
        raise ValueError("action AOT build must be Release")
    if value.get("private_build_directory") is not True or value.get(
            "build_directory_persisted") is not False:
        raise ValueError("action AOT build isolation is invalid")
    commands = value.get("compile_commands")
    if not isinstance(commands, dict) or set(commands) != {"server", "client"}:
        raise ValueError("action AOT compile commands are incomplete")
    if any(
            not isinstance(command, str)
            or "-O3" not in command
            or "-DNDEBUG" not in command
            for command in commands.values()):
        raise ValueError("action AOT build does not prove Release optimization")
    executables = value.get("executables")
    if not isinstance(executables, dict) or set(executables) != {"server", "client"}:
        raise ValueError("action AOT executable evidence is incomplete")
    for artifact in executables.values():
        if artifact.get("format") != "ELF" or not SHA256_PATTERN.fullmatch(
                artifact.get("sha256", "")):
            raise ValueError("action AOT executable identity is invalid")
    for key in ("server_source_sha256", "client_source_sha256", "cmake_sha256"):
        if not SHA256_PATTERN.fullmatch(value.get(key, "")):
            raise ValueError("action AOT source identity is invalid")
    if not _positive_int(value.get("build_elapsed_ns")):
        raise ValueError("action AOT build duration is invalid")


def _validate_identity(event: dict, sample: dict, *, expected_event: str, role: str) -> None:
    schema = CLIENT_SCHEMA if role == "client" else SERVER_SCHEMA
    pid_key = "%s_pid" % role
    if not isinstance(event, dict) or event.get("schema") != schema or event.get(
            "event") != expected_event:
        raise ValueError("action %s %s evidence is invalid" % (role, expected_event))
    for key in ("variant", "run_token"):
        if event.get(key) != sample[key]:
            raise ValueError("action %s identity is invalid" % role)
    if event.get("pid") != sample[pid_key] or event.get("process_group_id") != sample[pid_key]:
        raise ValueError("action %s process-group evidence is invalid" % role)


def _validate_client_ready(event: dict, sample: dict, cache: dict) -> None:
    _validate_identity(event, sample, expected_event="ready", role="client")
    variant = sample["variant"]
    spec = VARIANTS[variant]
    exact = {
        "node_name": sample["client_node"],
        "action_name": sample["action_name"],
        "loaded_rmw": RMW,
        "action_type": ACTION_TYPE,
        "execution_model": spec["execution_model"],
        "action_authority": spec["action_authority"],
        "action_implementation": spec["action_implementation"],
        "goal_representation": spec["goal_representation"],
        "qos": QOS,
        "endpoints": endpoint_names(sample["action_name"]),
        "warmup_goals": WARMUP_GOALS,
        "warmup_checksum": expected_checksum(WARMUP_GOALS),
        "warmup_feedback": WARMUP_GOALS * FEEDBACK_PER_GOAL,
        "warmup_results": WARMUP_GOALS,
        "warmup_terminal_success": WARMUP_GOALS,
        "active_goals": 0,
        "pending_operations": 0,
    }
    if any(event.get(key) != value for key, value in exact.items()):
        raise ValueError("action client READY contract is invalid")
    executor = event.get("executor")
    if not isinstance(executor, dict) or executor.get("kind") != "single_threaded" or executor.get(
            "threads") != 1 or executor.get("authority") != spec["action_authority"]:
        raise ValueError("action client executor marker is invalid")
    route_cache = event.get("cache")
    if not isinstance(route_cache, dict) or route_cache.get("kind") != spec["cache_kind"]:
        raise ValueError("action client cache marker is invalid")
    if variant == "compatible-rclcppyy":
        if route_cache != {"kind": "activation-only", "state": "activation-only"} or event.get(
                "activation") != {"profile": "compatible", "action_authority": "python"}:
            raise ValueError("compatible action lane must remain activation-only")
    elif variant == "direct-source-compatible":
        expected = cache["phases"]["warm"]["artifacts"]["native_action_client"]
        _validate_route_artifact(route_cache, expected)
        if event.get("activation") != {
            "profile": "direct_cpp",
            "action_authority": "cpp",
            "representations": "actual_cpp",
        }:
            raise ValueError("direct action lane activation marker is invalid")
    elif variant == "native-python-orchestrated":
        expected = cache["phases"]["warm"]["artifacts"]["native_action_client"]
        _validate_route_artifact(route_cache, expected)
    elif variant == "native-cpp-state-machine":
        expected = cache["phases"]["warm"]["artifacts"]["state_machine"]
        _validate_route_artifact(route_cache, expected)
    elif variant == "aot-staged":
        if route_cache != {"kind": "aot-binary", "state": "prebuilt"}:
            raise ValueError("action AOT cache marker is invalid")
    elif route_cache != {"kind": "stock-rclpy", "state": "not_applicable"}:
        raise ValueError("stock action cache marker is invalid")


def _validate_route_artifact(route: dict, expected: dict) -> None:
    if route.get("state") != "prebuilt" or route.get("hit") is not True:
        raise ValueError("native action route must use a prebuilt cache hit")
    if tuple(route.get(key) for key in ("path", "sha256", "size_bytes")) != tuple(
            expected[key] for key in ("path", "sha256", "size_bytes")):
        raise ValueError("native action route selected the wrong artifact")


def _validate_client_report(event: dict, sample: dict) -> None:
    _validate_identity(event, sample, expected_event="report", role="client")
    variant = sample["variant"]
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
        "python_crossings": expected_crossings(variant, total),
        "no_python_message_conversion": VARIANTS[variant]["no_python_message_conversion"],
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "teardown_clean": True,
        "executor_thread_joined": True,
    }
    if any(event.get(key) != value for key, value in exact.items()):
        raise ValueError("action client report violates the exact contract")
    if not _positive_int(event.get("cpu_time_ns")) or not _positive_int(
            event.get("wall_duration_ns")):
        raise ValueError("action client timing evidence is invalid")
    latencies = event.get("latency_ns")
    if not isinstance(latencies, dict) or set(latencies) != {
            "send_to_accept", "send_to_first_feedback", "send_to_result"}:
        raise ValueError("action client latency evidence is incomplete")
    for value in latencies.values():
        _validate_percentiles(value)
    _validate_rss(event.get("rss_guard"))
    if not _nonnegative_int(event.get("orchestration_poll_count")):
        raise ValueError("action orchestration polling diagnostic is invalid")


def _validate_server_ready(event: dict, sample: dict) -> None:
    _validate_identity(event, sample, expected_event="ready", role="server")
    exact = {
        "node_name": sample["server_node"],
        "action_name": sample["action_name"],
        "loaded_rmw": RMW,
        "action_type": ACTION_TYPE,
        "action_authority": "cpp",
        "qos": QOS,
        "endpoints": endpoint_names(sample["action_name"]),
        "executor": {"authority": "cpp", "kind": "single_threaded", "threads": 1},
    }
    if any(event.get(key) != value for key, value in exact.items()):
        raise ValueError("action server READY contract is invalid")


def _validate_server_report(event: dict, sample: dict) -> None:
    _validate_identity(event, sample, expected_event="report", role="server")
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
        "cpu_role": "drift_diagnostic_only",
        "teardown_clean": True,
    }
    if any(event.get(key) != value for key, value in exact.items()):
        raise ValueError("action server report violates the exact contract")
    if not _positive_int(event.get("cpu_time_ns")):
        raise ValueError("action server CPU diagnostic is invalid")
    _validate_rss(event.get("rss_guard"))


def _validate_graph(graph: dict) -> None:
    flags = (
        "server_present_after_ready", "client_present_after_ready",
        "exact_endpoints_present", "client_absent_after_exit",
        "server_absent_after_exit", "endpoints_absent_after_exit",
    )
    if not isinstance(graph, dict) or any(graph.get(key) is not True for key in flags):
        raise ValueError("action graph lifecycle evidence is incomplete")
    for key in ("ready_observation", "client_exit_observation", "final_observation"):
        value = graph.get(key)
        if not isinstance(value, dict) or value.get("observed") is not True or not _positive_int(
                value.get("observations")) or not _nonnegative_int(value.get("elapsed_ns")):
            raise ValueError("action graph observation is invalid")


def validate_sample(sample: dict, cache: dict, build: dict) -> None:
    if not isinstance(sample, dict) or sample.get("schema") != SAMPLE_SCHEMA:
        raise ValueError("unsupported action sample schema")
    variant = sample.get("variant")
    repetition = sample.get("repetition")
    if variant not in VARIANTS or not _is_int(repetition) or not 1 <= repetition <= REPETITIONS:
        raise ValueError("action sample variant or repetition is invalid")
    if sample.get("case_id") != "%s__rep_%d" % (variant, repetition):
        raise ValueError("action sample case identity is invalid")
    token = sample.get("run_token")
    if not isinstance(token, str) or TOKEN_PATTERN.fullmatch(token) is None:
        raise ValueError("action sample token is invalid")
    expected_action = "/rclcppyy/action_benchmark/run_%s" % token[7:]
    if sample.get("action_name") != expected_action:
        raise ValueError("action sample name is invalid")
    if sample.get("client_node") != "action_client_%s" % token[7:19] or sample.get(
            "server_node") != "action_server_%s" % token[7:19]:
        raise ValueError("action sample node names are invalid")
    if sample.get("requested_rmw") != RMW or sample.get("ros_distro") != ROS_DISTRO:
        raise ValueError("action sample runtime is not Jazzy/Cyclone")
    domain = sample.get("ros_domain_id")
    if not _is_int(domain) or not 0 <= domain <= 232:
        raise ValueError("action sample ROS domain is invalid")
    if not _positive_int(sample.get("server_pid")) or not _positive_int(
            sample.get("client_pid")) or sample["server_pid"] == sample["client_pid"]:
        raise ValueError("action sample process identities are invalid")
    topology = sample.get("topology")
    expected_topology = {
        "process_count": 2,
        "fresh_process_groups": True,
        "one_active_goal": True,
        "common_aot_server": True,
        "action_type": ACTION_TYPE,
        "qos": QOS,
        "endpoints": endpoint_names(sample["action_name"]),
    }
    if not isinstance(topology, dict) or topology != expected_topology:
        raise ValueError("action sample topology is invalid")
    _validate_graph(sample.get("graph"))
    _validate_server_ready(sample.get("server_ready"), sample)
    _validate_client_ready(sample.get("client_ready"), sample, cache)
    _validate_identity(sample.get("client_armed"), sample, expected_event="armed", role="client")
    if sample["client_armed"] != {
        "schema": CLIENT_SCHEMA,
        "event": "armed",
        "variant": variant,
        "run_token": token,
        "pid": sample["client_pid"],
        "process_group_id": sample["client_pid"],
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "measurement_reset": True,
    }:
        raise ValueError("action client ARMED evidence is invalid")
    _validate_client_report(sample.get("client_report"), sample)
    _validate_server_report(sample.get("server_report"), sample)
    timing = sample.get("timing")
    report = sample["client_report"]
    expected_cpu = report["cpu_time_ns"] / MEASURED_GOALS
    expected_rate = MEASURED_GOALS * 1e9 / report["wall_duration_ns"]
    if not isinstance(timing, dict) or not math.isclose(
            timing.get("client_cpu_ns_per_completed_goal", -1), expected_cpu, rel_tol=1e-12):
        raise ValueError("action primary CPU metric is inconsistent")
    if not math.isclose(timing.get("completed_goals_per_second", -1), expected_rate, rel_tol=1e-12):
        raise ValueError("action completion-rate metric is inconsistent")
    if timing.get("latency_ns") != report["latency_ns"]:
        raise ValueError("action latency summary differs from client evidence")
    diagnostic = sample.get("server_cpu_diagnostic")
    if diagnostic != {
        "role": "drift_only",
        "cpu_time_ns": sample["server_report"]["cpu_time_ns"],
        "cpu_ns_per_measured_goal": (
            sample["server_report"]["cpu_time_ns"] / MEASURED_GOALS),
    }:
        raise ValueError("action server CPU diagnostic is inconsistent")
    if sample.get("correctness_verified") is not True or sample.get(
            "teardown_verified") is not True:
        raise ValueError("action sample verification flags are incomplete")
    if variant == "aot-staged":
        validate_build(build)


def rotating_order(variants: list[str], repetition: int) -> list[str]:
    offset = (repetition - 1) % len(variants)
    return variants[offset:] + variants[:offset]


def validate_document(document: dict) -> None:
    if not isinstance(document, dict) or document.get("schema") != SCHEMA_ID:
        raise ValueError("unsupported action benchmark schema")
    if document.get("claims") != {"enabled": False, "reason": "characterization_only"}:
        raise ValueError("action benchmark claims must remain disabled")
    if document.get("interpretation") != {"enabled": False, "reason": "raw_evidence_only"}:
        raise ValueError("action benchmark interpretation must remain disabled")
    if document.get("mode") != "measurement" or not isinstance(document.get("environment"), dict):
        raise ValueError("action benchmark mode or environment is invalid")
    try:
        datetime.datetime.fromisoformat(document.get("generated_at"))
    except (TypeError, ValueError) as exc:
        raise ValueError("action benchmark timestamp is invalid") from exc
    command = document.get("command")
    if not isinstance(command, list) or not command or any(not isinstance(item, str) for item in command):
        raise ValueError("action benchmark command evidence is invalid")
    parameters = document.get("parameters")
    expected_parameters = {
        "variants": list(VARIANTS),
        "warmup_goals": WARMUP_GOALS,
        "measured_goals": MEASURED_GOALS,
        "feedback_per_goal": FEEDBACK_PER_GOAL,
        "repetitions": REPETITIONS,
        "requested_rmw": RMW,
        "ros_distro": ROS_DISTRO,
        "action_type": ACTION_TYPE,
        "qos": QOS,
    }
    if not isinstance(parameters, dict) or any(
            parameters.get(key) != value for key, value in expected_parameters.items()):
        raise ValueError("action benchmark parameters differ from the fixed contract")
    expected_order = [
        "%s__rep_%d" % (variant, repetition)
        for repetition in range(1, REPETITIONS + 1)
        for variant in rotating_order(list(VARIANTS), repetition)
    ]
    if parameters.get("execution_order") != expected_order:
        raise ValueError("action benchmark execution order is invalid")
    isolation = document.get("isolation")
    required_isolation = (
        "fresh_process_pair_per_sample", "fresh_process_groups_per_sample",
        "unique_action_and_nodes_per_sample", "one_leased_domain_per_run",
        "rotating_variant_order",
    )
    if not isinstance(isolation, dict) or any(isolation.get(key) is not True for key in required_isolation):
        raise ValueError("action benchmark isolation evidence is incomplete")
    if not _is_int(isolation.get("ros_domain_id")) or not 0 <= isolation["ros_domain_id"] <= 232:
        raise ValueError("action benchmark leased domain is invalid")
    validate_cache(document.get("cache"))
    validate_build(document.get("aot_build"))
    sources = document.get("source_files")
    expected_sources = {
        "runner", "protocol", "worker", "server_source", "client_source", "aot_cmake"}
    if not isinstance(sources, dict) or set(sources) != expected_sources or any(
            not isinstance(value, str) or not SHA256_PATTERN.fullmatch(value)
            for value in sources.values()):
        raise ValueError("action benchmark source identity is invalid")
    results = document.get("results")
    failures = document.get("failures")
    if not isinstance(results, list) or not isinstance(failures, list):
        raise ValueError("action result and failure collections are required")
    cases = {}
    for sample in results:
        validate_sample(sample, document["cache"], document["aot_build"])
        if sample["case_id"] in cases:
            raise ValueError("duplicate action sample case")
        cases[sample["case_id"]] = "result"
    for failure in failures:
        variant = failure.get("variant") if isinstance(failure, dict) else None
        repetition = failure.get("repetition") if isinstance(failure, dict) else None
        valid = variant in VARIANTS and _is_int(repetition) and 1 <= repetition <= REPETITIONS
        expected_case = "%s__rep_%d" % (variant, repetition) if valid else None
        if (
            not isinstance(failure, dict)
            or set(failure) != {"case_id", "variant", "repetition", "error"}
            or failure.get("case_id") != expected_case
            or not isinstance(failure.get("error"), str)
            or not failure["error"]
            or failure["case_id"] in cases
        ):
            raise ValueError("invalid or duplicate action failure case")
        cases[failure["case_id"]] = "failure"
    if set(cases) != set(expected_order):
        raise ValueError("action benchmark does not cover the exact sample matrix")


def build_document(
    *, repo_root: Path, parameters: dict, isolation: dict, aot_build: dict,
    cache: dict, source_files: dict, results: list[dict], failures: list[dict],
    command: list[str],
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
