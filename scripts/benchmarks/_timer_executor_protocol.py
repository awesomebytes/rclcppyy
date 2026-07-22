"""Strict evidence contract for the Jazzy/Cyclone timer-executor benchmark."""

from __future__ import annotations

import datetime
import json
import math
from pathlib import Path
import re
import statistics
import sys

from _result_schema import environment_metadata


SCHEMA_ID = "rclcppyy.timer-executor-benchmark/v1"
SAMPLE_SCHEMA = "rclcppyy.timer-executor-sample/v1"
EVENT_SCHEMA = "rclcppyy.timer-executor-event/v1"
PREWARM_SCHEMA = "rclcppyy.timer-executor-prewarm/v1"
RMW = "rmw_cyclonedds_cpp"
ROS_DISTRO = "jazzy"
PERIOD_NS = 1_000_000
WARMUP_FIRINGS = 500
MEASURED_FIRINGS = 5_000
REPETITIONS = 10
MTE_THREADS = 2
REGRESSION_REQUIRED_PAIRS = 10
REGRESSION_CPU_RATIO_LIMIT = 1.03
REGRESSION_VARIANTS = ("direct-public-ste", "direct-raw-ste-control")
MASK64 = (1 << 64) - 1
RECURRENCE_MULTIPLIER = 6_364_136_223_846_793_005
RECURRENCE_INCREMENT = 1_442_695_040_888_963_407
RECURRENCE_SEED = 0xC0DEC0FFEE123456
SHA256_PATTERN = re.compile(r"^[a-f0-9]{64}$")
COMMIT_PATTERN = re.compile(r"^[a-f0-9]{40}$")
ENTITY_DECISION_PATTERN = re.compile(r"^entity-[0-9]{8}$")

VARIANTS = {
    "stock-rclpy": {
        "execution_model": "stock-rclpy-steady-timer-python-callback",
        "timer_authority": "python",
        "executor_authority": "python",
        "callback_language": "python",
        "cache_kind": "stock-rclpy",
        "python_crossings_per_firing": 1,
        "executor_kind": "single_threaded",
        "executor_threads": 1,
    },
    "compatible-rclcppyy": {
        "execution_model": "compatible-activation-stock-python-timer-executor",
        "timer_authority": "python",
        "executor_authority": "python",
        "callback_language": "python",
        "cache_kind": "activation-only",
        "python_crossings_per_firing": 1,
        "executor_kind": "single_threaded",
        "executor_threads": 1,
    },
    "direct-cpp-rclcppyy": {
        "execution_model": "direct-rclcpp-wall-timer-python-callback",
        "timer_authority": "cpp",
        "executor_authority": "cpp",
        "callback_language": "python",
        "cache_kind": "direct-rclcpp-runtime",
        "python_crossings_per_firing": 1,
        "executor_surface": "rclpy-spin-once",
        "executor_kind": "single_threaded",
        "executor_threads": 1,
    },
    "direct-public-ste": {
        "execution_model": "direct-rclcpp-wall-timer-public-ste-python-callback",
        "timer_authority": "cpp",
        "executor_authority": "cpp",
        "callback_language": "python",
        "cache_kind": "direct-rclcpp-runtime",
        "python_crossings_per_firing": 1,
        "executor_surface": "rclpy-public-single-threaded-executor",
        "executor_kind": "single_threaded",
        "executor_threads": 1,
    },
    "direct-raw-ste-control": {
        "execution_model": "direct-rclcpp-wall-timer-raw-ste-python-callback",
        "timer_authority": "cpp",
        "executor_authority": "cpp",
        "callback_language": "python",
        "cache_kind": "direct-rclcpp-runtime",
        "python_crossings_per_firing": 1,
        "executor_surface": "native-session-raw-single-threaded-executor",
        "executor_kind": "single_threaded",
        "executor_threads": 1,
    },
    "direct-public-mte": {
        "execution_model": "direct-rclcpp-wall-timer-public-mte-python-callback",
        "timer_authority": "cpp",
        "executor_authority": "cpp",
        "callback_language": "python",
        "cache_kind": "direct-rclcpp-runtime",
        "python_crossings_per_firing": 1,
        "executor_surface": "rclpy-public-multi-threaded-executor",
        "executor_kind": "multi_threaded",
        "executor_threads": MTE_THREADS,
    },
    "direct-raw-mte-control": {
        "execution_model": "direct-rclcpp-wall-timer-raw-mte-python-callback",
        "timer_authority": "cpp",
        "executor_authority": "cpp",
        "callback_language": "python",
        "cache_kind": "direct-rclcpp-runtime",
        "python_crossings_per_firing": 1,
        "executor_surface": "native-session-raw-multi-threaded-executor",
        "executor_kind": "multi_threaded",
        "executor_threads": MTE_THREADS,
    },
    "native-python-callback": {
        "execution_model": "managed-rclcpp-wall-timer-python-callback",
        "timer_authority": "cpp",
        "executor_authority": "cpp",
        "callback_language": "python",
        "cache_kind": "managed-rclcpp-runtime",
        "python_crossings_per_firing": 1,
        "executor_kind": "single_threaded",
        "executor_threads": 1,
    },
    "native-cpp-callback": {
        "execution_model": "content-addressed-rclcpp-wall-timer-cpp-callback",
        "timer_authority": "cpp",
        "executor_authority": "cpp",
        "callback_language": "cpp",
        "cache_kind": "timer-probe-shared-library",
        "python_crossings_per_firing": 0,
        "executor_kind": "single_threaded",
        "executor_threads": 1,
    },
    "aot-staged": {
        "execution_model": "conventional-release-aot-rclcpp-wall-timer",
        "timer_authority": "cpp",
        "executor_authority": "cpp",
        "callback_language": "cpp",
        "cache_kind": "aot-binary",
        "python_crossings_per_firing": 0,
        "executor_kind": "single_threaded",
        "executor_threads": 1,
    },
}

DIRECT_VARIANTS = tuple(
    variant for variant, spec in VARIANTS.items()
    if "executor_surface" in spec
)
BASE_VARIANTS = tuple(
    variant for variant in VARIANTS
    if variant not in REGRESSION_VARIANTS
)


def _is_int(value) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _positive_int(value) -> bool:
    return _is_int(value) and value > 0


def _nonnegative_int(value) -> bool:
    return _is_int(value) and value >= 0


def _finite_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def recurrence_step(value: int) -> int:
    return (value * RECURRENCE_MULTIPLIER + RECURRENCE_INCREMENT) & MASK64


def expected_recurrence(firings: int) -> tuple[int, int]:
    state = RECURRENCE_SEED
    checksum = 0
    for _ in range(firings):
        state = recurrence_step(state)
        checksum = (checksum + state) & MASK64
    return state, checksum


def nearest_rank(values: list[int], percentile: int) -> int:
    if not values:
        raise ValueError("deadline observations cannot be empty")
    ordered = sorted(values)
    rank = max(1, math.ceil(percentile / 100.0 * len(ordered)))
    return ordered[rank - 1]


def deadline_summary(values: list[int]) -> dict[str, dict[str, int]]:
    absolute = [abs(value) for value in values]
    return {
        "signed_ns": {
            "p50": nearest_rank(values, 50),
            "p95": nearest_rank(values, 95),
            "p99": nearest_rank(values, 99),
            "max": max(values),
        },
        "absolute_ns": {
            "p50": nearest_rank(absolute, 50),
            "p95": nearest_rank(absolute, 95),
            "p99": nearest_rank(absolute, 99),
            "max": max(absolute),
        },
    }


def consecutive_interval_errors(phase_errors: list[int]) -> list[int]:
    if len(phase_errors) < 2:
        raise ValueError("consecutive interval evidence requires at least two firings")
    return [
        current - previous
        for previous, current in zip(phase_errors, phase_errors[1:])
    ]


def max_phase_slip_periods(errors: list[int]) -> int:
    return max(0, max(errors) // PERIOD_NS)


def _validate_artifact(artifact: dict, *, cached: bool) -> None:
    if not isinstance(artifact, dict) or artifact.get("cached") is not cached:
        raise ValueError("timer cache artifact hit state is invalid")
    if not isinstance(artifact.get("path"), str) or not artifact["path"]:
        raise ValueError("timer cache artifact path is required")
    if not isinstance(artifact.get("sha256"), str) or not SHA256_PATTERN.fullmatch(
            artifact["sha256"]):
        raise ValueError("timer cache artifact SHA-256 is invalid")
    if not _positive_int(artifact.get("size_bytes")):
        raise ValueError("timer cache artifact size is invalid")


def validate_prewarm(document: dict, *, expect_hit: bool) -> None:
    if not isinstance(document, dict) or document.get("schema") != PREWARM_SCHEMA:
        raise ValueError("unsupported timer prewarm schema")
    if not _positive_int(document.get("pid")):
        raise ValueError("timer prewarm PID is invalid")
    if document.get("loaded_rmw") != RMW:
        raise ValueError("timer prewarm loaded the wrong RMW")
    source_id = document.get("source_id")
    if not isinstance(source_id, str) or re.fullmatch(r"[a-f0-9]{16}", source_id) is None:
        raise ValueError("timer prewarm source identity is invalid")
    _validate_artifact(document.get("artifact"), cached=expect_hit)


def validate_cache(cache: dict) -> None:
    required_true = (
        "isolated_root", "autopch_disabled", "fresh_process_per_phase",
        "compilation_excluded_from_samples", "warm_hits_verified",
    )
    if not isinstance(cache, dict) or any(cache.get(key) is not True for key in required_true):
        raise ValueError("timer cache policy evidence is incomplete")
    if cache.get("persisted_after_run") is not False:
        raise ValueError("timer cache must be temporary")
    phases = cache.get("phases")
    if not isinstance(phases, dict) or set(phases) != {"cold", "warm"}:
        raise ValueError("timer cache requires cold and warm phases")
    validate_prewarm(phases["cold"], expect_hit=False)
    validate_prewarm(phases["warm"], expect_hit=True)
    cold = phases["cold"]
    warm = phases["warm"]
    if cold["source_id"] != warm["source_id"]:
        raise ValueError("timer helper source identity changed between cache phases")
    if tuple(cold["artifact"][key] for key in ("path", "sha256", "size_bytes")) != tuple(
            warm["artifact"][key] for key in ("path", "sha256", "size_bytes")):
        raise ValueError("timer cold and warm phases selected different artifacts")


def validate_build(build: dict) -> None:
    if not isinstance(build, dict) or build.get("build_type") != "Release":
        raise ValueError("timer AOT build must be Release")
    if build.get("private_build_directory") is not True or build.get(
            "build_directory_persisted") is not False:
        raise ValueError("timer AOT build isolation is invalid")
    command = build.get("compile_command")
    if not isinstance(command, str) or "-O3" not in command or "-DNDEBUG" not in command:
        raise ValueError("timer AOT build does not prove Release optimization")
    if build.get("executable_format") != "ELF":
        raise ValueError("timer AOT executable must be ELF")
    for key in (
            "source_sha256", "cmake_sha256", "compile_commands_sha256",
            "executable_sha256"):
        if not isinstance(build.get(key), str) or not SHA256_PATTERN.fullmatch(build[key]):
            raise ValueError("timer AOT %s is invalid" % key)
    if not _positive_int(build.get("build_elapsed_ns")):
        raise ValueError("timer AOT build duration is invalid")


def _validate_marker(
        marker: dict, *, authority: str, kind: str, expected_clock: str = "steady",
        expected_executor_kind: str = "single_threaded",
        expected_executor_threads: int = 1) -> None:
    if not isinstance(marker, dict) or marker.get("authority") != authority:
        raise ValueError("timer/executor authority marker is invalid")
    if not isinstance(marker.get("implementation"), str) or not marker["implementation"]:
        raise ValueError("timer/executor implementation marker is required")
    if kind == "timer":
        if marker.get("clock") != expected_clock or marker.get("period_ns") != PERIOD_NS:
            raise ValueError("timer clock or period marker is invalid")
    elif marker.get("kind") != expected_executor_kind or marker.get(
            "threads") != expected_executor_threads:
        raise ValueError("executor kind/thread-count marker is invalid")


def _validate_ready(ready: dict, sample: dict, cache: dict) -> None:
    variant = sample["variant"]
    spec = VARIANTS[variant]
    if not isinstance(ready, dict) or ready.get("schema") != EVENT_SCHEMA or ready.get(
            "event") != "ready":
        raise ValueError("timer READY evidence is invalid")
    for key in ("variant", "run_token"):
        if ready.get(key) != sample[key]:
            raise ValueError("timer READY identity is invalid")
    if ready.get("pid") != sample["worker_pid"] or ready.get("process_group_id") != sample[
            "worker_pid"]:
        raise ValueError("timer worker process-group evidence is invalid")
    if ready.get("node_name") != sample["node_name"] or ready.get("loaded_rmw") != RMW:
        raise ValueError("timer READY runtime identity is invalid")
    if ready.get("execution_model") != spec["execution_model"]:
        raise ValueError("timer execution model is invalid")
    if ready.get("warmup_firings") != WARMUP_FIRINGS or ready.get(
            "timer_canceled") is not True:
        raise ValueError("timer warmup did not self-cancel exactly")
    _validate_marker(
        ready.get("timer_marker"), authority=spec["timer_authority"], kind="timer",
        expected_clock="ros" if variant in DIRECT_VARIANTS else "steady")
    _validate_marker(
        ready.get("executor_marker"), authority=spec["executor_authority"], kind="executor",
        expected_executor_kind=spec["executor_kind"],
        expected_executor_threads=spec["executor_threads"])
    if ready["timer_marker"].get("callback_language") != spec["callback_language"]:
        raise ValueError("timer callback-language marker is invalid")
    artifact = ready.get("cache")
    if not isinstance(artifact, dict) or artifact.get("kind") != spec["cache_kind"]:
        raise ValueError("timer route cache marker is invalid")
    if variant == "native-cpp-callback":
        expected = cache["phases"]["warm"]["artifact"]
        if artifact.get("state") != "prebuilt" or artifact.get("hit") is not True:
            raise ValueError("native C++ timer helper must be a warm cache hit")
        if tuple(artifact.get(key) for key in ("path", "sha256", "size_bytes")) != tuple(
                expected[key] for key in ("path", "sha256", "size_bytes")):
            raise ValueError("native C++ timer helper differs from the warm artifact")
    elif variant == "aot-staged":
        if artifact != {"state": "prebuilt", "kind": "aot-binary"}:
            raise ValueError("AOT timer cache marker is invalid")
    elif variant == "compatible-rclcppyy":
        activation = ready.get("activation")
        if artifact != {"state": "activation-only", "kind": "activation-only"}:
            raise ValueError("compatible timer must be activation-only")
        if not isinstance(activation, dict) or activation.get("timer_status_backend") != "python":
            raise ValueError("compatible timer did not prove Python authority")
    elif variant in DIRECT_VARIANTS:
        activation = ready.get("activation")
        expected_activation = {
            "profile": "direct_cpp",
            "timer_status_backend": "cpp",
            "timer_decision_id": activation.get("timer_decision_id")
            if isinstance(activation, dict) else None,
            "timer_creation_route": "rclcpp_clock_timer",
            "callback_handoff": "direct_std_function",
            "executor_session_owned": True,
            "native_timer_type": ready["timer_marker"]["implementation"],
            "native_executor_type": ready["executor_marker"]["implementation"],
            "executor_surface": spec["executor_surface"],
        }
        if artifact != {"state": "process_warm", "kind": "direct-rclcpp-runtime"}:
            raise ValueError("direct timer runtime cache marker is invalid")
        decision_id = (
            activation.get("timer_decision_id")
            if isinstance(activation, dict) else None
        )
        if (
            activation != expected_activation
            or not isinstance(decision_id, str)
            or ENTITY_DECISION_PATTERN.fullmatch(decision_id) is None
        ):
            raise ValueError("direct timer did not prove its source-compatible route")
        if not ready["timer_marker"]["implementation"].startswith(
                "rclcpp::GenericTimer<"):
            raise ValueError("direct timer marker is not a native rclcpp clock timer")
        expected_executor_prefix = (
            "rclcpp::executors::MultiThreadedExecutor"
            if spec["executor_kind"] == "multi_threaded"
            else "rclcpp::executors::SingleThreadedExecutor"
        )
        if not ready[
                "executor_marker"]["implementation"].startswith(
                expected_executor_prefix):
            raise ValueError("direct executor marker is not the session native executor")
    elif artifact.get("state") not in ("not_applicable", "process_warm"):
        raise ValueError("timer cache state is invalid")


def _validate_armed(armed: dict, sample: dict) -> None:
    starts_after_emit = VARIANTS[sample["variant"]]["callback_language"] == "python"
    expected = {
        "schema": EVENT_SCHEMA,
        "event": "armed",
        "variant": sample["variant"],
        "run_token": sample["run_token"],
        "pid": sample["worker_pid"],
        "process_group_id": sample["worker_pid"],
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "timer_reset": not starts_after_emit,
        "measurement_starts_after_emit": starts_after_emit,
    }
    if armed != expected:
        raise ValueError("timer ARMED evidence is invalid")


def _validate_percentiles(summary: dict, *, absolute: bool) -> None:
    if not isinstance(summary, dict) or set(summary) != {"p50", "p95", "p99", "max"}:
        raise ValueError("deadline percentile fields are invalid")
    if any(not _is_int(value) for value in summary.values()):
        raise ValueError("deadline percentiles must be integer nanoseconds")
    if absolute and any(value < 0 for value in summary.values()):
        raise ValueError("absolute deadline error cannot be negative")
    if not summary["p50"] <= summary["p95"] <= summary["p99"] <= summary["max"]:
        raise ValueError("deadline percentiles are not ordered")


def _validate_report(report: dict, sample: dict) -> None:
    spec = VARIANTS[sample["variant"]]
    if not isinstance(report, dict) or report.get("schema") != EVENT_SCHEMA or report.get(
            "event") != "report":
        raise ValueError("timer report evidence is invalid")
    for key in ("variant", "run_token"):
        if report.get(key) != sample[key]:
            raise ValueError("timer report identity is invalid")
    expected_state, expected_checksum = expected_recurrence(MEASURED_FIRINGS)
    exact = {
        "warmup_firings": WARMUP_FIRINGS,
        "measured_firings": MEASURED_FIRINGS,
        "recurrence_state": expected_state,
        "checksum": expected_checksum,
        "post_cancel_firings": 0,
        "exceptions": 0,
    }
    if any(report.get(key) != value for key, value in exact.items()):
        raise ValueError("timer report counters violate the exact contract")
    crossing = spec["python_crossings_per_firing"]
    total = WARMUP_FIRINGS + MEASURED_FIRINGS
    if report.get("python_callback_count") != total * crossing or report.get(
            "measured_python_callback_count") != MEASURED_FIRINGS * crossing or report.get(
            "python_boundary_crossings") != total * crossing:
        raise ValueError("timer Python crossing counts are invalid")
    if report.get("cpu_clock") != "CLOCK_PROCESS_CPUTIME_ID" or not _positive_int(
            report.get("cpu_time_ns")):
        raise ValueError("timer CPU evidence is invalid")
    if not _positive_int(report.get("wall_duration_ns")):
        raise ValueError("timer wall duration is invalid")
    deadline = report.get("scheduled_deadline_error")
    if not isinstance(deadline, dict):
        raise ValueError("timer deadline evidence is required")
    _validate_percentiles(deadline.get("signed_ns"), absolute=False)
    _validate_percentiles(deadline.get("absolute_ns"), absolute=True)
    first_rearm = report.get("first_rearm_error_ns")
    if (
        not _is_int(first_rearm)
        or first_rearm > deadline["signed_ns"]["max"]
        or abs(first_rearm) > deadline["absolute_ns"]["max"]
    ):
        raise ValueError("timer first-rearm error is invalid")
    interval = report.get("consecutive_interval_error")
    if not isinstance(interval, dict):
        raise ValueError("timer consecutive-interval evidence is required")
    _validate_percentiles(interval.get("signed_ns"), absolute=False)
    _validate_percentiles(interval.get("absolute_ns"), absolute=True)
    if report.get("consecutive_interval_observations") != MEASURED_FIRINGS - 1:
        raise ValueError("timer consecutive-interval observation count is invalid")
    if report.get("max_phase_slip_periods") != max(
            0, deadline["signed_ns"]["max"] // PERIOD_NS):
        raise ValueError("timer maximum phase-slip count is inconsistent")
    if report.get("missed_periods") != report["max_phase_slip_periods"]:
        raise ValueError("timer legacy missed-period alias is inconsistent")
    if report.get("timer_canceled") is not True or report.get("teardown_clean") is not True:
        raise ValueError("timer cancellation or teardown evidence is invalid")
    if report.get("executor_thread_joined") is not True:
        raise ValueError("timer executor thread was not joined deterministically")


def validate_sample(sample: dict, cache: dict, build: dict) -> None:
    if not isinstance(sample, dict) or sample.get("schema") != SAMPLE_SCHEMA:
        raise ValueError("unsupported timer sample schema")
    if sample.get("variant") not in VARIANTS:
        raise ValueError("unknown timer variant")
    repetition = sample.get("repetition")
    if not _is_int(repetition) or not 1 <= repetition <= REPETITIONS:
        raise ValueError("timer repetition is invalid")
    variant = sample["variant"]
    if sample.get("case_id") != "%s__rep_%d" % (variant, repetition):
        raise ValueError("timer sample case identity is invalid")
    token = sample.get("run_token")
    if not isinstance(token, str) or re.fullmatch(r"timer_[a-f0-9]{32}", token) is None:
        raise ValueError("timer run token is invalid")
    if sample.get("node_name") != "timer_executor_%s" % token[6:18]:
        raise ValueError("timer node identity is invalid")
    domain_id = sample.get("ros_domain_id")
    if not _is_int(domain_id) or not 0 <= domain_id <= 232:
        raise ValueError("timer ROS domain is invalid")
    if sample.get("requested_rmw") != RMW or sample.get("ros_distro") != ROS_DISTRO:
        raise ValueError("timer sample runtime is not Jazzy/Cyclone")
    if not _positive_int(sample.get("worker_pid")) or sample.get(
            "worker_process_group_id") != sample["worker_pid"]:
        raise ValueError("timer worker process isolation is invalid")
    graph = sample.get("graph")
    if not isinstance(graph, dict) or graph.get("present_after_ready") is not True or graph.get(
            "absent_after_exit") is not True:
        raise ValueError("timer graph lifecycle evidence is incomplete")
    for key in ("presence_observation", "absence_observation"):
        observation = graph.get(key)
        if not isinstance(observation, dict) or observation.get("observed") is not True:
            raise ValueError("timer graph observation is invalid")
        if not _positive_int(observation.get("observations")) or not _nonnegative_int(
                observation.get("elapsed_ns")):
            raise ValueError("timer graph observation counters are invalid")
    _validate_ready(sample.get("worker_ready"), sample, cache)
    _validate_armed(sample.get("worker_armed"), sample)
    _validate_report(sample.get("worker_report"), sample)
    timing = sample.get("timing")
    report = sample["worker_report"]
    if not isinstance(timing, dict):
        raise ValueError("timer timing summary is required")
    expected_cpu = report["cpu_time_ns"] / MEASURED_FIRINGS
    expected_frequency = MEASURED_FIRINGS * 1e9 / report["wall_duration_ns"]
    if not _finite_number(timing.get("worker_cpu_ns_per_firing")) or not math.isclose(
            timing["worker_cpu_ns_per_firing"], expected_cpu, rel_tol=1e-12):
        raise ValueError("timer primary CPU metric is inconsistent")
    if not _finite_number(timing.get("effective_frequency_hz")) or not math.isclose(
            timing["effective_frequency_hz"], expected_frequency, rel_tol=1e-12):
        raise ValueError("timer effective frequency is inconsistent")
    secondary = (
        "scheduled_deadline_error",
        "first_rearm_error_ns",
        "consecutive_interval_error",
        "missed_periods",
        "max_phase_slip_periods",
    )
    if any(timing.get(key) != report[key] for key in secondary):
        raise ValueError("timer secondary metrics differ from worker evidence")
    if sample.get("correctness_verified") is not True or sample.get(
            "teardown_verified") is not True:
        raise ValueError("timer sample verification flags are incomplete")
    if sample["variant"] == "aot-staged":
        validate_build(build)


def rotating_order(variants: list[str], repetition: int) -> list[str]:
    offset = (repetition - 1) % len(variants)
    return variants[offset:] + variants[:offset]


def measurement_order(repetition: int) -> list[str]:
    """Keep the regression pair adjacent and alternate its local order."""
    pair = list(REGRESSION_VARIANTS)
    if repetition % 2 == 0:
        pair.reverse()
    return pair + rotating_order(list(BASE_VARIANTS), repetition)


def build_public_ste_regression(
    results: list[dict], *, characterization_only: bool = False
) -> dict:
    """Build the paired CPU gate and its secondary latency evidence."""
    by_case = {
        (row.get("variant"), row.get("repetition")): row
        for row in results
        if isinstance(row, dict)
    }
    pairs = []
    for repetition in range(1, REPETITIONS + 1):
        public = by_case.get((REGRESSION_VARIANTS[0], repetition))
        raw = by_case.get((REGRESSION_VARIANTS[1], repetition))
        if public is None or raw is None:
            continue
        public_cpu = float(public["timing"]["worker_cpu_ns_per_firing"])
        raw_cpu = float(raw["timing"]["worker_cpu_ns_per_firing"])
        public_latency = int(
            public["timing"]["scheduled_deadline_error"]["absolute_ns"]["p99"])
        raw_latency = int(
            raw["timing"]["scheduled_deadline_error"]["absolute_ns"]["p99"])
        pairs.append({
            "repetition": repetition,
            "public_case_id": public["case_id"],
            "raw_case_id": raw["case_id"],
            "public_cpu_ns_per_firing": public_cpu,
            "raw_cpu_ns_per_firing": raw_cpu,
            "cpu_ratio": public_cpu / raw_cpu,
            "public_absolute_latency_p99_ns": public_latency,
            "raw_absolute_latency_p99_ns": raw_latency,
        })

    public_cpu_values = [row["public_cpu_ns_per_firing"] for row in pairs]
    raw_cpu_values = [row["raw_cpu_ns_per_firing"] for row in pairs]
    public_latency_values = [row["public_absolute_latency_p99_ns"] for row in pairs]
    raw_latency_values = [row["raw_absolute_latency_p99_ns"] for row in pairs]
    if pairs:
        public_cpu_median = float(statistics.median(public_cpu_values))
        raw_cpu_median = float(statistics.median(raw_cpu_values))
        cpu_ratio = public_cpu_median / raw_cpu_median
        public_latency_median = float(statistics.median(public_latency_values))
        raw_latency_median = float(statistics.median(raw_latency_values))
        latency_ratio = (
            public_latency_median / raw_latency_median
            if raw_latency_median else None
        )
    else:
        public_cpu_median = None
        raw_cpu_median = None
        cpu_ratio = None
        public_latency_median = None
        raw_latency_median = None
        latency_ratio = None

    if characterization_only:
        mode = "characterization"
        status = "characterization"
        reason = "characterization_requested"
    elif len(pairs) < REGRESSION_REQUIRED_PAIRS:
        mode = "characterization"
        status = "characterization"
        reason = "incomplete_pairs"
    elif cpu_ratio <= REGRESSION_CPU_RATIO_LIMIT:
        mode = "enforced"
        status = "pass"
        reason = "within_cpu_ratio_limit"
    else:
        mode = "enforced"
        status = "fail"
        reason = "cpu_ratio_limit_exceeded"

    return {
        "schema": "rclcppyy.public-ste-regression/v1",
        "public_variant": REGRESSION_VARIANTS[0],
        "raw_control_variant": REGRESSION_VARIANTS[1],
        "primary_metric": "worker_cpu_ns_per_firing",
        "secondary_metric": "scheduled_deadline_error.absolute_ns.p99",
        "cpu_ratio_limit": REGRESSION_CPU_RATIO_LIMIT,
        "required_pairs": REGRESSION_REQUIRED_PAIRS,
        "pair_count": len(pairs),
        "mode": mode,
        "status": status,
        "reason": reason,
        "public_median_cpu_ns_per_firing": public_cpu_median,
        "raw_median_cpu_ns_per_firing": raw_cpu_median,
        "median_cpu_ratio": cpu_ratio,
        "public_median_absolute_latency_p99_ns": public_latency_median,
        "raw_median_absolute_latency_p99_ns": raw_latency_median,
        "median_absolute_latency_p99_ratio": latency_ratio,
        "pairs": pairs,
    }


def validate_public_ste_regression(regression: dict, results: list[dict]) -> None:
    if not isinstance(regression, dict):
        raise ValueError("public STE regression evidence is required")
    characterization = regression.get("mode") == "characterization"
    expected = build_public_ste_regression(
        results, characterization_only=(
            characterization and regression.get("reason") == "characterization_requested"
        ),
    )
    if regression != expected:
        raise ValueError("public STE regression evidence is inconsistent")


def validate_document(document: dict) -> None:
    if not isinstance(document, dict) or document.get("schema") != SCHEMA_ID:
        raise ValueError("unsupported timer benchmark schema")
    if document.get("claims") != {
            "enabled": False, "reason": "characterization_only"}:
        raise ValueError("timer benchmark claims must remain disabled")
    if document.get("interpretation") != {
            "enabled": False, "reason": "raw_evidence_only"}:
        raise ValueError("timer benchmark interpretation must remain disabled")
    if document.get("mode") != "measurement":
        raise ValueError("timer benchmark must remain in measurement mode")
    command = document.get("command")
    if not isinstance(command, list) or not command or any(
            not isinstance(value, str) for value in command):
        raise ValueError("timer benchmark command evidence is invalid")
    if not isinstance(document.get("environment"), dict):
        raise ValueError("timer benchmark environment evidence is invalid")
    generated_at = document.get("generated_at")
    try:
        datetime.datetime.fromisoformat(generated_at)
    except (TypeError, ValueError) as exc:
        raise ValueError("timer benchmark timestamp is invalid") from exc
    parameters = document.get("parameters")
    expected_parameters = {
        "variants": list(VARIANTS),
        "period_ns": PERIOD_NS,
        "warmup_firings": WARMUP_FIRINGS,
        "measured_firings": MEASURED_FIRINGS,
        "repetitions": REPETITIONS,
        "requested_rmw": RMW,
        "ros_distro": ROS_DISTRO,
    }
    if not isinstance(parameters, dict) or any(
            parameters.get(key) != value for key, value in expected_parameters.items()):
        raise ValueError("timer benchmark parameters differ from the fixed contract")
    expected_order = [
        "%s__rep_%d" % (variant, repetition)
        for repetition in range(1, REPETITIONS + 1)
        for variant in measurement_order(repetition)
    ]
    if parameters.get("execution_order") != expected_order:
        raise ValueError("timer execution order is not the paired rotating matrix")
    isolation = document.get("isolation")
    required_isolation = (
        "fresh_worker_process_per_sample", "fresh_process_group_per_sample",
        "unique_node_per_sample", "one_leased_domain_per_run", "rotating_variant_order",
    )
    if not isinstance(isolation, dict) or any(
            isolation.get(key) is not True for key in required_isolation):
        raise ValueError("timer process/domain isolation evidence is incomplete")
    domain_id = isolation.get("ros_domain_id")
    if not _is_int(domain_id) or not 0 <= domain_id <= 232:
        raise ValueError("timer leased domain evidence is invalid")
    source_files = document.get("source_files")
    expected_sources = {"runner", "protocol", "worker", "aot_source", "aot_cmake"}
    if not isinstance(source_files, dict) or set(source_files) != expected_sources or any(
            not isinstance(value, str) or not SHA256_PATTERN.fullmatch(value)
            for value in source_files.values()):
        raise ValueError("timer source identity evidence is invalid")
    validate_cache(document.get("cache"))
    validate_build(document.get("aot_build"))
    results = document.get("results")
    failures = document.get("failures")
    if not isinstance(results, list) or not isinstance(failures, list):
        raise ValueError("timer result and failure collections are required")
    cases = {}
    for row in results:
        validate_sample(row, document["cache"], document["aot_build"])
        if row["case_id"] in cases:
            raise ValueError("duplicate timer sample case")
        cases[row["case_id"]] = "result"
    for row in failures:
        variant = row.get("variant") if isinstance(row, dict) else None
        repetition = row.get("repetition") if isinstance(row, dict) else None
        valid_identity = (
            variant in VARIANTS
            and _is_int(repetition)
            and 1 <= repetition <= REPETITIONS
        )
        expected_case = (
            "%s__rep_%d" % (variant, repetition)
            if valid_identity else None
        )
        if (
            not isinstance(row, dict)
            or set(row) != {"case_id", "variant", "repetition", "error"}
            or not valid_identity
            or row.get("case_id") != expected_case
            or not isinstance(row.get("error"), str)
            or not row["error"]
            or row["case_id"] in cases
        ):
            raise ValueError("invalid or duplicate timer failure case")
        cases[row["case_id"]] = "failure"
    if set(cases) != set(expected_order):
        raise ValueError("timer document does not cover the exact sample matrix")
    validate_public_ste_regression(document.get("public_ste_regression"), results)


def build_document(
    *,
    repo_root: Path,
    mode: str,
    parameters: dict,
    isolation: dict,
    aot_build: dict,
    cache: dict,
    source_files: dict,
    results: list[dict],
    failures: list[dict],
    command: list[str],
    characterization_only: bool = False,
) -> dict:
    document = {
        "schema": SCHEMA_ID,
        "generated_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "mode": mode,
        "command": command,
        "environment": environment_metadata(repo_root),
        "parameters": parameters,
        "isolation": isolation,
        "aot_build": aot_build,
        "cache": cache,
        "source_files": source_files,
        "results": results,
        "failures": failures,
        "public_ste_regression": build_public_ste_regression(
            results, characterization_only=characterization_only),
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


if __name__ == "__main__":
    try:
        validate_document(json.load(sys.stdin))
    except (ValueError, json.JSONDecodeError) as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(2)
