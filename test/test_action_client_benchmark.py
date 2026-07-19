"""Contract and live smoke coverage for the action-client benchmark."""

from __future__ import annotations

import copy
import importlib.util
import json
import os
from pathlib import Path
import sys
import uuid

import pytest


REPO_ROOT = Path(__file__).resolve().parent.parent
BENCH_DIR = REPO_ROOT / "scripts" / "benchmarks"
SCHEMA_PATH = REPO_ROOT / "schemas" / "action-client-benchmark-v1.schema.json"
sys.path.insert(0, str(BENCH_DIR))


def _load(name):
    spec = importlib.util.spec_from_file_location(name, BENCH_DIR / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


protocol = _load("_action_client_protocol")
runner = _load("run_action_client_benchmark")
DIGEST = "a" * 64


def _artifact(*, cached):
    return {
        "cached": cached,
        "path": "/fixture/action-helper.so",
        "sha256": DIGEST,
        "size_bytes": 4096,
    }


def _cache():
    def phase(cached, pid):
        return {
            "schema": protocol.PREWARM_SCHEMA,
            "pid": pid,
            "loaded_rmw": protocol.RMW,
            "state_machine_source_id": "1" * 16,
            "artifacts": {
                "native_action_client": _artifact(cached=cached),
                "state_machine": _artifact(cached=cached),
            },
            "stdout_diagnostics": ["fixture"],
        }

    return {
        "isolated_root": True,
        "autopch_disabled": True,
        "fresh_process_per_phase": True,
        "compilation_excluded_from_samples": True,
        "warm_hits_verified": True,
        "persisted_after_run": False,
        "phases": {"cold": phase(False, 100), "warm": phase(True, 101)},
    }


def _build():
    return {
        "build_type": "Release",
        "private_build_directory": True,
        "build_directory_persisted": False,
        "compiler": "/fixture/c++",
        "compiler_version": "fixture c++ 1.0",
        "compile_commands": {
            "server": "c++ -O3 -DNDEBUG action_benchmark_server.cpp",
            "client": "c++ -O3 -DNDEBUG action_benchmark_client.cpp",
        },
        "server_source_sha256": DIGEST,
        "client_source_sha256": DIGEST,
        "cmake_sha256": DIGEST,
        "executables": {
            "server": {"format": "ELF", "sha256": DIGEST},
            "client": {"format": "ELF", "sha256": DIGEST},
        },
        "build_elapsed_ns": 100,
    }


def _rss():
    return {
        "kind": "post-warmup-peak-rss-growth",
        "unit": "bytes",
        "baseline_peak_bytes": 10_000,
        "final_peak_bytes": 12_000,
        "growth_bytes": 2_000,
        "limit_bytes": protocol.RSS_LIMIT_BYTES,
        "within_limit": True,
    }


def _latency():
    return {"p50": 100, "p95": 200, "p99": 300, "max": 400}


def _sample(variant, repetition=1, index=0):
    token = "action_" + format(index + 1, "032x")
    suffix = token[7:19]
    action_name = "/rclcppyy/action_benchmark/run_%s" % token[7:]
    server_node = "action_server_%s" % suffix
    client_node = "action_client_%s" % suffix
    server_pid = 1000 + index * 2
    client_pid = server_pid + 1
    spec = protocol.VARIANTS[variant]
    route_cache = {"kind": "stock-rclpy", "state": "not_applicable"}
    activation = None
    if variant == "compatible-rclcppyy":
        route_cache = {"kind": "activation-only", "state": "activation-only"}
        activation = {"profile": "compatible", "action_authority": "python"}
    elif variant in ("direct-source-compatible", "native-python-orchestrated"):
        artifact = _cache()["phases"]["warm"]["artifacts"]["native_action_client"]
        route_cache = {
            "kind": spec["cache_kind"],
            "state": "prebuilt",
            "hit": True,
            "path": artifact["path"],
            "sha256": artifact["sha256"],
            "size_bytes": artifact["size_bytes"],
        }
        if variant == "direct-source-compatible":
            activation = {
                "profile": "direct_cpp",
                "action_authority": "cpp",
                "representations": "actual_cpp",
            }
    elif variant == "native-cpp-state-machine":
        artifact = _cache()["phases"]["warm"]["artifacts"]["state_machine"]
        route_cache = {
            "kind": spec["cache_kind"],
            "state": "prebuilt",
            "hit": True,
            "path": artifact["path"],
            "sha256": artifact["sha256"],
            "size_bytes": artifact["size_bytes"],
        }
    elif variant == "aot-staged":
        route_cache = {"kind": "aot-binary", "state": "prebuilt"}

    client_ready = {
        "schema": protocol.CLIENT_SCHEMA,
        "event": "ready",
        "variant": variant,
        "run_token": token,
        "pid": client_pid,
        "process_group_id": client_pid,
        "node_name": client_node,
        "action_name": action_name,
        "loaded_rmw": protocol.RMW,
        "action_type": protocol.ACTION_TYPE,
        "execution_model": spec["execution_model"],
        "action_authority": spec["action_authority"],
        "action_implementation": spec["action_implementation"],
        "goal_representation": spec["goal_representation"],
        "qos": copy.deepcopy(protocol.QOS),
        "endpoints": protocol.endpoint_names(action_name),
        "executor": {
            "authority": spec["action_authority"],
            "kind": "single_threaded",
            "threads": 1,
            "implementation": "fixture executor",
        },
        "cache": route_cache,
        "warmup_goals": protocol.WARMUP_GOALS,
        "warmup_checksum": protocol.expected_checksum(protocol.WARMUP_GOALS),
        "warmup_feedback": protocol.WARMUP_GOALS * protocol.FEEDBACK_PER_GOAL,
        "warmup_results": protocol.WARMUP_GOALS,
        "warmup_terminal_success": protocol.WARMUP_GOALS,
        "active_goals": 0,
        "pending_operations": 0,
    }
    if activation is not None:
        client_ready["activation"] = activation

    server_ready = {
        "schema": protocol.SERVER_SCHEMA,
        "event": "ready",
        "variant": variant,
        "run_token": token,
        "pid": server_pid,
        "process_group_id": server_pid,
        "node_name": server_node,
        "action_name": action_name,
        "loaded_rmw": protocol.RMW,
        "action_type": protocol.ACTION_TYPE,
        "action_authority": "cpp",
        "qos": copy.deepcopy(protocol.QOS),
        "endpoints": protocol.endpoint_names(action_name),
        "executor": {"authority": "cpp", "kind": "single_threaded", "threads": 1},
    }
    total = protocol.WARMUP_GOALS + protocol.MEASURED_GOALS
    latency = {
        "send_to_accept": _latency(),
        "send_to_first_feedback": _latency(),
        "send_to_result": _latency(),
    }
    client_report = {
        "schema": protocol.CLIENT_SCHEMA,
        "event": "report",
        "variant": variant,
        "run_token": token,
        "pid": client_pid,
        "process_group_id": client_pid,
        "warmup_goals": protocol.WARMUP_GOALS,
        "measured_goals": protocol.MEASURED_GOALS,
        "goals_sent": total,
        "goals_accepted": total,
        "goals_rejected": 0,
        "feedback_received": total * protocol.FEEDBACK_PER_GOAL,
        "feedback_dropped": 0,
        "results_received": total,
        "terminal_succeeded": total,
        "sequence_checksum": protocol.expected_checksum(protocol.MEASURED_GOALS),
        "last_sequence": protocol.MEASURED_GOALS,
        "active_goals": 0,
        "pending_operations": 0,
        "exceptions": 0,
        "python_crossings": protocol.expected_crossings(variant, total),
        "no_python_message_conversion": spec["no_python_message_conversion"],
        "cpu_time_ns": 5_000_000,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "wall_duration_ns": 6_000_000_000,
        "latency_ns": latency,
        "rss_guard": _rss(),
        "orchestration_poll_count": 100 if spec["action_authority"] == "python" else 0,
        "teardown_clean": True,
        "executor_thread_joined": True,
    }
    server_report = {
        "schema": protocol.SERVER_SCHEMA,
        "event": "report",
        "variant": variant,
        "run_token": token,
        "pid": server_pid,
        "process_group_id": server_pid,
        "warmup_goals": protocol.WARMUP_GOALS,
        "measured_goals": protocol.MEASURED_GOALS,
        "goals_received": total,
        "goals_accepted": total,
        "goals_rejected": 0,
        "feedback_sent": total * protocol.FEEDBACK_PER_GOAL,
        "results_sent": total,
        "terminal_succeeded": total,
        "warmup_checksum": protocol.expected_checksum(protocol.WARMUP_GOALS),
        "measured_checksum": protocol.expected_checksum(protocol.MEASURED_GOALS),
        "active_goals": 0,
        "pending_operations": 0,
        "exceptions": 0,
        "cpu_time_ns": 4_000_000,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "cpu_role": "drift_diagnostic_only",
        "rss_guard": _rss(),
        "teardown_clean": True,
    }
    observation = {"observed": True, "observations": 2, "elapsed_ns": 100}
    return {
        "schema": protocol.SAMPLE_SCHEMA,
        "case_id": "%s__rep_%d" % (variant, repetition),
        "variant": variant,
        "repetition": repetition,
        "run_token": token,
        "action_name": action_name,
        "server_node": server_node,
        "client_node": client_node,
        "server_pid": server_pid,
        "client_pid": client_pid,
        "ros_domain_id": 77,
        "requested_rmw": protocol.RMW,
        "ros_distro": protocol.ROS_DISTRO,
        "topology": {
            "process_count": 2,
            "fresh_process_groups": True,
            "one_active_goal": True,
            "common_aot_server": True,
            "action_type": protocol.ACTION_TYPE,
            "qos": copy.deepcopy(protocol.QOS),
            "endpoints": protocol.endpoint_names(action_name),
        },
        "graph": {
            "server_present_after_ready": True,
            "client_present_after_ready": True,
            "exact_endpoints_present": True,
            "client_absent_after_exit": True,
            "server_absent_after_exit": True,
            "endpoints_absent_after_exit": True,
            "ready_observation": dict(observation),
            "client_exit_observation": dict(observation),
            "final_observation": dict(observation),
        },
        "server_ready": server_ready,
        "client_ready": client_ready,
        "client_armed": {
            "schema": protocol.CLIENT_SCHEMA,
            "event": "armed",
            "variant": variant,
            "run_token": token,
            "pid": client_pid,
            "process_group_id": client_pid,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "measurement_reset": True,
        },
        "client_report": client_report,
        "server_report": server_report,
        "timing": {
            "client_cpu_ns_per_completed_goal": (
                client_report["cpu_time_ns"] / protocol.MEASURED_GOALS),
            "completed_goals_per_second": (
                protocol.MEASURED_GOALS * 1e9 / client_report["wall_duration_ns"]),
            "latency_ns": client_report["latency_ns"],
        },
        "server_cpu_diagnostic": {
            "role": "drift_only",
            "cpu_time_ns": server_report["cpu_time_ns"],
            "cpu_ns_per_measured_goal": (
                server_report["cpu_time_ns"] / protocol.MEASURED_GOALS),
        },
        "correctness_verified": True,
        "teardown_verified": True,
        "diagnostics": {
            "server_stdout": [], "server_stderr": None,
            "client_stdout": [], "client_stderr": None,
        },
    }


def _document():
    results = []
    execution_order = []
    index = 0
    for repetition in range(1, protocol.REPETITIONS + 1):
        for variant in protocol.rotating_order(list(protocol.VARIANTS), repetition):
            execution_order.append("%s__rep_%d" % (variant, repetition))
            results.append(_sample(variant, repetition, index))
            index += 1
    return {
        "schema": protocol.SCHEMA_ID,
        "generated_at": "2026-01-01T00:00:00+00:00",
        "mode": "measurement",
        "command": ["python", "run_action_client_benchmark.py"],
        "environment": {"fixture": True},
        "parameters": {
            "variants": list(protocol.VARIANTS),
            "warmup_goals": protocol.WARMUP_GOALS,
            "measured_goals": protocol.MEASURED_GOALS,
            "feedback_per_goal": protocol.FEEDBACK_PER_GOAL,
            "repetitions": protocol.REPETITIONS,
            "requested_rmw": protocol.RMW,
            "ros_distro": protocol.ROS_DISTRO,
            "action_type": protocol.ACTION_TYPE,
            "qos": copy.deepcopy(protocol.QOS),
            "execution_order": execution_order,
        },
        "isolation": {
            "fresh_process_pair_per_sample": True,
            "fresh_process_groups_per_sample": True,
            "unique_action_and_nodes_per_sample": True,
            "one_leased_domain_per_run": True,
            "rotating_variant_order": True,
            "ros_domain_id": 77,
        },
        "aot_build": _build(),
        "cache": _cache(),
        "source_files": {
            "runner": DIGEST, "protocol": DIGEST, "worker": DIGEST,
            "server_source": DIGEST, "client_source": DIGEST, "aot_cmake": DIGEST,
        },
        "results": results,
        "failures": [],
        "claims": {"enabled": False, "reason": "characterization_only"},
        "interpretation": {"enabled": False, "reason": "raw_evidence_only"},
    }


def _replace(document, path, value):
    target = document
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value


def test_goal_identity_checksum_latency_and_crossings_are_fixed():
    assert protocol.goal_strings("measured", 7) == (
        "rclcppyy/action-benchmark/measured/7/target",
        "rclcppyy/action-benchmark/measured/7/source",
    )
    assert protocol.expected_checksum(500) == 125250
    assert protocol.latency_summary([4, 1, 8, 2]) == {
        "p50": 2, "p95": 8, "p99": 8, "max": 8}
    assert protocol.expected_crossings("native-python-orchestrated", 520) == {
        "goal": 520, "feedback": 1560, "result": 520, "total": 2600}
    assert protocol.expected_crossings("native-cpp-state-machine", 520)["total"] == 0


@pytest.mark.parametrize("variant", tuple(protocol.VARIANTS))
def test_each_action_lane_satisfies_the_exact_sample_contract(variant):
    protocol.validate_sample(_sample(variant), _cache(), _build())


@pytest.mark.parametrize(
    ("variant", "path", "value"),
    [
        ("stock-rclpy", ("requested_rmw",), "rmw_fastrtps_cpp"),
        ("stock-rclpy", ("action_name",), "/wrong"),
        ("stock-rclpy", ("topology", "one_active_goal"), False),
        ("stock-rclpy", ("client_ready", "action_authority"), "cpp"),
        ("stock-rclpy", ("client_ready", "action_implementation"), "unknown"),
        ("stock-rclpy", ("client_ready", "goal_representation"), "cpp-message"),
        ("stock-rclpy", ("client_ready", "warmup_checksum"), 0),
        ("stock-rclpy", ("client_ready", "executor", "threads"), 2),
        ("stock-rclpy", ("client_armed", "cpu_clock"), "CLOCK_THREAD_CPUTIME_ID"),
        ("stock-rclpy", ("client_report", "feedback_received"), 1559),
        ("stock-rclpy", ("client_report", "sequence_checksum"), 0),
        ("stock-rclpy", ("client_report", "terminal_succeeded"), 519),
        ("stock-rclpy", ("client_report", "active_goals"), 1),
        ("stock-rclpy", ("client_report", "feedback_dropped"), 1),
        ("stock-rclpy", ("client_report", "python_crossings", "total"), 0),
        ("stock-rclpy", ("client_report", "cpu_clock"), "CLOCK_MONOTONIC"),
        ("stock-rclpy", ("client_report", "rss_guard", "within_limit"), False),
        ("stock-rclpy", ("graph", "exact_endpoints_present"), False),
        ("stock-rclpy", ("server_report", "goals_accepted"), 519),
        ("stock-rclpy", ("server_report", "cpu_role"), "primary"),
        ("compatible-rclcppyy", ("client_ready", "cache", "state"), "prebuilt"),
        ("native-python-orchestrated", ("client_ready", "cache", "hit"), False),
        ("native-cpp-state-machine", ("client_report", "python_crossings", "goal"), 1),
        ("aot-staged", ("client_ready", "cache", "state"), "process_warm"),
    ],
)
def test_action_sample_rejects_tainted_evidence(variant, path, value):
    sample = _sample(variant)
    _replace(sample, path, value)
    with pytest.raises(ValueError):
        protocol.validate_sample(sample, _cache(), _build())


def test_action_latency_percentiles_must_be_ordered():
    sample = _sample("stock-rclpy")
    sample["client_report"]["latency_ns"]["send_to_result"]["p95"] = 50
    with pytest.raises(ValueError, match="not ordered"):
        protocol.validate_sample(sample, _cache(), _build())


@pytest.mark.parametrize(
    ("path", "value"),
    [
        (("phases", "cold", "artifacts", "state_machine", "cached"), True),
        (("phases", "warm", "state_machine_source_id"), "2" * 16),
        (("phases", "warm", "artifacts", "native_action_client", "sha256"), "b" * 64),
        (("warm_hits_verified",), False),
    ],
)
def test_action_cache_rejects_invalid_cold_warm_evidence(path, value):
    cache = _cache()
    _replace(cache, path, value)
    with pytest.raises(ValueError):
        protocol.validate_cache(cache)


def test_action_aot_build_requires_both_release_commands():
    build = _build()
    build["compile_commands"]["client"] = "c++ -O2 action_benchmark_client.cpp"
    with pytest.raises(ValueError, match="Release optimization"):
        protocol.validate_build(build)


def test_complete_action_document_and_rotating_order_validate():
    document = _document()
    protocol.validate_document(document)
    assert len(document["results"]) == 30
    assert document["parameters"]["execution_order"][6] == (
        "compatible-rclcppyy__rep_2")


@pytest.mark.parametrize(
    ("path", "value"),
    [
        (("claims", "enabled"), True),
        (("interpretation", "enabled"), True),
        (("parameters", "measured_goals"), 499),
        (("parameters", "execution_order"), []),
        (("isolation", "fresh_process_groups_per_sample"), False),
        (("source_files", "worker"), "invalid"),
    ],
)
def test_action_document_rejects_claims_or_matrix_drift(path, value):
    document = _document()
    _replace(document, path, value)
    with pytest.raises(ValueError):
        protocol.validate_document(document)


def test_action_schema_encodes_negative_contracts():
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    assert schema["properties"]["claims"]["const"]["enabled"] is False
    parameters = schema["properties"]["parameters"]["properties"]
    assert parameters["requested_rmw"]["const"] == protocol.RMW
    assert parameters["warmup_goals"]["const"] == 20
    assert parameters["measured_goals"]["const"] == 500
    report = schema["$defs"]["client_report"]["properties"]
    assert report["feedback_received"]["const"] == 1560
    assert report["cpu_clock"]["const"] == "CLOCK_PROCESS_CPUTIME_ID"
    assert report["active_goals"]["const"] == 0


def test_action_json_schema_accepts_fixture_and_rejects_claims_when_available():
    jsonschema = pytest.importorskip("jsonschema")
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    jsonschema.Draft202012Validator.check_schema(schema)
    jsonschema.validate(_document(), schema)
    invalid = _document()
    invalid["claims"]["enabled"] = True
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(invalid, schema)


@pytest.mark.skipif(
    os.environ.get("ROS_DISTRO") != protocol.ROS_DISTRO,
    reason="live action smoke requires ROS 2 Jazzy",
)
def test_live_cyclone_all_action_lanes(tmp_path, monkeypatch):
    from _domain_lease import acquire_domain

    build_directory = tmp_path / "build"
    cache_root = tmp_path / "cache"
    build_directory.mkdir()
    cache_root.mkdir()
    env = os.environ.copy()
    env.update({
        "RMW_IMPLEMENTATION": protocol.RMW,
        "XDG_CACHE_HOME": str(cache_root),
        "CPPYY_KIT_NO_AUTOPCH": "1",
        "PYTHONUNBUFFERED": "1",
    })
    env.pop("CPPYY_KIT_NO_CACHE", None)
    monkeypatch.setenv("RMW_IMPLEMENTATION", protocol.RMW)
    executables, build = runner._compile_aot(build_directory, env, 180.0)
    protocol.validate_build(build)
    observer = None
    with acquire_domain() as lease:
        env["ROS_DOMAIN_ID"] = str(lease.domain_id)
        monkeypatch.setenv("ROS_DOMAIN_ID", str(lease.domain_id))
        cache = runner._prewarm(cache_root, env, 180.0)
        protocol.validate_cache(cache)
        observer = runner.GraphObserver("action_smoke_observer_" + uuid.uuid4().hex[:10])
        try:
            for index, variant in enumerate(protocol.VARIANTS):
                token = "action_" + format(index + 1, "032x")
                suffix = token[7:19]
                action_name = "/rclcppyy/action_benchmark/run_%s" % token[7:]
                server_node = "action_server_%s" % suffix
                client_node = "action_client_%s" % suffix
                server = None
                client = None
                try:
                    server = runner._spawn([
                        str(executables["server"]), server_node, action_name,
                        token, variant, "1", "2",
                    ], env)
                    server_ready, _ = runner._read_document(
                        server, 60.0, "action smoke server ready")
                    assert server_ready["loaded_rmw"] == protocol.RMW
                    if variant == "aot-staged":
                        client_argv = [
                            str(executables["client"]), client_node, action_name,
                            token, variant, "1", "2",
                        ]
                    else:
                        client_argv = [
                            sys.executable, "-u", str(runner.WORKER),
                            "--variant", variant,
                            "--node-name", client_node,
                            "--action-name", action_name,
                            "--run-token", token,
                            "--warmup-goals", "1",
                            "--measured-goals", "2",
                        ]
                    client = runner._spawn(client_argv, env)
                    client_ready, _ = runner._read_document(
                        client, 60.0, "action smoke client ready")
                    assert client_ready["warmup_feedback"] == 3
                    observer.wait_ready(server_node, client_node, action_name, 20.0)
                    runner._write_control(client, "START", "action smoke client")
                    armed, _ = runner._read_document(
                        client, 60.0, "action smoke client armed")
                    report, _ = runner._read_document(
                        client, 60.0, "action smoke client report")
                    runner._finish(client, 60.0, "action smoke client")
                    observer.wait_client_exit(
                        server_node, client_node, action_name, 20.0)
                    runner._write_control(server, "STOP", "action smoke server")
                    server_report, _ = runner._read_document(
                        server, 60.0, "action smoke server report")
                    runner._finish(server, 60.0, "action smoke server")
                    observer.wait_final_exit(
                        server_node, client_node, action_name, 20.0)
                    assert armed["cpu_clock"] == "CLOCK_PROCESS_CPUTIME_ID"
                    assert report["goals_sent"] == 3
                    assert report["feedback_received"] == 9
                    assert report["sequence_checksum"] == 3
                    assert report["terminal_succeeded"] == 3
                    expected = protocol.expected_crossings(variant, 3)["total"]
                    assert report["python_crossings"]["total"] == expected
                    assert report["active_goals"] == 0
                    assert report["feedback_dropped"] == 0
                    assert server_report["results_sent"] == 3
                    assert server_report["feedback_sent"] == 9
                finally:
                    runner._stop_process(client)
                    runner._stop_process(server)
        finally:
            if observer is not None:
                observer.close()
