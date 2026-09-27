"""Contract and bounded live coverage for the action-server benchmark."""

from __future__ import annotations

import copy
import importlib.util
import json
import os
from pathlib import Path
import sys
import types
import uuid

import pytest


REPO_ROOT = Path(__file__).resolve().parent.parent
BENCH_DIR = REPO_ROOT / "scripts" / "benchmarks"
SCHEMA_PATH = REPO_ROOT / "schemas" / "action-server-benchmark-v1.schema.json"
sys.path.insert(0, str(BENCH_DIR))


def _load(name):
    spec = importlib.util.spec_from_file_location(name, BENCH_DIR / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


protocol = _load("_action_server_protocol")
runner = _load("run_action_server_benchmark")
worker = _load("action_server_worker")
DIGEST = "a" * 64


def _artifact(cached):
    return {
        "cached": cached,
        "path": "/fixture/action-server.so",
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
                "native_action_server": _artifact(cached),
                "state_machine": _artifact(cached),
            },
            "stdout_diagnostics": [],
        }

    return {
        "isolated_root": True,
        "autopch_disabled": True,
        "fresh_process_per_phase": True,
        "compilation_excluded_from_samples": True,
        "warm_hits_verified": True,
        "persisted_after_run": False,
        "phases": {"cold": phase(False, 101), "warm": phase(True, 102)},
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


def _route(variant):
    spec = protocol.VARIANTS[variant]
    if variant == "stock-rclpy":
        return {"kind": "stock-rclpy", "state": "not_applicable"}
    if variant == "aot-staged":
        return {"kind": "aot-binary", "state": "prebuilt"}
    name = "state_machine" if variant == "native-cpp-state-machine" else (
        "native_action_server")
    artifact = _cache()["phases"]["warm"]["artifacts"][name]
    return {
        "kind": spec["cache_kind"],
        "state": "prebuilt",
        "hit": True,
        "path": artifact["path"],
        "sha256": artifact["sha256"],
        "size_bytes": artifact["size_bytes"],
    }


def _server_boundary(variant):
    if variant in (
            "direct-source-compatible", "native-python-orchestrated",
            "native-cpp-state-machine"):
        return {
            "proof": "counter-backed-poison",
            "exact_generated_cpp": True,
            "python_message_conversions": 0,
            "python_serialization_calls": 0,
            "adapter_cdr_roundtrips": 0,
            "tripwires_armed": True,
            "tripwire_surfaces": list(protocol.BOUNDARY_TRIPWIRE_SURFACES),
        }
    if variant == "aot-staged":
        return {
            "proof": "cpp-only-process",
            "exact_generated_cpp": True,
            "python_message_conversions": 0,
            "python_serialization_calls": 0,
            "adapter_cdr_roundtrips": 0,
            "tripwires_armed": False,
            "tripwire_surfaces": [],
        }
    return {
        "proof": "python-message-lane",
        "exact_generated_cpp": False,
        "python_message_conversions": None,
        "python_serialization_calls": None,
        "adapter_cdr_roundtrips": None,
        "tripwires_armed": False,
        "tripwire_surfaces": [],
    }


def _sample(variant, repetition=1, index=0):
    token = "action_server_" + format(index + 1, "032x")
    suffix = token[14:26]
    action_name = "/rclcppyy/action_server_benchmark/run_%s" % token[14:]
    server_node = "action_server_%s" % suffix
    client_node = "action_client_%s" % suffix
    server_pid = 2000 + index * 2
    client_pid = server_pid + 1
    spec = protocol.VARIANTS[variant]
    total = protocol.WARMUP_GOALS + protocol.MEASURED_GOALS
    identity = {
        "variant": variant,
        "run_token": token,
    }
    server_ready = {
        "schema": protocol.SERVER_SCHEMA,
        "event": "ready",
        **identity,
        "pid": server_pid,
        "process_group_id": server_pid,
        "node_name": server_node,
        "action_name": action_name,
        "loaded_rmw": protocol.RMW,
        "action_type": protocol.ACTION_TYPE,
        "execution_model": spec["execution_model"],
        "action_authority": spec["authority"],
        "action_implementation": spec["implementation"],
        "goal_representation": spec["representation"],
        "feedback_representation": spec["representation"],
        "result_representation": spec["representation"],
        "goal_id_representation": spec["representation"],
        "envelope_representation": spec["representation"],
        "qos": copy.deepcopy(protocol.QOS),
        "endpoints": protocol.endpoint_names(action_name),
        "executor": {
            "authority": spec["authority"], "kind": "single_threaded", "threads": 1,
        },
        "cache": _route(variant),
    }
    client_ready = {
        "schema": protocol.CLIENT_SCHEMA,
        "event": "ready",
        **identity,
        "pid": client_pid,
        "process_group_id": client_pid,
        "node_name": client_node,
        "action_name": action_name,
        "loaded_rmw": protocol.RMW,
        "action_type": protocol.ACTION_TYPE,
        "execution_model": "conventional-release-aot-rclcpp-action-client",
        "action_authority": "cpp",
        "goal_representation": "cpp-message",
        "action_implementation": "rclcpp_action::Client<LookupTransform>",
        "qos": copy.deepcopy(protocol.QOS),
        "endpoints": protocol.endpoint_names(action_name),
        "executor": {"authority": "cpp", "kind": "single_threaded", "threads": 1},
        "cache": {"kind": "aot-binary", "state": "prebuilt"},
        "warmup_goals": protocol.WARMUP_GOALS,
        "warmup_checksum": protocol.expected_checksum(protocol.WARMUP_GOALS),
        "warmup_feedback": protocol.WARMUP_GOALS * protocol.FEEDBACK_PER_GOAL,
        "warmup_results": protocol.WARMUP_GOALS,
        "warmup_terminal_success": protocol.WARMUP_GOALS,
        "active_goals": 0,
        "pending_operations": 0,
    }
    latencies = {
        "send_to_accept": _latency(),
        "send_to_first_feedback": _latency(),
        "send_to_result": _latency(),
    }
    client_report = {
        "schema": protocol.CLIENT_SCHEMA,
        "event": "report",
        **identity,
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
        "cpu_time_ns": 3_000_000,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "wall_duration_ns": 4_000_000_000,
        "latency_ns": latencies,
        "rss_guard": _rss(),
        "orchestration_poll_count": 0,
        "teardown_clean": True,
        "executor_thread_joined": True,
    }
    server_report = {
        "schema": protocol.SERVER_SCHEMA,
        "event": "report",
        **identity,
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
        "cpu_time_ns": 2_000_000,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "cpu_role": "server_under_test",
        "rss_guard": _rss(),
        "python_crossings": protocol.expected_python_crossings(variant, total),
        "python_crossing_semantics": "callback_entries_only",
        "cpp_value_operations": protocol.expected_cpp_operations(variant, total),
        "boundary_evidence": _server_boundary(variant),
        "teardown_clean": True,
    }
    observation = {"observed": True, "observations": 1, "elapsed_ns": 100}
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
        "ros_domain_id": 88,
        "requested_rmw": protocol.RMW,
        "ros_distro": protocol.ROS_DISTRO,
        "topology": {
            "process_count": 2,
            "fresh_process_groups": True,
            "one_active_goal": True,
            "common_aot_client": True,
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
            "ready_observation": observation,
            "client_exit_observation": observation,
            "final_observation": observation,
        },
        "server_ready": server_ready,
        "client_ready": client_ready,
        "client_armed": {
            "schema": protocol.CLIENT_SCHEMA,
            "event": "armed",
            **identity,
            "pid": client_pid,
            "process_group_id": client_pid,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "measurement_reset": True,
            "measurement_window_started": False,
            "protocol_emission_excluded": True,
        },
        "client_report": client_report,
        "server_report": server_report,
        "timing": {
            "primary_metric": "server_cpu_ns_per_completed_goal",
            "server_cpu_ns_per_completed_goal": (
                server_report["cpu_time_ns"] / protocol.MEASURED_GOALS),
            "completed_goals_per_second": (
                protocol.MEASURED_GOALS * 1e9 / client_report["wall_duration_ns"]),
            "latency_ns": latencies,
            "client_cpu_diagnostic_ns_per_goal": (
                client_report["cpu_time_ns"] / protocol.MEASURED_GOALS),
        },
        "correctness_verified": True,
        "teardown_verified": True,
        "diagnostics": {},
    }


def _document():
    results = []
    order = []
    index = 0
    for repetition in range(1, protocol.REPETITIONS + 1):
        for variant in protocol.rotating_order(list(protocol.VARIANTS), repetition):
            order.append("%s__rep_%d" % (variant, repetition))
            results.append(_sample(variant, repetition, index))
            index += 1
    return {
        "schema": protocol.SCHEMA_ID,
        "generated_at": "2026-01-01T00:00:00+00:00",
        "mode": "measurement",
        "command": ["python", "run_action_server_benchmark.py"],
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
            "primary_metric": "server_cpu_ns_per_completed_goal",
            "common_driver": "conventional-release-aot-rclcpp-action-client",
            "execution_order": order,
        },
        "isolation": {
            "fresh_process_pair_per_sample": True,
            "fresh_process_groups_per_sample": True,
            "unique_action_and_nodes_per_sample": True,
            "one_leased_domain_per_run": True,
            "rotating_variant_order": True,
            "ros_domain_id": 88,
        },
        "aot_build": _build(),
        "cache": _cache(),
        "source_files": {
            "runner": DIGEST,
            "protocol": DIGEST,
            "worker": DIGEST,
            "aot_server": DIGEST,
            "aot_client": DIGEST,
            "aot_cmake": DIGEST,
        },
        "results": results,
        "failures": [],
        "claims": {"enabled": False, "reason": "characterization_only"},
        "interpretation": {"enabled": False, "reason": "raw_evidence_only"},
    }


def _replace(value, path, replacement):
    target = value
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = replacement


def test_action_server_fixed_cpu_contract_and_boundaries():
    assert protocol.MEASURED_GOALS == 500
    assert protocol.WARMUP_GOALS == 20
    assert protocol.FEEDBACK_PER_GOAL == 3
    assert protocol.expected_python_crossings("direct-source-compatible", 520) == {
        "goal_decision": 520,
        "accepted_goal": 520,
        "execute": 520,
        "total": 1560,
    }
    operations = protocol.expected_cpp_operations(
        "native-python-orchestrated", 520)
    assert operations["goal_shared_handoffs"] == 520
    assert operations["feedback_value_submissions"] == 1560
    assert operations["adapter_message_deep_copies"] == 2080
    assert protocol.expected_python_crossings(
        "native-cpp-state-machine", 520)["total"] == 0


def test_action_server_sources_pin_primary_role_and_public_feedback_qos():
    runner_source = (BENCH_DIR / "run_action_server_benchmark.py").read_text(
        encoding="utf-8")
    worker_source = (BENCH_DIR / "action_server_worker.py").read_text(
        encoding="utf-8")
    aot_source = (
        BENCH_DIR / "action_client_aot" / "action_benchmark_server.cpp"
    ).read_text(encoding="utf-8")
    assert '"server_under_test",' in runner_source
    assert protocol.QOS["feedback_topic"]["depth"] == 10
    assert protocol.QOS["feedback_topic"]["reliability"] == "reliable"
    assert "qos_profile_system_default" not in worker_source
    assert "feedback_pub_qos_profile=" not in worker_source
    assert 'cpu_role != "drift_diagnostic_only"' in aot_source
    assert 'cpu_role != "server_under_test"' in aot_source


def test_source_compatible_server_uses_one_public_executor_setup():
    source = (BENCH_DIR / "action_server_worker.py").read_text(encoding="utf-8")
    assert source.count("def _source_compatible_lane(") == 1
    assert "def _stock_lane(" not in source
    assert "def _direct_lane(" not in source
    assert "context = node.context" in source
    assert "SingleThreadedExecutor(context=context)" in source
    assert "executor.spin_once(timeout_sec=0.002)" in source


def test_exact_server_boundary_poison_is_counter_backed(monkeypatch):
    modules = {}

    def fake_import(name):
        return modules.setdefault(name, types.SimpleNamespace())

    monkeypatch.setattr(worker.importlib, "import_module", fake_import)
    guard = worker._poison_boundaries()
    assert guard["surfaces"] == list(protocol.BOUNDARY_TRIPWIRE_SURFACES)
    for surface in guard["surfaces"]:
        module_name, attribute = surface.rsplit(".", 1)
        with pytest.raises(AssertionError):
            getattr(modules[module_name], attribute)()
    assert guard["counters"] == {
        "python_message_conversions": 6,
        "python_serialization_calls": 6,
        "adapter_cdr_roundtrips": 4,
    }


def test_aot_server_uses_blocking_spin_once_without_poll_sleep():
    source = (
        BENCH_DIR / "action_client_aot" / "action_benchmark_server.cpp"
    ).read_text(encoding="utf-8")
    assert "executor.spin_once(2ms);" in source
    assert "executor.spin_some(2ms);" not in source
    assert "std::this_thread::sleep_for(100us);" not in source


@pytest.mark.parametrize("variant", tuple(protocol.VARIANTS))
def test_each_action_server_lane_satisfies_strict_sample_contract(variant):
    protocol.validate_sample(_sample(variant), _cache())


@pytest.mark.parametrize(
    ("variant", "path", "replacement"),
    [
        ("stock-rclpy", ("server_report", "cpu_time_ns"), 0),
        ("stock-rclpy", ("server_report", "feedback_sent"), 1559),
        ("stock-rclpy", ("server_report", "python_crossings", "total"), 0),
        ("direct-source-compatible", (
            "server_report", "boundary_evidence", "exact_generated_cpp"), False),
        ("direct-source-compatible", (
            "server_report", "boundary_evidence", "python_message_conversions"), 1),
        ("direct-source-compatible", (
            "server_report", "boundary_evidence", "tripwire_surfaces"), []),
        ("native-python-orchestrated", (
            "server_report", "cpp_value_operations", "goal_shared_handoffs"), 0),
        ("native-cpp-state-machine", (
            "server_report", "python_crossings", "execute"), 1),
        ("aot-staged", ("server_report", "cpu_role"), "drift_diagnostic_only"),
        ("aot-staged", ("server_ready", "cache", "state"), "process_warm"),
        ("aot-staged", ("topology", "common_aot_client"), False),
    ],
)
def test_action_server_sample_rejects_tainted_evidence(variant, path, replacement):
    sample = _sample(variant)
    _replace(sample, path, replacement)
    with pytest.raises(ValueError):
        protocol.validate_sample(sample, _cache())


def test_action_server_cache_requires_cold_then_identical_warm_artifacts():
    cache = _cache()
    protocol.validate_cache(cache)
    cache["phases"]["warm"]["artifacts"]["state_machine"]["sha256"] = "b" * 64
    with pytest.raises(ValueError, match="different artifacts"):
        protocol.validate_cache(cache)


def test_complete_document_has_rotating_five_by_five_matrix_and_no_claims():
    document = _document()
    protocol.validate_document(document)
    assert len(document["results"]) == 25
    assert document["parameters"]["execution_order"][5] == (
        "direct-source-compatible__rep_2")
    document["claims"]["enabled"] = True
    with pytest.raises(ValueError, match="claims"):
        protocol.validate_document(document)


def test_action_server_json_schema_matches_negative_contracts():
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    assert schema["properties"]["claims"]["const"]["enabled"] is False
    assert schema["properties"]["parameters"]["properties"][
        "requested_rmw"] == {"const": protocol.RMW}
    assert schema["$defs"]["server_report"]["properties"][
        "cpu_role"] == {"const": "server_under_test"}
    armed = schema["$defs"]["client_armed"]
    assert armed["properties"]["measurement_window_started"] == {
        "const": False}
    assert armed["properties"]["protocol_emission_excluded"] == {
        "const": True}
    assert schema["$defs"]["tripwire_surface_set"]["const"] == list(
        protocol.BOUNDARY_TRIPWIRE_SURFACES)
    assert schema["$defs"]["server_report"]["properties"][
        "python_crossing_semantics"] == {"const": "callback_entries_only"}
    jsonschema = pytest.importorskip("jsonschema")
    jsonschema.Draft202012Validator.check_schema(schema)
    jsonschema.validate(_document(), schema)

    invalid = _document()
    invalid["results"][1]["server_report"]["boundary_evidence"][
        "python_serialization_calls"] = 1
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(invalid, schema)

    invalid = _document()
    invalid["results"][0]["client_armed"]["measurement_window_started"] = True
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(invalid, schema)


@pytest.mark.skipif(
    os.environ.get("ROS_DISTRO") != protocol.ROS_DISTRO,
    reason="live action-server smoke requires ROS 2 Jazzy",
)
def test_live_cyclone_action_server_lanes_use_one_aot_client(tmp_path, monkeypatch):
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
        "RCLCPPYY_ACTION_SERVER_DEBUG": "1",
    })
    env.pop("CPPYY_KIT_NO_CACHE", None)
    monkeypatch.setenv("RMW_IMPLEMENTATION", protocol.RMW)
    executables, _build = runner._compile_aot(build_directory, env, 180.0)
    with acquire_domain() as lease:
        env["ROS_DOMAIN_ID"] = str(lease.domain_id)
        monkeypatch.setenv("ROS_DOMAIN_ID", str(lease.domain_id))
        cache = runner._prewarm(cache_root, env, 180.0)
        observer = runner.GraphObserver(
            "action_server_smoke_" + uuid.uuid4().hex[:10])
        try:
            for variant in protocol.VARIANTS:
                sample = runner._run_sample(
                    variant=variant,
                    repetition=1,
                    executables=executables,
                    cache=cache,
                    observer=observer,
                    domain_id=lease.domain_id,
                    env=env,
                    timeout=60.0,
                    warmup_goals=1,
                    measured_goals=2,
                    validate=False,
                )
                report = sample["server_report"]
                assert report["goals_received"] == 3
                assert report["feedback_sent"] == 9
                assert report["measured_checksum"] == 3
                assert report["cpu_time_ns"] > 0
                assert report["cpu_role"] == "server_under_test"
                assert sample["client_report"]["feedback_received"] == 9
                if protocol.VARIANTS[variant]["exact_cpp"]:
                    assert report["boundary_evidence"]["exact_generated_cpp"]
                    assert report["boundary_evidence"]["python_message_conversions"] == 0
                    assert report["boundary_evidence"]["python_serialization_calls"] == 0
        finally:
            observer.close()
