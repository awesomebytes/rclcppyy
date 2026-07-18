"""Contract and live smoke coverage for the timer/executor benchmark."""

from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys
import uuid

import pytest


REPO_ROOT = Path(__file__).resolve().parent.parent
BENCH_DIR = REPO_ROOT / "scripts" / "benchmarks"
SCHEMA_PATH = REPO_ROOT / "schemas" / "timer-executor-benchmark-v1.schema.json"
sys.path.insert(0, str(BENCH_DIR))


def _load(name):
    spec = importlib.util.spec_from_file_location(name, BENCH_DIR / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


protocol = _load("_timer_executor_protocol")
runner = _load("run_timer_executor_benchmark")
DIGEST = "a" * 64


def _artifact(*, cached):
    return {
        "cached": cached,
        "path": "/fixture/timer-probe.so",
        "sha256": DIGEST,
        "size_bytes": 4096,
    }


def _cache():
    def phase(*, cached, pid):
        return {
            "schema": protocol.PREWARM_SCHEMA,
            "pid": pid,
            "loaded_rmw": protocol.RMW,
            "source_id": "1" * 16,
            "artifact": _artifact(cached=cached),
            "stdout_diagnostics": ["fixture"],
        }

    return {
        "isolated_root": True,
        "autopch_disabled": True,
        "fresh_process_per_phase": True,
        "compilation_excluded_from_samples": True,
        "warm_hits_verified": True,
        "persisted_after_run": False,
        "phases": {
            "cold": phase(cached=False, pid=100),
            "warm": phase(cached=True, pid=101),
        },
    }


def _build():
    return {
        "build_type": "Release",
        "private_build_directory": True,
        "build_directory_persisted": False,
        "compiler": "/fixture/c++",
        "compiler_version": "fixture c++ 1.0",
        "compile_command": "c++ -O3 -DNDEBUG timer_executor_aot.cpp",
        "source_sha256": DIGEST,
        "cmake_sha256": DIGEST,
        "compile_commands_sha256": DIGEST,
        "executable_sha256": DIGEST,
        "executable_format": "ELF",
        "build_elapsed_ns": 100,
    }


def _deadline():
    return {
        "signed_ns": {"p50": 10, "p95": 20, "p99": 30, "max": 40},
        "absolute_ns": {"p50": 10, "p95": 20, "p99": 30, "max": 40},
    }


def _sample(variant, repetition=1, index=0):
    token = "timer_" + format(index + 1, "032x")
    node_name = "timer_executor_" + token[6:18]
    pid = 1000 + index
    spec = protocol.VARIANTS[variant]
    cache_marker = {
        "state": "not_applicable",
        "kind": spec["cache_kind"],
    }
    activation = None
    if variant == "compatible-rclcppyy":
        cache_marker = {"state": "activation-only", "kind": "activation-only"}
        activation = {
            "profile": "compatible",
            "timer_status_backend": "python",
            "timer_decision_id": 7,
        }
    elif variant == "native-python-callback":
        cache_marker = {
            "state": "process_warm",
            "kind": "managed-rclcpp-runtime",
        }
    elif variant == "native-cpp-callback":
        artifact = _cache()["phases"]["warm"]["artifact"]
        cache_marker = {
            "state": "prebuilt",
            "kind": "timer-probe-shared-library",
            "path": artifact["path"],
            "sha256": artifact["sha256"],
            "size_bytes": artifact["size_bytes"],
            "hit": True,
            "source_id": "1" * 16,
        }
    elif variant == "aot-staged":
        cache_marker = {"state": "prebuilt", "kind": "aot-binary"}

    ready = {
        "schema": protocol.EVENT_SCHEMA,
        "event": "ready",
        "variant": variant,
        "run_token": token,
        "pid": pid,
        "process_group_id": pid,
        "node_name": node_name,
        "loaded_rmw": protocol.RMW,
        "execution_model": spec["execution_model"],
        "warmup_firings": protocol.WARMUP_FIRINGS,
        "timer_canceled": True,
        "timer_marker": {
            "authority": spec["timer_authority"],
            "implementation": "fixture::SteadyTimer",
            "clock": "steady",
            "period_ns": protocol.PERIOD_NS,
            "callback_language": spec["callback_language"],
        },
        "executor_marker": {
            "authority": spec["executor_authority"],
            "implementation": "fixture::SingleThreadedExecutor",
            "kind": "single_threaded",
            "threads": 1,
        },
        "cache": cache_marker,
    }
    if activation is not None:
        ready["activation"] = activation

    state, checksum = protocol.expected_recurrence(protocol.MEASURED_FIRINGS)
    crossings = spec["python_crossings_per_firing"]
    total = protocol.WARMUP_FIRINGS + protocol.MEASURED_FIRINGS
    report = {
        "schema": protocol.EVENT_SCHEMA,
        "event": "report",
        "variant": variant,
        "run_token": token,
        "pid": pid,
        "process_group_id": pid,
        "warmup_firings": protocol.WARMUP_FIRINGS,
        "measured_firings": protocol.MEASURED_FIRINGS,
        "recurrence_state": state,
        "checksum": checksum,
        "python_callback_count": total * crossings,
        "measured_python_callback_count": protocol.MEASURED_FIRINGS * crossings,
        "python_boundary_crossings": total * crossings,
        "post_cancel_firings": 0,
        "exceptions": 0,
        "cpu_time_ns": 5_000_000,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "wall_duration_ns": 5_100_000_000,
        "scheduled_deadline_error": _deadline(),
        "missed_periods": 0,
        "timer_canceled": True,
        "teardown_clean": True,
        "executor_thread_joined": True,
    }
    observation = {"observed": True, "observations": 2, "elapsed_ns": 100}
    return {
        "schema": protocol.SAMPLE_SCHEMA,
        "case_id": "%s__rep_%d" % (variant, repetition),
        "variant": variant,
        "repetition": repetition,
        "run_token": token,
        "node_name": node_name,
        "worker_pid": pid,
        "worker_process_group_id": pid,
        "ros_domain_id": 77,
        "requested_rmw": protocol.RMW,
        "ros_distro": protocol.ROS_DISTRO,
        "graph": {
            "present_after_ready": True,
            "absent_after_exit": True,
            "presence_observation": dict(observation),
            "absence_observation": dict(observation),
        },
        "worker_ready": ready,
        "worker_armed": {
            "schema": protocol.EVENT_SCHEMA,
            "event": "armed",
            "variant": variant,
            "run_token": token,
            "pid": pid,
            "process_group_id": pid,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "timer_reset": True,
        },
        "worker_report": report,
        "timing": {
            "worker_cpu_ns_per_firing": (
                report["cpu_time_ns"] / protocol.MEASURED_FIRINGS),
            "effective_frequency_hz": (
                protocol.MEASURED_FIRINGS * 1e9 / report["wall_duration_ns"]),
            "scheduled_deadline_error": report["scheduled_deadline_error"],
            "missed_periods": report["missed_periods"],
        },
        "correctness_verified": True,
        "teardown_verified": True,
        "diagnostics": {"stdout": [], "stderr": None},
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
        "command": ["python", "run_timer_executor_benchmark.py"],
        "environment": {"fixture": True},
        "parameters": {
            "variants": list(protocol.VARIANTS),
            "period_ns": protocol.PERIOD_NS,
            "warmup_firings": protocol.WARMUP_FIRINGS,
            "measured_firings": protocol.MEASURED_FIRINGS,
            "repetitions": protocol.REPETITIONS,
            "requested_rmw": protocol.RMW,
            "ros_distro": protocol.ROS_DISTRO,
            "execution_order": order,
        },
        "isolation": {
            "fresh_worker_process_per_sample": True,
            "fresh_process_group_per_sample": True,
            "unique_node_per_sample": True,
            "one_leased_domain_per_run": True,
            "rotating_variant_order": True,
            "ros_domain_id": 77,
        },
        "aot_build": _build(),
        "cache": _cache(),
        "source_files": {
            "runner": DIGEST,
            "protocol": DIGEST,
            "worker": DIGEST,
            "aot_source": DIGEST,
            "aot_cmake": DIGEST,
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


def test_wrapping_recurrence_is_fixed():
    assert protocol.expected_recurrence(5_000) == (
        15_774_110_614_102_427_966,
        13_395_055_762_267_132_380,
    )
    assert protocol.deadline_summary([-4, -1, 2, 8]) == {
        "signed_ns": {"p50": -1, "p95": 8, "p99": 8, "max": 8},
        "absolute_ns": {"p50": 2, "p95": 8, "p99": 8, "max": 8},
    }


@pytest.mark.parametrize("variant", tuple(protocol.VARIANTS))
def test_each_variant_satisfies_the_exact_sample_contract(variant):
    protocol.validate_sample(_sample(variant), _cache(), _build())


@pytest.mark.parametrize(
    ("variant", "path", "value"),
    [
        ("stock-rclpy", ("requested_rmw",), "rmw_fastrtps_cpp"),
        ("stock-rclpy", ("run_token",), "invalid"),
        ("stock-rclpy", ("worker_ready", "warmup_firings"), 499),
        ("stock-rclpy", ("worker_ready", "timer_marker", "clock"), "system"),
        ("stock-rclpy", ("worker_ready", "timer_marker", "period_ns"), 2_000_000),
        ("stock-rclpy", ("worker_ready", "timer_marker", "authority"), "cpp"),
        ("stock-rclpy", ("worker_ready", "executor_marker", "threads"), 2),
        ("stock-rclpy", ("worker_armed", "timer_reset"), False),
        ("stock-rclpy", ("worker_report", "checksum"), 0),
        ("stock-rclpy", ("worker_report", "python_boundary_crossings"), 0),
        ("stock-rclpy", ("worker_report", "post_cancel_firings"), 1),
        ("stock-rclpy", ("worker_report", "cpu_clock"), "CLOCK_THREAD_CPUTIME_ID"),
        ("stock-rclpy", ("worker_report", "missed_periods"), 1),
        ("stock-rclpy", ("worker_report", "teardown_clean"), False),
        ("stock-rclpy", ("graph", "present_after_ready"), False),
        ("stock-rclpy", ("graph", "absence_observation", "observed"), False),
        ("native-cpp-callback", ("worker_report", "python_callback_count"), 1),
        ("native-cpp-callback", ("worker_ready", "cache", "hit"), False),
        ("aot-staged", ("worker_ready", "cache", "state"), "process_warm"),
    ],
)
def test_sample_contract_rejects_tainted_evidence(variant, path, value):
    sample = _sample(variant)
    _replace(sample, path, value)
    with pytest.raises(ValueError):
        protocol.validate_sample(sample, _cache(), _build())


def test_aot_sample_rejects_non_release_build():
    build = _build()
    build["compile_command"] = "c++ -O2 timer_executor_aot.cpp"
    with pytest.raises(ValueError, match="Release optimization"):
        protocol.validate_sample(_sample("aot-staged"), _cache(), build)


@pytest.mark.parametrize(
    ("path", "value"),
    [
        (("phases", "cold", "artifact", "cached"), True),
        (("phases", "warm", "source_id"), "2" * 16),
        (("phases", "warm", "artifact", "sha256"), "b" * 64),
        (("warm_hits_verified",), False),
    ],
)
def test_cache_contract_rejects_invalid_cold_warm_evidence(path, value):
    cache = _cache()
    _replace(cache, path, value)
    with pytest.raises(ValueError):
        protocol.validate_cache(cache)


def test_complete_document_and_rotating_order_validate():
    document = _document()
    protocol.validate_document(document)
    assert len(document["results"]) == 25
    assert document["parameters"]["execution_order"][:6] == [
        "stock-rclpy__rep_1",
        "compatible-rclcppyy__rep_1",
        "native-python-callback__rep_1",
        "native-cpp-callback__rep_1",
        "aot-staged__rep_1",
        "compatible-rclcppyy__rep_2",
    ]


@pytest.mark.parametrize(
    ("path", "value"),
    [
        (("claims", "enabled"), True),
        (("interpretation", "enabled"), True),
        (("mode",), "smoke"),
        (("parameters", "period_ns"), 2_000_000),
        (("parameters", "execution_order"), []),
        (("isolation", "fresh_process_group_per_sample"), False),
        (("source_files", "worker"), "invalid"),
    ],
)
def test_document_contract_rejects_interpretation_or_matrix_drift(path, value):
    document = _document()
    _replace(document, path, value)
    with pytest.raises(ValueError):
        protocol.validate_document(document)


def test_failure_rows_preserve_exact_matrix_identity():
    document = _document()
    removed = document["results"].pop()
    document["failures"].append({
        "case_id": removed["case_id"],
        "variant": removed["variant"],
        "repetition": removed["repetition"],
        "error": "fixture failure",
    })
    protocol.validate_document(document)
    document["failures"][0]["case_id"] = "wrong"
    with pytest.raises(ValueError, match="failure case"):
        protocol.validate_document(document)


def test_schema_encodes_the_same_negative_contracts():
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    properties = schema["properties"]
    assert properties["schema"]["const"] == protocol.SCHEMA_ID
    assert properties["claims"]["const"]["enabled"] is False
    assert properties["interpretation"]["const"]["enabled"] is False
    assert properties["parameters"]["properties"]["requested_rmw"]["const"] == (
        protocol.RMW)
    report = schema["$defs"]["report"]["allOf"][1]["properties"]
    assert report["cpu_clock"]["const"] == "CLOCK_PROCESS_CPUTIME_ID"
    assert report["post_cancel_firings"]["const"] == 0
    patterns = schema["$defs"]["aot_build"]["properties"][
        "compile_command"]["allOf"]
    assert {item["pattern"] for item in patterns} == {
        "(^| )-O3( |$)", "(^| )-DNDEBUG( |$)"}


def test_json_schema_accepts_fixture_and_rejects_claims_when_available():
    jsonschema = pytest.importorskip("jsonschema")
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    jsonschema.Draft202012Validator.check_schema(schema)
    jsonschema.validate(_document(), schema)
    invalid = _document()
    invalid["claims"]["enabled"] = True
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(invalid, schema)


@pytest.mark.skipif(shutil.which("cmake") is None, reason="cmake is unavailable")
def test_private_aot_helper_builds_release_o3_ndebug(tmp_path):
    executable, evidence = runner._compile_aot(tmp_path, os.environ.copy(), 120.0)
    assert executable.read_bytes()[:4] == b"\x7fELF"
    assert evidence["build_type"] == "Release"
    assert "-O3" in evidence["compile_command"]
    assert "-DNDEBUG" in evidence["compile_command"]


@pytest.mark.skipif(
    os.environ.get("ROS_DISTRO") != protocol.ROS_DISTRO,
    reason="live timer smoke requires ROS 2 Jazzy",
)
def test_live_cyclone_stock_timer_graph_and_protocol(monkeypatch):
    from _domain_lease import acquire_domain

    token = "timer_" + uuid.uuid4().hex
    node_name = "timer_executor_" + token[6:18]
    env = os.environ.copy()
    env["RMW_IMPLEMENTATION"] = protocol.RMW
    env["PYTHONUNBUFFERED"] = "1"
    monkeypatch.setenv("RMW_IMPLEMENTATION", protocol.RMW)
    process = None
    observer = None
    with acquire_domain() as lease:
        env["ROS_DOMAIN_ID"] = str(lease.domain_id)
        monkeypatch.setenv("ROS_DOMAIN_ID", str(lease.domain_id))
        observer = runner.GraphObserver("timer_executor_observer_" + uuid.uuid4().hex[:12])
        try:
            process = runner._spawn([
                sys.executable,
                "-u",
                str(runner.WORKER),
                "--variant",
                "stock-rclpy",
                "--node-name",
                node_name,
                "--run-token",
                token,
                "--warmup-firings",
                "5",
                "--measured-firings",
                "20",
            ], env)
            ready, _ = runner._read_document(process, 30.0, "timer smoke ready")
            assert ready["loaded_rmw"] == protocol.RMW
            assert ready["pid"] == process.pid
            assert ready["process_group_id"] == process.pid
            observer.wait_for(node_name, present=True, timeout=10.0)
            runner._write_start(process, "timer smoke")
            armed, _ = runner._read_document(process, 30.0, "timer smoke armed")
            report, _ = runner._read_document(process, 30.0, "timer smoke report")
            runner._finish(process, 30.0, "timer smoke")
            observer.wait_for(node_name, present=False, timeout=10.0)
            assert armed["cpu_clock"] == "CLOCK_PROCESS_CPUTIME_ID"
            assert (report["recurrence_state"], report["checksum"]) == (
                protocol.expected_recurrence(20))
            assert report["python_boundary_crossings"] == 25
            assert report["post_cancel_firings"] == 0
            assert report["teardown_clean"] is True
        finally:
            runner._stop_process(process)
            if observer is not None:
                observer.close()
