"""Contract and live smoke coverage for the SetBool callback benchmark."""

from __future__ import annotations

import importlib.util
import inspect
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parent.parent
BENCH_DIR = REPO_ROOT / "scripts" / "benchmarks"
sys.path.insert(0, str(BENCH_DIR))


def _load(name):
    spec = importlib.util.spec_from_file_location(name, BENCH_DIR / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


protocol = _load("_service_callback_protocol")
worker = _load("service_callback_worker")
runner = _load("run_service_callback_benchmark")
DIGEST = "a" * 64
RMW = "rmw_cyclonedds_cpp"
MESSAGES = 4
WARMUP = 2


def _artifact(*, cached):
    return {
        "cached": cached,
        "reason": "hit" if cached else "miss-built",
        "path": "/fixture/cache/artifact.so",
        "sha256": DIGEST,
        "size_bytes": 4096,
    }


def _cache():
    def phase(cached):
        return {
            "schema": protocol.PREWARM_SCHEMA,
            "pid": 42 if not cached else 43,
            "loaded_rmw": RMW,
            "native_cpp_source_id": "1" * 16,
            "artifacts": {
                "python_bridge": _artifact(cached=cached),
                "native_cpp_service": _artifact(cached=cached),
            },
            "stdout_diagnostics": ["fixture initialization"],
        }

    return {
        "isolated_root": True,
        "autopch_disabled": True,
        "fresh_process_per_phase": True,
        "compilation_excluded_from_samples": True,
        "warm_hits_verified": True,
        "persisted_after_run": False,
        "phases": {"cold": phase(False), "warm": phase(True)},
    }


def _build():
    return {
        "build_type": "Release",
        "private_build_directory": True,
        "build_directory_persisted": False,
        "compiler": "/fixture/c++",
        "compiler_version": "fixture c++ 1.0",
        "compile_command": "c++ -O3 -DNDEBUG service_callback_aot.cpp",
        "source_sha256": DIGEST,
        "cmake_sha256": DIGEST,
        "compile_commands_sha256": DIGEST,
        "executable_sha256": DIGEST,
        "executable_format": "ELF",
        "build_elapsed_ns": 100,
    }


def _rss_guard():
    return {
        "kind": "post-warmup-peak-rss-growth",
        "unit": "bytes",
        "baseline_peak_bytes": 10_000,
        "final_peak_bytes": 12_000,
        "growth_bytes": 2_000,
        "limit_bytes": protocol.RSS_LIMIT_BYTES,
        "within_limit": True,
    }


def _backend_marker(evidence):
    return {
        "schema": "rclcppyy.benchmark-backend/v1",
        "role": "server",
        "backend": "python",
        "evidence": evidence,
        "metadata": {"fixture": True},
    }


def _sample(variant, index=0):
    token = "run_" + format(index + 1, "032x")
    server_pid = 2000 + index * 2
    client_pid = server_pid + 1
    server_node = "service_server_%d" % index
    client_node = "service_client_%d" % index
    service_name = "/fixture/" + token
    spec = protocol.VARIANTS[variant]
    ready = {
        "schema": protocol.SERVER_SCHEMA,
        "event": "ready",
        "variant": variant,
        "run_token": token,
        "pid": server_pid,
        "process_group_id": server_pid,
        "node_name": server_node,
        "loaded_rmw": RMW,
        "execution_model": spec["model"],
        "service_authority": spec["authority"],
    }
    if variant == "aot-staged":
        ready["cache"] = {"state": "prebuilt", "kind": "aot-binary"}
        ready["entity_type"] = "rclcpp::Service<std_srvs::srv::SetBool>"
    elif variant in ("stock-rclpy", "compatible-rclcppyy"):
        ready["cache"] = {"state": "not_applicable", "kind": spec["cache"]}
        ready["entity_type"] = "rclpy.service.Service"
        ready["backend_marker"] = _backend_marker(
            "stock_rclpy_entity" if variant == "stock-rclpy"
            else "rclcppyy_status_entity")
    else:
        artifact_name = (
            "python_bridge" if variant == "native-python-callback"
            else "native_cpp_service")
        ready["cache"] = {
            **_cache()["phases"]["warm"]["artifacts"][artifact_name],
            "state": "prebuilt",
            "kind": spec["cache"],
        }
        ready["entity_type"] = "rclcpp::Service<std_srvs::srv::SetBool>"

    total = WARMUP + MESSAGES
    python = spec["python"]
    report = {
        "schema": protocol.SERVER_SCHEMA,
        "event": "report",
        "variant": variant,
        "run_token": token,
        "warmup_requests": WARMUP,
        "total_requests": total,
        "measured_requests": MESSAGES,
        "true_total": protocol.true_requests(1, total),
        "true_measured": protocol.true_requests(WARMUP + 1, MESSAGES),
        "response_checksum": protocol.response_checksum(WARMUP + 1, MESSAGES),
        "python_callback_count_total": total if python else 0,
        "python_boundary_crossings_measured": MESSAGES if python else 0,
        "exceptions": 0,
        "pending_requests": 0,
        "cpu_time_ns": 800,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "rss_guard": _rss_guard(),
        "correct": True,
        "teardown_clean": True,
    }
    if variant == "native-cpp-callback":
        report.update(
            true_total=None, true_measured=None, response_checksum=None)
    latency = [100, 200, 300, 400]
    client = {
        "schema": protocol.CLIENT_SCHEMA,
        "event": "measured",
        "run_token": token,
        "pid": client_pid,
        "process_group_id": client_pid,
        "loaded_rmw": RMW,
        "execution_model": "identical-release-aot-one-outstanding-client",
        "messages": MESSAGES,
        "response_checksum": protocol.response_checksum(WARMUP + 1, MESSAGES),
        "elapsed_ns": 2_000,
        "cpu_time_ns": 600,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "rss_guard": _rss_guard(),
        "topology_verified": True,
        "qos_verified": True,
        "qos": dict(protocol.QOS),
        "server_node": server_node,
        "service_name": service_name,
        "service_type": "std_srvs/srv/SetBool",
        "pending_requests": 0,
        "latency_ns": latency,
    }
    return {
        "schema": protocol.SAMPLE_SCHEMA,
        "case_id": "%s__rep_1" % variant,
        "variant": variant,
        "repetition": 1,
        "run_token": token,
        "ros_domain_id": 77,
        "requested_rmw": RMW,
        "qos": dict(protocol.QOS),
        "server_pid": server_pid,
        "client_pid": client_pid,
        "topology": {
            "process_count": 2,
            "fresh_process_groups": True,
            "server_node": server_node,
            "client_node": client_node,
            "service_name": service_name,
        },
        "server_ready": ready,
        "server_armed": {
            "schema": protocol.SERVER_SCHEMA,
            "event": "armed",
            "variant": variant,
            "run_token": token,
            "pid": server_pid,
            "process_group_id": server_pid,
            "warmup_requests": WARMUP,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        },
        "server_report": report,
        "client_result": client,
        "client_teardown": {
            "schema": protocol.CLIENT_SCHEMA,
            "event": "teardown",
            "run_token": token,
            "pid": client_pid,
            "process_group_id": client_pid,
            "endpoint_disappeared": True,
            "teardown_clean": True,
        },
        "timing": {
            "server_cpu_time_ns": 800,
            "server_cpu_ns_per_request": 800 / MESSAGES,
            "client_cpu_time_ns": 600,
            "client_cpu_ns_per_request": 600 / MESSAGES,
            "elapsed_ns": 2_000,
            "requests_per_second": MESSAGES * 1e9 / 2_000,
            "rtt_ns": protocol.latency_summary(latency),
        },
        "backend_verified": True,
        "correctness_verified": True,
        "teardown_verified": True,
        "diagnostics": {},
    }


def _parameters():
    return {
        "variants": list(protocol.VARIANTS),
        "messages": MESSAGES,
        "warmup_requests": WARMUP,
        "repetitions": 1,
        "requested_rmw": RMW,
        "qos": dict(protocol.QOS),
        "execution_order": ["fixture"],
    }


@pytest.mark.parametrize("variant", protocol.VARIANTS)
def test_each_service_variant_satisfies_the_strict_contract(variant):
    protocol.validate_sample(
        _sample(variant), _parameters(), _build(), _cache())


@pytest.mark.parametrize(
    ("variant", "mutation", "error"),
    [
        ("compatible-rclcppyy", lambda row: row["server_ready"].update(
            service_authority="cpp"), "identity"),
        ("stock-rclpy", lambda row: row["server_report"].update(
            python_boundary_crossings_measured=0), "crossing"),
        ("native-cpp-callback", lambda row: row["server_report"].update(
            python_boundary_crossings_measured=1), "crossing"),
        ("native-python-callback", lambda row: row["server_ready"]["cache"].update(
            sha256="b" * 64), "differs from warm manifest"),
        ("aot-staged", lambda row: row["client_result"]["qos"].update(
            depth=1), "parity/topology"),
        ("aot-staged", lambda row: row["client_result"].update(
            response_checksum=1), "parity/topology"),
        ("stock-rclpy", lambda row: row.update(
            client_pid=row["server_pid"]), "distinct process"),
        ("aot-staged", lambda row: row["client_teardown"].update(
            endpoint_disappeared=False), "endpoint disappearance"),
        ("aot-staged", lambda row: row["server_armed"].update(
            warmup_requests=99), "armed"),
        ("stock-rclpy", lambda row: row["server_report"]["rss_guard"].update(
            within_limit=False), "RSS"),
    ],
)
def test_tainted_service_evidence_is_rejected(variant, mutation, error):
    sample = _sample(variant, 1)
    mutation(sample)
    with pytest.raises(ValueError, match=error):
        protocol.validate_sample(sample, _parameters(), _build(), _cache())


def test_cold_and_warm_artifacts_must_have_identical_content():
    cache = _cache()
    cache["phases"]["warm"]["artifacts"]["python_bridge"]["sha256"] = "b" * 64
    with pytest.raises(ValueError, match="identity changed"):
        protocol.validate_cache(cache, RMW)


def test_python_compatibility_lanes_share_one_callback_implementation():
    source = inspect.getsource(worker._run_python_server)
    dispatcher = inspect.getsource(worker._run_server)
    assert source.count("def callback(request, response):") == 1
    assert "if activate:" in source
    assert "active.enable_cpp_acceleration()" in source
    assert "_run_python_server(args, False)" in dispatcher
    assert "_run_python_server(args, True)" in dispatcher


def test_runner_rejects_any_armed_record_drift():
    sample = _sample("stock-rclpy")
    armed = sample["server_armed"]
    runner._validate_armed_before_measurement(
        armed,
        variant="stock-rclpy",
        token=sample["run_token"],
        server_pid=sample["server_pid"],
        warmup=WARMUP,
    )
    armed["cpu_clock"] = "wall"
    with pytest.raises(RuntimeError, match="armed evidence"):
        runner._validate_armed_before_measurement(
            armed,
            variant="stock-rclpy",
            token=sample["run_token"],
            server_pid=sample["server_pid"],
            warmup=WARMUP,
        )


def test_document_and_portable_schema_forbid_performance_claims(monkeypatch):
    monkeypatch.setattr(protocol, "environment_metadata", lambda _root: {
        "source": {"commit": "1" * 40, "dirty": True},
        "source_dependencies": {
            "rclcpp_kit": {"commit": "2" * 40, "dirty": False},
        },
    })
    results = [
        _sample(variant, index)
        for index, variant in enumerate(protocol.VARIANTS)
    ]
    document = protocol.build_document(
        repo_root=REPO_ROOT,
        mode="measurement",
        parameters=_parameters(),
        isolation={
            "fresh_process_pair_per_sample": True,
            "two_process_groups_per_sample": True,
            "unique_service_per_sample": True,
            "one_leased_domain_per_run": True,
            "rotating_variant_order": True,
            "ros_domain_id": 77,
        },
        aot_build=_build(),
        cache=_cache(),
        source_files={"runner": DIGEST},
        results=results,
        failures=[],
        command=["service-callback-bench"],
    )
    assert document["benchmark"]["performance_claims_allowed"] is False
    assert document["comparison"]["interpretation_allowed"] is False
    assert "rss" not in json.dumps(document["comparison"]).lower()
    assert json.loads(protocol.dumps(document)) == document

    schema = json.loads((
        REPO_ROOT / "schemas" / "service-callback-benchmark-v1.schema.json"
    ).read_text(encoding="utf-8"))
    assert schema["properties"]["benchmark"]["properties"][
        "performance_claims_allowed"] == {"const": False}
    assert schema["properties"]["comparison"]["properties"][
        "interpretation_allowed"] == {"const": False}
    assert schema["properties"]["benchmark"]["properties"]["parameters"][
        "properties"]["requested_rmw"] == {"const": RMW}
    assert len(schema["properties"]["results"]["items"]["allOf"]) == 5


def test_all_service_variants_run_with_one_aot_client_and_exact_parity(tmp_path):
    output = tmp_path / "service-callback-smoke.json"
    environment = os.environ.copy()
    environment["RMW_IMPLEMENTATION"] = RMW
    process = subprocess.run(
        [
            sys.executable,
            str(BENCH_DIR / "run_service_callback_benchmark.py"),
            "--smoke",
            "--messages", "4",
            "--warmup-requests", "2",
            "--repetitions", "1",
            "--output", str(output),
            "--json",
        ],
        cwd=REPO_ROOT,
        env=environment,
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert process.returncode == 0, (
        f"stdout:\n{process.stdout}\nstderr:\n{process.stderr}")
    document = json.loads(process.stdout)
    assert document == json.loads(output.read_text(encoding="utf-8"))
    assert document["failures"] == []
    assert document["benchmark"]["mode"] == "smoke"
    assert document["benchmark"]["parameters"]["requested_rmw"] == RMW
    assert document["benchmark"]["performance_claims_allowed"] is False
    assert document["comparison"]["interpretation_allowed"] is False
    assert {row["variant"] for row in document["results"]} == set(
        protocol.VARIANTS)
    assert len({
        pid for row in document["results"]
        for pid in (row["server_pid"], row["client_pid"])
    }) == 10
    for row in document["results"]:
        report = row["server_report"]
        assert report["total_requests"] == 6
        assert report["measured_requests"] == 4
        assert row["client_result"]["messages"] == 4
        assert row["client_result"]["response_checksum"] == 250
        assert len(row["client_result"]["latency_ns"]) == 4
        assert row["client_teardown"]["endpoint_disappeared"] is True
    crossings = {
        row["variant"]: row["server_report"][
            "python_boundary_crossings_measured"]
        for row in document["results"]
    }
    assert crossings == {
        "stock-rclpy": 4,
        "compatible-rclcppyy": 4,
        "native-python-callback": 4,
        "native-cpp-callback": 0,
        "aot-staged": 0,
    }
