"""Tests for the local repeated compatibility evidence gate."""

import copy
import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "benchmarks" / "analyze_compatibility_evidence.py"
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location("compatibility_evidence", SCRIPT)
evidence = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(evidence)


def _marker(role, backend):
    return {
        "schema": "rclcppyy.benchmark-backend/v1",
        "role": role,
        "backend": backend,
        "evidence": "fixture_entity",
        "metadata": {"entity_type": role},
    }


def _row(backend, workload, repetition):
    publisher_backend = "python" if backend == "rclpy" else "cpp"
    contract = {
        "small-string": "std_msgs/String:sequence-timestamp-padding/v1",
        "nested-header": "std_msgs/Header:stamp-frame-sequence-padding/v1",
    }[workload]
    candidate = backend == "rclcppyy"
    received = 1000 - repetition if candidate else 1000
    latency = 120.0 + repetition if candidate else 100.0 + repetition
    publisher_cpu = 14.0 + repetition if candidate else 10.0 + repetition
    return {
        "case_id": f"{backend}__{workload}__1000hz__0b",
        "backend": backend,
        "workload": workload,
        "target_rate_hz": 1000,
        "payload_bytes": 0,
        "backend_verified": True,
        "expected_backends": {"publisher": publisher_backend, "subscriber": "python"},
        "publisher_backend": _marker("publisher", publisher_backend),
        "subscriber_backend": _marker("subscriber", "python"),
        "wire_values": {
            "schema": "rclcppyy.benchmark-wire-values/v1",
            "contract_id": contract,
            "expected_payload_bytes": 0,
            "checked_messages": received,
            "violations": 0,
            "violation_types": {},
            "value_contract_verified": True,
        },
        "messages": {
            "received": received,
            "dropped": 0,
            "effective_rate_hz": float(received),
        },
        "latency_us": {
            "count": received,
            "mean": latency / 2,
            "p50": latency / 2,
            "p95": latency * 0.8,
            "p99": latency,
            "min": 1.0,
            "max": latency * 1.2,
        },
        "cpu_pct": {
            "publisher": {"mean": publisher_cpu},
            "subscriber": {"mean": 12.0},
        },
    }


def _document(repetition):
    workloads = ["small-string", "nested-header"]
    return {
        "schema": "rclcppyy.benchmark/v3",
        "generated_at": f"2026-07-18T12:00:0{repetition}Z",
        "command": ["pixi", "run", "bench"],
        "environment": {
            "source": {
                "repository": "/fixture/checkout",
                "commit": "abc123",
                "dirty": False,
            },
            "host": {
                "architecture": "x86_64",
                "system": "Linux",
                "kernel": "fixture-kernel",
                "cpu_model": "Fixture CPU",
                "logical_cpu_count": 8,
            },
            "runtime": {"python": "3.12.0", "packages": {"rclcppyy": "0.3.0"}},
            "source_dependencies": {
                "rclcppyy": {
                    "repository": "/fixture/checkout",
                    "commit": "abc123",
                    "dirty": False,
                    "is_benchmark_repository": True,
                },
                "rclcpp_kit": {
                    "repository": "/fixture/suite",
                    "commit": "def456",
                    "dirty": False,
                    "is_benchmark_repository": False,
                },
            },
            "ros": {
                "distribution": "jazzy",
                "rmw_implementation": "rmw_cyclonedds_cpp",
                "domain_id": str(40 + repetition),
            },
            "cache": {"CPPYY_KIT_NO_CACHE": None},
        },
        "benchmark": {
            "name": "pubsub_backend_workload_matrix",
            "mode": "measurement",
            "performance_claims_allowed": False,
            "matrix": {
                "run_token": f"run_{repetition}",
                "ros_domain_id": 40 + repetition,
                "backends": ["rclpy", "rclcppyy"],
                "workloads": workloads,
                "target_rates_hz": [1000],
                "payload_bytes": [0],
                "duration_s": 15.0,
                "sample_hz": 4.0,
                "warmup_timeout_s": 60.0,
                "case_count": 4,
            },
            "statistics": {"latency": "nearest-rank", "cpu": "sample mean"},
        },
        "results": [
            _row(backend, workload, repetition)
            for backend in ("rclpy", "rclcppyy")
            for workload in workloads
        ],
        "failures": [],
    }


def test_repeated_gate_maps_route_parity_and_negative_observations_deterministically():
    documents = [_document(index) for index in range(1, 6)]

    first = evidence.analyze_documents(documents)
    second = evidence.analyze_documents(list(reversed(documents)))

    assert first == second
    assert first["decision"] == "evidence_complete"
    assert first["performance_claims_allowed"] is False
    assert first["interpretation_allowed"] is False
    assert first["evidence"]["benchmark_schema"] == "rclcppyy.benchmark/v3"
    assert first["evidence"]["environment"]["source"]["commit"] == "abc123"
    assert "domain_id" not in first["evidence"]["environment"]["ros"]
    assert "run_token" not in first["evidence"]["matrix"]
    route = first["routes"][0]
    assert route["path_id"] == "compatibility.publisher.same_handle_publish"
    assert route["backend_markers_verified"] is True
    assert route["wire_result_parity"] is True
    assert route["advertised_performance_benefit"] is False
    assert route["performance_conclusion"] == "not_established"
    assert route["workloads"] == ["nested-header", "small-string"]
    comparison = first["comparisons"][0]
    assert comparison["wire_result_parity"] is True
    assert comparison["metrics"]["cpu_pct.publisher.mean"]["consistency"] == (
        "stock_better_in_all_repetitions")


def test_gate_rejects_wire_violations_backend_drift_and_incomplete_workloads():
    documents = [_document(index) for index in range(1, 6)]
    documents[0]["results"][0]["wire_values"]["violations"] = 1
    documents[0]["results"][0]["wire_values"]["violation_types"] = {"ValueError": 1}
    documents[0]["results"][0]["wire_values"]["value_contract_verified"] = False
    with pytest.raises(ValueError, match="verified wire values"):
        evidence.analyze_documents(documents)

    documents = [_document(index) for index in range(1, 6)]
    documents[2]["results"][3]["publisher_backend"]["backend"] = "python"
    with pytest.raises(ValueError, match="observed publisher backend mismatch"):
        evidence.analyze_documents(documents)

    documents = [_document(index) for index in range(1, 6)]
    for document in documents:
        document["benchmark"]["matrix"]["workloads"] = ["small-string"]
        document["benchmark"]["matrix"]["case_count"] = 2
        document["results"] = [
            row for row in document["results"] if row["workload"] == "small-string"]
    with pytest.raises(ValueError, match="cover every workload"):
        evidence.analyze_documents(documents)


def test_gate_rejects_dirty_or_changing_environment_and_too_few_runs():
    documents = [_document(index) for index in range(1, 6)]
    documents[0]["environment"]["source"]["dirty"] = True
    with pytest.raises(ValueError, match="source must be clean"):
        evidence.analyze_documents(documents)

    documents = [_document(index) for index in range(1, 6)]
    documents[3]["environment"]["host"]["cpu_model"] = "Other CPU"
    with pytest.raises(ValueError, match="machine/runtime environment changed"):
        evidence.analyze_documents(documents)

    documents = [_document(index) for index in range(1, 6)]
    documents[1]["environment"]["source_dependencies"]["rclcpp_kit"]["dirty"] = True
    with pytest.raises(ValueError, match="source dependency rclcpp_kit must be clean"):
        evidence.analyze_documents(documents)

    with pytest.raises(ValueError, match="not enough"):
        evidence.analyze_documents([_document(1), _document(2)])


def test_cli_writes_evidence_and_portable_schema_forbids_claims(tmp_path):
    paths = []
    for index in range(1, 6):
        path = tmp_path / f"run-{index}.json"
        path.write_text(json.dumps(_document(index)), encoding="utf-8")
        paths.append(path)
    output = tmp_path / "evidence.json"

    exit_code = evidence.main([
        "--output", str(output), *(str(path) for path in paths)])

    assert exit_code == 0
    assert json.loads(output.read_text(encoding="utf-8"))["decision"] == "evidence_complete"
    schema = json.loads(
        (ROOT / "schemas" / "compatibility-performance-evidence-v1.schema.json")
        .read_text(encoding="utf-8"))
    assert schema["properties"]["performance_claims_allowed"] == {"const": False}
    assert schema["properties"]["interpretation_allowed"] == {"const": False}


def test_analysis_does_not_mutate_raw_documents():
    documents = [_document(index) for index in range(1, 6)]
    before = copy.deepcopy(documents)

    evidence.analyze_documents(documents)

    assert documents == before
