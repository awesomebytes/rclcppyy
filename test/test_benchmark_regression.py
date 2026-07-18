"""Tests for repeated-run architecture-specific regression gates."""

import importlib.util
import json
from pathlib import Path
import sys

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "benchmarks" / "compare_regressions.py"
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location("benchmark_regression", SCRIPT)
regression = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(regression)


def _marker(role, backend):
    return {
        "schema": "rclcppyy.benchmark-backend/v1",
        "role": role,
        "backend": backend,
        "evidence": "fixture_entity",
        "metadata": {"entity_type": role},
    }


def _row(backend, p99, rate=1000.0):
    expected = {
        "publisher": "python",
        "subscriber": "python",
    }
    return {
        "case_id": f"{backend}__small_string__1000hz__0b",
        "backend": backend,
        "workload": "small-string",
        "target_rate_hz": 1000,
        "payload_bytes": 0,
        "backend_verified": True,
        "expected_backends": expected,
        "publisher_backend": _marker("publisher", expected["publisher"]),
        "subscriber_backend": _marker("subscriber", expected["subscriber"]),
        "messages": {"received": 1000, "dropped": 0, "effective_rate_hz": rate},
        "wire_values": {
            "schema": "rclcppyy.benchmark-wire-values/v1",
            "contract_id": "std_msgs/String:sequence-timestamp-padding/v1",
            "expected_payload_bytes": 0,
            "checked_messages": 1000,
            "violations": 0,
            "violation_types": {},
            "value_contract_verified": True,
        },
        "latency_us": {
            "count": 1000,
            "mean": p99 / 2,
            "p50": p99 / 2,
            "p95": p99 * 0.8,
            "p99": p99,
            "min": 1.0,
            "max": p99 * 1.2,
        },
        "cpu_pct": {
            "publisher": {"mean": 10.0},
            "subscriber": {"mean": 12.0},
        },
    }


def _document(index, candidate_p99=102.0, candidate_rate=1000.0):
    return {
        "schema": "rclcppyy.benchmark/v3",
        "generated_at": f"2026-07-18T12:00:0{index}Z",
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
            "runtime": {
                "python": "3.12.0",
                "python_implementation": "CPython",
                "python_abi": "cpython-312",
                "packages": {"rclcppyy": "0.3.0"},
            },
            "ros": {
                "distribution": "jazzy",
                "rmw_implementation": "rmw_cyclonedds_cpp",
                "domain_id": str(40 + index),
                "automatic_discovery_range": "LOCALHOST",
            },
            "cache": {"CPPYY_KIT_NO_CACHE": None},
        },
        "benchmark": {
            "name": "pubsub_backend_workload_matrix",
            "mode": "measurement",
            "performance_claims_allowed": False,
            "matrix": {
                "run_token": f"run_{index}",
                "ros_domain_id": 40 + index,
                "backends": ["rclpy", "rclcppyy"],
                "workloads": ["small-string"],
                "target_rates_hz": [1000],
                "payload_bytes": [0],
                "duration_s": 15.0,
                "sample_hz": 4.0,
                "warmup_timeout_s": 60.0,
                "case_count": 2,
            },
            "statistics": {"latency": "nearest-rank", "cpu": "sample mean"},
        },
        "results": [
            _row("rclpy", 100.0, 1000.0),
            _row("rclcppyy", candidate_p99, candidate_rate),
        ],
        "failures": [],
    }


def _budget(status="reviewed"):
    budget = {
        "schema": "rclcppyy.benchmark-regression-budget/v1",
        "architecture": "x86_64",
        "status": status,
        "minimum_repetitions": 5,
        "machine": {
            "cpu_model": "Fixture CPU" if status == "reviewed" else None,
            "logical_cpu_count": 8 if status == "reviewed" else None,
        },
        "environment": {
            "ros_distribution": "jazzy",
            "rmw_implementation": "rmw_cyclonedds_cpp",
        },
        "comparisons": [],
    }
    if status == "reviewed":
        budget.update({"reviewed_at": "2026-07-18T12:00:00Z", "reviewed_by": "fixture"})
        budget["comparisons"] = [{
            "id": "compatibility_vs_stock",
            "candidate_backend": "rclcppyy",
            "reference_backend": "rclpy",
            "selectors": {
                "workloads": ["small-string"],
                "target_rates_hz": [1000],
                "payload_bytes": [0],
            },
            "metrics": {
                "latency_us.p99": {"max_ratio": 1.10},
                "messages.effective_rate_hz": {"min_ratio": 0.95},
            },
        }]
    return budget


def test_reviewed_budget_uses_robust_paired_median_and_is_deterministic():
    values = [100.0, 105.0, 500.0, 102.0, 101.0]
    documents = [_document(index, value) for index, value in enumerate(values, 1)]

    first = regression.compare_documents(documents, _budget())
    second = regression.compare_documents(list(reversed(documents)), _budget())

    assert first == second
    assert first["decision"] == "pass"
    metric_case = first["comparisons"][0]["metrics"]["latency_us.p99"]["cases"][0]
    assert metric_case["paired_ratios"] == [1.0, 1.05, 5.0, 1.02, 1.01]
    assert metric_case["median_ratio"] == 1.02
    aggregate = next(
        item for item in first["aggregates"] if item["case"]["backend"] == "rclcppyy")
    assert aggregate["metrics"]["latency_us.p99"]["median"] == 102.0


def test_reviewed_budget_fails_a_relative_regression():
    documents = [_document(index, 125.0) for index in range(1, 6)]

    result = regression.compare_documents(documents, _budget())

    assert result["decision"] == "fail"
    assert result["violations"] == [{
        "comparison_id": "compatibility_vs_stock",
        "metric": "latency_us.p99",
        "dimensions": {
            "workload": "small-string",
            "target_rate_hz": 1000,
            "payload_bytes": 0,
        },
        "direction": "max_ratio",
        "limit": 1.1,
        "median_ratio": 1.25,
    }]


def test_reviewed_budget_checks_higher_is_better_metrics():
    documents = [_document(index, 100.0, 900.0) for index in range(1, 6)]

    result = regression.compare_documents(documents, _budget())

    violation = result["violations"][0]
    assert violation["metric"] == "messages.effective_rate_hz"
    assert violation["direction"] == "min_ratio"
    assert violation["median_ratio"] == 0.9


def test_rejects_machine_or_case_matrix_drift_between_repetitions():
    documents = [_document(index) for index in range(1, 6)]
    documents[2]["environment"]["host"]["cpu_model"] = "Other CPU"
    with pytest.raises(ValueError, match="machine/runtime environment mismatch"):
        regression.compare_documents(documents, _budget())

    documents = [_document(index) for index in range(1, 6)]
    documents[2]["results"].pop()
    with pytest.raises(ValueError, match="case matrix mismatch"):
        regression.compare_documents(documents, _budget())


def test_rejects_unverified_or_contradictory_backend_evidence():
    documents = [_document(index) for index in range(1, 6)]
    documents[1]["results"][1]["publisher_backend"]["backend"] = "cpp"

    with pytest.raises(ValueError, match="publisher backend evidence mismatch"):
        regression.compare_documents(documents, _budget())


def test_calibration_budget_writes_evidence_and_returns_distinct_exit(tmp_path):
    budget_path = tmp_path / "budget.json"
    budget_path.write_text(json.dumps(_budget("calibration_required")), encoding="utf-8")
    input_paths = []
    for index in range(1, 6):
        path = tmp_path / f"run-{index}.json"
        path.write_text(json.dumps(_document(index)), encoding="utf-8")
        input_paths.append(path)
    output = tmp_path / "regression.json"

    exit_code = regression.main([
        "--budget", str(budget_path), "--output", str(output),
        *(str(path) for path in input_paths),
    ])

    assert exit_code == regression.CALIBRATION_EXIT
    assert json.loads(output.read_text(encoding="utf-8"))["decision"] == "calibration_required"


def test_committed_budgets_are_explicitly_uncalibrated_and_schemas_are_json():
    for architecture in ("x86_64", "aarch64"):
        budget = json.loads(
            (ROOT / "benchmarks" / "regression-budgets" / f"{architecture}.json")
            .read_text(encoding="utf-8"))
        regression.validate_budget(budget)
        assert budget["status"] == "calibration_required"
        assert budget["comparisons"] == []
    for name in (
        "benchmark-regression-budget-v1.schema.json",
        "benchmark-regression-v1.schema.json",
    ):
        json.loads((ROOT / "schemas" / name).read_text(encoding="utf-8"))


def test_dedicated_workflow_gates_all_five_runs_and_preserves_evidence():
    workflow = yaml.safe_load(
        (ROOT / ".github" / "workflows" / "scheduled.yml").read_text(encoding="utf-8"))
    steps = workflow["jobs"]["dedicated-benchmark"]["steps"]
    gate = next(
        step for step in steps
        if step.get("name") == "Gate repeated results against reviewed architecture budget")
    assert "bench-regression" in gate["run"]
    assert "run-{1,2,3,4,5}.json" in gate["run"]
    upload = next(step for step in steps if step.get("name") == "Upload raw benchmark evidence")
    assert upload["if"] == "always()"
