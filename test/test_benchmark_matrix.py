"""Unit and isolated integration coverage for the benchmark matrix."""

import importlib.util
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


matrix = _load("_benchmark_matrix")
protocol = _load("_benchmark_protocol")


def test_declarative_matrix_crosses_frequency_and_payload_variants():
    cases = matrix.build_cases(
        ["rclpy", "rclcppyy"],
        ["small-string", "nested-header"],
        [100, 1000],
        [0, 4096],
    )

    assert len(cases) == 16
    assert len({case["case_id"] for case in cases}) == 16
    assert {case["workload"] for case in cases} == {"small-string", "nested-header"}
    assert {case["target_rate_hz"] for case in cases} == {100, 1000}
    assert {case["payload_bytes"] for case in cases} == {0, 4096}
    assert all(case["expected_backends"] for case in cases)


def test_measurement_window_uses_actual_counts_drops_and_nearest_rank():
    clock_values = iter((1_000_000_000, 2_000_000_000))
    state = protocol.MeasurementState(clock_ns=lambda: next(clock_values))
    state.observe(9, 999.0)  # warmup is excluded, but anchors gap detection
    state.start("run")
    state.observe(10, 10.0)
    state.observe(11, 20.0)
    state.observe(14, 100.0)

    result = state.stop("run")

    assert result["window"]["duration_s"] == 1.0
    assert result["messages"] == {
        "received": 3,
        "dropped": 2,
        "effective_rate_hz": 3.0,
    }
    assert result["latency_us"]["count"] == 3
    assert result["latency_us"]["mean"] == 130.0 / 3.0
    assert result["latency_us"]["p50"] == 20.0
    assert result["latency_us"]["p99"] == 100.0


@pytest.mark.parametrize("field,value", [
    ("duration_s", "1.0"),
    ("duration_s", float("nan")),
])
def test_window_protocol_rejects_malformed_numeric_fields(field, value):
    document = {
        "schema": protocol.RESULT_SCHEMA,
        "run_id": "run",
        "window": {"duration_s": value},
        "messages": {"received": 0, "dropped": 0, "effective_rate_hz": 0.0},
        "latency_us": {
            "count": 0,
            "mean": None,
            "p50": None,
            "p95": None,
            "p99": None,
            "min": None,
            "max": None,
        },
    }

    with pytest.raises(ValueError, match="requires duration"):
        protocol.validate_window_result(document)


def test_smoke_matrix_runs_flat_and_nested_in_isolated_processes():
    command = [
        sys.executable,
        str(BENCH_DIR / "run_benchmarks.py"),
        "--smoke",
        "--backends", "rclpy",
        "--workloads", "small-string,nested-header",
        "--rate", "200",
        "--payload-bytes", "8",
        "--duration", "0.35",
        "--warmup-timeout", "20",
        "--json",
    ]
    proc = subprocess.run(
        command,
        cwd=REPO_ROOT,
        env=os.environ.copy(),
        capture_output=True,
        text=True,
        timeout=60,
    )

    assert proc.returncode == 0, f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
    document = json.loads(proc.stdout)
    assert document["schema"] == "rclcppyy.benchmark/v2"
    assert document["benchmark"]["mode"] == "smoke"
    assert document["benchmark"]["performance_claims_allowed"] is False
    assert document["failures"] == []
    assert {row["workload"] for row in document["results"]} == {
        "small-string", "nested-header"}
    assert all(row["backend_verified"] for row in document["results"])
    assert all(row["messages"]["received"] > 0 for row in document["results"])
    assert all(
        row["latency_us"]["count"] == row["messages"]["received"]
        for row in document["results"])
