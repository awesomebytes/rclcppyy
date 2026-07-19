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
domains = _load("_domain_lease")


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


def test_run_token_namespaces_topics_without_changing_case_identity():
    plain = matrix.build_cases(["rclpy"], ["small-string"], [100], [0])[0]
    scoped = matrix.build_cases(
        ["rclpy"], ["small-string"], [100], [0], run_token="run123")[0]

    assert scoped["case_id"] == plain["case_id"]
    assert scoped["topic"] == "/rclcppyy_bench/run123/" + plain["case_id"]


def test_direct_cpp_matrix_lanes_cover_flat_and_nested_generated_cpp_values():
    direct = {
        name: matrix.BACKENDS[name]
        for name in ("rclcppyy-direct-copy", "rclcppyy-direct-lease")
    }
    assert {item["worker_backend"] for item in direct.values()} == {
        "direct-copy", "direct-lease"}
    assert all(item["expected_backends"] == {
        "publisher": "cpp", "subscriber": "cpp"}
        for item in direct.values())
    assert all(item["workloads"] == ("small-string", "nested-header")
               for item in direct.values())


def test_domain_leases_are_distinct_and_reusable(tmp_path, monkeypatch):
    monkeypatch.setenv("RCLCPPYY_BENCH_DOMAIN_MIN", "220")
    monkeypatch.setenv("RCLCPPYY_BENCH_DOMAIN_MAX", "221")
    first = domains.acquire_domain(tmp_path)
    second = domains.acquire_domain(tmp_path)
    assert {first.domain_id, second.domain_id} == {220, 221}
    first_id = first.domain_id
    first.release()
    second.release()
    replacement = domains.acquire_domain(tmp_path)
    assert replacement.domain_id in {220, 221}
    replacement.release()
    assert first_id in {220, 221}


def test_measurement_window_uses_actual_counts_drops_and_nearest_rank():
    clock_values = iter((1_000_000_000, 2_000_000_000))
    state = protocol.MeasurementState(
        "fixture/value-contract/v1", 0, clock_ns=lambda: next(clock_values))
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
    assert result["wire_values"] == {
        "schema": protocol.WIRE_SCHEMA,
        "contract_id": "fixture/value-contract/v1",
        "expected_payload_bytes": 0,
        "checked_messages": 3,
        "violations": 0,
        "violation_types": {},
        "value_contract_verified": True,
    }


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
        "wire_values": {
            "schema": protocol.WIRE_SCHEMA,
            "contract_id": "fixture/value-contract/v1",
            "expected_payload_bytes": 0,
            "checked_messages": 0,
            "violations": 0,
            "violation_types": {},
            "value_contract_verified": False,
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
    assert document["schema"] == "rclcppyy.benchmark/v3"
    assert document["benchmark"]["mode"] == "smoke"
    assert document["benchmark"]["performance_claims_allowed"] is False
    assert document["environment"]["ros"]["domain_id"] is not None
    assert document["benchmark"]["matrix"]["ros_domain_id"] == int(
        document["environment"]["ros"]["domain_id"])
    assert document["benchmark"]["matrix"]["run_token"]
    assert document["failures"] == []
    assert {row["workload"] for row in document["results"]} == {
        "small-string", "nested-header"}
    assert all(row["backend_verified"] for row in document["results"])
    assert all(row["messages"]["received"] > 0 for row in document["results"])
    assert all(
        row["latency_us"]["count"] == row["messages"]["received"]
        for row in document["results"])
    assert all(row["wire_values"]["value_contract_verified"] for row in document["results"])
    assert all(row["wire_values"]["violations"] == 0 for row in document["results"])


def test_direct_cpp_smoke_proves_copy_and_lease_representation_boundaries():
    command = [
        sys.executable,
        str(BENCH_DIR / "run_benchmarks.py"),
        "--smoke",
        "--backends", "rclcppyy-direct-copy,rclcppyy-direct-lease",
        "--workloads", "small-string,nested-header",
        "--rate", "200",
        "--payload-bytes", "8",
        "--duration", "0.35",
        "--warmup-timeout", "30",
        "--json",
    ]
    proc = subprocess.run(
        command,
        cwd=REPO_ROOT,
        env=os.environ.copy(),
        capture_output=True,
        text=True,
        timeout=120,
    )

    assert proc.returncode == 0, f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
    document = json.loads(proc.stdout)
    assert document["failures"] == []
    assert len(document["results"]) == 4
    assert {(row["backend"], row["workload"]) for row in document["results"]} == {
        (backend, workload)
        for backend in ("rclcppyy-direct-copy", "rclcppyy-direct-lease")
        for workload in ("small-string", "nested-header")
    }
    for row in document["results"]:
        assert row["backend_verified"]
        assert row["wire_values"]["value_contract_verified"]
        assert row["wire_values"]["violations"] == 0
        expected_name = (
            "std_msgs::msg::Header_<std::allocator<void>>"
            if row["workload"] == "nested-header"
            else "std_msgs::msg::String_<std::allocator<void>>"
        )
        for marker in (row["publisher_backend"], row["subscriber_backend"]):
            assert marker["backend"] == "cpp"
            assert marker["evidence"] == "rclcppyy_direct_status"
            assert "no_conversion" in marker["metadata"]["policies"]
            assert marker["metadata"]["decision_metadata"][
                "message_type"] == expected_name
        subscriber = row["subscriber_backend"]["metadata"]
        if row["backend"] == "rclcppyy-direct-copy":
            assert "owning_cpp_callback_copy" in subscriber["policies"]
            assert subscriber["decision_metadata"]["callback_handoff"] == (
                "one_native_cpp_copy")
        else:
            assert {"subscription_shared_lease", "actual_cpp_message"} <= set(
                subscriber["policies"])
            evidence = subscriber["decision_metadata"]
            assert evidence["callback_handoff"] == "shared_cpp_message_lease"
            assert evidence["message_representation"] == "actual_cpp"
            assert evidence["message_deep_copies_per_callback"] == 0
            assert evidence["python_message_conversions"] == 0
            assert evidence["serialization_operations"] == 0


def test_measurement_window_rejects_wire_contract_violations():
    clock_values = iter((1_000_000_000, 2_000_000_000))
    state = protocol.MeasurementState(
        "fixture/value-contract/v1", 8, clock_ns=lambda: next(clock_values))
    state.start("run")
    state.observe(1, 10.0)
    state.reject_wire_value("ValueError")

    result = state.stop("run")

    assert result["wire_values"]["checked_messages"] == 1
    assert result["wire_values"]["violations"] == 1
    assert result["wire_values"]["value_contract_verified"] is False


def test_measurement_window_rejects_non_monotonic_sequence_values():
    clock_values = iter((1_000_000_000, 2_000_000_000))
    state = protocol.MeasurementState(
        "fixture/value-contract/v1", 0, clock_ns=lambda: next(clock_values))
    state.start("run")
    state.observe(2, 10.0)
    state.observe(2, 11.0)

    result = state.stop("run")

    assert result["wire_values"]["violation_types"] == {
        "non_monotonic_sequence": 1}
    assert result["wire_values"]["value_contract_verified"] is False
