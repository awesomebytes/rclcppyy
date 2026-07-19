"""Contract and focused smoke tests for clock-timer / rate-sleep CPU evidence."""

from __future__ import annotations

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
    spec = importlib.util.spec_from_file_location(name, BENCH_DIR / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


protocol = _load("_clock_timer_benchmark_protocol")
runner = _load("run_clock_timer_benchmark")


def _sample(item, index, warmup=5, operations=25):
    native = item["variant"] not in ("stock-rclpy", "compatible-rclcppyy")
    native_type = None
    if native and item["workload"] == "clock-timer":
        native_type = "rclcpp::GenericTimer<std::function<void()>,nullptr>"
    return {
        "schema": protocol.SAMPLE_SCHEMA,
        **item,
        "order_index": index,
        "pid": 12345 + index,
        "run_token": "fixture_%d" % index,
        "warmup_operations": warmup,
        "measured_operations": operations,
        "runtime": {
            "fresh_process": True,
            "setup_excluded": True,
            "jit_excluded": True,
            "warmup_completed": True,
            "fixed_work": True,
            "teardown_clean": True,
            "cleanup_error": None,
            "ros_distribution": "jazzy",
            "rmw_implementation": "rmw_cyclonedds_cpp",
            "domain_id": "75",
            "period_ns": protocol.PERIOD_NS,
        },
        "correctness": {
            "completed_operations": operations,
            "expected_operations": operations,
            "exceptions": 0,
            "verified": True,
        },
        "timing": {
            "primary_metric": protocol.PRIMARY_METRIC,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "process_cpu_time_ns": operations * (1000 + index),
            "process_cpu_ns_per_operation": 1000.0 + index,
            "wall_time_ns": operations * 2000,
            "wall_ns_per_operation": 2000.0,
            "operations_per_second": 500_000.0,
        },
        "boundary_evidence": {
            "conversion_poison_installed": True,
            "serialization_poison_installed": True,
            "application_message_conversions": 0,
            "serialization_calls": 0,
            "cdr_calls": 0,
        },
        "backend_evidence": {
            "verified": True,
            "operation_route": protocol.OPERATION_ROUTES[
                item["variant"]][item["workload"]],
            "exact_cpp_entity": native,
            "native_type": native_type,
        },
    }


def test_order_rotates_variants_deterministically_by_workload_and_repetition():
    order = protocol.execution_order(2)
    assert len(order) == 16
    assert order[:4] == [
        {"repetition": 0, "workload": "clock-timer", "variant": "stock-rclpy"},
        {"repetition": 0, "workload": "clock-timer", "variant": "compatible-rclcppyy"},
        {"repetition": 0, "workload": "clock-timer", "variant": "direct-cpp-rclcppyy"},
        {"repetition": 0, "workload": "clock-timer", "variant": "native-orchestrated"},
    ]
    assert [item["variant"] for item in order[4:8]] == [
        "compatible-rclcppyy", "direct-cpp-rclcppyy", "native-orchestrated",
        "stock-rclpy",
    ]


def test_document_is_cpu_first_strict_and_claims_disabled():
    order = protocol.execution_order(1)
    samples = [_sample(item, index) for index, item in enumerate(order)]
    document = protocol.build_document(
        environment={"fixture": True},
        command=["fixture"],
        warmup=5,
        operations=25,
        repetitions=1,
        samples=samples,
    )
    assert document["benchmark"]["performance_claims_allowed"] is False
    assert document["benchmark"]["primary_metric"] == (
        "process_cpu_ns_per_operation")
    assert document["benchmark"]["variants"] == list(protocol.VARIANTS)
    assert document["benchmark"]["workloads"] == list(protocol.WORKLOADS)
    assert document["benchmark"]["period_ns"] == protocol.PERIOD_NS
    assert json.loads(protocol.dumps(document)) == document

    invalid = json.loads(json.dumps(document))
    invalid["samples"][1]["boundary_evidence"]["serialization_calls"] = 1
    with pytest.raises(ValueError, match="forbidden conversion or serialization"):
        protocol.validate_document(invalid)

    invalid = json.loads(json.dumps(document))
    invalid["samples"][2]["backend_evidence"]["exact_cpp_entity"] = False
    with pytest.raises(ValueError, match="lacks exact C\\+\\+ entity"):
        protocol.validate_document(invalid)

    invalid = json.loads(json.dumps(document))
    invalid["samples"][2]["backend_evidence"]["native_type"] = (
        "rclcpp::WallTimer<std::function<void()>>")
    with pytest.raises(ValueError, match="not backed by a native GenericTimer"):
        protocol.validate_document(invalid)


def test_runner_smoke_and_argument_bounds_are_fixed():
    args = runner._arguments(["--smoke"])
    runner._validate_arguments(args)
    assert (args.warmup_operations, args.operations, args.repetitions) == (5, 25, 1)
    with pytest.raises(ValueError, match="measured operations"):
        runner._validate_arguments(runner._arguments(["--operations", "0"]))


def test_direct_clock_timer_worker_smoke_is_exact_cpp():
    environment = os.environ.copy()
    environment.update({
        "ROS_DISTRO": "jazzy",
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    })
    completed = subprocess.run(
        [
            sys.executable,
            str(BENCH_DIR / "clock_timer_benchmark_worker.py"),
            "--variant", "direct-cpp-rclcppyy",
            "--workload", "clock-timer",
            "--warmup-operations", "5",
            "--operations", "10",
            "--repetition", "0",
            "--order-index", "0",
            "--run-token", "test_direct_clock_timer",
        ],
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        capture_output=True,
        timeout=90,
        check=False,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
    records = [
        line for line in completed.stdout.splitlines()
        if line.startswith(runner.PREFIX)
    ]
    assert len(records) == 1
    sample = json.loads(records[0][len(runner.PREFIX):])
    protocol.validate_sample(
        sample, warmup=5, operations=10, repetition=0, order_index=0)
    assert sample["backend_evidence"]["exact_cpp_entity"] is True
    assert "GenericTimer" in sample["backend_evidence"]["native_type"]


def test_direct_rate_sleep_worker_smoke_is_exact_cpp():
    environment = os.environ.copy()
    environment.update({
        "ROS_DISTRO": "jazzy",
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    })
    completed = subprocess.run(
        [
            sys.executable,
            str(BENCH_DIR / "clock_timer_benchmark_worker.py"),
            "--variant", "direct-cpp-rclcppyy",
            "--workload", "rate-sleep",
            "--warmup-operations", "5",
            "--operations", "10",
            "--repetition", "0",
            "--order-index", "0",
            "--run-token", "test_direct_rate_sleep",
        ],
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        capture_output=True,
        timeout=90,
        check=False,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
    records = [
        line for line in completed.stdout.splitlines()
        if line.startswith(runner.PREFIX)
    ]
    assert len(records) == 1
    sample = json.loads(records[0][len(runner.PREFIX):])
    protocol.validate_sample(
        sample, warmup=5, operations=10, repetition=0, order_index=0)
    assert sample["backend_evidence"]["exact_cpp_entity"] is True
    assert sample["backend_evidence"]["native_type"] is None


@pytest.mark.skipif(
    os.environ.get("RCLCPPYY_RUN_LIVE_CLOCK_TIMER_BENCH") != "1",
    reason="set RCLCPPYY_RUN_LIVE_CLOCK_TIMER_BENCH=1 for the full matrix smoke",
)
def test_live_clock_timer_matrix_smoke(tmp_path):
    output = tmp_path / "clock-timer-smoke.json"
    completed = subprocess.run(
        [
            sys.executable,
            str(BENCH_DIR / "run_clock_timer_benchmark.py"),
            "--smoke",
            "--output", str(output),
        ],
        cwd=REPO_ROOT,
        env=os.environ.copy(),
        text=True,
        capture_output=True,
        timeout=240,
        check=False,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
    document = json.loads(output.read_text(encoding="utf-8"))
    protocol.validate_document(document)
    assert document["benchmark"]["performance_claims_allowed"] is False
