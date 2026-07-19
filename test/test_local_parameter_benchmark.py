"""Contract and focused smoke tests for local parameter CPU evidence."""

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


protocol = _load("_local_parameter_benchmark_protocol")
runner = _load("run_local_parameter_benchmark")


def _sample(item, index, warmup=5, operations=25):
    native = item["variant"] != "stock-rclpy"
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
            "ros_distribution": "jazzy",
            "rmw_implementation": "rmw_cyclonedds_cpp",
        },
        "correctness": {
            "completed_operations": operations,
            "expected_operations": operations,
            "exceptions": 0,
            "final_value": 73,
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
            "value_snapshot_in_cpu_window": (
                item["workload"] == "get-value-snapshot"),
        },
        "backend_evidence": {
            "verified": True,
            "operation_route": protocol.OPERATION_ROUTES[
                item["variant"]][item["workload"]],
            "parameter_representation": (
                "rclcpp::Parameter" if native else "rclpy.parameter.Parameter"),
            "exact_cpp_parameter": native,
            "exact_cpp_node": native,
            "exact_cpp_result": native and item["workload"] == "set-atomically",
            "measured_parameter_values_exact_cpp": native,
            "cpp_parameter_address": 12345 if native else None,
            "parameter_implementation": "rclcpp::Parameter" if native else None,
            "node_implementation": "rclcpp::Node" if native else "rclpy.node.Node",
        },
    }


def test_order_rotates_variants_deterministically_by_workload_and_repetition():
    order = protocol.execution_order(2)
    assert len(order) == 24
    assert order[:3] == [
        {"repetition": 0, "workload": "declare", "variant": "stock-rclpy"},
        {"repetition": 0, "workload": "declare", "variant": "direct-rclcppyy"},
        {"repetition": 0, "workload": "declare", "variant": "raw-rclcpp"},
    ]
    assert [item["variant"] for item in order[3:6]] == [
        "direct-rclcppyy", "raw-rclcpp", "stock-rclpy"]
    assert [item["variant"] for item in order[12:15]] == [
        "direct-rclcppyy", "raw-rclcpp", "stock-rclpy"]


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
    assert len(document["summary"]["snapshot_materialization_ratios"]) == 3
    assert json.loads(protocol.dumps(document)) == document

    invalid = json.loads(json.dumps(document))
    invalid["samples"][1]["boundary_evidence"]["serialization_calls"] = 1
    with pytest.raises(ValueError, match="forbidden conversion or serialization"):
        protocol.validate_document(invalid)


def test_worker_keeps_setup_warmup_and_poison_outside_cpu_window():
    source = (BENCH_DIR / "local_parameter_benchmark_worker.py").read_text(
        encoding="utf-8")
    run_source = source[source.index("def run(args):"):]
    assert run_source.index("counters = _install_boundary_poison()") < (
        run_source.index("measured = _operation(state, args)"))
    assert source.index("for index in range(args.warmup_operations)") < source.index(
        "cpu_started = time.process_time_ns()")
    assert "return _get(state, target_name)" in source
    assert "return _snapshot(state, _get(state, target_name))" in source
    assert '"application_message_conversions": 0' in source
    assert '"performance_claims_allowed": False' in (
        BENCH_DIR / "_local_parameter_benchmark_protocol.py").read_text(
            encoding="utf-8")


def test_runner_smoke_and_argument_bounds_are_fixed():
    args = runner._arguments(["--smoke"])
    runner._validate_arguments(args)
    assert (args.warmup_operations, args.operations, args.repetitions) == (5, 25, 1)
    with pytest.raises(ValueError, match="measured operations"):
        runner._validate_arguments(runner._arguments(["--operations", "0"]))


def test_direct_native_get_worker_smoke_is_exact_cpp_and_snapshot_free():
    environment = os.environ.copy()
    environment.update({
        "ROS_DISTRO": "jazzy",
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    })
    completed = subprocess.run(
        [
            sys.executable,
            str(BENCH_DIR / "local_parameter_benchmark_worker.py"),
            "--variant", "direct-rclcppyy",
            "--workload", "get-native",
            "--warmup-operations", "5",
            "--operations", "25",
            "--repetition", "0",
            "--order-index", "0",
            "--run-token", "test_direct_native_get",
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
        sample, warmup=5, operations=25, repetition=0, order_index=0)
    assert sample["backend_evidence"]["exact_cpp_parameter"] is True
    assert sample["boundary_evidence"]["value_snapshot_in_cpu_window"] is False


@pytest.mark.skipif(
    os.environ.get("RCLCPPYY_RUN_LIVE_LOCAL_PARAMETER_BENCH") != "1",
    reason="set RCLCPPYY_RUN_LIVE_LOCAL_PARAMETER_BENCH=1 for full matrix smoke",
)
def test_live_local_parameter_matrix_smoke(tmp_path):
    output = tmp_path / "local-parameter-smoke.json"
    completed = subprocess.run(
        [
            sys.executable,
            str(BENCH_DIR / "run_local_parameter_benchmark.py"),
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
    protocol.validate_document(json.loads(output.read_text(encoding="utf-8")))
