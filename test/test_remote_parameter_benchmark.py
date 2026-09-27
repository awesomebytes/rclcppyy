"""Contract and focused smoke tests for remote parameter CPU evidence."""

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
SCHEMA_PATH = REPO_ROOT / "schemas" / "remote-parameter-benchmark-v1.schema.json"
sys.path.insert(0, str(BENCH_DIR))


def _load(name):
    spec = importlib.util.spec_from_file_location(name, BENCH_DIR / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


protocol = _load("_remote_parameter_benchmark_protocol")
runner = _load("run_remote_parameter_benchmark")


def _sample(item, index, warmup=2, operations=4):
    direct = item["variant"] == "direct-rclcppyy"
    workload = item["workload"]
    latency = [100 + index, 200 + index, 300 + index, 400 + index]
    cpu_per_operation = 1000.0 + index
    source_digest = "a" * 64
    return {
        "schema": protocol.SAMPLE_SCHEMA,
        **item,
        "order_index": index,
        "run_token": "fixture_%d" % index,
        "pid": 12000 + index,
        "warmup_operations": warmup,
        "measured_operations": operations,
        "runtime": {
            "fresh_process": True,
            "setup_excluded": True,
            "jit_excluded": True,
            "warmup_completed": True,
            "post_init_settle_completed": True,
            "post_init_settle_ns": protocol.POST_INIT_SETTLE_NS,
            "fixed_work": True,
            "teardown_clean": True,
            "cleanup_error": None,
            "ros_distribution": protocol.ROS_DISTRO,
            "rmw_implementation": protocol.RMW,
            "domain_id": "77",
        },
        "topology": {
            "process_count": 1,
            "node_count": 2,
            "server_node": "remote_parameter_server_fixture",
            "client_node": "remote_parameter_client_fixture",
            "remote_node_name": "/remote_parameter_server_fixture",
            "executor": "rclpy.executors.SingleThreadedExecutor",
            "one_outstanding_request": True,
            "server_parameter_service_count": protocol.SERVICE_COUNT,
            "client_parameter_client_count": protocol.SERVICE_COUNT,
            "same_context": True,
            "graph_verified": True,
            "service_qos": protocol.QOS,
        },
        "correctness": {
            "completed_operations": operations,
            "expected_operations": operations,
            "exceptions": 0,
            "responses_verified": operations,
            "pending_futures": 0,
            "final_value": 73,
            "verified": True,
        },
        "timing": {
            "primary_metric": protocol.PRIMARY_METRIC,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "cpu_scope": "combined-server-client-process",
            "process_cpu_time_ns": int(cpu_per_operation * operations),
            "process_cpu_ns_per_operation": cpu_per_operation,
            "wall_time_ns": operations * 2000,
            "wall_ns_per_operation": 2000.0,
            "operations_per_second": 500_000.0,
            "latency_ns": latency,
            "latency": protocol.latency_summary(latency),
        },
        "boundary_evidence": {
            "conversion_poison_installed": True,
            "serialization_poison_installed": True,
            "application_message_conversions": 0,
            "serialization_calls": 0,
            "cdr_calls": 0,
            "exact_cpp_data_path_required": direct,
        },
        "backend_evidence": {
            "verified": True,
            "operation_route": protocol.OPERATION_ROUTES[workload],
            "async_parameter_client_source": (
                "rclpy.parameter_client.AsyncParameterClient"),
            "source_shape_verified": True,
            "source_path": "/fixture/rclpy/parameter_client.py",
            "source_sha256": source_digest,
            "python_payload_cache": False,
            "retained_response_verified": True,
            "server_authority": "cpp" if direct else "python",
            "client_authority": "cpp" if direct else "python",
            "request_representation": (
                "actual-generated-cpp" if direct else "generated-python"),
            "response_representation": (
                "actual-generated-cpp" if direct else "generated-python"),
            "parameter_representation": (
                "actual-generated-cpp" if direct else "generated-python"),
            "all_parameter_service_aliases_exact_cpp": direct,
            "request_alias_exact_cpp": direct,
            "response_exact_cpp": direct,
            "parameter_value_exact_cpp": direct,
            "parameter_message_alias_exact_cpp": direct,
            "client_entities_exact_cpp": direct,
            "set_result_exact_cpp": direct and workload == "set-atomically-one",
            "server_node_exact_cpp": direct,
            "client_node_exact_cpp": direct,
            "parameter_owner_exact_cpp": direct,
            "cpp_node_addresses": [1234, 5678] if direct else None,
        },
    }


def test_order_rotates_two_lanes_by_workload_and_repetition():
    order = protocol.execution_order(2)
    assert len(order) == 8
    assert order[:4] == [
        {"repetition": 0, "workload": "get-one", "variant": "stock-rclpy"},
        {"repetition": 0, "workload": "get-one", "variant": "direct-rclcppyy"},
        {
            "repetition": 0,
            "workload": "set-atomically-one",
            "variant": "direct-rclcppyy",
        },
        {
            "repetition": 0,
            "workload": "set-atomically-one",
            "variant": "stock-rclpy",
        },
    ]
    assert [item["variant"] for item in order[4:6]] == [
        "direct-rclcppyy", "stock-rclpy"]


def test_document_is_strict_cpu_first_exact_cpp_and_claims_disabled():
    order = protocol.execution_order(1)
    samples = [_sample(item, index) for index, item in enumerate(order)]
    document = protocol.build_document(
        environment={"fixture": True},
        command=["fixture"],
        warmup=2,
        operations=4,
        repetitions=1,
        samples=samples,
    )
    assert document["benchmark"]["performance_claims_allowed"] is False
    assert document["benchmark"]["primary_metric"] == (
        "process_cpu_ns_per_operation")
    assert document["benchmark"]["cpu_scope"] == (
        "combined-server-client-process")
    assert set(document["summary"]["median_ratios"]) == set(protocol.WORKLOADS)
    assert json.loads(protocol.dumps(document)) == document

    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    jsonschema = pytest.importorskip("jsonschema")
    jsonschema.Draft202012Validator.check_schema(schema)
    jsonschema.validate(document, schema)

    invalid = json.loads(json.dumps(document))
    invalid["samples"][1]["boundary_evidence"]["cdr_calls"] = 1
    with pytest.raises(ValueError, match="conversion, serialization, or CDR"):
        protocol.validate_document(invalid)
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(invalid, schema)

    invalid = json.loads(json.dumps(document))
    invalid["samples"][1]["backend_evidence"][
        "request_alias_exact_cpp"] = False
    with pytest.raises(ValueError, match=r"exact-C\+\+"):
        protocol.validate_document(invalid)

    invalid = json.loads(json.dumps(document))
    invalid["samples"][1]["backend_evidence"]["source_sha256"] = "b" * 64
    with pytest.raises(ValueError, match="different client source"):
        protocol.validate_document(invalid)


def test_worker_source_keeps_common_shape_setup_and_warmup_outside_cpu_window():
    source = (BENCH_DIR / "remote_parameter_benchmark_worker.py").read_text(
        encoding="utf-8")
    assert source.count("client = AsyncParameterClient(") == 1
    assert source.count("executor = SingleThreadedExecutor(") == 1
    assert source.index("rclcppyy.enable_cpp_acceleration(") < source.index(
        "    import rclpy\n")
    run_source = source[source.index("def run(args):"):]
    assert run_source.index("state = _setup(args.variant, suffix)") < (
        run_source.index("counters = _install_boundary_poison()"))
    assert run_source.index("counters = _install_boundary_poison()") < (
        run_source.index("measured = _operation(state, args)"))
    operation_source = source[
        source.index("def _operation(state, args):"):
        source.index("def _topology(state):")
    ]
    assert operation_source.index(
        "for index in range(args.warmup_operations)") < operation_source.index(
            "cpu_started = time.process_time_ns()")
    assert "state[\"client\"].get_parameters([\"target\"])" in operation_source
    assert "state[\"client\"].set_parameters_atomically" in operation_source
    assert "convert_python_msg_to_cpp(" not in operation_source


def test_runner_smoke_and_argument_bounds_are_fixed():
    args = runner._arguments(["--smoke"])
    runner._validate_arguments(args)
    assert (args.warmup_operations, args.operations, args.repetitions) == (2, 5, 1)
    with pytest.raises(ValueError, match="measured operations"):
        runner._validate_arguments(runner._arguments(["--operations", "0"]))


def test_direct_atomic_worker_smoke_is_exact_cpp_and_retained():
    environment = os.environ.copy()
    environment.update({
        "ROS_DISTRO": "jazzy",
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    })
    completed = subprocess.run(
        [
            sys.executable,
            str(BENCH_DIR / "remote_parameter_benchmark_worker.py"),
            "--variant", "direct-rclcppyy",
            "--workload", "set-atomically-one",
            "--warmup-operations", "2",
            "--operations", "3",
            "--repetition", "0",
            "--order-index", "0",
            "--run-token", "test_direct_remote_parameter",
        ],
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        capture_output=True,
        timeout=120,
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
        sample, warmup=2, operations=3, repetition=0, order_index=0)
    backend = sample["backend_evidence"]
    assert backend["all_parameter_service_aliases_exact_cpp"] is True
    assert backend["request_alias_exact_cpp"] is True
    assert backend["response_exact_cpp"] is True
    assert backend["parameter_message_alias_exact_cpp"] is True
    assert backend["client_entities_exact_cpp"] is True
    assert backend["set_result_exact_cpp"] is True
    assert backend["parameter_owner_exact_cpp"] is True
    assert backend["retained_response_verified"] is True
    assert sample["boundary_evidence"]["application_message_conversions"] == 0
    assert sample["boundary_evidence"]["serialization_calls"] == 0
    assert sample["boundary_evidence"]["cdr_calls"] == 0


@pytest.mark.skipif(
    os.environ.get("RCLCPPYY_RUN_LIVE_REMOTE_PARAMETER_BENCH") != "1",
    reason="set RCLCPPYY_RUN_LIVE_REMOTE_PARAMETER_BENCH=1 for matrix smoke",
)
def test_live_remote_parameter_matrix_smoke(tmp_path):
    output = tmp_path / "remote-parameter-smoke.json"
    completed = subprocess.run(
        [
            sys.executable,
            str(BENCH_DIR / "run_remote_parameter_benchmark.py"),
            "--smoke",
            "--output", str(output),
        ],
        cwd=REPO_ROOT,
        env=os.environ.copy(),
        text=True,
        capture_output=True,
        timeout=300,
        check=False,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
    protocol.validate_document(json.loads(output.read_text(encoding="utf-8")))
