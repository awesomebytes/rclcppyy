"""Static and protocol tests for facade characterization."""

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
    spec = importlib.util.spec_from_file_location(
        name, BENCH_DIR / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


protocol = _load("_message_facade_benchmark_protocol")
runner = _load("run_message_facade_benchmark")


def _sample(item, index, warmup=10, messages=50):
    facade = item["variant"] == "message-facade-rclcppyy"
    backend = {
        "verified": True,
        "profile": "message_facade" if facade else "stock",
        "publisher": "cpp" if facade else "python",
        "subscription_take": "cpp" if facade else "python",
        "representation": (
            "std_msgs::msg::" + item["message_type"]
            if facade else "generated_python_message"),
        "converter_forbidden": facade,
        "python_to_cpp_whole_message_conversions": 0 if facade else None,
        "fallback_operations": 0,
        "message_type": item["message_type"],
        "entity_types": {
            "node": "rclpy.node.Node",
            "publisher": "rclpy.publisher.Publisher",
            "subscription": "rclpy.subscription.Subscription",
            "executor": "rclpy.executors.SingleThreadedExecutor",
        },
        "callback_message_type": {
            "UInt64": "std_msgs.msg._u_int64.UInt64",
            "String": "std_msgs.msg._string.String",
        }[item["message_type"]],
        "graph_node_identity_count": 1,
    }
    if facade:
        backend.update({
            "publish_route_messages": warmup + messages,
            "take_route_messages": warmup + messages,
            "hidden_original_entity_mapping": True,
            "facade_storage_address": 12345,
            "total_route_messages": warmup + messages,
        })
    first = warmup + 1
    return {
        "schema": protocol.SAMPLE_SCHEMA,
        **item,
        "order_index": index,
        "warmup_messages": warmup,
        "measured_messages": messages,
        "runtime": {
            "fresh_process": True,
            "setup_excluded": True,
            "discovery_complete": True,
            "single_threaded_executor": True,
            "fixed_work": True,
            "teardown_clean": True,
            "rmw_implementation": "rmw_cyclonedds_cpp",
            "ros_distribution": "jazzy",
        },
        "correctness": {
            "received": messages,
            "dropped": 0,
            "exceptions": 0,
            "checksum": protocol.expected_checksum(
                item["message_type"], first, messages),
            "last_sequence": warmup + messages,
            "value_contract_verified": True,
        },
        "timing": {
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "cpu_time_ns": 5_000_000,
            "cpu_ns_per_message": 100_000.0,
            "elapsed_ns": 10_000_000,
            "throughput_messages_per_second": 5_000.0,
            "latency": {
                "count": messages,
                "p50_ns": 50_000,
                "p99_ns": 80_000,
                "max_ns": 90_000,
            },
            "latency_p50_ns": 50_000,
            "latency_p99_ns": 80_000,
        },
        "rss_guard": {
            "baseline_bytes": 100_000_000,
            "peak_bytes": 101_000_000,
            "growth_bytes": 1_000_000,
            "limit_bytes": protocol.RSS_GUARD_LIMIT_BYTES,
            "passed": True,
        },
        "backend_evidence": backend,
    }


def test_execution_order_rotates_types_and_paired_variant_order():
    order = protocol.execution_order(2)
    assert len(order) == 8
    assert order[:4] == [
        {"repetition": 0, "message_type": "UInt64", "variant": "stock-rclpy"},
        {
            "repetition": 0,
            "message_type": "UInt64",
            "variant": "message-facade-rclcppyy",
        },
        {
            "repetition": 0,
            "message_type": "String",
            "variant": "message-facade-rclcppyy",
        },
        {"repetition": 0, "message_type": "String", "variant": "stock-rclpy"},
    ]
    assert order[4]["message_type"] == "String"
    assert order[4]["variant"] == "message-facade-rclcppyy"


def test_document_requires_complete_correctness_gated_pairs_and_disabled_claims():
    order = protocol.execution_order(1)
    samples = [_sample(item, index) for index, item in enumerate(order)]
    document = protocol.build_document(
        environment={"fixture": True},
        command=["fixture"],
        warmup=10,
        messages=50,
        repetitions=1,
        samples=samples,
    )
    protocol.validate_document(document)
    assert document["benchmark"]["performance_claims_allowed"] is False
    assert document["benchmark"]["rss_is_guard_only"] is True
    invalid = json.loads(json.dumps(document))
    invalid["samples"][1]["backend_evidence"]["converter_forbidden"] = False
    with pytest.raises(ValueError, match="converter-forbidden"):
        protocol.validate_document(invalid)


def test_worker_excludes_setup_and_forbids_facade_conversion():
    source = (BENCH_DIR / "message_facade_benchmark_worker.py").read_text(
        encoding="utf-8")
    assert source.index(
        "borrowed_publish.convert_python_msg_to_cpp = conversion_forbidden"
    ) < source.index("cpu_started = time.process_time_ns()")
    assert source.index("for sequence in range(1, args.warmup_messages + 1)") < (
        source.index("cpu_started = time.process_time_ns()"))
    assert "SingleThreadedExecutor" in source
    assert '"python_to_cpp_whole_message_conversions": 0' in source
    assert '"performance_claims_allowed": False' in (
        BENCH_DIR / "_message_facade_benchmark_protocol.py").read_text(
            encoding="utf-8")


def test_runner_argument_bounds_and_smoke_work_are_fixed():
    args = runner._arguments(["--smoke"])
    runner._validate_arguments(args)
    assert (args.warmup_messages, args.messages, args.repetitions) == (20, 100, 1)
    args = runner._arguments(["--messages", "0"])
    with pytest.raises(ValueError, match="messages"):
        runner._validate_arguments(args)


@pytest.mark.skipif(
    os.environ.get("RCLCPPYY_RUN_LIVE_MESSAGE_FACADE_BENCH") != "1",
    reason="set RCLCPPYY_RUN_LIVE_MESSAGE_FACADE_BENCH=1 for live smoke",
)
def test_live_message_facade_benchmark_smoke(tmp_path):
    output = tmp_path / "message-facade-smoke.json"
    completed = subprocess.run(
        [
            sys.executable,
            str(BENCH_DIR / "run_message_facade_benchmark.py"),
            "--smoke",
            "--output",
            str(output),
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
