#!/usr/bin/env python3
"""Run a stock Python relay or its explicit fused C++ rewrite."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time


FIXTURE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIXTURE_ROOT))

from evidence_protocol import PAYLOADS, build_case, emit_case  # noqa: E402
from stock_app import CommandAnnotator  # noqa: E402

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402


INPUT_TOPIC = "/rewrite_fixture/t3/input"
OUTPUT_TOPIC = "/rewrite_fixture/t3/output"


def _spin_rclpy_until(executor, predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return
    raise TimeoutError("stock fused-relay condition did not become true")


def _spin_cpp_until(executor, predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_some()
        if predicate():
            return
        time.sleep(0.002)
    raise TimeoutError("fused native condition did not become true")


def _cpp_type_name(value):
    name = getattr(type(value), "__cpp_name__", "")
    if not name:
        name = getattr(value, "__cpp_name__", "")
    return str(name)


def _run_stock():
    context = rclpy.context.Context()
    context.init(args=[])
    relay = CommandAnnotator(context=context)
    peer = Node("fused_relay_probe", context=context)
    outputs = []
    sink = peer.create_subscription(
        String, OUTPUT_TOPIC, lambda message: outputs.append(str(message.data)), 10)
    source = peer.create_publisher(String, INPUT_TOPIC, 10)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(relay)
    executor.add_node(peer)
    teardown_clean = False
    callback_count = 0
    try:
        _spin_rclpy_until(
            executor,
            lambda: source.get_subscription_count() == 1
            and relay.publisher.get_subscription_count() == 1,
        )
        started_ns = time.perf_counter_ns()
        started_cpu_ns = time.process_time_ns()
        for payload in PAYLOADS:
            expected_count = len(outputs) + 1
            source.publish(String(data=payload))
            _spin_rclpy_until(executor, lambda: len(outputs) == expected_count)
        cpu_time_ns = time.process_time_ns() - started_cpu_ns
        elapsed_ns = time.perf_counter_ns() - started_ns
        assert outputs == [payload + ":annotated" for payload in PAYLOADS]
        assert relay.callback_count == len(PAYLOADS)
        callback_count = relay.callback_count
        assert type(peer) is Node
        assert type(source).__module__ == "rclpy.publisher"
        assert type(sink).__module__ == "rclpy.subscription"
    finally:
        executor.remove_node(peer)
        executor.remove_node(relay)
        executor.shutdown(timeout_sec=2.0)
        peer.destroy_node()
        relay.destroy_node()
        context.shutdown()
        teardown_clean = True

    return build_case(
        project="fused_native_relay",
        variant="stock",
        tier=3,
        outputs=outputs,
        elapsed_ns=elapsed_ns,
        cpu_time_ns=cpu_time_ns,
        backend_roles={
            "node": {
                "backend": "python",
                "evidence": "exact rclpy.node.Node peer and Python subclass relay",
            },
            "publisher": {
                "backend": "python",
                "evidence": "exact rclpy.publisher.Publisher type",
            },
            "subscription": {
                "backend": "python",
                "evidence": "exact rclpy.subscription.Subscription type",
            },
            "transform_callback": {
                "backend": "python",
                "evidence": "CommandAnnotator.on_message invocation counter",
            },
        },
        transform_crossings=callback_count,
        api_coverage=[
            "stock Python transform callback",
            "every-message delivery",
            "ordered annotated String values",
            "normal teardown",
        ],
        contract_delta=[],
        teardown_clean=teardown_clean,
    )


def _run_rewrite():
    import rclcppyy

    outputs = []
    session = rclcppyy.native(["fused-native-relay"])
    with session as ros:
        relay = ros.create_node("command_annotator")
        peer = ros.create_node("fused_relay_probe")
        executor = ros.create_executor()
        executor.add_node(relay)
        executor.add_node(peer)
        pipeline = ros.create_fused_pipeline(
            relay,
            String,
            String,
            INPUT_TOPIC,
            OUTPUT_TOPIC,
            'output.data = input.data + ":annotated";',
            delivery="every",
        )
        source = peer.create_publisher(String, INPUT_TOPIC, 10)
        sink = peer.create_subscription(
            String, OUTPUT_TOPIC, lambda message: outputs.append(str(message.data)), 10)
        assert sink is not None
        _spin_cpp_until(
            executor,
            lambda: source.get_subscription_count() == 1
            and peer.count_publishers(OUTPUT_TOPIC) == 1,
        )
        started_ns = time.perf_counter_ns()
        started_cpu_ns = time.process_time_ns()
        for payload in PAYLOADS:
            expected_count = len(outputs) + 1
            source.publish(String(data=payload))
            _spin_cpp_until(executor, lambda: len(outputs) == expected_count)
        cpu_time_ns = time.process_time_ns() - started_cpu_ns
        elapsed_ns = time.perf_counter_ns() - started_ns
        assert outputs == [payload + ":annotated" for payload in PAYLOADS]
        stats = pipeline.stats()
        assert stats.received == len(PAYLOADS)
        assert stats.processed == len(PAYLOADS)
        assert stats.published == len(PAYLOADS)
        assert stats.dropped == 0
        assert stats.coalesced == 0
        assert stats.exceptions == 0
        assert stats.python_boundary_crossings == 0
        cpp_types = {
            "node": _cpp_type_name(relay),
            "executor": _cpp_type_name(executor),
            "publisher": _cpp_type_name(source),
            "subscription": _cpp_type_name(sink),
        }
        assert all("rclcpp" in name for name in cpp_types.values()), cpp_types
        pipeline_evidence = (
            "native pipeline counters received=%d processed=%d published=%d crossings=%d"
            % (
                stats.received,
                stats.processed,
                stats.published,
                stats.python_boundary_crossings,
            )
        )

    assert pipeline.closed
    return build_case(
        project="fused_native_relay",
        variant="rewrite",
        tier=3,
        outputs=outputs,
        elapsed_ns=elapsed_ns,
        cpu_time_ns=cpu_time_ns,
        backend_roles={
            "node": {
                "backend": "cpp",
                "evidence": "cppyy C++ type %s" % cpp_types["node"],
            },
            "executor": {
                "backend": "cpp",
                "evidence": "cppyy C++ type %s" % cpp_types["executor"],
            },
            "publisher": {
                "backend": "cpp",
                "evidence": "cppyy C++ type %s" % cpp_types["publisher"],
            },
            "subscription": {
                "backend": "cpp",
                "evidence": "cppyy C++ type %s" % cpp_types["subscription"],
            },
            "transform_callback": {
                "backend": "cpp",
                "evidence": pipeline_evidence,
            },
            "sink_observer": {
                "backend": "python",
                "evidence": "Python observer retained only for bounded output proof",
            },
        },
        transform_crossings=stats.python_boundary_crossings,
        api_coverage=[
            "editable C++ String transform",
            "fused subscription-transform-publisher",
            "every-message delivery with zero drops",
            "ordered annotated String values",
            "native resource teardown",
        ],
        contract_delta=[
            "relay Node and entities move to a managed rclcpp session",
            "the transform callback crosses no Python boundary",
        ],
        teardown_clean=session.closed and pipeline.closed,
    )


def run(variant):
    if variant == "stock":
        return _run_stock()
    return _run_rewrite()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", choices=("stock", "rewrite"), required=True)
    args = parser.parse_args(argv)
    emit_case(run(args.variant))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
