#!/usr/bin/env python3
"""Run stock ownership or an explicit managed-rclcpp rewrite."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time


FIXTURE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIXTURE_ROOT))

from evidence_protocol import PAYLOADS, build_case, emit_case  # noqa: E402
from stock_app import TelemetryNormalizer, make_executor  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402


INPUT_TOPIC = "/rewrite_fixture/t2/input"
OUTPUT_TOPIC = "/rewrite_fixture/t2/output"


def _spin_rclpy_until(executor, predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return
        time.sleep(0.002)
    raise TimeoutError("stock managed-worker condition did not become true")


def _spin_cpp_until(executor, predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_some()
        if predicate():
            return
        time.sleep(0.002)
    raise TimeoutError("managed native condition did not become true")


def _cpp_type_name(value):
    name = getattr(type(value), "__cpp_name__", "")
    if not name:
        name = getattr(value, "__cpp_name__", "")
    return str(name)


def _run_stock():
    context = rclpy.context.Context()
    context.init(args=[])
    worker = TelemetryNormalizer(context=context)
    peer = Node("managed_worker_probe", context=context)
    outputs = []
    sink = peer.create_subscription(
        String, OUTPUT_TOPIC, lambda message: outputs.append(str(message.data)), 10)
    source = peer.create_publisher(String, INPUT_TOPIC, 10)
    executor = make_executor(context)
    executor.add_node(worker)
    executor.add_node(peer)
    teardown_clean = False
    callback_count = 0
    try:
        _spin_rclpy_until(
            executor,
            lambda: source.get_subscription_count() == 1
            and worker.publisher.get_subscription_count() == 1,
        )
        started_ns = time.perf_counter_ns()
        started_cpu_ns = time.process_time_ns()
        for payload in PAYLOADS:
            expected_count = len(outputs) + 1
            source.publish(String(data=payload))
            _spin_rclpy_until(executor, lambda: len(outputs) == expected_count)
        cpu_time_ns = time.process_time_ns() - started_cpu_ns
        elapsed_ns = time.perf_counter_ns() - started_ns
        expected = [payload.strip().lower() + ":normalized" for payload in PAYLOADS]
        assert outputs == expected
        assert worker.callback_count == len(PAYLOADS)
        callback_count = worker.callback_count
        assert type(peer) is Node
        assert type(source).__module__ == "rclpy.publisher"
        assert type(sink).__module__ == "rclpy.subscription"
    finally:
        executor.remove_node(peer)
        executor.remove_node(worker)
        executor.shutdown(timeout_sec=2.0)
        peer.destroy_node()
        worker.destroy_node()
        context.shutdown()
        teardown_clean = True

    return build_case(
        project="managed_native_worker",
        variant="stock",
        tier=2,
        outputs=outputs,
        elapsed_ns=elapsed_ns,
        cpu_time_ns=cpu_time_ns,
        backend_roles={
            "node": {
                "backend": "python",
                "evidence": "exact rclpy.node.Node peer and Python subclass worker",
            },
            "executor": {
                "backend": "python",
                "evidence": "rclpy.executors.MultiThreadedExecutor factory",
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
                "evidence": "TelemetryNormalizer.on_message invocation counter",
            },
        },
        transform_crossings=callback_count,
        api_coverage=[
            "ReentrantCallbackGroup",
            "two-thread rclpy executor",
            "ordered normalized String values",
            "normal teardown",
        ],
        contract_delta=[],
        teardown_clean=teardown_clean,
    )


def _run_rewrite():
    import rclcppyy

    outputs = []
    callback_count = 0
    session = rclcppyy.native(["managed-native-worker"])
    with session as ros:
        worker_options = ros.rclcpp.NodeOptions()
        peer_options = ros.rclcpp.NodeOptions()
        worker = ros.create_node(
            "telemetry_normalizer",
            options=worker_options,
            use_intra_process=True,
        )
        peer = ros.create_node(
            "managed_worker_probe",
            options=peer_options,
            use_intra_process=True,
        )
        callback_group = ros.create_callback_group(worker, "reentrant")
        assert callback_group is not None
        assert bool(worker.get_node_options().use_intra_process_comms())
        publisher = worker.create_publisher(String, OUTPUT_TOPIC, 10)

        def normalize(message):
            nonlocal callback_count
            callback_count += 1
            value = str(message.data).strip().lower() + ":normalized"
            publisher.publish(String(data=value))

        subscription = worker.create_subscription(String, INPUT_TOPIC, normalize, 10)
        sink = peer.create_subscription(
            String, OUTPUT_TOPIC, lambda message: outputs.append(str(message.data)), 10)
        assert sink is not None
        source = peer.create_publisher(String, INPUT_TOPIC, 10)
        executor = ros.create_executor("multi_threaded", threads=2)
        executor.add_node(worker)
        executor.add_node(peer)

        _spin_cpp_until(
            executor,
            lambda: source.get_subscription_count() == 1
            and publisher.get_subscription_count() == 1,
        )
        started_ns = time.perf_counter_ns()
        started_cpu_ns = time.process_time_ns()
        for payload in PAYLOADS:
            expected_count = len(outputs) + 1
            source.publish(String(data=payload))
            _spin_cpp_until(executor, lambda: len(outputs) == expected_count)
        cpu_time_ns = time.process_time_ns() - started_cpu_ns
        elapsed_ns = time.perf_counter_ns() - started_ns
        expected = [payload.strip().lower() + ":normalized" for payload in PAYLOADS]
        assert outputs == expected
        assert callback_count == len(PAYLOADS)
        cpp_types = {
            "node": _cpp_type_name(worker),
            "executor": _cpp_type_name(executor),
            "publisher": _cpp_type_name(publisher),
            "subscription": _cpp_type_name(subscription),
        }
        assert all("rclcpp" in name for name in cpp_types.values()), cpp_types
        status = rclcppyy.status()
        assert any(
            record["backend"] == "cpp"
            and record["metadata"].get("operation") == "native"
            for record in status["operations"]
        ), status

    return build_case(
        project="managed_native_worker",
        variant="rewrite",
        tier=2,
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
                "backend": "python",
                "evidence": "explicit normalize callback invocation counter",
            },
        },
        transform_crossings=callback_count,
        api_coverage=[
            "managed custom Context",
            "NodeOptions with intra-process communication",
            "real rclcpp nodes, entities, callback group, and two-thread executor",
            "ordered normalized String values",
            "managed teardown",
        ],
        contract_delta=[
            "Node, Context, entity, and executor ownership move to rclcpp",
            "intra-process communication is explicitly enabled",
        ],
        teardown_clean=session.closed,
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
