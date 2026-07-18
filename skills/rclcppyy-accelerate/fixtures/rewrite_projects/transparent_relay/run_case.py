#!/usr/bin/env python3
"""Run one isolated stock or transparent-activation relay case."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time


FIXTURE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIXTURE_ROOT))

from evidence_protocol import PAYLOADS, build_case, emit_case  # noqa: E402
from stock_app import TextRelay  # noqa: E402

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402


INPUT_TOPIC = "/rewrite_fixture/t0/input"
OUTPUT_TOPIC = "/rewrite_fixture/t0/output"


def _spin_until(executor, predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return
    raise TimeoutError("transparent relay condition did not become true")


def _matching_status(status, entity_type, topic, backend):
    return [
        record for record in status["entities"]
        if record["backend"] == backend
        and record["metadata"].get("entity_type") == entity_type
        and record["metadata"].get("topic") == topic
    ]


def run(variant):
    if variant == "rewrite":
        import rclcppyy
        rclcppyy.enable_cpp_acceleration(profile="compatible")

    context = rclpy.context.Context()
    context.init(args=[])
    relay = TextRelay(context=context)
    peer = Node("transparent_relay_probe", context=context)
    outputs = []
    sink = peer.create_subscription(
        String, OUTPUT_TOPIC, lambda message: outputs.append(str(message.data)), 10)
    source = peer.create_publisher(String, INPUT_TOPIC, 10)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(relay)
    executor.add_node(peer)
    teardown_clean = False
    try:
        _spin_until(
            executor,
            lambda: source.get_subscription_count() == 1
            and relay.publisher.get_subscription_count() == 1,
        )
        started_ns = time.perf_counter_ns()
        started_cpu_ns = time.process_time_ns()
        for payload in PAYLOADS:
            expected_count = len(outputs) + 1
            source.publish(String(data=payload))
            _spin_until(executor, lambda: len(outputs) == expected_count)
        cpu_time_ns = time.process_time_ns() - started_cpu_ns
        elapsed_ns = time.perf_counter_ns() - started_ns
        expected = [payload + ":relayed" for payload in PAYLOADS]
        assert outputs == expected
        assert relay.callback_count == len(PAYLOADS)

        if variant == "stock":
            assert type(peer) is Node
            assert type(source).__module__ == "rclpy.publisher"
            assert type(sink).__module__ == "rclpy.subscription"
            roles = {
                "node": {
                    "backend": "python",
                    "evidence": "exact rclpy.node.Node type",
                },
                "publisher": {
                    "backend": "python",
                    "evidence": "exact rclpy.publisher.Publisher type without activation",
                },
                "subscription": {
                    "backend": "python",
                    "evidence": "exact rclpy.subscription.Subscription type without activation",
                },
                "transform_callback": {
                    "backend": "python",
                    "evidence": "TextRelay.on_message invocation counter",
                },
            }
            coverage = [
                "stock Node/Context/executor ownership",
                "reliable ordered String relay",
                "normal teardown",
            ]
        else:
            import rclcppyy
            status = rclcppyy.status()
            publisher_records = _matching_status(
                status, "publisher", INPUT_TOPIC, "python")
            publisher_records += _matching_status(
                status, "publisher", OUTPUT_TOPIC, "python")
            subscription_records = _matching_status(
                status, "subscription", INPUT_TOPIC, "python")
            subscription_records += _matching_status(
                status, "subscription", OUTPUT_TOPIC, "python")
            assert len(publisher_records) == 2, status
            assert len(subscription_records) == 2, status
            assert status["counts"]["nodes"]["python"] >= 2
            roles = {
                "node": {
                    "backend": "python",
                    "evidence": "rclcppyy status stock_node_authority records",
                },
                "publisher": {
                    "backend": "python",
                    "evidence": "stock publish-authority status records for both topics",
                },
                "subscription": {
                    "backend": "python",
                    "evidence": "stock subscription authority status records for both topics",
                },
                "transform_callback": {
                    "backend": "python",
                    "evidence": "TextRelay.on_message invocation counter",
                },
            }
            coverage = [
                "unchanged TextRelay source",
                "compatible profile stock-authority status",
                "reliable ordered String relay",
                "stock-owned teardown",
            ]
    finally:
        executor.remove_node(peer)
        executor.remove_node(relay)
        executor.shutdown(timeout_sec=2.0)
        peer.destroy_node()
        relay.destroy_node()
        context.shutdown()
        teardown_clean = True

    return build_case(
        project="transparent_relay",
        variant=variant,
        tier=0,
        outputs=outputs,
        elapsed_ns=elapsed_ns,
        cpu_time_ns=cpu_time_ns,
        backend_roles=roles,
        transform_crossings=relay.callback_count,
        api_coverage=coverage,
        contract_delta=[],
        teardown_clean=teardown_clean,
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", choices=("stock", "rewrite"), required=True)
    args = parser.parse_args(argv)
    emit_case(run(args.variant))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
