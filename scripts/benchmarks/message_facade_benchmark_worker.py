#!/usr/bin/env python3
"""One fresh-process stock or facade self-pub/sub characterization sample."""

from __future__ import annotations

import argparse
import json
import os
import time

import psutil

from _message_facade_benchmark_protocol import (
    MESSAGE_TYPES,
    RSS_GUARD_LIMIT_BYTES,
    SAMPLE_SCHEMA,
    VARIANTS,
    expected_checksum,
    latency_summary,
)


PREFIX = "@@RCLCPPYY_MESSAGE_FACADE_V1@@"
TIMEOUT_S = 10.0
STRING_PADDING = "x" * 51


def _arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", choices=VARIANTS, required=True)
    parser.add_argument("--message-type", choices=MESSAGE_TYPES, required=True)
    parser.add_argument("--warmup-messages", type=int, required=True)
    parser.add_argument("--messages", type=int, required=True)
    parser.add_argument("--repetition", type=int, required=True)
    parser.add_argument("--order-index", type=int, required=True)
    parser.add_argument("--run-token", required=True)
    return parser.parse_args(argv)


def _encoded(message_type, sequence):
    if message_type == "UInt64":
        return sequence
    value = "%012d|%s" % (sequence, STRING_PADDING)
    assert len(value) == 64
    return value


def _decoded(message_type, value):
    if message_type == "UInt64":
        return int(value)
    assert len(value) == 64 and value[12] == "|"
    return int(value[:12])


def _entity_types(node, publisher, subscription, executor):
    return {
        "node": type(node).__module__ + "." + type(node).__qualname__,
        "publisher": (
            type(publisher).__module__ + "." + type(publisher).__qualname__),
        "subscription": (
            type(subscription).__module__ + "." +
            type(subscription).__qualname__),
        "executor": (
            type(executor).__module__ + "." + type(executor).__qualname__),
    }


def _facade_backend_evidence(
        message_type, publisher, subscription, total_messages):
    import cppyy
    import rclcppyy
    from rclcpp_kit import message_facade

    binding = message_facade.binding_for_type(publisher.msg_type)
    assert type(publisher._message_facade_benchmark_message) is binding.facade_type
    assert publisher._rclcppyy_publish_route.message_type is binding.original_type
    assert subscription._rclcppyy_take_route.original_type is binding.original_type
    status = rclcppyy.status()
    route_operations = [
        record for record in status["operations"]
        if record["metadata"].get("operation") in (
            "publish", "subscription_take")
    ]
    cpp_operations = [
        record for record in route_operations
        if record["backend"] == "cpp" and
        "direct_cpp_message" in record["policies"]
    ]
    fallback_operations = [
        record for record in route_operations if record["backend"] != "cpp"]
    entities = [
        record for record in status["entities"]
        if record["backend"] == "cpp" and
        record["metadata"].get("message_type") == binding.cpp_type_name
    ]
    assert len(cpp_operations) == 2
    assert len(entities) == 2
    assert not fallback_operations
    assert publisher._rclcppyy_last_publish_backend == "cpp"
    assert subscription._rclcppyy_last_take_backend == "cpp"
    facade_message = publisher._message_facade_benchmark_message
    return {
        "verified": True,
        "profile": "message_facade",
        "publisher": "cpp",
        "subscription_take": "cpp",
        "representation": binding.cpp_type_name,
        "converter_forbidden": True,
        "python_to_cpp_whole_message_conversions": 0,
        "publish_route_messages": publisher._rclcppyy_publish_route._scratch_stats()[
            "publishes"],
        "take_route_messages": subscription._rclcppyy_take_route.stats()["takes"],
        "fallback_operations": len(fallback_operations),
        "hidden_original_entity_mapping": True,
        "binding_original_type": (
            binding.original_type.__module__ + "." +
            binding.original_type.__qualname__),
        "facade_storage_address": int(cppyy.addressof(
            facade_message._rclcpp_kit_cpp_message)),
        "status_direct_operation_count": len(cpp_operations),
        "status_direct_entity_count": len(entities),
        "message_type": message_type,
        "total_route_messages": total_messages,
    }


def _stock_backend_evidence(message_type):
    return {
        "verified": True,
        "profile": "stock",
        "publisher": "python",
        "subscription_take": "python",
        "representation": "generated_python_message",
        "converter_forbidden": False,
        "python_to_cpp_whole_message_conversions": None,
        "fallback_operations": 0,
        "message_type": message_type,
    }


def run(args):
    if args.variant == "message-facade-rclcppyy":
        import rclcppyy

        rclcppyy.enable_cpp_acceleration(profile="message_facade")

    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.publisher import Publisher
    from rclpy.subscription import Subscription
    from std_msgs.msg import String, UInt64

    message_class = UInt64 if args.message_type == "UInt64" else String
    if args.variant == "message-facade-rclcppyy":
        from rclcpp_kit import borrowed_publish

        def conversion_forbidden(*_args, **_kwargs):
            raise AssertionError("whole-message Python-to-C++ conversion was used")

        borrowed_publish.convert_python_msg_to_cpp = conversion_forbidden

    context = Context()
    context.init(args=[])
    suffix = args.run_token[-12:]
    topic = "/rclcppyy_message_facade_bench/run_" + suffix
    node = rclpy.create_node("facade_bench_" + suffix, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    state = {
        "received_sequence": 0,
        "callback_time_ns": 0,
        "last_message": None,
        "exceptions": 0,
    }

    def callback(message):
        try:
            sequence = _decoded(args.message_type, message.data)
            expected = state["received_sequence"] + 1
            if sequence != expected:
                raise AssertionError(
                    "received sequence %d, expected %d" % (sequence, expected))
            state["received_sequence"] = sequence
            state["callback_time_ns"] = time.perf_counter_ns()
            state["last_message"] = message
        except Exception:
            state["exceptions"] += 1
            raise

    subscription = node.create_subscription(message_class, topic, callback, 10)
    publisher = node.create_publisher(message_class, topic, 10)
    assert type(node) is Node
    assert type(publisher) is Publisher
    assert type(subscription) is Subscription
    assert type(executor) is SingleThreadedExecutor
    message = message_class()
    publisher._message_facade_benchmark_message = message

    discovery_deadline = time.monotonic() + TIMEOUT_S
    while (publisher.get_subscription_count() < 1 and
           time.monotonic() < discovery_deadline):
        executor.spin_once(timeout_sec=0.01)
    if publisher.get_subscription_count() < 1:
        raise RuntimeError("self subscription discovery timed out")

    def one(sequence):
        message.data = _encoded(args.message_type, sequence)
        sent_ns = time.perf_counter_ns()
        publisher.publish(message)
        deadline = time.monotonic() + TIMEOUT_S
        while state["received_sequence"] < sequence and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        if state["received_sequence"] != sequence:
            raise RuntimeError("message %d timed out" % sequence)
        return state["callback_time_ns"] - sent_ns

    for sequence in range(1, args.warmup_messages + 1):
        one(sequence)

    process = psutil.Process()
    baseline_rss = process.memory_info().rss
    peak_rss = baseline_rss
    latencies = []
    checksum = 0
    first = args.warmup_messages + 1
    last = args.warmup_messages + args.messages
    cpu_started = time.process_time_ns()
    elapsed_started = time.perf_counter_ns()
    for sequence in range(first, last + 1):
        latencies.append(one(sequence))
        checksum += sequence
        if sequence % 128 == 0 or sequence == last:
            peak_rss = max(peak_rss, process.memory_info().rss)
    elapsed_ns = time.perf_counter_ns() - elapsed_started
    cpu_time_ns = time.process_time_ns() - cpu_started

    total_messages = args.warmup_messages + args.messages
    backend = (
        _facade_backend_evidence(
            args.message_type, publisher, subscription, total_messages)
        if args.variant == "message-facade-rclcppyy" else
        _stock_backend_evidence(args.message_type)
    )
    backend["entity_types"] = _entity_types(
        node, publisher, subscription, executor)
    backend["callback_message_type"] = (
        type(state["last_message"]).__module__ + "." +
        type(state["last_message"]).__qualname__)
    identity = (node.get_name(), node.get_namespace())
    backend["graph_node_identity_count"] = (
        node.get_node_names_and_namespaces().count(identity))

    destroyed_publisher = node.destroy_publisher(publisher)
    destroyed_subscription = node.destroy_subscription(subscription)
    executor.remove_node(node)
    removed_node = node not in executor.get_nodes()
    executor.shutdown(timeout_sec=2.0)
    node.destroy_node()
    context.shutdown()
    teardown_clean = bool(
        destroyed_publisher and destroyed_subscription and removed_node and
        not context.ok())

    latency = latency_summary(latencies)
    growth = max(0, peak_rss - baseline_rss)
    return {
        "schema": SAMPLE_SCHEMA,
        "variant": args.variant,
        "message_type": args.message_type,
        "repetition": args.repetition,
        "order_index": args.order_index,
        "run_token": args.run_token,
        "pid": os.getpid(),
        "warmup_messages": args.warmup_messages,
        "measured_messages": args.messages,
        "runtime": {
            "fresh_process": True,
            "setup_excluded": True,
            "discovery_complete": True,
            "single_threaded_executor": True,
            "fixed_work": True,
            "teardown_clean": teardown_clean,
            "ros_distribution": os.environ.get("ROS_DISTRO"),
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
            "domain_id": os.environ.get("ROS_DOMAIN_ID"),
        },
        "correctness": {
            "received": args.messages,
            "dropped": 0,
            "exceptions": state["exceptions"],
            "checksum": checksum,
            "expected_checksum": expected_checksum(
                args.message_type, first, args.messages),
            "last_sequence": state["received_sequence"],
            "value_contract_verified": checksum == expected_checksum(
                args.message_type, first, args.messages),
        },
        "timing": {
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "cpu_time_ns": cpu_time_ns,
            "cpu_ns_per_message": cpu_time_ns / args.messages,
            "elapsed_ns": elapsed_ns,
            "throughput_messages_per_second": args.messages * 1e9 / elapsed_ns,
            "latency": {"count": len(latencies), **latency},
            "latency_p50_ns": latency["p50_ns"],
            "latency_p99_ns": latency["p99_ns"],
        },
        "rss_guard": {
            "baseline_bytes": baseline_rss,
            "peak_bytes": peak_rss,
            "growth_bytes": growth,
            "limit_bytes": RSS_GUARD_LIMIT_BYTES,
            "passed": growth <= RSS_GUARD_LIMIT_BYTES,
            "interpretation": "guard_only_not_a_performance_metric",
        },
        "backend_evidence": backend,
    }


def main(argv=None):
    args = _arguments(argv)
    if args.warmup_messages <= 0 or args.messages <= 0:
        raise SystemExit("warmup and messages must be positive")
    sample = run(args)
    print(PREFIX + json.dumps(sample, sort_keys=True, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
