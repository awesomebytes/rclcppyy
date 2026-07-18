#!/usr/bin/env python3
"""Prebuild and run the two dynamic relay-boundary variants."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import resource
import sys
import time


TRANSFORM_BODY = "output.data = input.data * 2ULL + 1ULL;"
CPP_TYPE = "std_msgs::msg::UInt64"
HEADER = "std_msgs/msg/u_int64.hpp"
PREWARM_SCHEMA = "rclcppyy.relay-boundary-prewarm/v1"
RELAY_SCHEMA = "rclcppyy.relay-boundary-relay-event/v1"
BACKEND_SCHEMA = "rclcppyy.benchmark-backend/v1"
PROTOCOL_PREFIX = "@@RCLCPPYY_RELAY_BOUNDARY_V1@@"
RSS_GUARD_LIMIT_BYTES = 64 * 1024 * 1024


def _sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _emit(document: dict) -> None:
    print(
        PROTOCOL_PREFIX + json.dumps(document, sort_keys=True, allow_nan=False),
        flush=True,
    )


def _cpp_name(value) -> str:
    return str(
        getattr(type(value), "__cpp_name__", "")
        or getattr(value, "__cpp_name__", "")
    )


def _stock_marker(role: str, entity) -> dict:
    entity_type = type(entity)
    return {
        "schema": BACKEND_SCHEMA,
        "role": role,
        "backend": "python",
        "evidence": "stock_rclpy_entity",
        "metadata": {
            "entity_type": "%s.%s" % (
                entity_type.__module__, entity_type.__qualname__),
        },
    }


def _status_marker(snapshot: dict, role: str, *, operation: bool = False) -> dict:
    if operation:
        records = [
            record for record in snapshot["operations"]
            if record["metadata"].get("operation") == "publish"
        ]
    else:
        entity_type = "publisher" if role == "publisher" else "subscription"
        records = [
            record for record in snapshot["entities"]
            if record["metadata"].get("entity_type") == entity_type
        ]
    if not records:
        raise RuntimeError("activated relay emitted no %s status decision" % role)
    decision = records[-1]
    return {
        "schema": BACKEND_SCHEMA,
        "role": role,
        "backend": decision["backend"],
        "evidence": "rclcppyy_status_operation" if operation else "rclcppyy_status_entity",
        "metadata": {
            "decision_id": decision["id"],
            "reason": decision["reason"],
            "policies": decision["policies"],
            "entity_type": "publisher" if role == "publisher" else "subscription",
        },
    }


def _loaded_rmw() -> str:
    from rclpy.utilities import get_rmw_implementation_identifier

    loaded = get_rmw_implementation_identifier()
    requested = os.environ.get("RMW_IMPLEMENTATION")
    if not requested or loaded != requested:
        raise RuntimeError(
            "loaded RMW %r does not match requested RMW %r" % (loaded, requested))
    return loaded


def _peak_rss_bytes() -> int:
    # Linux reports ru_maxrss in KiB. This benchmark targets ROS 2 Linux hosts.
    return int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) * 1024


def _rss_guard(baseline: int, final: int) -> dict:
    growth = max(0, final - baseline)
    return {
        "kind": "post-warmup-peak-rss-growth",
        "unit": "bytes",
        "baseline_peak_bytes": baseline,
        "final_peak_bytes": final,
        "growth_bytes": growth,
        "limit_bytes": RSS_GUARD_LIMIT_BYTES,
        "within_limit": growth <= RSS_GUARD_LIMIT_BYTES,
    }


def _prewarm() -> int:
    import cppyy_kit
    from cppyy_kit import cache
    from rclcpp_kit.native import native
    from rclcpp_kit import subscription_cache
    from std_msgs.msg import UInt64

    with native(["relay-boundary-prewarm"]) as ros:
        node = ros.create_node("relay_boundary_prewarm")
        compile_args = subscription_cache._compile_args(CPP_TYPE, HEADER)
        code = compile_args.pop("code")
        subscription_result = cppyy_kit.cppdef_cached(code, **compile_args)
        pipeline = ros.create_fused_pipeline(
            node,
            UInt64,
            UInt64,
            "/relay_boundary_prewarm/input",
            "/relay_boundary_prewarm/output",
            TRANSFORM_BODY,
            qos_depth=1,
            delivery="every",
            output_memory="fresh",
        )
        pipeline_result = dict(pipeline.compile_result)
        source_id = pipeline.source_id

    artifacts = {}
    for name, result in (
        ("subscription", subscription_result),
        ("fused_pipeline", pipeline_result),
    ):
        artifact = result.get("so")
        if not artifact or not Path(artifact).is_file():
            raise RuntimeError("%s prewarm produced no shared library" % name)
        artifacts[name] = {
            "cached": bool(result.get("cached")),
            "reason": result.get("reason"),
            "path": str(Path(artifact).resolve()),
            "sha256": _sha256(artifact),
            "size_bytes": Path(artifact).stat().st_size,
        }
    subscription_paths = cache.artifact_paths(
        code,
        decls=compile_args["decls"],
        name=compile_args["name"],
        include_paths=compile_args["include_paths"],
        libraries=compile_args["libraries"],
        directory=compile_args["directory"],
    )
    if Path(subscription_paths[0]).resolve() != Path(
            artifacts["subscription"]["path"]):
        raise RuntimeError("subscription prewarm selected an unexpected artifact")
    _emit({
        "schema": PREWARM_SCHEMA,
        "pid": os.getpid(),
        "loaded_rmw": _loaded_rmw(),
        "fused_source_id": source_id,
        "artifacts": artifacts,
    })
    return 0


def _subscription_artifact() -> dict:
    from cppyy_kit import cache
    from rclcpp_kit import subscription_cache

    compile_args = subscription_cache._compile_args(CPP_TYPE, HEADER)
    code = compile_args["code"]
    artifact = cache.artifact_paths(
        code,
        decls=compile_args["decls"],
        name=compile_args["name"],
        include_paths=compile_args["include_paths"],
        libraries=compile_args["libraries"],
        directory=compile_args["directory"],
    )[0]
    path = Path(artifact).resolve()
    if not path.is_file():
        raise RuntimeError("prebuilt subscription artifact is missing")
    return {
        "state": "prebuilt",
        "kind": "subscription-trampoline",
        "path": str(path),
        "sha256": _sha256(path),
        "size_bytes": path.stat().st_size,
        "hit": True,
    }


def _run_python_relay(args, *, activate: bool) -> tuple[dict, dict, bool]:
    rclcppyy = None
    if activate:
        import rclcppyy as active_rclcppyy

        active_rclcppyy.enable_cpp_acceleration()
        rclcppyy = active_rclcppyy

    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import UInt64
    import threading

    received = 0
    published = 0
    checksum = 0
    last = 0
    context = Context()
    context.init(args=[])
    node = Node(args.node_name, context=context)
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    publisher = node.create_publisher(UInt64, args.output_topic, qos)

    def on_message(message):
        nonlocal received, published, checksum, last
        value = int(message.data)
        received += 1
        checksum += value
        last = value
        publisher.publish(UInt64(data=value * 2 + 1))
        published += 1

    subscription = node.create_subscription(UInt64, args.input_topic, on_message, qos)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=False)
    executor_thread.start()
    if activate:
        snapshot = rclcppyy.status()
        markers = {
            "publisher": _status_marker(snapshot, "publisher"),
            "subscriber": _status_marker(snapshot, "subscriber"),
        }
        cache_evidence = {
            "state": "process-warm",
            "kind": "compatible-borrowed-publish-route",
            "prepared_before_measurement": True,
        }
    else:
        markers = {
            "publisher": _stock_marker("publisher", publisher),
            "subscriber": _stock_marker("subscriber", subscription),
        }
        cache_evidence = {
            "state": "not_applicable",
            "kind": "stock-rclpy",
        }
    ready = {
        "schema": RELAY_SCHEMA,
        "event": "ready",
        "variant": args.variant,
        "run_token": args.run_token,
        "pid": os.getpid(),
        "process_group_id": os.getpgrp(),
        "node_name": args.node_name,
        "loaded_rmw": _loaded_rmw(),
        "execution_model": (
            "same-python-relay-compatible-activation"
            if activate else "same-python-relay-stock-rclpy"
        ),
        "cache": cache_evidence,
        "entity_types": {
            "node": "%s.%s" % (type(node).__module__, type(node).__qualname__),
            "publisher": "%s.%s" % (
                type(publisher).__module__, type(publisher).__qualname__),
            "subscription": "%s.%s" % (
                type(subscription).__module__, type(subscription).__qualname__),
            "executor": "%s.%s" % (
                type(executor).__module__, type(executor).__qualname__),
        },
        "backend_markers": markers,
    }
    _emit(ready)
    if sys.stdin.readline().rstrip("\n") != "START":
        raise RuntimeError("relay expected START control")
    rss_baseline = _peak_rss_bytes()
    cpu_start = time.process_time_ns()
    if sys.stdin.readline().rstrip("\n") != "REPORT":
        raise RuntimeError("relay expected REPORT control")
    cpu_time_ns = time.process_time_ns() - cpu_start
    rss_final = _peak_rss_bytes()
    publish_marker = None
    fallback_publish_operations = 0
    last_publish_backend = "python"
    if activate:
        final_status = rclcppyy.status()
        publish_marker = _status_marker(final_status, "publisher", operation=True)
        fallback_publish_operations = len([
            record for record in final_status["operations"]
            if record["metadata"].get("operation") == "publish"
            and record["backend"] == "python"
        ])
        last_publish_backend = getattr(
            publisher, "_rclcppyy_last_publish_backend", None)
    executor.remove_node(node)
    executor.shutdown(timeout_sec=2.0)
    executor_thread.join(timeout=2.0)
    thread_clean = not executor_thread.is_alive()
    node.destroy_subscription(subscription)
    node.destroy_publisher(publisher)
    node.destroy_node()
    context.shutdown()
    counters = {
        "received": received,
        "processed": received,
        "published": published,
        "checksum": checksum,
        "last": last,
        "python_callback_count": received,
        "python_boundary_crossings": received,
        "dropped": 0,
        "exceptions": 0,
        "publish_operation_marker": publish_marker,
        "fallback_publish_operations": fallback_publish_operations,
        "last_publish_backend": last_publish_backend,
        "cpu_time_ns": cpu_time_ns,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "rss_guard": _rss_guard(rss_baseline, rss_final),
    }
    return ready, counters, bool(thread_clean and not context.ok())


def _run_python_callback(args) -> tuple[dict, dict, bool]:
    import cppyy
    from rclcpp_kit import subscription_cache
    from rclcpp_kit.native import native

    received = 0
    published = 0
    checksum = 0
    last = 0
    session = native(["relay-boundary-python-callback"])
    with session as ros:
        cppyy.include(HEADER)
        cpp_type = cppyy.gbl.std_msgs.msg.UInt64
        node = ros.create_node(args.node_name)
        qos = ros.rclcpp.QoS(ros.rclcpp.KeepLast(1))
        publisher = node.create_publisher[cpp_type](args.output_topic, qos)

        def on_message(message):
            nonlocal received, published, checksum, last
            value = int(message.data)
            received += 1
            checksum += value
            last = value
            output = cpp_type()
            output.data = value * 2 + 1
            publisher.publish(output)
            published += 1

        callback = cppyy.gbl.std.function[
            "void(std::shared_ptr<const %s>)" % CPP_TYPE
        ](on_message)
        subscription = subscription_cache.make_subscription(
            node, CPP_TYPE, HEADER, args.input_topic, qos, callback)
        if subscription is None:
            raise RuntimeError("native Python callback did not use the prebuilt subscription route")
        executor = ros.create_executor("single_threaded", threads=1)
        executor.add_node(node)
        executor_thread = ros.start_executor(executor)
        cache_evidence = _subscription_artifact()
        ready = {
            "schema": RELAY_SCHEMA,
            "event": "ready",
            "variant": args.variant,
            "run_token": args.run_token,
            "pid": os.getpid(),
            "process_group_id": os.getpgrp(),
            "node_name": args.node_name,
            "loaded_rmw": _loaded_rmw(),
            "execution_model": "native-rclcpp-python-transform-callback",
            "cache": cache_evidence,
            "entity_types": {
                "node": _cpp_name(node),
                "publisher": _cpp_name(publisher),
                "subscription": _cpp_name(subscription),
                "executor": _cpp_name(executor),
            },
        }
        _emit(ready)
        if sys.stdin.readline().rstrip("\n") != "START":
            raise RuntimeError("relay expected START control")
        rss_baseline = _peak_rss_bytes()
        cpu_start = time.process_time_ns()
        if sys.stdin.readline().rstrip("\n") != "REPORT":
            raise RuntimeError("relay expected REPORT control")
        cpu_time_ns = time.process_time_ns() - cpu_start
        rss_final = _peak_rss_bytes()
        thread_exceptions = int(executor_thread.exceptions)
        counters = {
            "received": received,
            "processed": received,
            "published": published,
            "checksum": checksum,
            "last": last,
            "python_callback_count": received,
            "python_boundary_crossings": received,
            "dropped": 0,
            "exceptions": thread_exceptions,
            "cpu_time_ns": cpu_time_ns,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "rss_guard": _rss_guard(rss_baseline, rss_final),
        }
    return ready, counters, bool(session.closed and executor_thread.closed)


def _run_fused(args) -> tuple[dict, dict, bool]:
    from rclcpp_kit.native import native
    from std_msgs.msg import UInt64

    session = native(["relay-boundary-fused"])
    with session as ros:
        node = ros.create_node(args.node_name)
        pipeline = ros.create_fused_pipeline(
            node,
            UInt64,
            UInt64,
            args.input_topic,
            args.output_topic,
            TRANSFORM_BODY,
            qos_depth=1,
            delivery="every",
            output_memory="fresh",
        )
        compile_result = dict(pipeline.compile_result)
        artifact = compile_result.get("so")
        if compile_result.get("cached") is not True or not artifact:
            raise RuntimeError("fused relay did not load its prebuilt artifact")
        artifact_path = Path(artifact).resolve()
        if not artifact_path.is_file():
            raise RuntimeError("fused relay artifact is missing")
        executor = ros.create_executor("single_threaded", threads=1)
        executor.add_node(node)
        executor_thread = ros.start_executor(executor)
        ready = {
            "schema": RELAY_SCHEMA,
            "event": "ready",
            "variant": args.variant,
            "run_token": args.run_token,
            "pid": os.getpid(),
            "process_group_id": os.getpgrp(),
            "node_name": args.node_name,
            "loaded_rmw": _loaded_rmw(),
            "execution_model": "native-content-addressed-fused-cpp-relay",
            "cache": {
                "state": "prebuilt",
                "kind": "fused-pipeline-shared-library",
                "path": str(artifact_path),
                "sha256": _sha256(artifact_path),
                "size_bytes": artifact_path.stat().st_size,
                "hit": True,
                "source_id": pipeline.source_id,
            },
            "entity_types": {
                "node": _cpp_name(node),
                "executor": _cpp_name(executor),
            },
        }
        _emit(ready)
        if sys.stdin.readline().rstrip("\n") != "START":
            raise RuntimeError("relay expected START control")
        rss_baseline = _peak_rss_bytes()
        cpu_start = time.process_time_ns()
        if sys.stdin.readline().rstrip("\n") != "REPORT":
            raise RuntimeError("relay expected REPORT control")
        cpu_time_ns = time.process_time_ns() - cpu_start
        rss_final = _peak_rss_bytes()
        stats = pipeline.stats()
        thread_exceptions = int(executor_thread.exceptions)
        counters = {
            "received": stats.received,
            "processed": stats.processed,
            "published": stats.published,
            "checksum": None,
            "last": None,
            "python_callback_count": 0,
            "python_boundary_crossings": stats.python_boundary_crossings,
            "dropped": stats.dropped + stats.coalesced,
            "exceptions": stats.exceptions + thread_exceptions,
            "output_instances": stats.output_instances,
            "compile_cache_hits": stats.compile_cache_hits,
            "compile_cache_misses": stats.compile_cache_misses,
            "cpu_time_ns": cpu_time_ns,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "rss_guard": _rss_guard(rss_baseline, rss_final),
        }
    return ready, counters, bool(
        session.closed and pipeline.closed and executor_thread.closed)


def _run_relay(args) -> int:
    if args.variant == "stock-rclpy":
        _ready, counters, teardown_clean = _run_python_relay(args, activate=False)
    elif args.variant == "compatible-rclcppyy":
        _ready, counters, teardown_clean = _run_python_relay(args, activate=True)
    elif args.variant == "native-python-callback":
        _ready, counters, teardown_clean = _run_python_callback(args)
    else:
        _ready, counters, teardown_clean = _run_fused(args)
    total = args.warmup_messages + args.messages
    correct = (
        counters["received"] == total
        and counters["processed"] == total
        and counters["published"] == total
        and counters["dropped"] == 0
        and counters["exceptions"] == 0
    )
    _emit({
        "schema": RELAY_SCHEMA,
        "event": "report",
        "variant": args.variant,
        "run_token": args.run_token,
        **counters,
        "correct": correct,
        "teardown_clean": teardown_clean,
    })
    return 0 if correct and teardown_clean else 2


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prewarm", action="store_true")
    parser.add_argument(
        "--variant", choices=(
            "stock-rclpy", "compatible-rclcppyy",
            "native-python-callback", "native-fused"))
    parser.add_argument("--input-topic")
    parser.add_argument("--output-topic")
    parser.add_argument("--node-name")
    parser.add_argument("--warmup-messages", type=int)
    parser.add_argument("--messages", type=int)
    parser.add_argument("--run-token")
    return parser


def main() -> int:
    parser = _parser()
    args = parser.parse_args()
    if args.prewarm:
        return _prewarm()
    required = (
        "variant", "input_topic", "output_topic", "node_name",
        "warmup_messages", "messages", "run_token",
    )
    missing = [name for name in required if getattr(args, name) is None]
    if missing:
        parser.error("relay mode requires: " + ", ".join(missing))
    if args.warmup_messages < 0 or args.messages <= 0:
        parser.error("message counts must be non-negative with messages positive")
    return _run_relay(args)


if __name__ == "__main__":
    raise SystemExit(main())
