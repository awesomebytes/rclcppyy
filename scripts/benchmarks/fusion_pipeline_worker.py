#!/usr/bin/env python3
"""Prebuild and run the cppyy-loaded C++ fusion pipeline benchmark lane."""

from __future__ import annotations

import argparse
import faulthandler
import hashlib
import json
import os
from pathlib import Path
import resource
import signal
import sys
import time


PREFIX = "@@RCLCPPYY_FUSION_PIPELINE_V1@@"
PREWARM_SCHEMA = "rclcppyy.fusion-pipeline-prewarm/v1"
RELAY_SCHEMA = "rclcppyy.fusion-pipeline-relay/v1"
RSS_LIMIT = 64 * 1024 * 1024
TRANSFORM_BODY = (
    "output.data = (((input.data + 1ULL) * 3ULL) + 5ULL) ^ 0x5aULL;"
)


def _emit(document: dict) -> None:
    print(PREFIX + json.dumps(document, sort_keys=True, allow_nan=False), flush=True)


def _sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _loaded_rmw() -> str:
    from rclpy.utilities import get_rmw_implementation_identifier

    loaded = get_rmw_implementation_identifier()
    requested = os.environ.get("RMW_IMPLEMENTATION")
    if not requested or loaded != requested:
        raise RuntimeError(
            "loaded RMW %r does not match requested RMW %r" % (loaded, requested))
    return loaded


def _peak_rss() -> int:
    return int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) * 1024


def _rss_guard(baseline: int, final: int) -> dict:
    growth = max(0, final - baseline)
    return {
        "kind": "post-warmup-peak-rss-growth",
        "unit": "bytes",
        "baseline_peak_bytes": baseline,
        "final_peak_bytes": final,
        "growth_bytes": growth,
        "limit_bytes": RSS_LIMIT,
        "within_limit": growth <= RSS_LIMIT,
    }


def _pipeline(ros, node, input_topic: str, output_topic: str):
    from std_msgs.msg import UInt64

    return ros.create_fused_pipeline(
        node,
        UInt64,
        UInt64,
        input_topic,
        output_topic,
        TRANSFORM_BODY,
        qos_depth=1,
        delivery="every",
        output_memory="fresh",
    )


def _artifact(pipeline) -> dict:
    result = dict(pipeline.compile_result)
    path_value = result.get("so")
    if not path_value:
        raise RuntimeError("fused pipeline produced no shared library")
    path = Path(path_value).resolve()
    if not path.is_file():
        raise RuntimeError("fused pipeline shared library is missing")
    return {
        "cached": bool(result.get("cached")),
        "reason": result.get("reason"),
        "path": str(path),
        "sha256": _sha256(path),
        "size_bytes": path.stat().st_size,
        "source_id": pipeline.source_id,
    }


def _prewarm() -> int:
    from rclcpp_kit.native import native

    with native(["fusion-pipeline-prewarm"]) as ros:
        node = ros.create_node(
            "fusion_pipeline_prewarm", use_intra_process=True)
        pipeline = _pipeline(
            ros, node, "/fusion_pipeline_prewarm/input",
            "/fusion_pipeline_prewarm/output")
        artifact = _artifact(pipeline)
    _emit({
        "schema": PREWARM_SCHEMA,
        "pid": os.getpid(),
        "loaded_rmw": _loaded_rmw(),
        "artifact": artifact,
    })
    return 0


def _run(args) -> int:
    from rclcpp_kit.native import native

    session = native(["fusion-pipeline-cppyy-fused"])
    counters = None
    artifact = None
    executor_thread = None
    pipeline = None
    with session as ros:
        node = ros.create_node(args.node_name, use_intra_process=True)
        pipeline = _pipeline(ros, node, args.input_topic, args.output_topic)
        artifact = _artifact(pipeline)
        if not artifact["cached"]:
            raise RuntimeError("measured cppyy lane did not load its prewarmed artifact")
        executor = ros.create_executor("single_threaded", threads=1)
        executor.add_node(node)
        executor_thread = ros.start_executor(executor)
        _emit({
            "schema": RELAY_SCHEMA,
            "event": "ready",
            "variant": "cppyy-fused",
            "run_token": args.run_token,
            "pid": os.getpid(),
            "process_group_id": os.getpgrp(),
            "node_name": args.node_name,
            "loaded_rmw": _loaded_rmw(),
            "execution_model": "cppyy-loaded-single-callback-fused-cpp",
            "representation": "std_msgs::msg::UInt64",
            "python_message_conversions": 0,
            "ros_entity_count": 2,
            "observable_topic_count": 2,
            "composition": {
                "relay_processes": 1,
                "relay_nodes": 1,
                "executor": "single_threaded",
                "executor_threads": 1,
                "use_intra_process_comms": True,
            },
            "cache": {**artifact, "state": "prebuilt", "kind": "fused-shared-library"},
        })
        if sys.stdin.readline().rstrip("\n") != "ARM":
            raise RuntimeError("relay expected ARM")
        rss_start = _peak_rss()
        cpu_start = time.process_time_ns()
        _emit({
            "schema": RELAY_SCHEMA,
            "event": "armed",
            "variant": "cppyy-fused",
            "run_token": args.run_token,
            "pid": os.getpid(),
            "process_group_id": os.getpgrp(),
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        })
        if sys.stdin.readline().rstrip("\n") != "REPORT":
            raise RuntimeError("relay expected REPORT")
        cpu_time_ns = time.process_time_ns() - cpu_start
        rss_stop = _peak_rss()
        stats = pipeline.stats()
        counters = {
            "received": int(stats.received),
            "published": int(stats.published),
            "logical_stage_events": int(stats.processed) * 4,
            "python_callback_count": 0,
            "python_boundary_crossings": int(stats.python_boundary_crossings),
            "python_message_conversions": 0,
            "cpu_time_ns": cpu_time_ns,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "rss_guard": _rss_guard(rss_start, rss_stop),
            "dropped": int(stats.dropped + stats.coalesced),
            "exceptions": int(stats.exceptions + executor_thread.exceptions),
            "compile_cache_hits": int(stats.compile_cache_hits),
            "compile_cache_misses": int(stats.compile_cache_misses),
        }
    total = args.warmup_messages + args.messages
    teardown_clean = bool(
        session.closed and pipeline.closed and executor_thread.closed)
    correct = bool(
        counters["received"] == total
        and counters["published"] == total
        and counters["logical_stage_events"] == total * 4
        and counters["dropped"] == 0
        and counters["exceptions"] == 0
        and counters["python_boundary_crossings"] == 0
    )
    _emit({
        "schema": RELAY_SCHEMA,
        "event": "report",
        "variant": "cppyy-fused",
        "run_token": args.run_token,
        **counters,
        "correct": correct,
        "teardown_clean": teardown_clean,
    })
    return 0 if correct and teardown_clean else 2


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prewarm", action="store_true")
    parser.add_argument("--input-topic")
    parser.add_argument("--output-topic")
    parser.add_argument("--node-name")
    parser.add_argument("--warmup-messages", type=int)
    parser.add_argument("--messages", type=int)
    parser.add_argument("--run-token")
    return parser


def main() -> int:
    faulthandler.enable(file=sys.stderr, all_threads=True)
    faulthandler.register(signal.SIGUSR1, file=sys.stderr, all_threads=True)
    args = _parser().parse_args()
    if args.prewarm:
        return _prewarm()
    required = (
        "input_topic", "output_topic", "node_name", "warmup_messages",
        "messages", "run_token")
    missing = [name for name in required if getattr(args, name) is None]
    if missing:
        raise SystemExit("missing relay arguments: " + ", ".join(missing))
    if args.warmup_messages <= 0 or args.messages <= 0:
        raise SystemExit("message counts must be positive")
    return _run(args)


if __name__ == "__main__":
    raise SystemExit(main())
