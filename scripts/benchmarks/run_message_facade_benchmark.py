#!/usr/bin/env python3
"""Run paired stock/facade UInt64 and String characterization samples."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import uuid

from _domain_lease import acquire_domain
from _message_facade_benchmark_protocol import (
    METRICS,
    build_document,
    dumps,
    execution_order,
    validate_sample,
)
from _result_schema import environment_metadata


HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
WORKER = HERE / "message_facade_benchmark_worker.py"
PREFIX = "@@RCLCPPYY_MESSAGE_FACADE_V1@@"
MAX_REPETITIONS = 30
MAX_MESSAGES = 100_000


def _arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--warmup-messages", type=int, default=500)
    parser.add_argument("--messages", type=int, default=5_000)
    parser.add_argument("--repetitions", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=REPO_ROOT / "build" / "message-facade-characterization.json",
    )
    parser.add_argument("--smoke", action="store_true")
    return parser.parse_args(argv)


def _validate_arguments(args):
    if args.smoke:
        args.warmup_messages = 20
        args.messages = 100
        args.repetitions = 1
    if not 0 < args.warmup_messages <= MAX_MESSAGES:
        raise ValueError("warmup messages must be between 1 and %d" % MAX_MESSAGES)
    if not 0 < args.messages <= MAX_MESSAGES:
        raise ValueError("messages must be between 1 and %d" % MAX_MESSAGES)
    if not 0 < args.repetitions <= MAX_REPETITIONS:
        raise ValueError("repetitions must be between 1 and %d" % MAX_REPETITIONS)
    if args.timeout <= 0:
        raise ValueError("timeout must be positive")


def _sample_command(item, args, order_index):
    return [
        sys.executable,
        "-u",
        str(WORKER),
        "--variant",
        item["variant"],
        "--message-type",
        item["message_type"],
        "--warmup-messages",
        str(args.warmup_messages),
        "--messages",
        str(args.messages),
        "--repetition",
        str(item["repetition"]),
        "--order-index",
        str(order_index),
        "--run-token",
        "run_" + uuid.uuid4().hex,
    ]


def _parse_sample(stdout, label):
    lines = [line for line in stdout.splitlines() if line.strip()]
    records = [line for line in lines if line.startswith(PREFIX)]
    if len(records) != 1:
        raise RuntimeError("%s emitted %d protocol records" % (label, len(records)))
    try:
        sample = json.loads(records[0][len(PREFIX):])
    except json.JSONDecodeError as exc:
        raise RuntimeError("%s emitted invalid protocol JSON" % label) from exc
    diagnostics = [line for line in lines if not line.startswith(PREFIX)]
    sample["stdout_diagnostics"] = diagnostics
    return sample


def _run_sample(item, args, order_index, env):
    label = "%s/%s/repetition-%d" % (
        item["message_type"], item["variant"], item["repetition"])
    completed = subprocess.run(
        _sample_command(item, args, order_index),
        cwd=REPO_ROOT,
        env=env,
        text=True,
        capture_output=True,
        timeout=args.timeout,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "%s failed with code %d:\n%s\n%s" % (
                label,
                completed.returncode,
                completed.stdout[-3000:],
                completed.stderr[-3000:],
            )
        )
    sample = _parse_sample(completed.stdout, label)
    sample["stderr_diagnostics"] = completed.stderr.strip() or None
    validate_sample(
        sample,
        warmup=args.warmup_messages,
        messages=args.messages,
        repetition=item["repetition"],
        order_index=order_index,
    )
    return sample


def _environment(domain_id, args):
    env = os.environ.copy()
    env.update({
        "ROS_DISTRO": "jazzy",
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
        "ROS_DOMAIN_ID": str(domain_id),
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        "PYTHONHASHSEED": "0",
        "OMP_NUM_THREADS": "1",
        "OPENBLAS_NUM_THREADS": "1",
        "MKL_NUM_THREADS": "1",
    })
    metadata = environment_metadata(REPO_ROOT)
    metadata["benchmark_controls"] = {
        "domain_id": domain_id,
        "rmw_implementation": "rmw_cyclonedds_cpp",
        "ros_distribution": "jazzy",
        "fresh_process_per_sample": True,
        "single_threaded_executor": True,
        "fixed_warmup_messages": args.warmup_messages,
        "fixed_measured_messages": args.messages,
        "setup_jit_discovery_warmup_excluded": True,
        "rotating_paired_order": True,
        "rss_is_guard_only": True,
    }
    return env, metadata


def _write(document, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(dumps(document), encoding="utf-8")
    temporary.replace(path)


def _print_summary(document, output):
    print("message facade characterization (claims disabled)")
    print("raw evidence: %s" % output)
    print("ratio is message_facade / stock_rclpy")
    for message_type, values in document["summary"][
            "median_facade_over_stock"].items():
        print("%s:" % message_type)
        for metric in METRICS:
            print("  %-32s %.4f" % (metric, values[metric]))


def main(argv=None):
    args = _arguments(argv)
    try:
        _validate_arguments(args)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    samples = []
    with acquire_domain() as lease:
        env, metadata = _environment(lease.domain_id, args)
        order = execution_order(args.repetitions)
        for index, item in enumerate(order):
            samples.append(_run_sample(item, args, index, env))
    document = build_document(
        environment=metadata,
        command=[sys.executable, *sys.argv],
        warmup=args.warmup_messages,
        messages=args.messages,
        repetitions=args.repetitions,
        samples=samples,
    )
    _write(document, args.output)
    _print_summary(document, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
