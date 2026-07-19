#!/usr/bin/env python3
"""Run fixed-order stock/direct/raw local parameter CPU samples."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import uuid

from _domain_lease import acquire_domain
from _local_parameter_benchmark_protocol import (
    PRIMARY_METRIC,
    build_document,
    dumps,
    execution_order,
    validate_sample,
)
from _result_schema import environment_metadata


HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
WORKER = HERE / "local_parameter_benchmark_worker.py"
PREFIX = "@@RCLCPPYY_LOCAL_PARAMETER_V1@@"
MAX_REPETITIONS = 30
MAX_OPERATIONS = 100_000


def _arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--warmup-operations", type=int, default=1_000)
    parser.add_argument("--operations", type=int, default=10_000)
    parser.add_argument("--repetitions", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=REPO_ROOT / "build" / "local-parameter-characterization.json",
    )
    parser.add_argument("--smoke", action="store_true")
    return parser.parse_args(argv)


def _validate_arguments(args):
    if args.smoke:
        args.warmup_operations = 5
        args.operations = 25
        args.repetitions = 1
    if not 0 < args.warmup_operations <= MAX_OPERATIONS:
        raise ValueError(
            "warmup operations must be between 1 and %d" % MAX_OPERATIONS)
    if not 0 < args.operations <= MAX_OPERATIONS:
        raise ValueError(
            "measured operations must be between 1 and %d" % MAX_OPERATIONS)
    if not 0 < args.repetitions <= MAX_REPETITIONS:
        raise ValueError("repetitions must be between 1 and %d" % MAX_REPETITIONS)
    if args.timeout <= 0:
        raise ValueError("timeout must be positive")


def _sample_command(item, args, order_index):
    return [
        sys.executable,
        "-u",
        str(WORKER),
        "--variant", item["variant"],
        "--workload", item["workload"],
        "--warmup-operations", str(args.warmup_operations),
        "--operations", str(args.operations),
        "--repetition", str(item["repetition"]),
        "--order-index", str(order_index),
        "--run-token", "run_" + uuid.uuid4().hex,
    ]


def _parse_sample(stdout, label):
    lines = [line for line in stdout.splitlines() if line.strip()]
    records = [line for line in lines if line.startswith(PREFIX)]
    if len(records) != 1:
        raise RuntimeError("%s emitted %d protocol records" % (label, len(records)))
    try:
        sample = json.loads(records[0][len(PREFIX):])
    except json.JSONDecodeError as exception:
        raise RuntimeError("%s emitted invalid protocol JSON" % label) from exception
    sample["stdout_diagnostics"] = [
        line for line in lines if not line.startswith(PREFIX)]
    return sample


def _run_sample(item, args, order_index, environment):
    label = "%s/%s/repetition-%d" % (
        item["workload"], item["variant"], item["repetition"])
    completed = subprocess.run(
        _sample_command(item, args, order_index),
        cwd=REPO_ROOT,
        env=environment,
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
            ))
    sample = _parse_sample(completed.stdout, label)
    sample["stderr_diagnostics"] = completed.stderr.strip() or None
    validate_sample(
        sample,
        warmup=args.warmup_operations,
        operations=args.operations,
        repetition=item["repetition"],
        order_index=order_index,
    )
    return sample


def _environment(domain_id, args):
    environment = os.environ.copy()
    environment.update({
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
    metadata["ros"].update({
        "distribution": "jazzy",
        "rmw_implementation": "rmw_cyclonedds_cpp",
        "domain_id": str(domain_id),
        "automatic_discovery_range": "LOCALHOST",
    })
    metadata["benchmark_controls"] = {
        "domain_id": domain_id,
        "ros_distribution": "jazzy",
        "rmw_implementation": "rmw_cyclonedds_cpp",
        "fresh_process_per_sample": True,
        "deterministic_rotating_order": True,
        "order_policy": "rotate-variant-by-repetition-and-workload",
        "repetitions": args.repetitions,
        "fixed_warmup_operations": args.warmup_operations,
        "fixed_measured_operations": args.operations,
        "init_jit_setup_warmup_excluded": True,
        "primary_metric": PRIMARY_METRIC,
    }
    return environment, metadata


def _write(document, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(dumps(document), encoding="utf-8")
    temporary.replace(path)


def _print_summary(document, output):
    print("local parameter CPU characterization (claims disabled)")
    print("primary metric: %s" % PRIMARY_METRIC)
    print("raw evidence: %s" % output)
    for workload, values in document["summary"]["median_cpu_ratios"].items():
        print(
            "%s: direct/stock=%.4f raw/stock=%.4f" % (
                workload,
                values["direct_over_stock_process_cpu"],
                values["raw_over_stock_process_cpu"],
            ))


def main(argv=None):
    args = _arguments(argv)
    try:
        _validate_arguments(args)
    except ValueError as exception:
        raise SystemExit(str(exception)) from exception
    samples = []
    with acquire_domain() as lease:
        environment, metadata = _environment(lease.domain_id, args)
        order = execution_order(args.repetitions)
        for index, item in enumerate(order):
            samples.append(_run_sample(item, args, index, environment))
    document = build_document(
        environment=metadata,
        command=[sys.executable, *sys.argv],
        warmup=args.warmup_operations,
        operations=args.operations,
        repetitions=args.repetitions,
        samples=samples,
    )
    _write(document, args.output)
    _print_summary(document, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
