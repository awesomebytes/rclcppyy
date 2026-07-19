#!/usr/bin/env python3
"""Run a bounded direct ActionServer copy-versus-shared diagnostic."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from statistics import median
import sys
import tempfile
import uuid

from _action_server_protocol import RMW, ROS_DISTRO
from _domain_lease import acquire_domain
from run_action_client_benchmark import GraphObserver, _compile_aot
from run_action_server_benchmark import _prewarm, _run_sample


HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
DEFAULT_BASELINE = REPO_ROOT / "build/action-server-cyclone-b2f0f2d-corrected.json"


def _artifact_baseline(path: Path) -> dict | None:
    if not path.is_file():
        return None
    document = json.loads(path.read_text(encoding="utf-8"))
    values = {"direct-source-compatible": {}, "aot-staged": {}}
    for sample in document.get("results", []):
        variant = sample.get("variant")
        if variant in values:
            values[variant][sample["repetition"]] = (
                sample["timing"]["server_cpu_ns_per_completed_goal"])
    if not all(values.values()):
        return None
    direct_values = values["direct-source-compatible"]
    aot_values = values["aot-staged"]
    repetitions = sorted(set(direct_values) & set(aot_values))
    if not repetitions:
        return None
    direct = median(direct_values.values())
    aot = median(aot_values.values())
    paired_ratio = median(
        direct_values[repetition] / aot_values[repetition]
        for repetition in repetitions)
    return {
        "path": str(path),
        "direct_cpu_ns_per_goal_median": direct,
        "aot_cpu_ns_per_goal_median": aot,
        "direct_to_aot_ratio_of_medians": direct / aot,
        "direct_to_aot_paired_ratio_median": paired_ratio,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--warmup-goals", type=int, default=5)
    parser.add_argument("--measured-goals", type=int, default=30)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE)
    parser.add_argument("--output", type=Path)
    return parser


def main() -> int:
    args = _parser().parse_args()
    if min(args.warmup_goals, args.measured_goals, args.repetitions) <= 0:
        raise SystemExit("goal counts and repetitions must be positive")
    if args.timeout <= 0:
        raise SystemExit("--timeout must be positive")
    if os.environ.get("ROS_DISTRO") != ROS_DISTRO:
        raise SystemExit("shared action diagnostic requires ROS_DISTRO=jazzy")

    env = os.environ.copy()
    env.update({
        "RMW_IMPLEMENTATION": RMW,
        "CPPYY_KIT_NO_AUTOPCH": "1",
        "PYTHONUNBUFFERED": "1",
    })
    env.pop("CPPYY_KIT_NO_CACHE", None)
    os.environ["RMW_IMPLEMENTATION"] = RMW
    rows = []
    with tempfile.TemporaryDirectory(
            prefix="rclcppyy-action-shared-diagnostic-") as temporary:
        root = Path(temporary)
        build_directory = root / "aot-build"
        cache_root = root / "cache"
        build_directory.mkdir(mode=0o700)
        cache_root.mkdir(mode=0o700)
        env["XDG_CACHE_HOME"] = str(cache_root)
        executables, _build = _compile_aot(
            build_directory, env, args.timeout)
        with acquire_domain() as lease:
            env["ROS_DOMAIN_ID"] = str(lease.domain_id)
            os.environ["ROS_DOMAIN_ID"] = str(lease.domain_id)
            cache = _prewarm(cache_root, env, args.timeout)
            observer = GraphObserver(
                "action_shared_diagnostic_%s" % uuid.uuid4().hex[:12])
            try:
                for repetition in range(1, args.repetitions + 1):
                    modes = (False, True) if repetition % 2 else (True, False)
                    for shared_values in modes:
                        sample = _run_sample(
                            variant="direct-source-compatible",
                            repetition=repetition,
                            executables=executables,
                            cache=cache,
                            observer=observer,
                            domain_id=lease.domain_id,
                            env=env,
                            timeout=args.timeout,
                            warmup_goals=args.warmup_goals,
                            measured_goals=args.measured_goals,
                            validate=False,
                            shared_values=shared_values,
                        )
                        operations = sample[
                            "server_report"]["cpp_value_operations"]
                        boundary = sample["server_report"]["boundary_evidence"]
                        if any(boundary[key] != 0 for key in (
                                "python_message_conversions",
                                "python_serialization_calls",
                                "adapter_cdr_roundtrips")):
                            raise RuntimeError("shared diagnostic crossed a poisoned boundary")
                        rows.append({
                            "mode": "shared" if shared_values else "copy",
                            "repetition": repetition,
                            "server_cpu_ns_per_goal": sample[
                                "timing"]["server_cpu_ns_per_completed_goal"],
                            "adapter_message_deep_copies": operations[
                                "adapter_message_deep_copies"],
                            "feedback_shared_handoffs": operations.get(
                                "feedback_shared_handoffs", 0),
                            "result_shared_handoffs": operations.get(
                                "result_shared_handoffs", 0),
                        })
            finally:
                observer.close()

    copy_cpu = median(
        row["server_cpu_ns_per_goal"] for row in rows if row["mode"] == "copy")
    shared_cpu = median(
        row["server_cpu_ns_per_goal"] for row in rows if row["mode"] == "shared")
    by_mode_and_repetition = {
        (row["mode"], row["repetition"]): row["server_cpu_ns_per_goal"]
        for row in rows
    }
    paired_ratio = median(
        by_mode_and_repetition[("shared", repetition)] /
        by_mode_and_repetition[("copy", repetition)]
        for repetition in range(1, args.repetitions + 1))
    baseline = _artifact_baseline(args.baseline)
    summary = {
        "copy_cpu_ns_per_goal_median": copy_cpu,
        "shared_cpu_ns_per_goal_median": shared_cpu,
        "shared_to_copy_cpu_ratio": shared_cpu / copy_cpu,
        "cpu_reduction_percent": (copy_cpu - shared_cpu) * 100.0 / copy_cpu,
        "paired_shared_to_copy_cpu_ratio_median": paired_ratio,
        "paired_cpu_reduction_percent_median": (1.0 - paired_ratio) * 100.0,
    }
    if baseline is not None:
        summary.update({
            "corrected_baseline": baseline,
            "diagnostic_copy_to_baseline_aot_ratio": (
                copy_cpu / baseline["aot_cpu_ns_per_goal_median"]),
            "diagnostic_shared_to_baseline_aot_ratio": (
                shared_cpu / baseline["aot_cpu_ns_per_goal_median"]),
        })
    document = {
        "schema": "rclcppyy.action-server-shared-diagnostic/v1",
        "parameters": {
            "warmup_goals": args.warmup_goals,
            "measured_goals": args.measured_goals,
            "repetitions": args.repetitions,
            "rmw": RMW,
            "ros_distro": ROS_DISTRO,
        },
        "samples": rows,
        "summary": summary,
    }
    rendered = json.dumps(document, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    sys.stdout.write(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
