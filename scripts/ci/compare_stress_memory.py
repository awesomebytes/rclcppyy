#!/usr/bin/env python3
"""Compare equal-work stock and compatible stress memory evidence."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import uuid


SCHEMA = "rclcppyy.stress-memory-comparison/v1"
STRESS_SCHEMA = "rclcppyy.runtime-stress/v4"
MATCHED_PARAMETERS = (
    "cycles",
    "threads",
    "messages_per_thread",
    "timeout_s",
    "repetitions",
    "signal_repetitions",
    "min_duration_s",
    "round_period_s",
    "signal_settle_s",
    "seed",
    "max_rss_growth_kib",
)


def _load(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("%s is not a JSON object" % path)
    return value


def _source_identity(document: dict) -> tuple:
    source = document.get("source", {})
    product = source.get("product", {})
    suite = source.get("suite", {})
    if product.get("dirty") is not False or suite.get("dirty") is not False:
        raise ValueError("stress evidence must come from clean product and suite trees")
    if not product.get("commit") or not suite.get("active_commit"):
        raise ValueError("stress evidence is missing exact source commits")
    if suite.get("active_commit") != suite.get("locked_commit"):
        raise ValueError("active suite source does not match the product lock")
    return product["commit"], suite["active_commit"]


def _validate(document: dict, expected_profile: str, minimum_cycles: int) -> float:
    if document.get("schema") != STRESS_SCHEMA:
        raise ValueError("stress evidence does not use %s" % STRESS_SCHEMA)
    if document.get("profile") != expected_profile:
        raise ValueError("expected %s profile evidence" % expected_profile)
    if document.get("performance_claims_allowed") is not False:
        raise ValueError("raw stress evidence must forbid performance claims")
    summary = document.get("summary", {})
    if summary.get("result") != "pass" or document.get("failures") != []:
        raise ValueError("stress evidence must pass before memory comparison")
    parameters = document.get("parameters", {})
    cycles = parameters.get("cycles")
    repetitions = parameters.get("repetitions")
    if not isinstance(cycles, int) or not isinstance(repetitions, int):
        raise ValueError("stress evidence is missing fixed-work counts")
    expected_cycles = cycles * repetitions
    if summary.get("rounds") != repetitions:
        raise ValueError("stress evidence did not complete every fixed-work round")
    if summary.get("entity_cycles") != expected_cycles:
        raise ValueError("stress evidence did not complete every entity cycle")
    memory = document.get("memory", {})
    if memory.get("warmup_rounds_excluded") != 1:
        raise ValueError("exactly one warmup round must be excluded")
    measured_cycles = memory.get("post_warmup_entity_cycles")
    if measured_cycles != cycles * (repetitions - 1):
        raise ValueError("post-warm-up memory evidence has incomplete entity cycles")
    if measured_cycles < minimum_cycles:
        raise ValueError("stress evidence has too few post-warm-up entity cycles")
    slope = memory.get("post_warmup_anonymous_rss_kib_per_1000_cycles")
    if not isinstance(slope, (int, float)):
        raise ValueError("anonymous RSS slope is missing")
    _source_identity(document)
    return float(slope)


def compare(
    reference: dict,
    candidate: dict,
    *,
    max_relative_slope: float,
    max_absolute_delta_kib_per_1000_cycles: float,
    minimum_cycles: int,
) -> dict:
    reference_slope = _validate(reference, "stock", minimum_cycles)
    candidate_slope = _validate(candidate, "compatible", minimum_cycles)
    for field in ("architecture", "python", "rmw_implementation"):
        if reference.get(field) != candidate.get(field):
            raise ValueError("stress evidence differs in %s" % field)
    if _source_identity(reference) != _source_identity(candidate):
        raise ValueError("stress evidence differs in source identity")
    for parameter in MATCHED_PARAMETERS:
        if reference["parameters"].get(parameter) != candidate["parameters"].get(parameter):
            raise ValueError("stress evidence differs in parameter %s" % parameter)
    if reference["parameters"].get("signal_repetitions") != 0:
        raise ValueError("paired memory evidence must disable signal subprocesses")
    if reference["parameters"].get("min_duration_s") != 0:
        raise ValueError("paired memory evidence must use equal fixed work")
    if reference["summary"]["entity_cycles"] != candidate["summary"]["entity_cycles"]:
        raise ValueError("stress evidence differs in completed entity cycles")

    delta = candidate_slope - reference_slope
    ratio = None
    if reference_slope > 0:
        ratio = candidate_slope / reference_slope
    failures = []
    if ratio is not None and ratio > max_relative_slope:
        failures.append(
            "candidate anonymous RSS slope ratio %.6f exceeds %.6f" % (
                ratio, max_relative_slope))
    if delta > max_absolute_delta_kib_per_1000_cycles:
        failures.append(
            "candidate anonymous RSS slope delta %.6f exceeds %.6f KiB/1000 cycles" % (
                delta, max_absolute_delta_kib_per_1000_cycles))
    return {
        "schema": SCHEMA,
        "architecture": reference["architecture"],
        "python": reference["python"],
        "rmw_implementation": reference["rmw_implementation"],
        "source": reference["source"],
        "entity_cycles": reference["summary"]["entity_cycles"],
        "post_warmup_entity_cycles": (
            reference["memory"]["post_warmup_entity_cycles"]),
        "reference": {
            "profile": "stock",
            "anonymous_rss_kib_per_1000_cycles": reference_slope,
        },
        "candidate": {
            "profile": "compatible",
            "anonymous_rss_kib_per_1000_cycles": candidate_slope,
        },
        "comparison": {
            "relative_slope": ratio,
            "absolute_delta_kib_per_1000_cycles": delta,
            "max_relative_slope": max_relative_slope,
            "max_absolute_delta_kib_per_1000_cycles": (
                max_absolute_delta_kib_per_1000_cycles),
        },
        "result": "fail" if failures else "pass",
        "failures": failures,
        "performance_claims_allowed": False,
    }


def _write(path: Path, document: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(".%s.%s.tmp" % (path.name, uuid.uuid4().hex))
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-relative-slope", type=float, default=1.5)
    parser.add_argument(
        "--max-absolute-delta-kib-per-1000-cycles",
        type=float,
        default=2048.0,
    )
    parser.add_argument("--minimum-cycles", type=int, default=1000)
    args = parser.parse_args(argv)
    if min(
        args.max_relative_slope,
        args.max_absolute_delta_kib_per_1000_cycles,
        args.minimum_cycles,
    ) <= 0:
        parser.error("comparison budgets and minimum cycles must be positive")
    try:
        result = compare(
            _load(args.reference),
            _load(args.candidate),
            max_relative_slope=args.max_relative_slope,
            max_absolute_delta_kib_per_1000_cycles=(
                args.max_absolute_delta_kib_per_1000_cycles),
            minimum_cycles=args.minimum_cycles,
        )
    except (KeyError, TypeError, ValueError) as exception:
        print("STRESS_MEMORY_INVALID: %s" % exception, file=sys.stderr)
        return 2
    _write(args.output, result)
    print(
        "STRESS_MEMORY_%s reference=%.6f candidate=%.6f" % (
            result["result"].upper(),
            result["reference"]["anonymous_rss_kib_per_1000_cycles"],
            result["candidate"]["anonymous_rss_kib_per_1000_cycles"],
        )
    )
    return 1 if result["failures"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
