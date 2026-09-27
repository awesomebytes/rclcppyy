#!/usr/bin/env python3
"""Gate repeated benchmark matrices against a reviewed machine budget."""

from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
import statistics
import sys

from _result_schema import validate_document


BUDGET_SCHEMA = "rclcppyy.benchmark-regression-budget/v1"
RESULT_SCHEMA = "rclcppyy.benchmark-regression/v1"
BACKEND_SCHEMA = "rclcppyy.benchmark-backend/v1"
CALIBRATION_EXIT = 2

METRICS = {
    "messages.effective_rate_hz": ("min_ratio", ("messages", "effective_rate_hz")),
    "latency_us.p50": ("max_ratio", ("latency_us", "p50")),
    "latency_us.p95": ("max_ratio", ("latency_us", "p95")),
    "latency_us.p99": ("max_ratio", ("latency_us", "p99")),
    "cpu_pct.publisher.mean": ("max_ratio", ("cpu_pct", "publisher", "mean")),
    "cpu_pct.subscriber.mean": ("max_ratio", ("cpu_pct", "subscriber", "mean")),
}
CASE_FIELDS = ("backend", "workload", "target_rate_hz", "payload_bytes")
PAIR_FIELDS = ("workload", "target_rate_hz", "payload_bytes")
MATRIX_RUN_FIELDS = ("run_token", "ros_domain_id")


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _finite_number(value, label, *, positive=False):
    _require(
        isinstance(value, (int, float)) and not isinstance(value, bool),
        f"{label} must be numeric",
    )
    _require(math.isfinite(value), f"{label} must be finite")
    if positive:
        _require(value > 0, f"{label} must be positive")
    else:
        _require(value >= 0, f"{label} must be non-negative")
    return value


def _nested(document, path, label):
    value = document
    for field in path:
        _require(isinstance(value, dict) and field in value, f"{label} is missing")
        value = value[field]
    return value


def _metric_value(row, metric):
    _direction, path = METRICS[metric]
    return _finite_number(_nested(row, path, metric), metric)


def _validate_marker(row, role):
    marker = row.get(f"{role}_backend")
    expected = row.get("expected_backends")
    _require(isinstance(expected, dict), "expected_backends must be an object")
    _require(expected.get(role) in ("python", "cpp"), f"invalid expected {role} backend")
    _require(isinstance(marker, dict), f"{role}_backend evidence is required")
    _require(marker.get("schema") == BACKEND_SCHEMA, f"invalid {role} backend schema")
    _require(marker.get("role") == role, f"invalid {role} backend role")
    _require(marker.get("backend") in ("python", "cpp"), f"invalid {role} backend value")
    _require(marker.get("backend") == expected[role], f"{role} backend evidence mismatch")
    _require(isinstance(marker.get("evidence"), str) and marker["evidence"],
             f"{role} backend evidence kind is required")
    _require(isinstance(marker.get("metadata"), dict),
             f"{role} backend metadata must be an object")


def validate_budget(budget):
    """Validate the versioned budget without requiring a JSON Schema runtime."""
    _require(isinstance(budget, dict), "budget must be an object")
    _require(budget.get("schema") == BUDGET_SCHEMA, "unsupported regression budget schema")
    _require(budget.get("architecture") in ("x86_64", "aarch64"),
             "budget architecture must be x86_64 or aarch64")
    _require(budget.get("status") in ("calibration_required", "reviewed"),
             "budget status must be calibration_required or reviewed")
    repetitions = budget.get("minimum_repetitions")
    _require(isinstance(repetitions, int) and not isinstance(repetitions, bool)
             and repetitions >= 3,
             "minimum_repetitions must be an integer of at least three")
    _require(repetitions % 2 == 1, "minimum_repetitions must be odd")

    machine = budget.get("machine")
    _require(isinstance(machine, dict), "budget machine must be an object")
    _require("cpu_model" in machine and "logical_cpu_count" in machine,
             "budget machine identity is incomplete")
    environment = budget.get("environment")
    _require(isinstance(environment, dict), "budget environment must be an object")
    _require(isinstance(environment.get("ros_distribution"), str)
             and environment["ros_distribution"], "ROS distribution is required")
    _require(isinstance(environment.get("rmw_implementation"), str)
             and environment["rmw_implementation"], "RMW implementation is required")

    comparisons = budget.get("comparisons")
    _require(isinstance(comparisons, list), "budget comparisons must be an array")
    if budget["status"] == "calibration_required":
        _require(not comparisons, "uncalibrated budgets cannot contain thresholds")
        _require(machine["cpu_model"] is None and machine["logical_cpu_count"] is None,
                 "uncalibrated budgets cannot pin an unreviewed machine")
        return

    _require(isinstance(budget.get("reviewed_at"), str) and budget["reviewed_at"],
             "reviewed_at is required for a reviewed budget")
    _require(isinstance(budget.get("reviewed_by"), str) and budget["reviewed_by"],
             "reviewed_by is required for a reviewed budget")
    _require(isinstance(machine["cpu_model"], str) and machine["cpu_model"],
             "reviewed budget requires an exact CPU model")
    _require(isinstance(machine["logical_cpu_count"], int)
             and not isinstance(machine["logical_cpu_count"], bool)
             and machine["logical_cpu_count"] > 0,
             "reviewed budget requires a logical CPU count")
    _require(bool(comparisons), "reviewed budget requires comparisons")

    comparison_ids = set()
    for comparison in comparisons:
        _require(isinstance(comparison, dict), "comparison must be an object")
        comparison_id = comparison.get("id")
        _require(isinstance(comparison_id, str) and comparison_id,
                 "comparison id is required")
        _require(comparison_id not in comparison_ids, f"duplicate comparison id: {comparison_id}")
        comparison_ids.add(comparison_id)
        candidate = comparison.get("candidate_backend")
        reference = comparison.get("reference_backend")
        _require(isinstance(candidate, str) and candidate, "candidate_backend is required")
        _require(isinstance(reference, str) and reference, "reference_backend is required")
        _require(candidate != reference, "candidate and reference backends must differ")
        selectors = comparison.get("selectors")
        _require(isinstance(selectors, dict), "comparison selectors must be an object")
        for field in ("workloads", "target_rates_hz", "payload_bytes"):
            values = selectors.get(field)
            _require(isinstance(values, list) and values, f"selector {field} must be non-empty")
            _require(len(set(values)) == len(values), f"selector {field} contains duplicates")
        _require(all(isinstance(value, str) and value for value in selectors["workloads"]),
                 "workload selectors must be non-empty strings")
        _require(all(isinstance(value, int) and not isinstance(value, bool) and value > 0
                     for value in selectors["target_rates_hz"]),
                 "target rate selectors must be positive integers")
        _require(all(isinstance(value, int) and not isinstance(value, bool) and value >= 0
                     for value in selectors["payload_bytes"]),
                 "payload byte selectors must be non-negative integers")
        metrics = comparison.get("metrics")
        _require(isinstance(metrics, dict) and metrics, "comparison metrics must be non-empty")
        for metric, threshold in metrics.items():
            _require(metric in METRICS, f"unsupported regression metric: {metric}")
            _require(isinstance(threshold, dict), f"threshold for {metric} must be an object")
            direction, _path = METRICS[metric]
            _require(set(threshold) == {direction},
                     f"{metric} requires exactly {direction}")
            _finite_number(threshold[direction], f"{metric}.{direction}", positive=True)


def _normalized_matrix(document):
    matrix = copy.deepcopy(document["benchmark"]["matrix"])
    for field in MATRIX_RUN_FIELDS:
        matrix.pop(field, None)
    return matrix


def _stable_environment(document):
    environment = copy.deepcopy(document["environment"])
    environment.get("ros", {}).pop("domain_id", None)
    environment.get("source", {}).pop("repository", None)
    return environment


def _case_key(row):
    return tuple(row[field] for field in CASE_FIELDS)


def _pair_key(row):
    return tuple(row[field] for field in PAIR_FIELDS)


def _validate_document_evidence(document, index):
    validate_document(document)
    label = f"repetition {index}"
    _require(document["benchmark"]["mode"] == "measurement",
             f"{label} must use measurement mode")
    _require(not document["failures"], f"{label} contains benchmark failures")
    _require(bool(document["results"]), f"{label} has no benchmark results")
    source = document["environment"].get("source")
    _require(isinstance(source, dict), f"{label} source metadata is required")
    _require(isinstance(source.get("commit"), str) and source["commit"],
             f"{label} source commit is required")
    _require(source.get("dirty") is False, f"{label} source must be clean")
    matrix = document["benchmark"]["matrix"]
    _require(isinstance(matrix.get("run_token"), str) and matrix["run_token"],
             f"{label} run token is required")

    rows = {}
    for row in document["results"]:
        for field in CASE_FIELDS:
            _require(field in row, f"{label} result is missing {field}")
        _require(isinstance(row["backend"], str) and row["backend"],
                 f"{label} result backend must be a non-empty string")
        _require(isinstance(row["workload"], str) and row["workload"],
                 f"{label} result workload must be a non-empty string")
        _require(isinstance(row["target_rate_hz"], int)
                 and not isinstance(row["target_rate_hz"], bool)
                 and row["target_rate_hz"] > 0,
                 f"{label} result target rate must be a positive integer")
        _require(isinstance(row["payload_bytes"], int)
                 and not isinstance(row["payload_bytes"], bool)
                 and row["payload_bytes"] >= 0,
                 f"{label} result payload bytes must be a non-negative integer")
        key = _case_key(row)
        _require(key not in rows, f"{label} contains duplicate case dimensions: {key}")
        _validate_marker(row, "publisher")
        _validate_marker(row, "subscriber")
        for metric in METRICS:
            _metric_value(row, metric)
        rows[key] = row
    return rows


def _select_pairs(rows, comparison):
    selectors = comparison["selectors"]
    selected = {}
    for row in rows.values():
        if row["backend"] not in (
                comparison["candidate_backend"], comparison["reference_backend"]):
            continue
        if row["workload"] not in selectors["workloads"]:
            continue
        if row["target_rate_hz"] not in selectors["target_rates_hz"]:
            continue
        if row["payload_bytes"] not in selectors["payload_bytes"]:
            continue
        selected[(row["backend"],) + _pair_key(row)] = row

    expected_dimensions = {
        (workload, rate, payload)
        for workload in selectors["workloads"]
        for rate in selectors["target_rates_hz"]
        for payload in selectors["payload_bytes"]
    }
    for dimensions in expected_dimensions:
        for backend in (comparison["candidate_backend"], comparison["reference_backend"]):
            _require((backend,) + dimensions in selected,
                     f"comparison {comparison['id']} is missing {backend} case {dimensions}")
    _require(len(selected) == 2 * len(expected_dimensions),
             f"comparison {comparison['id']} selected an unexpected case set")
    return selected, sorted(expected_dimensions)


def compare_documents(documents, budget):
    """Return one deterministic regression artifact from repeated raw runs."""
    validate_budget(budget)
    _require(len(documents) >= budget["minimum_repetitions"],
             "not enough repetitions for the selected budget")
    validated = []
    for index, document in enumerate(documents, 1):
        validated.append((document, _validate_document_evidence(document, index)))

    first = validated[0][0]
    first_rows = validated[0][1]
    run_tokens = []
    for index, (document, rows) in enumerate(validated, 1):
        _require(document["benchmark"]["name"] == first["benchmark"]["name"],
                 f"repetition {index} benchmark name mismatch")
        _require(_normalized_matrix(document) == _normalized_matrix(first),
                 f"repetition {index} benchmark matrix mismatch")
        _require(_stable_environment(document) == _stable_environment(first),
                 f"repetition {index} machine/runtime environment mismatch")
        _require(set(rows) == set(first_rows), f"repetition {index} case matrix mismatch")
        run_tokens.append(document["benchmark"]["matrix"]["run_token"])
    _require(len(set(run_tokens)) == len(run_tokens), "repetitions must have unique run tokens")

    host = first["environment"].get("host", {})
    ros = first["environment"].get("ros", {})
    _require(host.get("architecture") == budget["architecture"],
             "budget architecture does not match benchmark host")
    _require(ros.get("distribution") == budget["environment"]["ros_distribution"],
             "budget ROS distribution does not match benchmark environment")
    _require(ros.get("rmw_implementation") == budget["environment"]["rmw_implementation"],
             "budget RMW implementation does not match benchmark environment")
    if budget["status"] == "reviewed":
        _require(host.get("cpu_model") == budget["machine"]["cpu_model"],
                 "reviewed budget CPU model does not match benchmark host")
        _require(host.get("logical_cpu_count") == budget["machine"]["logical_cpu_count"],
                 "reviewed budget CPU count does not match benchmark host")

    ordered = sorted(validated, key=lambda item: item[0]["benchmark"]["matrix"]["run_token"])
    aggregates = []
    for key in sorted(first_rows):
        values = {metric: [] for metric in METRICS}
        for _document, rows in ordered:
            for metric in METRICS:
                values[metric].append(_metric_value(rows[key], metric))
        aggregates.append({
            "case": dict(zip(CASE_FIELDS, key)),
            "metrics": {
                metric: {"values": values[metric], "median": statistics.median(values[metric])}
                for metric in sorted(values)
            },
        })

    comparison_results = []
    violations = []
    if budget["status"] == "reviewed":
        for comparison in sorted(budget["comparisons"], key=lambda item: item["id"]):
            selected_runs = []
            dimensions = None
            for _document, rows in ordered:
                selected, selected_dimensions = _select_pairs(rows, comparison)
                dimensions = selected_dimensions if dimensions is None else dimensions
                selected_runs.append(selected)

            metric_results = {}
            for metric in sorted(comparison["metrics"]):
                direction, _path = METRICS[metric]
                limit = comparison["metrics"][metric][direction]
                case_results = []
                for dimension in dimensions:
                    paired_ratios = []
                    for selected in selected_runs:
                        candidate = selected[(comparison["candidate_backend"],) + dimension]
                        reference = selected[(comparison["reference_backend"],) + dimension]
                        reference_value = _metric_value(reference, metric)
                        _require(reference_value > 0,
                                 f"comparison {comparison['id']} has zero reference {metric}")
                        paired_ratios.append(
                            _metric_value(candidate, metric) / reference_value)
                    median_ratio = statistics.median(paired_ratios)
                    passed = (
                        median_ratio <= limit if direction == "max_ratio"
                        else median_ratio >= limit
                    )
                    case_result = {
                        "dimensions": dict(zip(PAIR_FIELDS, dimension)),
                        "paired_ratios": paired_ratios,
                        "median_ratio": median_ratio,
                        "passed": passed,
                    }
                    case_results.append(case_result)
                    if not passed:
                        violations.append({
                            "comparison_id": comparison["id"],
                            "metric": metric,
                            "dimensions": case_result["dimensions"],
                            "direction": direction,
                            "limit": limit,
                            "median_ratio": median_ratio,
                        })
                metric_results[metric] = {
                    "direction": direction,
                    "limit": limit,
                    "cases": case_results,
                    "passed": all(case["passed"] for case in case_results),
                }
            comparison_results.append({
                "id": comparison["id"],
                "candidate_backend": comparison["candidate_backend"],
                "reference_backend": comparison["reference_backend"],
                "selectors": copy.deepcopy(comparison["selectors"]),
                "metrics": metric_results,
            })

    decision = "calibration_required"
    if budget["status"] == "reviewed":
        decision = "fail" if violations else "pass"
    source = first["environment"]["source"]
    return {
        "schema": RESULT_SCHEMA,
        "decision": decision,
        "performance_claims_allowed": False,
        "budget": {
            "schema": budget["schema"],
            "architecture": budget["architecture"],
            "status": budget["status"],
            "reviewed_at": budget.get("reviewed_at"),
            "reviewed_by": budget.get("reviewed_by"),
        },
        "evidence": {
            "repetitions": len(documents),
            "benchmark_name": first["benchmark"]["name"],
            "source_commit": source["commit"],
            "architecture": host.get("architecture"),
            "cpu_model": host.get("cpu_model"),
            "logical_cpu_count": host.get("logical_cpu_count"),
            "ros_distribution": ros.get("distribution"),
            "rmw_implementation": ros.get("rmw_implementation"),
            "run_tokens": sorted(run_tokens),
            "matrix": _normalized_matrix(first),
        },
        "aggregates": aggregates,
        "comparisons": comparison_results,
        "violations": violations,
    }


def dumps(document):
    return json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n"


def write(document, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(dumps(document), encoding="utf-8")
    temporary.replace(path)


def _parser():
    parser = argparse.ArgumentParser(
        description="Compare repeated benchmark matrices with a reviewed machine budget.")
    parser.add_argument("inputs", nargs="+", type=Path, help="raw benchmark v2 JSON files")
    parser.add_argument("--budget", required=True, type=Path, help="architecture budget JSON")
    parser.add_argument("--output", required=True, type=Path, help="regression evidence JSON")
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    try:
        budget = json.loads(args.budget.read_text(encoding="utf-8"))
        documents = [json.loads(path.read_text(encoding="utf-8")) for path in args.inputs]
        result = compare_documents(documents, budget)
        write(result, args.output)
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        print(f"benchmark regression gate: {exc}", file=sys.stderr)
        return 1
    print(f"benchmark regression decision: {result['decision']}")
    if result["decision"] == "calibration_required":
        return CALIBRATION_EXIT
    return 1 if result["decision"] == "fail" else 0


if __name__ == "__main__":
    raise SystemExit(main())
