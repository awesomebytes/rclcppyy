#!/usr/bin/env python3
"""Validate repeated stock/compatibility runs without making speed claims."""

from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
import statistics
import sys

from _benchmark_matrix import WORKLOADS
from _result_schema import validate_document


SCHEMA_ID = "rclcppyy.compatibility-performance-evidence/v1"
BACKEND_SCHEMA = "rclcppyy.benchmark-backend/v1"
WIRE_SCHEMA = "rclcppyy.benchmark-wire-values/v1"
MINIMUM_REPETITIONS = 5
REFERENCE_BACKEND = "rclpy"
CANDIDATE_BACKEND = "rclcppyy"
PAIR_FIELDS = ("workload", "target_rate_hz", "payload_bytes")
METRICS = {
    "messages.effective_rate_hz": ("higher", ("messages", "effective_rate_hz")),
    "latency_us.p50": ("lower", ("latency_us", "p50")),
    "latency_us.p95": ("lower", ("latency_us", "p95")),
    "latency_us.p99": ("lower", ("latency_us", "p99")),
    "cpu_pct.publisher.mean": ("lower", ("cpu_pct", "publisher", "mean")),
    "cpu_pct.subscriber.mean": ("lower", ("cpu_pct", "subscriber", "mean")),
}


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _finite_number(value, label):
    _require(
        isinstance(value, (int, float)) and not isinstance(value, bool),
        f"{label} must be numeric",
    )
    _require(math.isfinite(value) and value >= 0, f"{label} must be finite and non-negative")
    return value


def _nested(document, path, label):
    value = document
    for field in path:
        _require(isinstance(value, dict) and field in value, f"{label} is missing")
        value = value[field]
    return value


def _metric_value(row, metric):
    _preference, path = METRICS[metric]
    return _finite_number(_nested(row, path, metric), metric)


def _normalized_matrix(document):
    matrix = copy.deepcopy(document["benchmark"]["matrix"])
    matrix.pop("run_token", None)
    matrix.pop("ros_domain_id", None)
    return matrix


def _stable_environment(document):
    environment = copy.deepcopy(document["environment"])
    environment.get("source", {}).pop("repository", None)
    environment.get("ros", {}).pop("domain_id", None)
    return environment


def _validate_marker(row, role, expected_backend, label):
    expected = row.get("expected_backends")
    _require(isinstance(expected, dict), f"{label} expected_backends is required")
    _require(expected.get(role) == expected_backend, f"{label} expected {role} backend mismatch")
    marker = row.get(f"{role}_backend")
    _require(isinstance(marker, dict), f"{label} {role} backend marker is required")
    _require(marker.get("schema") == BACKEND_SCHEMA, f"{label} invalid {role} marker schema")
    _require(marker.get("role") == role, f"{label} invalid {role} marker role")
    _require(marker.get("backend") == expected_backend, f"{label} observed {role} backend mismatch")
    _require(
        isinstance(marker.get("evidence"), str) and marker["evidence"],
        f"{label} {role} marker requires evidence",
    )
    _require(isinstance(marker.get("metadata"), dict), f"{label} {role} metadata is required")


def _validate_wire_values(row, label):
    wire = row.get("wire_values")
    _require(isinstance(wire, dict), f"{label} wire-value evidence is required")
    _require(wire.get("schema") == WIRE_SCHEMA, f"{label} invalid wire-value schema")
    expected_contract = WORKLOADS[row["workload"]]["wire_contract"]
    _require(wire.get("contract_id") == expected_contract, f"{label} wire contract mismatch")
    _require(
        wire.get("expected_payload_bytes") == row["payload_bytes"],
        f"{label} payload contract mismatch",
    )
    received = row.get("messages", {}).get("received")
    _require(isinstance(received, int) and received > 0, f"{label} received no messages")
    _require(wire.get("checked_messages") == received, f"{label} wire count mismatch")
    _require(wire.get("violations") == 0, f"{label} contains wire-value violations")
    _require(wire.get("violation_types") == {}, f"{label} contains wire violation types")
    _require(wire.get("value_contract_verified") is True, f"{label} wire values are unverified")


def _validate_document(document, repetition):
    validate_document(document)
    label = f"repetition {repetition}"
    _require(document["benchmark"]["mode"] == "measurement", f"{label} must use measurement mode")
    _require(not document["failures"], f"{label} contains benchmark failures")
    source = document["environment"].get("source")
    _require(isinstance(source, dict), f"{label} source metadata is required")
    _require(isinstance(source.get("commit"), str) and source["commit"], f"{label} commit is required")
    _require(source.get("dirty") is False, f"{label} source must be clean")
    source_dependencies = document["environment"].get("source_dependencies")
    _require(isinstance(source_dependencies, dict), f"{label} source dependencies are required")
    for module_name, dependency in source_dependencies.items():
        if dependency is None or dependency.get("is_benchmark_repository") is True:
            continue
        _require(
            dependency.get("dirty") is False,
            f"{label} source dependency {module_name} must be clean",
        )

    matrix = document["benchmark"]["matrix"]
    run_token = matrix.get("run_token")
    _require(isinstance(run_token, str) and run_token, f"{label} run token is required")
    _require(set(matrix.get("workloads", ())) == set(WORKLOADS), f"{label} must cover every workload")
    _require(
        {REFERENCE_BACKEND, CANDIDATE_BACKEND}.issubset(matrix.get("backends", ())),
        f"{label} must contain stock and compatibility backends",
    )
    rates = matrix.get("target_rates_hz")
    payloads = matrix.get("payload_bytes")
    _require(isinstance(rates, list) and rates, f"{label} target rates are required")
    _require(isinstance(payloads, list) and payloads, f"{label} payload sizes are required")

    rows = {}
    for row in document["results"]:
        if row.get("backend") not in (REFERENCE_BACKEND, CANDIDATE_BACKEND):
            continue
        key = tuple(row.get(field) for field in PAIR_FIELDS)
        row_key = (row["backend"],) + key
        _require(row_key not in rows, f"{label} duplicate compatibility case {row_key}")
        _require(row.get("workload") in WORKLOADS, f"{label} unknown workload")
        expected_publisher = "python" if row["backend"] == REFERENCE_BACKEND else "cpp"
        _validate_marker(row, "publisher", expected_publisher, label)
        _validate_marker(row, "subscriber", "python", label)
        _validate_wire_values(row, label)
        for metric in METRICS:
            _metric_value(row, metric)
        rows[row_key] = row

    dimensions = {
        (workload, rate, payload)
        for workload in WORKLOADS
        for rate in rates
        for payload in payloads
    }
    for dimensions_key in dimensions:
        for backend in (REFERENCE_BACKEND, CANDIDATE_BACKEND):
            _require(
                (backend,) + dimensions_key in rows,
                f"{label} is missing {backend} case {dimensions_key}",
            )
    _require(len(rows) == len(dimensions) * 2, f"{label} has an unexpected compatibility case set")
    return run_token, rows, sorted(dimensions)


def _direction(candidate, reference, preference):
    if candidate == reference:
        return "tie"
    candidate_better = candidate > reference if preference == "higher" else candidate < reference
    return "candidate_better" if candidate_better else "stock_better"


def _consistency(counts, repetitions):
    if counts["candidate_better"] == repetitions:
        return "candidate_better_in_all_repetitions"
    if counts["stock_better"] == repetitions:
        return "stock_better_in_all_repetitions"
    if counts["tie"] == repetitions:
        return "equal_in_all_repetitions"
    return "mixed_observation"


def analyze_documents(documents, minimum_repetitions=MINIMUM_REPETITIONS):
    """Build deterministic compatibility evidence from controlled repetitions."""
    _require(
        isinstance(minimum_repetitions, int) and minimum_repetitions >= 2,
        "minimum repetitions must be at least two",
    )
    _require(len(documents) >= minimum_repetitions, "not enough benchmark repetitions")

    validated = []
    for repetition, document in enumerate(documents, 1):
        run_token, rows, dimensions = _validate_document(document, repetition)
        validated.append((run_token, document, rows, dimensions))
    first = validated[0]
    for _token, document, rows, dimensions in validated[1:]:
        _require(_normalized_matrix(document) == _normalized_matrix(first[1]), "benchmark matrix changed")
        _require(_stable_environment(document) == _stable_environment(first[1]), "machine/runtime environment changed")
        _require(set(rows) == set(first[2]), "compatibility case matrix changed")
        _require(dimensions == first[3], "comparison dimensions changed")
    tokens = [item[0] for item in validated]
    _require(len(set(tokens)) == len(tokens), "repetitions must use unique run tokens")

    ordered = sorted(validated, key=lambda item: item[0])
    comparisons = []
    for dimension in first[3]:
        metric_results = {}
        for metric, (preference, _path) in sorted(METRICS.items()):
            observations = []
            counts = {"candidate_better": 0, "stock_better": 0, "tie": 0}
            candidate_values = []
            reference_values = []
            for token, _document, rows, _dimensions in ordered:
                candidate = _metric_value(rows[(CANDIDATE_BACKEND,) + dimension], metric)
                reference = _metric_value(rows[(REFERENCE_BACKEND,) + dimension], metric)
                direction = _direction(candidate, reference, preference)
                counts[direction] += 1
                candidate_values.append(candidate)
                reference_values.append(reference)
                observations.append({
                    "run_token": token,
                    "candidate": candidate,
                    "stock": reference,
                    "direction": direction,
                })
            metric_results[metric] = {
                "preferred_direction": preference,
                "observations": observations,
                "direction_counts": counts,
                "consistency": _consistency(counts, len(ordered)),
                "candidate_median": statistics.median(candidate_values),
                "stock_median": statistics.median(reference_values),
            }
        comparisons.append({
            "dimensions": dict(zip(PAIR_FIELDS, dimension)),
            "wire_result_parity": True,
            "metrics": metric_results,
        })

    result = {
        "schema": SCHEMA_ID,
        "decision": "evidence_complete",
        "performance_claims_allowed": False,
        "interpretation_allowed": False,
        "controls": {
            "minimum_repetitions": minimum_repetitions,
            "observed_repetitions": len(ordered),
            "unique_run_tokens": [item[0] for item in ordered],
            "source_clean": True,
            "source_dependencies_clean": True,
            "stable_machine_runtime_environment": True,
            "stable_matrix": True,
        },
        "evidence": {
            "benchmark_schema": first[1]["schema"],
            "benchmark_name": first[1]["benchmark"]["name"],
            "environment": _stable_environment(first[1]),
            "matrix": _normalized_matrix(first[1]),
        },
        "routes": [{
            "path_id": "compatibility.publisher.same_handle_publish",
            "operation": "rclpy.publisher.Publisher.publish",
            "candidate_backend": CANDIDATE_BACKEND,
            "reference_backend": REFERENCE_BACKEND,
            "transparent": True,
            "candidate_publisher_backend": "cpp",
            "candidate_subscriber_backend": "python",
            "backend_markers_verified": True,
            "wire_result_parity": True,
            "workloads": sorted(WORKLOADS),
            "advertised_performance_benefit": False,
            "performance_conclusion": "not_established",
            "reason": (
                "Direction counts are raw repeated observations; no reviewed noise "
                "threshold or architecture-specific benefit policy was applied."
            ),
        }],
        "comparisons": comparisons,
    }
    validate_evidence(result)
    return result


def validate_evidence(document):
    _require(document.get("schema") == SCHEMA_ID, "unsupported compatibility evidence schema")
    _require(document.get("decision") == "evidence_complete", "compatibility evidence is incomplete")
    _require(document.get("performance_claims_allowed") is False, "performance claims are forbidden")
    _require(document.get("interpretation_allowed") is False, "performance interpretation is forbidden")
    _require(isinstance(document.get("controls"), dict), "evidence controls are required")
    _require(isinstance(document.get("evidence"), dict), "source evidence is required")
    _require(isinstance(document.get("comparisons"), list) and document["comparisons"], "comparisons are required")
    routes = document.get("routes")
    _require(isinstance(routes, list) and len(routes) == 1, "one compatibility route is required")
    route = routes[0]
    _require(route.get("backend_markers_verified") is True, "route backend markers are unverified")
    _require(route.get("wire_result_parity") is True, "route wire-result parity is unverified")
    _require(route.get("advertised_performance_benefit") is False, "unreviewed benefit advertisement")
    _require(route.get("performance_conclusion") == "not_established", "unreviewed performance conclusion")


def _write(document, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Gate repeated stock/compatibility benchmark evidence without speed claims.")
    parser.add_argument("inputs", nargs="+", type=Path, help="benchmark-v3 JSON repetitions")
    parser.add_argument("--minimum-repetitions", type=int, default=MINIMUM_REPETITIONS)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--json", action="store_true", help="also emit the evidence JSON on stdout")
    args = parser.parse_args(argv)
    try:
        documents = [json.loads(path.read_text(encoding="utf-8")) for path in args.inputs]
        result = analyze_documents(documents, args.minimum_repetitions)
        _write(result, args.output)
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        print(f"compatibility evidence gate: {exc}", file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    else:
        print("compatibility evidence gate: evidence_complete")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
