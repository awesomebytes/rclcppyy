#!/usr/bin/env python3
"""Validate and summarize a compatibility manifest without extra dependencies."""

from __future__ import annotations

import argparse
import collections
import datetime
import json
import sys
from pathlib import Path


SCHEMA_ID = "rclcppyy.compatibility/v2"
UPSTREAM_SCHEMA_VERSION = 1
SUPPORT = {
    "certified", "stock_authoritative", "experimental", "unsupported",
    "unassessed",
}
BACKENDS = {"cpp", "python", "mixed", "none"}
FALLBACKS = {"none", "process_only", "safe", "unsafe", "not_applicable"}
AREAS = {
    "activation", "node", "message", "publisher", "subscription", "timer",
    "executor", "service", "action", "parameter", "lifecycle", "context",
    "serialization", "logging", "qos", "time", "graph", "utility",
    "type_support",
}
ENTRY_KEYS = {
    "id", "area", "api", "support", "backend", "fallback", "default",
    "evidence", "notes",
}
UPSTREAM_KEYS = {"manifest", "coverage"}
COVERAGE_KEYS = {"path", "disposition", "entry_id"}
COVERAGE_DISPOSITIONS = {"selected", "reviewed_exclusion"}
MANIFEST_KEYS = {
    "schema", "ros_distribution", "reviewed_at", "upstream_contract", "entries",
}


class ManifestError(ValueError):
    pass


def _require(condition, message):
    if not condition:
        raise ManifestError(message)


def _load_json(path: Path, label: str) -> dict:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ManifestError(f"cannot read {label} {path}: {exc}") from exc
    _require(isinstance(document, dict), f"{label} must be an object")
    return document


def _safe_repo_path(repo_root: Path, value: object, label: str) -> Path:
    _require(isinstance(value, str) and value, f"{label} is required")
    relative = Path(value)
    _require(
        not relative.is_absolute() and ".." not in relative.parts,
        f"{label} must be a safe relative path",
    )
    return repo_root / relative


def _contract_partition(document: dict) -> tuple[set[str], set[str]]:
    _require(
        document.get("schema_version") == UPSTREAM_SCHEMA_VERSION,
        "unsupported upstream contract schema_version",
    )
    selection = document.get("selection")
    exclusions = document.get("reviewed_exclusions")
    _require(isinstance(selection, list), "upstream contract selection must be a list")
    _require(
        isinstance(exclusions, list),
        "upstream contract reviewed_exclusions must be a list",
    )
    selected = [entry.get("path") for entry in selection if isinstance(entry, dict)]
    excluded = [
        path
        for group in exclusions if isinstance(group, dict)
        for path in group.get("paths", []) if isinstance(path, str)
    ]
    _require(
        len(selected) == len(selection) and all(isinstance(path, str) for path in selected),
        "upstream contract has an invalid selection entry",
    )
    _require(len(selected) == len(set(selected)), "upstream contract selection has duplicates")
    _require(len(excluded) == len(set(excluded)), "upstream contract exclusions have duplicates")
    _require(
        not set(selected) & set(excluded),
        "upstream contract selected and excluded paths overlap",
    )
    return set(selected), set(excluded)


def _validate_upstream_coverage(
    manifest: dict,
    repo_root: Path,
    entries_by_id: dict[str, dict],
) -> dict:
    upstream = manifest.get("upstream_contract")
    _require(isinstance(upstream, dict), "upstream_contract must be an object")
    _require(set(upstream) == UPSTREAM_KEYS, "upstream_contract fields differ from v2 schema")
    contract_path = _safe_repo_path(
        repo_root, upstream["manifest"], "upstream_contract.manifest")
    selected, excluded = _contract_partition(
        _load_json(contract_path, "upstream contract"))

    coverage = upstream.get("coverage")
    _require(isinstance(coverage, list), "upstream_contract.coverage must be a list")
    seen_paths = set()
    mapped = {disposition: set() for disposition in COVERAGE_DISPOSITIONS}
    mapped_entry_ids = {disposition: set() for disposition in COVERAGE_DISPOSITIONS}
    for index, mapping in enumerate(coverage):
        prefix = f"upstream_contract.coverage[{index}]"
        _require(isinstance(mapping, dict), f"{prefix} must be an object")
        _require(set(mapping) == COVERAGE_KEYS, f"{prefix} fields differ from v2 schema")
        path = mapping["path"]
        _require(isinstance(path, str) and path, f"{prefix}.path is required")
        _require(path not in seen_paths, f"duplicate upstream coverage path: {path}")
        seen_paths.add(path)
        disposition = mapping["disposition"]
        _require(
            disposition in COVERAGE_DISPOSITIONS,
            f"{path}: unknown upstream coverage disposition",
        )
        entry_id = mapping["entry_id"]
        _require(entry_id in entries_by_id, f"{path}: unknown compatibility entry {entry_id}")
        entry = entries_by_id[entry_id]
        if disposition == "selected":
            _require(path in selected, f"unknown selected upstream coverage path: {path}")
            _require(
                entry["support"] not in {"unsupported", "unassessed"},
                f"{path}: selected coverage maps to {entry['support']} behavior",
            )
        else:
            _require(path in excluded, f"unknown reviewed exclusion coverage path: {path}")
            _require(
                entry["support"] == "unassessed",
                f"{path}: reviewed exclusion must map to unassessed behavior",
            )
        mapped[disposition].add(path)
        mapped_entry_ids[disposition].add(entry_id)

    for disposition, expected in (
        ("selected", selected),
        ("reviewed_exclusion", excluded),
    ):
        missing = sorted(expected - mapped[disposition])
        _require(
            not missing,
            f"missing {disposition} upstream coverage: {', '.join(missing)}",
        )
    for entry_id, entry in entries_by_id.items():
        if entry["support"] == "stock_authoritative":
            _require(
                entry_id in mapped_entry_ids["selected"],
                f"{entry_id}: stock-authoritative entry has no selected upstream coverage",
            )
        if entry["support"] == "unassessed":
            _require(
                entry_id in mapped_entry_ids["reviewed_exclusion"],
                f"{entry_id}: unassessed entry has no reviewed exclusion coverage",
            )
    return {
        "manifest": upstream["manifest"],
        "selected": len(mapped["selected"]),
        "reviewed_exclusions": len(mapped["reviewed_exclusion"]),
        "total": len(coverage),
    }


def validate(manifest: dict, repo_root: Path) -> dict:
    """Validate structure, semantic invariants, and referenced evidence files."""
    _require(isinstance(manifest, dict), "manifest must be an object")
    _require(set(manifest) == MANIFEST_KEYS, "manifest fields differ from the v2 schema")
    _require(manifest.get("schema") == SCHEMA_ID, f"schema must be {SCHEMA_ID}")
    _require(isinstance(manifest.get("ros_distribution"), str), "ros_distribution is required")
    try:
        datetime.date.fromisoformat(manifest.get("reviewed_at", ""))
    except ValueError as exc:
        raise ManifestError("reviewed_at must be an ISO date") from exc

    entries = manifest.get("entries")
    _require(isinstance(entries, list) and entries, "entries must be a non-empty list")
    seen = set()
    counts = collections.Counter()
    backend_counts = collections.Counter()
    area_counts = collections.defaultdict(collections.Counter)

    for index, entry in enumerate(entries):
        prefix = f"entries[{index}]"
        _require(isinstance(entry, dict), f"{prefix} must be an object")
        _require(set(entry) == ENTRY_KEYS, f"{prefix} fields differ from the v2 schema")
        entry_id = entry["id"]
        _require(isinstance(entry_id, str) and entry_id, f"{prefix}.id is required")
        _require(entry_id not in seen, f"duplicate entry id: {entry_id}")
        seen.add(entry_id)
        _require(entry["area"] in AREAS, f"{entry_id}: unknown area")
        _require(entry["support"] in SUPPORT, f"{entry_id}: unknown support status")
        _require(entry["backend"] in BACKENDS, f"{entry_id}: unknown backend")
        _require(entry["fallback"] in FALLBACKS, f"{entry_id}: unknown fallback")
        _require(isinstance(entry["default"], bool), f"{entry_id}: default must be boolean")
        _require(isinstance(entry["api"], str) and entry["api"], f"{entry_id}: api is required")
        _require(isinstance(entry["notes"], str) and entry["notes"], f"{entry_id}: notes are required")
        evidence = entry["evidence"]
        _require(isinstance(evidence, list), f"{entry_id}: evidence must be a list")
        _require(
            len(evidence) == len(set(evidence)),
            f"{entry_id}: evidence paths must be unique",
        )
        if entry["support"] == "certified":
            _require(evidence, f"{entry_id}: certified entries require evidence")
            _require(entry["backend"] != "none", f"{entry_id}: certified entry has no backend")
            _require(entry["fallback"] != "unsafe", f"{entry_id}: certified entry has unsafe fallback")
        if entry["support"] == "stock_authoritative":
            _require(evidence, f"{entry_id}: stock-authoritative entries require evidence")
            _require(
                entry["backend"] == "python",
                f"{entry_id}: stock-authoritative entry must use the Python backend",
            )
            _require(
                entry["fallback"] != "unsafe",
                f"{entry_id}: stock-authoritative entry has unsafe fallback",
            )
        if entry["support"] in {"unsupported", "unassessed"}:
            _require(
                entry["backend"] == "none",
                f"{entry_id}: {entry['support']} entry must have no backend",
            )
            _require(
                entry["default"] is False,
                f"{entry_id}: {entry['support']} entry cannot be default",
            )
        for relative_path in evidence:
            _require(isinstance(relative_path, str) and relative_path, f"{entry_id}: invalid evidence path")
            _require((repo_root / relative_path).exists(), f"{entry_id}: missing evidence {relative_path}")
        counts[entry["support"]] += 1
        backend_counts[entry["backend"]] += 1
        area_counts[entry["area"]][entry["support"]] += 1

    entries_by_id = {entry["id"]: entry for entry in entries}
    upstream_summary = _validate_upstream_coverage(manifest, repo_root, entries_by_id)

    return {
        "schema": SCHEMA_ID,
        "ros_distribution": manifest["ros_distribution"],
        "reviewed_at": manifest["reviewed_at"],
        "entries": len(entries),
        "support": dict(sorted(counts.items())),
        "backend": dict(sorted(backend_counts.items())),
        "areas": {area: dict(sorted(values.items())) for area, values in sorted(area_counts.items())},
        "upstream_contract": upstream_summary,
    }


def load_and_validate(path: Path, repo_root: Path) -> dict:
    manifest = _load_json(path, "compatibility manifest")
    return validate(manifest, repo_root)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--json", action="store_true", help="print the structured coverage summary")
    args = parser.parse_args(argv)
    repo_root = Path(__file__).resolve().parent.parent
    try:
        summary = load_and_validate(args.manifest, repo_root)
    except ManifestError as exc:
        print(f"compatibility manifest invalid: {exc}", file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))
    else:
        support = ", ".join(f"{key}={value}" for key, value in summary["support"].items())
        backend = ", ".join(f"{key}={value}" for key, value in summary["backend"].items())
        print(
            f"{args.manifest}: {summary['entries']} entries "
            f"(support: {support}; backend: {backend})"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
