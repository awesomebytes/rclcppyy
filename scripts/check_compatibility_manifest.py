#!/usr/bin/env python3
"""Validate and summarize a compatibility manifest without extra dependencies."""

from __future__ import annotations

import argparse
import collections
import datetime
import json
import sys
from pathlib import Path


SCHEMA_ID = "rclcppyy.compatibility/v1"
SUPPORT = {"certified", "experimental", "unsupported"}
BACKENDS = {"cpp", "python", "mixed", "none"}
FALLBACKS = {"none", "process_only", "safe", "unsafe", "not_applicable"}
AREAS = {
    "activation", "node", "message", "publisher", "subscription", "timer",
    "executor", "service", "action", "parameter", "lifecycle", "context",
    "serialization",
}
ENTRY_KEYS = {
    "id", "area", "api", "support", "backend", "fallback", "default",
    "evidence", "notes",
}


class ManifestError(ValueError):
    pass


def _require(condition, message):
    if not condition:
        raise ManifestError(message)


def validate(manifest: dict, repo_root: Path) -> dict:
    """Validate structure, semantic invariants, and referenced evidence files."""
    _require(isinstance(manifest, dict), "manifest must be an object")
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
    area_counts = collections.defaultdict(collections.Counter)

    for index, entry in enumerate(entries):
        prefix = f"entries[{index}]"
        _require(isinstance(entry, dict), f"{prefix} must be an object")
        _require(set(entry) == ENTRY_KEYS, f"{prefix} fields differ from the v1 schema")
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
        if entry["support"] == "certified":
            _require(evidence, f"{entry_id}: certified entries require evidence")
            _require(entry["backend"] != "none", f"{entry_id}: certified entry has no backend")
            _require(entry["fallback"] != "unsafe", f"{entry_id}: certified entry has unsafe fallback")
        for relative_path in evidence:
            _require(isinstance(relative_path, str) and relative_path, f"{entry_id}: invalid evidence path")
            _require((repo_root / relative_path).exists(), f"{entry_id}: missing evidence {relative_path}")
        counts[entry["support"]] += 1
        area_counts[entry["area"]][entry["support"]] += 1

    return {
        "schema": SCHEMA_ID,
        "ros_distribution": manifest["ros_distribution"],
        "reviewed_at": manifest["reviewed_at"],
        "entries": len(entries),
        "support": dict(sorted(counts.items())),
        "areas": {area: dict(sorted(values.items())) for area, values in sorted(area_counts.items())},
    }


def load_and_validate(path: Path, repo_root: Path) -> dict:
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ManifestError(f"cannot read {path}: {exc}") from exc
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
        print(f"{args.manifest}: {summary['entries']} entries ({support})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
