#!/usr/bin/env python3
"""Verify that source tests use the one reviewed supporting-suite identity."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import subprocess


SCHEMA = "rclcppyy.suite-source/v1"
COMMIT_RE = re.compile(r"[0-9a-f]{40}")
VERSION_RE = re.compile(r"[0-9]+\.[0-9]+\.[0-9]+")


def _git(suite: Path, *args: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(suite), *args], text=True).strip()


def load_lock(path: Path) -> dict:
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("schema") != SCHEMA:
        raise ValueError("unsupported suite source lock schema")
    if not COMMIT_RE.fullmatch(str(document.get("commit", ""))):
        raise ValueError("suite source lock requires a full lowercase commit")
    if not VERSION_RE.fullmatch(str(document.get("package_version", ""))):
        raise ValueError("suite source lock requires an exact package version")
    if not document.get("repository"):
        raise ValueError("suite source lock requires a repository")
    return document


def recipe_versions(suite: Path) -> set[str]:
    versions = set()
    for recipe in sorted((suite / "recipe").glob("*/recipe.yaml")):
        match = re.search(
            r"^  version: \"([^\"]+)\"$",
            recipe.read_text(encoding="utf-8"),
            flags=re.MULTILINE,
        )
        if match is None:
            raise ValueError("recipe has no context version: %s" % recipe)
        versions.add(match.group(1))
    return versions


def verify(repo_root: Path, suite: Path, *, allow_dirty: bool = False) -> dict:
    lock = load_lock(repo_root / "suite-source.lock.json")
    suite = suite.resolve()
    expected_paths = (
        suite / "cppyy_kit" / "__init__.py",
        suite / "rclcpp_kit" / "rclcpp_kit" / "__init__.py",
    )
    missing = [str(path) for path in expected_paths if not path.is_file()]
    if missing:
        raise ValueError("suite source checkout is incomplete: %s" % ", ".join(missing))

    actual_commit = _git(suite, "rev-parse", "HEAD")
    if actual_commit != lock["commit"]:
        raise ValueError(
            "suite source commit mismatch: expected %s, observed %s" % (
                lock["commit"], actual_commit))
    dirty = bool(_git(suite, "status", "--porcelain"))
    if dirty and not allow_dirty:
        raise ValueError("suite source checkout has uncommitted changes")
    versions = recipe_versions(suite)
    if versions != {lock["package_version"]}:
        raise ValueError(
            "suite recipe version mismatch: expected %s, observed %s" % (
                lock["package_version"], sorted(versions)))

    python_path = [
        Path(value).resolve()
        for value in os.environ.get("PYTHONPATH", "").split(os.pathsep)
        if value
    ]
    required_roots = (suite, suite / "rclcpp_kit")
    absent = [str(path) for path in required_roots if path.resolve() not in python_path]
    if absent:
        raise ValueError(
            "suite source roots are not active on PYTHONPATH: %s" % ", ".join(absent))

    return {
        "schema": SCHEMA,
        "repository": lock["repository"],
        "commit": actual_commit,
        "package_version": lock["package_version"],
        "suite_path": str(suite),
        "dirty": dirty,
        "python_roots_verified": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--suite", type=Path)
    parser.add_argument("--allow-dirty", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    repo_root = Path(__file__).resolve().parents[2]
    suite = args.suite or Path(
        os.environ.get("RCLCPPYY_SUITE_SRC", repo_root.parent / "cppyy_kit"))
    report = verify(repo_root, suite, allow_dirty=args.allow_dirty)
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print("SUITE_SOURCE_OK %s" % report["commit"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
