"""Stable, strict-JSON result envelope for rclcppyy benchmarks."""

from __future__ import annotations

import datetime
import importlib.metadata
import json
import math
import os
import platform
import subprocess
import sys
from pathlib import Path


SCHEMA_ID = "rclcppyy.benchmark/v1"
PACKAGE_NAMES = (
    "rclcppyy",
    "ros-jazzy-rclcppyy",
    "rclcpp-kit",
    "ros-jazzy-rclcpp-kit",
    "cppyy-kit",
    "cppyy",
    "numpy",
    "psutil",
)
CACHE_ENV_NAMES = (
    "CPPYY_KIT_NO_AUTOPCH",
    "CPPYY_KIT_NO_CACHE",
    "RCLCPPYY_DISABLE_CACHE",
    "XDG_CACHE_HOME",
)


def _run_git(repo_root: Path, *args: str) -> str | None:
    try:
        proc = subprocess.run(
            ["git", *args],
            cwd=repo_root,
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if proc.returncode != 0:
        return None
    return proc.stdout.strip()


def _source_metadata(repo_root: Path) -> dict:
    commit = _run_git(repo_root, "rev-parse", "HEAD")
    status = _run_git(repo_root, "status", "--porcelain", "--untracked-files=no")
    return {
        "repository": str(repo_root),
        "commit": commit,
        "dirty": bool(status) if status is not None else None,
    }


def _cpu_model() -> str | None:
    cpuinfo = Path("/proc/cpuinfo")
    if not cpuinfo.exists():
        return platform.processor() or None
    candidates = {}
    try:
        for line in cpuinfo.read_text(encoding="utf-8", errors="replace").splitlines():
            key, separator, value = line.partition(":")
            key = key.strip().lower()
            if separator and key in ("model name", "hardware", "processor"):
                value = value.strip()
                if value:
                    candidates.setdefault(key, value)
    except OSError:
        pass
    for key in ("model name", "hardware", "processor"):
        if key in candidates:
            return candidates[key]
    return platform.processor() or None


def _package_versions() -> dict[str, str | None]:
    versions = {}
    for package in PACKAGE_NAMES:
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = None
    return versions


def environment_metadata(repo_root: Path) -> dict:
    """Collect enough immutable context to interpret or reproduce a result."""
    uname = platform.uname()
    return {
        "source": _source_metadata(repo_root),
        "host": {
            "architecture": platform.machine(),
            "system": uname.system,
            "kernel": uname.release,
            "cpu_model": _cpu_model(),
            "logical_cpu_count": os.cpu_count(),
        },
        "runtime": {
            "python": platform.python_version(),
            "python_implementation": platform.python_implementation(),
            "python_abi": getattr(sys.implementation, "cache_tag", None),
            "packages": _package_versions(),
        },
        "ros": {
            "distribution": os.environ.get("ROS_DISTRO"),
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
            "domain_id": os.environ.get("ROS_DOMAIN_ID"),
            "automatic_discovery_range": os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE"),
        },
        "cache": {name: os.environ.get(name) for name in CACHE_ENV_NAMES},
    }


def _strict_json_value(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _strict_json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_strict_json_value(item) for item in value]
    return value


def build_document(
    *,
    repo_root: Path,
    benchmark_name: str,
    parameters: dict,
    results_by_rate: dict,
    failures: list,
    command: list[str] | None = None,
) -> dict:
    """Build and validate one versioned benchmark result document."""
    rows = []
    for rate, results in results_by_rate.items():
        for result in results:
            row = dict(result)
            row.setdefault("target_rate_hz", int(rate))
            rows.append(row)

    document = {
        "schema": SCHEMA_ID,
        "generated_at": datetime.datetime.now(datetime.timezone.utc).isoformat().replace("+00:00", "Z"),
        "command": list(command if command is not None else sys.argv),
        "environment": environment_metadata(repo_root),
        "benchmark": {
            "name": benchmark_name,
            "parameters": parameters,
        },
        "results": rows,
        "failures": failures,
    }
    document = _strict_json_value(document)
    validate_document(document)
    return document


def validate_document(document: dict) -> None:
    """Reject malformed documents before they are stored or published."""
    if document.get("schema") != SCHEMA_ID:
        raise ValueError("unsupported benchmark schema")
    if not isinstance(document.get("generated_at"), str):
        raise ValueError("generated_at must be an ISO-8601 string")
    if not isinstance(document.get("environment"), dict):
        raise ValueError("environment metadata is required")
    benchmark = document.get("benchmark")
    if not isinstance(benchmark, dict) or not benchmark.get("name"):
        raise ValueError("benchmark name is required")
    if not isinstance(benchmark.get("parameters"), dict):
        raise ValueError("benchmark parameters must be an object")
    if not isinstance(document.get("results"), list):
        raise ValueError("results must be a list")
    if not isinstance(document.get("failures"), list):
        raise ValueError("failures must be a list")


def dumps(document: dict) -> str:
    """Serialize strict JSON; NaN and Infinity indicate a schema bug."""
    validate_document(document)
    return json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n"


def write(document: dict, path: Path) -> None:
    """Atomically write a result document."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(dumps(document), encoding="utf-8")
    temporary.replace(path)
