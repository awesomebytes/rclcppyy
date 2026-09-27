#!/usr/bin/env python3
"""Prove a generated rclcpp factory cache miss and fresh-process hit."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import uuid


SCHEMA = "rclcppyy.generated-cache-proof/v1"
WORKER_SCHEMA = "rclcppyy.generated-cache-worker/v1"
CPP_TYPE = "std_msgs::msg::String"
HEADER = "std_msgs/msg/string.hpp"


def _write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(".%s.%s.tmp" % (path.name, uuid.uuid4().hex))
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _worker(cache_root: Path) -> dict:
    import cppyy
    import cppyy_kit
    from rclcpp_kit import subscription_cache

    compile_args = subscription_cache._compile_args(CPP_TYPE, HEADER)
    code = compile_args.pop("code")
    function_name = compile_args["name"]
    result = cppyy_kit.cppdef_cached(code, **compile_args)
    artifact = Path(result["so"]).resolve() if result.get("so") else None
    return {
        "schema": WORKER_SCHEMA,
        "cache_root": str(cache_root.resolve()),
        "cpp_type": CPP_TYPE,
        "function": function_name,
        "symbol_resolved": callable(getattr(cppyy.gbl, function_name)),
        "result": result,
        "artifact_exists": bool(artifact and artifact.is_file()),
    }


def _run_worker(cache_root: Path) -> dict:
    environment = os.environ.copy()
    environment["XDG_CACHE_HOME"] = str(cache_root.resolve())
    environment["CPPYY_KIT_NO_AUTOPCH"] = "1"
    environment.pop("CPPYY_KIT_NO_CACHE", None)
    environment.pop("RCLCPP_KIT_NO_SUB_CACHE", None)
    process = subprocess.run(
        [
            sys.executable,
            str(Path(__file__).resolve()),
            "--worker",
            "--cache-root",
            str(cache_root),
        ],
        capture_output=True,
        text=True,
        timeout=180,
        env=environment,
        check=False,
    )
    if process.returncode != 0:
        raise RuntimeError(
            "generated-cache worker failed (%d):\n%s\n%s" % (
                process.returncode, process.stdout, process.stderr))
    lines = [line for line in process.stdout.splitlines() if line.strip()]
    if not lines:
        raise RuntimeError("generated-cache worker produced no JSON")
    try:
        value = json.loads(lines[-1])
    except json.JSONDecodeError as exc:
        raise RuntimeError(
            "generated-cache worker produced invalid JSON:\n%s" % process.stdout
        ) from exc
    if value.get("schema") != WORKER_SCHEMA:
        raise RuntimeError("generated-cache worker schema mismatch")
    return value


def prove(cache_root: Path) -> dict:
    cache_root = cache_root.resolve()
    if cache_root.exists() and any(cache_root.iterdir()):
        raise ValueError("cache root must be absent or empty: %s" % cache_root)

    cold = _run_worker(cache_root)
    warm = _run_worker(cache_root)
    cold_result = cold["result"]
    warm_result = warm["result"]
    if cold_result.get("cached") is not False:
        raise AssertionError("cold process did not record a cache miss")
    if cold_result.get("reason") != "miss-built":
        raise AssertionError("cold process did not build a cache artifact")
    if warm_result.get("cached") is not True:
        raise AssertionError("warm process did not record a cache hit")
    if not cold["symbol_resolved"] or not warm["symbol_resolved"]:
        raise AssertionError("generated rclcpp factory symbol was not resolved")

    artifact = Path(cold_result["so"]).resolve()
    if artifact != Path(warm_result["so"]).resolve():
        raise AssertionError("cold and warm processes selected different artifacts")
    try:
        artifact.relative_to(cache_root)
    except ValueError as exc:
        raise AssertionError("cache artifact escaped the isolated root") from exc
    if not artifact.is_file() or artifact.stat().st_size <= 0:
        raise AssertionError("generated cache artifact is absent or empty")
    header = Path(cold_result["header"]).resolve()
    metadata = artifact.with_suffix(".json")
    if not header.is_file() or not metadata.is_file():
        raise AssertionError("generated cache sidecar artifacts are incomplete")

    digest = hashlib.sha256(artifact.read_bytes()).hexdigest()
    return {
        "schema": SCHEMA,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "architecture": platform.machine(),
        "python": platform.python_version(),
        "cache_root": str(cache_root),
        "policy": {
            "isolated": True,
            "autopch_disabled": True,
            "fresh_process_per_phase": True,
        },
        "phases": {"cold": cold, "warm": warm},
        "artifact": {
            "path": str(artifact),
            "header": str(header),
            "metadata": str(metadata),
            "size_bytes": artifact.stat().st_size,
            "sha256": digest,
        },
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache-root", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.worker:
        print(json.dumps(_worker(args.cache_root), sort_keys=True))
        return 0
    evidence = prove(args.cache_root)
    if args.output is not None:
        _write_json(args.output, evidence)
    print(
        "GENERATED_CACHE_OK cold=miss-built warm=hit sha256=%s" %
        evidence["artifact"]["sha256"]
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
