"""Strict evidence protocol shared by the runnable rewrite fixtures."""

from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
from typing import Any


CASE_SCHEMA = "rclcppyy.rewrite-case/v1"
EVIDENCE_SCHEMA = "rclcppyy.rewrite-evidence/v1"
CASE_PREFIX = "RCLCPPYY_REWRITE_CASE "
PAYLOADS = ("alpha", "Beta-17", "gamma payload", "DELTA")
BENCHMARK_BLOCKER = (
    "bounded acknowledgement-driven smoke samples are not representative performance "
    "evidence; use the main benchmark runner for at least five interleaved repetitions "
    "on an isolated machine with CPU and frequency controls"
)


def checksum(values: list[str]) -> str:
    encoded = "\0".join(values).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def build_case(
    *,
    project: str,
    variant: str,
    tier: int,
    outputs: list[str],
    elapsed_ns: int,
    cpu_time_ns: int,
    backend_roles: dict[str, dict[str, Any]],
    transform_crossings: int,
    api_coverage: list[str],
    contract_delta: list[str],
    teardown_clean: bool,
) -> dict[str, Any]:
    if elapsed_ns <= 0 or cpu_time_ns <= 0:
        raise ValueError("elapsed and CPU time must be positive")
    document = {
        "schema": CASE_SCHEMA,
        "project": project,
        "variant": variant,
        "tier": tier,
        "application_output": {
            "inputs": list(PAYLOADS),
            "outputs": outputs,
            "checksum_sha256": checksum(outputs),
        },
        "backend": {
            "verified": True,
            "roles": backend_roles,
        },
        "python_boundary_crossings": {
            "scope": "transform callback only",
            "count": transform_crossings,
        },
        "api_coverage": api_coverage,
        "contract_delta": contract_delta,
        "teardown_clean": teardown_clean,
        "benchmark": {
            "mode": "smoke",
            "performance_claims_allowed": False,
            "blocker": BENCHMARK_BLOCKER,
            "raw_sample": {
                "elapsed_ns": elapsed_ns,
                "cpu_time_ns": cpu_time_ns,
                "cpu_pct": (cpu_time_ns / elapsed_ns) * 100.0,
                "messages": len(outputs),
                "effective_rate_hz": len(outputs) / (elapsed_ns / 1e9),
            },
        },
    }
    validate_case(document)
    return document


def emit_case(document: dict[str, Any]) -> None:
    validate_case(document)
    print(CASE_PREFIX + json.dumps(document, sort_keys=True, allow_nan=False), flush=True)


def validate_case(document: dict[str, Any]) -> None:
    if document.get("schema") != CASE_SCHEMA:
        raise ValueError("unsupported rewrite case schema")
    if document.get("variant") not in ("stock", "rewrite"):
        raise ValueError("rewrite case variant must be stock or rewrite")
    if document.get("tier") not in (0, 2, 3):
        raise ValueError("fixture tier must be 0, 2, or 3")
    application = document.get("application_output")
    if not isinstance(application, dict):
        raise ValueError("application output is required")
    outputs = application.get("outputs")
    if application.get("inputs") != list(PAYLOADS) or not isinstance(outputs, list):
        raise ValueError("case must use the bounded fixture inputs")
    if application.get("checksum_sha256") != checksum(outputs):
        raise ValueError("application output checksum mismatch")
    backend = document.get("backend")
    if not isinstance(backend, dict) or backend.get("verified") is not True:
        raise ValueError("explicit backend verification is required")
    roles = backend.get("roles")
    if not isinstance(roles, dict) or not roles:
        raise ValueError("at least one backend role is required")
    for role, evidence in roles.items():
        if not role or not isinstance(evidence, dict):
            raise ValueError("backend roles require evidence objects")
        if evidence.get("backend") not in ("cpp", "python"):
            raise ValueError("backend role must identify cpp or python")
        if not evidence.get("evidence"):
            raise ValueError("backend role requires independent evidence")
    crossings = document.get("python_boundary_crossings")
    if not isinstance(crossings, dict) or not isinstance(crossings.get("count"), int):
        raise ValueError("transform callback crossing count is required")
    if crossings["count"] < 0:
        raise ValueError("transform callback crossing count cannot be negative")
    if not isinstance(document.get("api_coverage"), list) or not document["api_coverage"]:
        raise ValueError("explicit API coverage is required")
    if not isinstance(document.get("contract_delta"), list):
        raise ValueError("contract delta must be a list")
    if document.get("teardown_clean") is not True:
        raise ValueError("successful cases require clean teardown")
    benchmark = document.get("benchmark")
    if not isinstance(benchmark, dict):
        raise ValueError("benchmark smoke evidence is required")
    if benchmark.get("mode") != "smoke":
        raise ValueError("fixture evidence must remain smoke-only")
    if benchmark.get("performance_claims_allowed") is not False:
        raise ValueError("fixture smoke data cannot allow performance claims")
    if benchmark.get("blocker") != BENCHMARK_BLOCKER:
        raise ValueError("fixture evidence must retain the benchmark blocker")
    sample = benchmark.get("raw_sample")
    if not isinstance(sample, dict):
        raise ValueError("a raw benchmark sample is required")
    for field in ("elapsed_ns", "cpu_time_ns", "cpu_pct", "effective_rate_hz"):
        value = sample.get(field)
        if (not isinstance(value, (int, float)) or isinstance(value, bool)
                or not math.isfinite(value) or value <= 0):
            raise ValueError("raw sample %s must be positive and finite" % field)
    if sample.get("messages") != len(outputs):
        raise ValueError("raw sample message count must match application output")


def _parse_case(stdout: str) -> dict[str, Any]:
    matches = [line for line in stdout.splitlines() if line.startswith(CASE_PREFIX)]
    if len(matches) != 1:
        raise RuntimeError("child emitted %d rewrite case documents" % len(matches))
    document = json.loads(matches[0][len(CASE_PREFIX):])
    validate_case(document)
    return document


def _run_case(project_dir: Path, variant: str, domain_id: int, timeout: float) -> dict[str, Any]:
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(domain_id)
    process = subprocess.run(
        [sys.executable, str(project_dir / "run_case.py"), "--variant", variant],
        capture_output=True,
        text=True,
        timeout=timeout,
        env=environment,
        start_new_session=True,
    )
    if process.returncode != 0:
        raise RuntimeError(
            "%s %s failed with code %d\nstdout:\n%s\nstderr:\n%s" % (
                project_dir.name,
                variant,
                process.returncode,
                process.stdout,
                process.stderr,
            )
        )
    return _parse_case(process.stdout)


def _pair(project_dir: Path, tier: int, repetition: int, timeout: float) -> dict[str, Any]:
    # Keep each process isolated while staying inside ROS_DOMAIN_ID's portable range.
    domain_base = 40 + ((os.getpid() + repetition * 11 + tier * 17) % 150)
    before = _run_case(project_dir, "stock", domain_base, timeout)
    after = _run_case(project_dir, "rewrite", domain_base + 1, timeout)
    before_output = before["application_output"]
    after_output = after["application_output"]
    if before_output != after_output:
        raise RuntimeError("%s rewrite changed bounded application output" % project_dir.name)
    return {
        "repetition": repetition,
        "before": before,
        "after": after,
        "parity": {
            "outputs_equal": True,
            "checksums_equal": True,
            "messages": len(before_output["outputs"]),
        },
    }


PROJECTS = {
    "transparent_relay": 0,
    "managed_native_worker": 2,
    "fused_native_relay": 3,
}


def _git_value(repo_root: Path, *arguments: str) -> str | None:
    try:
        process = subprocess.run(
            ["git", *arguments],
            cwd=repo_root,
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if process.returncode != 0:
        return None
    return process.stdout.strip()


def _environment(repo_root: Path) -> dict[str, Any]:
    status = _git_value(repo_root, "status", "--porcelain", "--untracked-files=no")
    return {
        "source": {
            "commit": _git_value(repo_root, "rev-parse", "HEAD"),
            "dirty": bool(status) if status is not None else None,
        },
        "host": {
            "architecture": platform.machine(),
            "system": platform.system(),
            "kernel": platform.release(),
            "processor": platform.processor() or None,
            "logical_cpu_count": os.cpu_count(),
        },
        "runtime": {
            "python": platform.python_version(),
            "implementation": platform.python_implementation(),
        },
        "ros": {
            "distribution": os.environ.get("ROS_DISTRO"),
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
            "automatic_discovery_range": os.environ.get(
                "ROS_AUTOMATIC_DISCOVERY_RANGE"),
        },
        "cache": {
            name: os.environ.get(name)
            for name in (
                "CPPYY_KIT_NO_AUTOPCH",
                "CPPYY_KIT_NO_CACHE",
                "RCLCPPYY_DISABLE_CACHE",
                "XDG_CACHE_HOME",
            )
        },
    }


def build_evidence(
    root: Path,
    projects: list[str],
    repetitions: int,
    timeout: float,
    command: list[str] | None = None,
) -> dict[str, Any]:
    if repetitions < 1:
        raise ValueError("repetitions must be positive")
    results = []
    for project in projects:
        tier = PROJECTS[project]
        project_dir = root / project
        pairs = [
            _pair(project_dir, tier, repetition, timeout)
            for repetition in range(repetitions)
        ]
        results.append({
            "project": project,
            "selected_tier": tier,
            "pairs": pairs,
        })
    document = {
        "schema": EVIDENCE_SCHEMA,
        "generated_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "command": list(command or sys.argv),
        "environment": _environment(root.parents[3]),
        "benchmark": {
            "mode": "smoke",
            "performance_claims_allowed": False,
            "blocker": BENCHMARK_BLOCKER,
            "repetitions": repetitions,
            "workload": {
                "message_type": "std_msgs/msg/String",
                "qos": {"history": "keep_last", "depth": 10},
                "payloads": list(PAYLOADS),
                "send_mode": "one message followed by output acknowledgement",
                "timing_scope": "post-discovery application processing only",
            },
            "statistics": {
                "wall_time": "perf_counter_ns around acknowledged sequential messages",
                "cpu_time": (
                    "process_time_ns including middleware threads; CPU percentage may "
                    "exceed 100 on multicore hosts"
                ),
                "comparison": "none; raw samples are retained without ranking",
            },
        },
        "projects": results,
        "failures": [],
    }
    validate_evidence(document)
    return document


def validate_evidence(document: dict[str, Any]) -> None:
    if document.get("schema") != EVIDENCE_SCHEMA:
        raise ValueError("unsupported rewrite evidence schema")
    benchmark = document.get("benchmark")
    if not isinstance(benchmark, dict):
        raise ValueError("benchmark metadata is required")
    if benchmark.get("mode") != "smoke":
        raise ValueError("rewrite fixture evidence is smoke-only")
    if benchmark.get("performance_claims_allowed") is not False:
        raise ValueError("rewrite fixture evidence cannot support performance claims")
    if benchmark.get("blocker") != BENCHMARK_BLOCKER:
        raise ValueError("rewrite fixture benchmark blocker is required")
    if document.get("failures") != []:
        raise ValueError("successful rewrite evidence cannot contain failures")
    projects = document.get("projects")
    if not isinstance(projects, list) or not projects:
        raise ValueError("at least one rewrite project is required")
    for project in projects:
        if project.get("selected_tier") != PROJECTS.get(project.get("project")):
            raise ValueError("rewrite project tier mismatch")
        pairs = project.get("pairs")
        if not isinstance(pairs, list) or not pairs:
            raise ValueError("rewrite project requires paired samples")
        for pair in pairs:
            before = pair.get("before", {})
            after = pair.get("after", {})
            validate_case(before)
            validate_case(after)
            if before.get("variant") != "stock" or after.get("variant") != "rewrite":
                raise ValueError("rewrite pair must contain stock before rewrite")
            if any(
                    case.get("project") != project["project"]
                    or case.get("tier") != project["selected_tier"]
                    for case in (before, after)):
                raise ValueError("rewrite pair project or tier mismatch")
            if before["application_output"] != after["application_output"]:
                raise ValueError("rewrite pair changed bounded application output")
            parity = pair.get("parity")
            if not isinstance(parity, dict) or not all(
                    parity.get(field) is True
                    for field in ("outputs_equal", "checksums_equal")):
                raise ValueError("rewrite pair did not preserve bounded behavior")
            if parity.get("messages") != len(before["application_output"]["outputs"]):
                raise ValueError("rewrite parity message count mismatch")


def write_evidence(document: dict[str, Any], output: Path) -> None:
    validate_evidence(document)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.name + ".tmp")
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(output)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--project",
        action="append",
        choices=tuple(PROJECTS),
        help="project to run; repeat the option to select more than one",
    )
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    root = Path(__file__).resolve().parent
    projects = args.project or list(PROJECTS)
    document = build_evidence(root, projects, args.repetitions, args.timeout)
    write_evidence(document, args.output)
    print(str(args.output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
