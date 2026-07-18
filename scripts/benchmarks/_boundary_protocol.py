"""Strict protocol and result envelope for the callback-boundary benchmark."""

from __future__ import annotations

import datetime
import json
import math
from pathlib import Path
import re
import statistics
import sys

from _result_schema import environment_metadata


SCHEMA_ID = "rclcppyy.boundary-benchmark/v1"
SAMPLE_SCHEMA = "rclcppyy.boundary-sample/v1"
BACKEND_SCHEMA = "rclcppyy.boundary-backend/v1"
VARIANTS = {
    "cppyy-python-boundary": {
        "backend": "cppyy",
        "execution_model": "cppyy-cpp-to-python-to-cpp-callback",
        "callback_count": "iterations",
    },
    "cppyy-fused": {
        "backend": "cppyy",
        "execution_model": "cppyy-jit-fused-cpp",
        "callback_count": "zero",
    },
    "aot-cpp": {
        "backend": "aot-cpp",
        "execution_model": "standalone-optimized-executable",
        "callback_count": "zero",
    },
}
SHA256_PATTERN = re.compile(r"^[a-f0-9]{64}$")


def _is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


def _is_sha256(value):
    return isinstance(value, str) and SHA256_PATTERN.fullmatch(value) is not None


def validate_sample(
        sample, *, variant, run_token, iterations, checksum, process_id):
    if not isinstance(sample, dict) or sample.get("schema") != SAMPLE_SCHEMA:
        raise ValueError("invalid boundary sample schema")
    if sample.get("variant") != variant:
        raise ValueError("boundary sample variant mismatch")
    if sample.get("run_token") != run_token:
        raise ValueError("boundary sample run token mismatch")
    for field in (
            "pid", "process_group_id", "iterations", "checksum", "elapsed_ns",
            "cpu_time_ns", "python_callback_count"):
        if not _is_int(sample.get(field)):
            raise ValueError(f"boundary sample requires integer {field}")
    if sample["pid"] != process_id or sample["process_group_id"] != process_id:
        raise ValueError("boundary sample did not run in its isolated process group")
    if sample["iterations"] != iterations:
        raise ValueError("boundary sample iteration count mismatch")
    if sample["checksum"] != checksum:
        raise ValueError("boundary sample checksum mismatch")
    if sample["elapsed_ns"] <= 0 or sample["cpu_time_ns"] < 0:
        raise ValueError("boundary sample timing must be non-negative")

    expected = VARIANTS[variant]
    backend = sample.get("backend")
    if not isinstance(backend, dict) or backend.get("schema") != BACKEND_SCHEMA:
        raise ValueError("invalid boundary backend evidence schema")
    if backend.get("backend") != expected["backend"]:
        raise ValueError("boundary backend mismatch")
    if backend.get("execution_model") != expected["execution_model"]:
        raise ValueError("boundary execution model mismatch")
    if not isinstance(backend.get("evidence"), str) or not backend["evidence"]:
        raise ValueError("boundary backend evidence is required")
    callback_count = iterations if expected["callback_count"] == "iterations" else 0
    if sample["python_callback_count"] != callback_count:
        raise ValueError("boundary sample callback count contradicts backend evidence")


def summarize(results, variants):
    medians = {}
    for variant in variants:
        values = [
            row["ns_per_iteration"] for row in results
            if row["variant"] == variant
        ]
        if values:
            medians[variant] = statistics.median(values)
    ratios = {}
    aot = medians.get("aot-cpp")
    if aot is not None and aot > 0:
        ratios = {
            variant: value / aot for variant, value in medians.items()
        }
    boundary = medians.get("cppyy-python-boundary")
    fused = medians.get("cppyy-fused")
    incremental = None
    boundary_to_fused = None
    if boundary is not None and fused is not None:
        incremental = boundary - fused
        if fused > 0:
            boundary_to_fused = boundary / fused
    return {
        "checksum_consistent": bool(results) and len({
            row["checksum"] for row in results}) == 1,
        "raw_median_ns_per_iteration": medians,
        "raw_ratio_to_aot": ratios,
        "raw_boundary_increment_over_cppyy_fused_ns_per_iteration": incremental,
        "raw_boundary_ratio_to_cppyy_fused": boundary_to_fused,
        "interpretation_allowed": False,
        "note": (
            "Raw same-run characterization only; ratios combine callback crossing "
            "and Python transform cost and do not establish a performance claim."
        ),
    }


def build_document(
        *, repo_root: Path, mode, parameters, isolation, aot_build,
        results, failures, command=None):
    document = {
        "schema": SCHEMA_ID,
        "generated_at": datetime.datetime.now(
            datetime.timezone.utc).isoformat().replace("+00:00", "Z"),
        "command": list(command if command is not None else sys.argv),
        "environment": environment_metadata(repo_root),
        "benchmark": {
            "name": "native_python_boundary_vs_aot",
            "mode": mode,
            "performance_claims_allowed": False,
            "parameters": parameters,
            "isolation": isolation,
            "aot_build": aot_build,
            "statistics": {
                "timed_region": (
                    "deterministic uint64 transform loop only; import, JIT/AOT "
                    "compilation, process startup, and warmup are excluded"
                ),
                "sample": "one fresh process per variant and repetition",
                "summary": "median of raw nanoseconds per iteration",
            },
            "scope": {
                "measures": (
                    "C++ to Python to C++ callback path including the small Python "
                    "transform body, compared with fused JIT C++ and standalone AOT C++"
                ),
                "does_not_measure": (
                    "ROS transport, executor scheduling, DDS, end-to-end latency, or "
                    "a pure ABI crossing isolated from Python work"
                ),
            },
        },
        "results": list(results),
        "comparison": summarize(results, parameters["variants"]),
        "failures": list(failures),
    }
    validate_document(document)
    return document


def validate_document(document):
    if not isinstance(document, dict) or document.get("schema") != SCHEMA_ID:
        raise ValueError("unsupported boundary benchmark schema")
    if not isinstance(document.get("generated_at"), str):
        raise ValueError("boundary benchmark generated_at is required")
    benchmark = document.get("benchmark")
    if not isinstance(benchmark, dict):
        raise ValueError("boundary benchmark metadata is required")
    if benchmark.get("mode") not in ("smoke", "measurement"):
        raise ValueError("boundary benchmark mode must be smoke or measurement")
    if benchmark.get("performance_claims_allowed") is not False:
        raise ValueError("raw boundary benchmarks cannot allow performance claims")
    isolation = benchmark.get("isolation")
    required_isolation = (
        "fresh_process_per_sample", "new_process_group_per_sample",
        "unique_run_token_per_sample", "private_aot_build_directory",
    )
    if not isinstance(isolation, dict) or any(
            isolation.get(field) is not True for field in required_isolation):
        raise ValueError("boundary benchmark requires process and build isolation")
    domain_id = isolation.get("ros_domain_id")
    if not _is_int(domain_id) or not 0 <= domain_id <= 232:
        raise ValueError("boundary benchmark requires a valid leased ROS domain")
    aot_build = benchmark.get("aot_build")
    if not isinstance(aot_build, dict):
        raise ValueError("boundary benchmark AOT build evidence is required")
    if not isinstance(aot_build.get("compiler"), str) or not aot_build["compiler"]:
        raise ValueError("boundary benchmark AOT compiler evidence is required")
    if not isinstance(aot_build.get("compiler_version"), str) or not aot_build[
            "compiler_version"]:
        raise ValueError("boundary benchmark AOT compiler version is required")
    if not isinstance(aot_build.get("flags"), list) or not all(
            isinstance(flag, str) for flag in aot_build["flags"]):
        raise ValueError("boundary benchmark AOT flags are invalid")
    for field in ("source_sha256", "kernel_sha256", "executable_sha256"):
        if not _is_sha256(aot_build.get(field)):
            raise ValueError(f"boundary benchmark AOT {field} is invalid")
    if aot_build.get("executable_format") != "ELF" or aot_build.get(
            "build_directory_persisted") is not False:
        raise ValueError("boundary benchmark AOT executable evidence is invalid")
    if not _is_int(aot_build.get("build_elapsed_ns")) or aot_build[
            "build_elapsed_ns"] <= 0:
        raise ValueError("boundary benchmark AOT build timing is invalid")
    results = document.get("results")
    failures = document.get("failures")
    if not isinstance(results, list) or not isinstance(failures, list):
        raise ValueError("boundary results and failures must be arrays")
    parameters = benchmark.get("parameters")
    if not isinstance(parameters, dict):
        raise ValueError("boundary benchmark parameters are required")
    variants = parameters.get("variants")
    repetitions = parameters.get("repetitions")
    iterations = parameters.get("iterations")
    expected_checksum = parameters.get("expected_checksum")
    if not isinstance(variants, list) or not variants or any(
            variant not in VARIANTS for variant in variants):
        raise ValueError("boundary benchmark variants are invalid")
    if not _is_int(repetitions) or repetitions <= 0:
        raise ValueError("boundary benchmark repetitions are invalid")
    if not _is_int(iterations) or iterations <= 0:
        raise ValueError("boundary benchmark iterations are invalid")
    if not _is_int(expected_checksum) or expected_checksum < 0:
        raise ValueError("boundary benchmark expected checksum is invalid")
    tokens = []
    process_ids = []
    for row in results:
        if not isinstance(row, dict) or row.get("backend_verified") is not True:
            raise ValueError("boundary results require verified backend evidence")
        if row.get("variant") not in VARIANTS:
            raise ValueError("boundary result has unknown variant")
        if not isinstance(row.get("ns_per_iteration"), (int, float)) or not math.isfinite(
                row["ns_per_iteration"]):
            raise ValueError("boundary result requires finite ns_per_iteration")
        validate_sample(
            row,
            variant=row["variant"],
            run_token=row.get("run_token"),
            iterations=iterations,
            checksum=expected_checksum,
            process_id=row.get("pid"),
        )
        if row["ns_per_iteration"] != row["elapsed_ns"] / iterations:
            raise ValueError("boundary result ns_per_iteration is inconsistent")
        if not _is_sha256(row.get("worker_source_sha256")) or row.get(
                "kernel_sha256") != aot_build["kernel_sha256"]:
            raise ValueError("boundary result source identity is invalid")
        if row["variant"] == "aot-cpp" and row["backend"].get(
                "executable_sha256") != aot_build["executable_sha256"]:
            raise ValueError("boundary AOT executable identity mismatch")
        tokens.append(row.get("run_token"))
        process_ids.append(row.get("pid"))
    if len(tokens) != len(set(tokens)) or len(process_ids) != len(set(process_ids)):
        raise ValueError("boundary result samples must use unique processes and tokens")
    if not failures:
        expected_cases = {
            (variant, repetition)
            for variant in variants
            for repetition in range(1, repetitions + 1)
        }
        observed_cases = {
            (row.get("variant"), row.get("repetition")) for row in results}
        if observed_cases != expected_cases or len(results) != len(expected_cases):
            raise ValueError("boundary benchmark result matrix is incomplete")
    if isolation.get("sample_process_ids") != process_ids:
        raise ValueError("boundary isolation PID evidence does not match results")
    if isolation.get("sample_run_tokens") != tokens:
        raise ValueError("boundary isolation token evidence does not match results")
    comparison = document.get("comparison")
    if not isinstance(comparison, dict) or comparison.get(
            "interpretation_allowed") is not False:
        raise ValueError("raw boundary comparison cannot permit interpretation")
    if results and comparison.get("checksum_consistent") is not True:
        raise ValueError("boundary result checksums must agree")
    if comparison != summarize(results, variants):
        raise ValueError("boundary raw comparison is inconsistent with samples")


def dumps(document):
    validate_document(document)
    return json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n"


def write(document, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(dumps(document), encoding="utf-8")
    temporary.replace(path)


__all__ = [
    "SCHEMA_ID", "VARIANTS", "build_document", "dumps", "summarize",
    "validate_document", "validate_sample", "write",
]
