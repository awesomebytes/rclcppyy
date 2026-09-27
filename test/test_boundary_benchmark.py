"""Protocol, backend-evidence, and isolated smoke tests for boundary evidence."""

from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parent.parent
BENCH_DIR = REPO_ROOT / "scripts" / "benchmarks"
sys.path.insert(0, str(BENCH_DIR))


def _load(name):
    spec = importlib.util.spec_from_file_location(name, BENCH_DIR / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


protocol = _load("_boundary_protocol")
worker = _load("boundary_worker")


def _sample(variant="cppyy-python-boundary"):
    models = {
        "cppyy-python-boundary": (
            "cppyy", "cppyy-cpp-to-python-to-cpp-callback", 10),
        "cppyy-fused": ("cppyy", "cppyy-jit-fused-cpp", 0),
        "aot-cpp": ("aot-cpp", "standalone-optimized-executable", 0),
    }
    backend, model, callbacks = models[variant]
    return {
        "schema": "rclcppyy.boundary-sample/v1",
        "run_token": "run_fixture",
        "variant": variant,
        "pid": 123,
        "process_group_id": 123,
        "iterations": 10,
        "checksum": 99,
        "elapsed_ns": 1000,
        "cpu_time_ns": 900,
        "python_callback_count": callbacks,
        "backend": {
            "schema": "rclcppyy.boundary-backend/v1",
            "backend": backend,
            "execution_model": model,
            "evidence": "fixture evidence",
        },
    }


def _aot_build():
    digest = "a" * 64
    return {
        "compiler": "/fixture/c++",
        "compiler_version": "fixture c++ 1.0",
        "flags": ["-O3"],
        "source_sha256": digest,
        "kernel_sha256": digest,
        "executable_sha256": digest,
        "executable_format": "ELF",
        "build_elapsed_ns": 100,
        "build_directory_persisted": False,
    }


def test_python_transform_has_stable_uint64_reference_vector():
    state = 1469598103934665603
    for index in range(1000):
        state = worker.python_transform(state, index)

    assert state == 830812027954845773


def test_sample_requires_observed_callback_count_and_exact_backend():
    sample = _sample()
    protocol.validate_sample(
        sample,
        variant="cppyy-python-boundary",
        run_token="run_fixture",
        iterations=10,
        checksum=99,
        process_id=123,
    )

    sample["python_callback_count"] = 0
    with pytest.raises(ValueError, match="callback count"):
        protocol.validate_sample(
            sample,
            variant="cppyy-python-boundary",
            run_token="run_fixture",
            iterations=10,
            checksum=99,
            process_id=123,
        )


def test_raw_document_rejects_claims_and_missing_isolation(tmp_path):
    row = _sample("aot-cpp")
    row.update({
        "case_id": "aot-cpp__rep_1",
        "repetition": 1,
        "backend_verified": True,
        "ns_per_iteration": 100.0,
        "worker_source_sha256": "a" * 64,
        "kernel_sha256": "a" * 64,
    })
    row["backend"]["executable_sha256"] = "a" * 64
    document = protocol.build_document(
        repo_root=REPO_ROOT,
        mode="measurement",
        parameters={
            "variants": ["aot-cpp"],
            "iterations": 10,
            "repetitions": 1,
            "expected_checksum": 99,
        },
        isolation={
            "fresh_process_per_sample": True,
            "new_process_group_per_sample": True,
            "unique_run_token_per_sample": True,
            "private_aot_build_directory": True,
            "ros_domain_id": 123,
            "sample_process_ids": [123],
            "sample_run_tokens": ["run_fixture"],
        },
        aot_build=_aot_build(),
        results=[row],
        failures=[],
        command=["boundary-benchmark"],
    )
    assert document["comparison"]["interpretation_allowed"] is False
    assert document["benchmark"]["performance_claims_allowed"] is False
    assert json.loads(protocol.dumps(document)) == document

    document["benchmark"]["performance_claims_allowed"] = True
    with pytest.raises(ValueError, match="cannot allow performance claims"):
        protocol.validate_document(document)
    document["benchmark"]["performance_claims_allowed"] = False
    document["results"][0]["backend"]["execution_model"] = "cppyy-jit-fused-cpp"
    with pytest.raises(ValueError, match="execution model mismatch"):
        protocol.validate_document(document)
    document["results"][0]["backend"][
        "execution_model"] = "standalone-optimized-executable"
    document["benchmark"]["isolation"]["fresh_process_per_sample"] = False
    with pytest.raises(ValueError, match="requires process and build isolation"):
        protocol.validate_document(document)


def test_portable_schema_forbids_claims_and_requires_variant_evidence():
    schema_path = REPO_ROOT / "schemas" / "boundary-benchmark-v1.schema.json"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))

    benchmark = schema["properties"]["benchmark"]
    assert benchmark["properties"]["performance_claims_allowed"] == {
        "const": False}
    comparison = schema["properties"]["comparison"]
    assert comparison["properties"]["interpretation_allowed"] == {
        "const": False}
    result = schema["properties"]["results"]["items"]
    assert len(result["allOf"]) == 3


def test_all_variants_run_in_fresh_processes_with_verified_backend_and_parity(tmp_path):
    output = tmp_path / "boundary-smoke.json"
    command = [
        sys.executable,
        str(BENCH_DIR / "run_boundary_benchmark.py"),
        "--smoke",
        "--iterations", "250",
        "--output", str(output),
        "--json",
    ]
    process = subprocess.run(
        command,
        cwd=REPO_ROOT,
        env=os.environ.copy(),
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert process.returncode == 0, (
        f"stdout:\n{process.stdout}\nstderr:\n{process.stderr}")

    document = json.loads(process.stdout)
    assert document == json.loads(output.read_text(encoding="utf-8"))
    assert document["schema"] == "rclcppyy.boundary-benchmark/v1"
    assert document["benchmark"]["mode"] == "smoke"
    assert document["benchmark"]["performance_claims_allowed"] is False
    assert document["comparison"]["interpretation_allowed"] is False
    assert document["comparison"]["checksum_consistent"] is True
    assert document["failures"] == []
    assert {row["variant"] for row in document["results"]} == {
        "cppyy-python-boundary", "cppyy-fused", "aot-cpp"}
    assert len({row["pid"] for row in document["results"]}) == 3
    assert len({row["run_token"] for row in document["results"]}) == 3
    assert all(row["pid"] == row["process_group_id"] for row in document["results"])
    assert all(row["backend_verified"] for row in document["results"])
    callbacks = {
        row["variant"]: row["python_callback_count"]
        for row in document["results"]
    }
    assert callbacks == {
        "cppyy-python-boundary": 250,
        "cppyy-fused": 0,
        "aot-cpp": 0,
    }
