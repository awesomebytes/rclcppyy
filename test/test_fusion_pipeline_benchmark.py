"""Focused contract coverage for the controlled C++ fusion benchmark."""

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
    spec = importlib.util.spec_from_file_location(name, BENCH_DIR / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


protocol = _load("_fusion_pipeline_protocol")
runner = _load("run_fusion_pipeline_benchmark")
worker = _load("fusion_pipeline_worker")
DIGEST = "a" * 64
RMW = "rmw_cyclonedds_cpp"


def _phase(cached):
    return {
        "schema": protocol.PREWARM_SCHEMA,
        "pid": 42 if not cached else 43,
        "loaded_rmw": RMW,
        "artifact": {
            "cached": cached,
            "reason": "hit" if cached else "miss-built",
            "path": "/fixture/fused.so",
            "sha256": DIGEST,
            "size_bytes": 4096,
            "source_id": "1" * 16,
        },
        "stdout_diagnostics": [],
    }


def _cache():
    return {
        "isolated_root": True,
        "autopch_disabled": True,
        "fresh_process_per_phase": True,
        "compilation_excluded_from_samples": True,
        "warm_hits_verified": True,
        "persisted_after_run": False,
        "phases": {"cold": _phase(False), "warm": _phase(True)},
    }


def test_four_stage_transform_matches_cppyy_body():
    assert [protocol.stage(index, 7) for index in range(4)] == [8, 21, 12, 93]
    assert protocol.fused(7) == 71
    assert protocol.expected_checksum(2, 3) == sum(
        protocol.fused(value) for value in (3, 4, 5))
    assert worker.TRANSFORM_BODY == (
        "output.data = (((input.data + 1ULL) * 3ULL) + 5ULL) ^ 0x5aULL;")


def test_cache_contract_requires_exact_cold_then_warm_artifact():
    protocol.validate_cache(_cache(), RMW)
    invalid = _cache()
    invalid["phases"]["warm"]["artifact"]["sha256"] = "b" * 64
    with pytest.raises(ValueError, match="different artifacts"):
        protocol.validate_cache(invalid, RMW)


def test_cpp_lanes_keep_generated_data_in_cpp_representation():
    worker_source = (BENCH_DIR / "fusion_pipeline_worker.py").read_text(encoding="utf-8")
    aot_source = (
        BENCH_DIR / "fusion_pipeline_aot" / "fusion_pipeline_aot.cpp"
    ).read_text(encoding="utf-8")
    kernel_source = (
        BENCH_DIR / "fusion_pipeline_kernel.hpp").read_text(encoding="utf-8")
    assert "std_msgs::msg::UInt64" in aot_source
    assert "Message::ConstSharedPtr" in aot_source
    assert "create_fused_pipeline" in worker_source
    assert '"python_message_conversions": 0' in worker_source
    assert "serialize_message" not in worker_source
    assert "from_py" not in worker_source
    assert "def on_message" not in worker_source
    assert "inline std::uint64_t fused" in kernel_source


def test_staged_baseline_and_ceiling_are_release_aot_and_composed():
    source = (
        BENCH_DIR / "fusion_pipeline_aot" / "fusion_pipeline_aot.cpp"
    ).read_text(encoding="utf-8")
    runner_source = (
        BENCH_DIR / "run_fusion_pipeline_benchmark.py").read_text(encoding="utf-8")
    assert "class StagedRelay" in source
    assert "class FusedRelay" in source
    assert "options.use_intra_process_comms(true)" in source
    assert "SingleThreadedExecutor" in source
    assert 'variant == "aot-staged"' in source
    assert 'variant == "aot-fused"' in source
    assert '"-DCMAKE_BUILD_TYPE=Release"' in runner_source
    assert '"-O3" not in compile_command' in runner_source
    assert '"-DNDEBUG" not in compile_command' in runner_source


def test_schema_requires_disabled_claims_and_cpp_representation():
    jsonschema = pytest.importorskip("jsonschema")
    schema = json.loads(
        (BENCH_DIR / "fusion_pipeline.schema.json").read_text(encoding="utf-8"))
    jsonschema.Draft202012Validator.check_schema(schema)
    assert schema["properties"]["benchmark"]["properties"][
        "performance_claims_allowed"] == {"const": False}
    ready = schema["properties"]["results"]["items"]["properties"]["relay_ready"]
    assert ready["properties"]["representation"] == {
        "const": "std_msgs::msg::UInt64"}
    assert ready["properties"]["python_message_conversions"] == {"const": 0}


def test_variant_parser_deduplicates_and_rejects_unknown_values():
    assert runner._parse_variants("cppyy-fused,aot-fused,cppyy-fused") == [
        "cppyy-fused", "aot-fused"]
    with pytest.raises(ValueError, match="unknown fusion variant"):
        runner._parse_variants("unknown")


@pytest.mark.skipif(
    os.environ.get("RCLCPPYY_RUN_LIVE_FUSION_SMOKE") != "1",
    reason="set RCLCPPYY_RUN_LIVE_FUSION_SMOKE=1 for the controlled live smoke",
)
def test_live_jazzy_cyclone_fusion_smoke(tmp_path):
    output = tmp_path / "fusion-smoke.json"
    env = os.environ.copy()
    env.update({
        "RMW_IMPLEMENTATION": RMW,
        "ROS_DISTRO": "jazzy",
    })
    completed = subprocess.run(
        [
            sys.executable,
            str(BENCH_DIR / "run_fusion_pipeline_benchmark.py"),
            "--smoke",
            "--output",
            str(output),
        ],
        cwd=REPO_ROOT,
        env=env,
        text=True,
        capture_output=True,
        timeout=240,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr + completed.stdout
    document = json.loads(output.read_text(encoding="utf-8"))
    protocol.validate_document(document)
    assert [row["variant"] for row in document["results"]] == list(protocol.VARIANTS)
    assert all(row["relay_ready"]["python_message_conversions"] == 0 for row in document["results"])
    assert all(row["relay_report"]["python_boundary_crossings"] == 0 for row in document["results"])
