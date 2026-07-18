"""End-to-end evidence gates for the acceleration rewrite fixtures."""

import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parent.parent
SKILL = ROOT / "skills" / "rclcppyy-accelerate"
FIXTURES = SKILL / "fixtures" / "rewrite_projects"
SCANNER_PATH = SKILL / "scripts" / "scan_project.py"
PROTOCOL_PATH = FIXTURES / "evidence_protocol.py"


def _load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


scanner = _load_module("rclcppyy_fixture_scanner", SCANNER_PATH)
protocol = _load_module("rclcppyy_fixture_evidence", PROTOCOL_PATH)


@pytest.mark.parametrize(
    "project,selected_tier",
    [
        ("transparent_relay", 0),
        ("managed_native_worker", 2),
        ("fused_native_relay", 3),
    ],
)
def test_runnable_fixture_scan_has_correctness_and_benchmark_gates(
        project, selected_tier):
    result = scanner.scan(FIXTURES / project)

    assert result["parse_errors"] == []
    assert result["signals"]["evidence"]["scanner_only"] is False
    assert result["signals"]["evidence"]["blockers"] == []
    assert result["signals"]["evidence"]["test_files"] == ["test_behavior.py"]
    assert result["signals"]["evidence"]["benchmark_files"] == [
        "benchmark_rewrite.py"]
    assert selected_tier in {
        recommendation["tier"] for recommendation in result["recommendation_inputs"]}
    assert all(
        recommendation["evidence"]
        for recommendation in result["recommendation_inputs"]
    )


def test_case_protocol_rejects_unverified_backend_and_performance_claims():
    outputs = [payload + ":result" for payload in protocol.PAYLOADS]
    document = protocol.build_case(
        project="transparent_relay",
        variant="stock",
        tier=0,
        outputs=outputs,
        elapsed_ns=1000,
        cpu_time_ns=500,
        backend_roles={
            "node": {"backend": "python", "evidence": "unit-test marker"},
        },
        transform_crossings=len(outputs),
        api_coverage=["protocol unit test"],
        contract_delta=[],
        teardown_clean=True,
    )

    document["backend"]["verified"] = False
    with pytest.raises(ValueError, match="backend verification"):
        protocol.validate_case(document)

    document["backend"]["verified"] = True
    document["benchmark"]["performance_claims_allowed"] = True
    with pytest.raises(ValueError, match="cannot allow performance claims"):
        protocol.validate_case(document)


def test_all_rewrite_projects_emit_paired_bounded_evidence(tmp_path):
    output = tmp_path / "rewrite-evidence.json"
    process = subprocess.run(
        [sys.executable, str(PROTOCOL_PATH), "--output", str(output)],
        capture_output=True,
        text=True,
        timeout=360,
    )
    assert process.returncode == 0, (
        "stdout:\n%s\nstderr:\n%s" % (process.stdout, process.stderr))

    document = json.loads(output.read_text(encoding="utf-8"))
    protocol.validate_evidence(document)
    assert document["benchmark"]["mode"] == "smoke"
    assert document["benchmark"]["performance_claims_allowed"] is False
    assert document["benchmark"]["blocker"] == protocol.BENCHMARK_BLOCKER
    assert document["benchmark"]["repetitions"] == 1
    assert document["benchmark"]["workload"]["qos"]["depth"] == 10
    projects = {item["project"]: item for item in document["projects"]}
    assert set(projects) == set(protocol.PROJECTS)

    transparent = projects["transparent_relay"]["pairs"][0]
    assert transparent["before"]["backend"]["roles"]["publisher"]["backend"] == "python"
    assert transparent["after"]["backend"]["roles"]["publisher"]["backend"] == "cpp"
    assert transparent["after"]["contract_delta"] == []

    managed = projects["managed_native_worker"]["pairs"][0]
    assert managed["after"]["backend"]["roles"]["node"]["backend"] == "cpp"
    assert managed["after"]["python_boundary_crossings"]["count"] == len(
        protocol.PAYLOADS)
    assert managed["after"]["contract_delta"]

    fused = projects["fused_native_relay"]["pairs"][0]
    assert fused["before"]["python_boundary_crossings"]["count"] == len(
        protocol.PAYLOADS)
    assert fused["after"]["backend"]["roles"]["transform_callback"]["backend"] == "cpp"
    assert fused["after"]["python_boundary_crossings"]["count"] == 0
    assert fused["after"]["contract_delta"]

    for project in projects.values():
        pair = project["pairs"][0]
        assert pair["parity"]["outputs_equal"] is True
        assert pair["parity"]["checksums_equal"] is True
        assert pair["before"]["benchmark"]["raw_sample"]["elapsed_ns"] > 0
        assert pair["after"]["benchmark"]["raw_sample"]["elapsed_ns"] > 0
        assert pair["before"]["benchmark"]["raw_sample"]["cpu_time_ns"] > 0
        assert pair["after"]["benchmark"]["raw_sample"]["cpu_time_ns"] > 0
