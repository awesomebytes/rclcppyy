import copy
import importlib.util
from pathlib import Path
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "benchmarks" / "render_report.py"
sys.path.insert(0, str(SCRIPT.parent))
spec = importlib.util.spec_from_file_location("benchmark_report", SCRIPT)
report_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(report_module)


def _document(mode="smoke"):
    return {
        "schema": "rclcppyy.benchmark/v3",
        "generated_at": "2026-07-18T12:00:00Z",
        "command": ["pixi", "run", "bench", "--smoke"],
        "environment": {
            "source": {"commit": "abc123", "dirty": False},
            "host": {"architecture": "x86_64", "cpu_model": "Fixture CPU"},
            "ros": {"distribution": "jazzy", "rmw_implementation": "rmw_fixture"},
        },
        "benchmark": {
            "name": "fixture",
            "mode": mode,
            "performance_claims_allowed": False,
            "matrix": {},
            "statistics": {"latency": "nearest-rank", "cpu": "sample mean"},
        },
        "results": [{
            "case_id": "case",
            "backend": "rclcppyy",
            "workload": "small-string",
            "target_rate_hz": 1000,
            "payload_bytes": 0,
            "backend_verified": True,
            "expected_backends": {"publisher": "cpp", "subscriber": "python"},
            "messages": {"received": 100, "dropped": 0, "effective_rate_hz": 999.5},
            "wire_values": {
                "schema": "rclcppyy.benchmark-wire-values/v1",
                "contract_id": "std_msgs/String:sequence-timestamp-padding/v1",
                "expected_payload_bytes": 0,
                "checked_messages": 100,
                "violations": 0,
                "violation_types": {},
                "value_contract_verified": True,
            },
            "latency_us": {
                "count": 100, "mean": 10.0, "p50": 9.0, "p95": 15.0,
                "p99": 20.0, "min": 5.0, "max": 25.0,
            },
            "cpu_pct": {
                "publisher": {"mean": 3.5},
                "subscriber": {"mean": 4.5},
            },
        }],
        "failures": [],
    }


def test_smoke_report_forbids_claims_and_shows_backend_route():
    first = report_module.render(_document())
    reordered = copy.deepcopy(_document())
    reordered["benchmark"]["statistics"] = {
        "cpu": "sample mean", "latency": "nearest-rank"}
    second = report_module.render(reordered)
    assert first == second
    assert "Smoke artifact. Performance claims are forbidden" in first
    assert "cpp -> python" in first
    assert "verified: std_msgs/String:sequence-timestamp-padding/v1" in first
    assert "| 100 | 0 | 999.500 |" in first


def test_reproduction_command_is_shell_quoted():
    document = _document()
    document["command"].append("value with spaces")
    assert "'value with spaces'" in report_module.render(document)


def test_measurement_report_does_not_declare_a_winner():
    report = report_module.render(_document(mode="measurement"))
    assert "Characterization artifact. Performance claims are forbidden" in report
    assert "winner" not in report.lower()


def test_invalid_smoke_claim_is_rejected():
    document = _document()
    document["benchmark"]["performance_claims_allowed"] = True
    with pytest.raises(ValueError, match="smoke results"):
        report_module.render(document)
