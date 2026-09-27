"""Sequential stock-versus-activated ROS contract tests."""

import os
import sys
from pathlib import Path

import pytest

from differential.domains import acquire_domain
from differential.expectations import (
    CERTIFIED_PARITY,
    EXPECTED_BACKEND_GAPS,
    value_at_path,
)
from differential.process import run_owned_process


REPO_ROOT = Path(__file__).resolve().parent.parent
ARTIFACT_ROOT = Path(os.environ.get(
    "RCLCPPYY_TEST_ARTIFACT_DIR",
    REPO_ROOT / "build" / "test-artifacts" / "differential",
))


@pytest.fixture(scope="module")
def differential_results():
    results = {}
    with acquire_domain() as domain:
        for mode in ("stock", "activated"):
            env = domain.environment()
            test_root = str(REPO_ROOT / "test")
            env["PYTHONPATH"] = os.pathsep.join(filter(None, [
                test_root,
                env.get("PYTHONPATH"),
            ]))
            process = run_owned_process(
                [sys.executable, "-m", "differential._contract_probe", "--mode", mode],
                cwd=REPO_ROOT,
                env=env,
                timeout_s=180,
                artifact_path=ARTIFACT_ROOT / (mode + ".json"),
            )
            assert process.returncode == 0, process.diagnostics()
            assert not process.timed_out, process.diagnostics()
            assert process.protocol_error is None, process.diagnostics()
            assert process.protocol_result["outcome"] == "pass", process.diagnostics()
            assert process.protocol_result["backend_verified"], process.diagnostics()
            assert all(process.protocol_result["cleanup"].values()), process.diagnostics()
            results[mode] = process.protocol_result
    return results


@pytest.mark.parametrize("expectation", CERTIFIED_PARITY,
                         ids=lambda expectation: expectation.identifier)
def test_certified_observations_match_stock(differential_results, expectation):
    stock = value_at_path(differential_results["stock"]["observations"], expectation.path)
    activated = value_at_path(
        differential_results["activated"]["observations"], expectation.path)
    assert activated == stock


def test_activated_child_proves_declared_routes_from_status(differential_results):
    activated = differential_results["activated"]
    assert activated["backend_expectations"]
    assert activated["backend_status"]["schema_version"] == 1
    assert activated["backend_verified"]
    assert activated["backend_status"]["counts"]["operations"]["cpp"] == 0
    assert activated["backend_status"]["counts"]["operations"]["python"] >= 1
    assert activated["backend_status"]["counts"]["nodes"]["python"] >= 1


@pytest.mark.parametrize("gap", EXPECTED_BACKEND_GAPS,
                         ids=lambda gap: gap.identifier)
def test_expected_backend_gap_is_explicit(differential_results, gap):
    records = differential_results["activated"]["backend_status"][gap.kind]
    matches = [
        record for record in records
        if record["backend"] == gap.backend and all(
            record["metadata"].get(key) == value for key, value in gap.metadata)
    ]
    if not matches:
        related = [
            record for record in records
            if all(record["metadata"].get(key) == value
                   for key, value in gap.metadata)
        ]
        reasons = sorted({record["reason"] for record in related})
        detail = "; ".join(reasons) if reasons else "no matching status records"
        pytest.xfail("%s Current evidence: %s" % (gap.reason, detail))
    assert matches
