"""Sequential differential application corpus with reviewed limitations."""

import json
import os
import sys
from pathlib import Path

import pytest

from differential.application_expectations import APPLICATION_PARITY
from differential.domains import acquire_domain
from differential.expectations import value_at_path
from differential.process import run_owned_process


REPO_ROOT = Path(__file__).resolve().parent.parent
ARTIFACT_ROOT = Path(os.environ.get(
    "RCLCPPYY_TEST_ARTIFACT_DIR",
    REPO_ROOT / "build" / "test-artifacts" / "differential",
))
LIMITATIONS_PATH = REPO_ROOT / "test" / "differential" / "application_limitations.json"


def _load_limitations():
    return json.loads(LIMITATIONS_PATH.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def application_results():
    results = {}
    with acquire_domain() as domain:
        for mode in ("stock", "activated"):
            env = domain.environment()
            test_root = str(REPO_ROOT / "test")
            env["PYTHONPATH"] = os.pathsep.join(filter(None, [
                test_root,
                env.get("PYTHONPATH"),
            ]))
            artifact = ARTIFACT_ROOT / ("application-" + mode + ".json")
            process = run_owned_process(
                [sys.executable, "-m", "differential._application_probe", "--mode", mode],
                cwd=REPO_ROOT,
                env=env,
                timeout_s=180,
                artifact_path=artifact,
            )
            details = "artifact=%s\n%s" % (artifact, process.diagnostics())
            assert process.returncode == 0, details
            assert not process.timed_out, details
            assert process.protocol_error is None, details
            assert process.protocol_result["outcome"] == "pass", details
            assert process.protocol_result["backend_verified"], details
            assert all(process.protocol_result["cleanup"].values()), details
            results[mode] = process.protocol_result
    return results


@pytest.mark.parametrize("expectation", APPLICATION_PARITY,
                         ids=lambda expectation: expectation.identifier)
def test_application_behavior_matches_stock(application_results, expectation):
    stock = value_at_path(application_results["stock"]["observations"], expectation.path)
    activated = value_at_path(
        application_results["activated"]["observations"], expectation.path)
    assert activated == stock


def test_application_backend_evidence_is_self_consistent(application_results):
    activated = application_results["activated"]
    assert activated["backend_verified"]
    status = activated["backend_status"]
    assert status["counts"]["entities"]["cpp"] == 0
    assert status["counts"]["entities"]["python"] >= 1
    assert status["counts"]["nodes"]["python"] >= 1
    raw_subscriptions = [
        record for record in status["entities"]
        if record["metadata"].get("entity_type") == "subscription"
        and record["metadata"].get("raw_requested")
    ]
    assert len(raw_subscriptions) == 1
    assert raw_subscriptions[0]["metadata"]["topic"] == "raw_topic"
    event_publishers = [
        record for record in status["entities"]
        if record["metadata"].get("entity_type") == "publisher"
        and record["metadata"].get("topic") == "/incompatible_qos_topic"
    ]
    assert len(event_publishers) == 1
    assert event_publishers[0]["metadata"]["event_callbacks_requested"] is True


def test_reviewed_limitations_are_well_formed():
    limitations = _load_limitations()
    assert limitations["schema"] == "rclcppyy.differential-limitations/v1"
    assert limitations["reviewed_at"]
    entries = limitations["entries"]
    assert len({entry["id"] for entry in entries}) == len(entries)
    assert {entry["coverage"] for entry in entries} <= {"probed", "excluded"}
    assert {entry["check"] for entry in entries} <= {"status_record", "none"}
    assert all(entry["reason"] for entry in entries)


STATUS_LIMITATIONS = [
    entry for entry in _load_limitations()["entries"]
    if entry["check"] == "status_record"
]


@pytest.mark.parametrize("limitation", STATUS_LIMITATIONS,
                         ids=lambda limitation: limitation["id"])
def test_backend_evidence_limitation_is_explicit(application_results, limitation):
    expected = limitation["expected"]
    records = application_results["activated"]["backend_status"][expected["kind"]]
    matches = [
        record for record in records
        if record["backend"] == expected["backend"] and all(
            record["metadata"].get(key) == value
            for key, value in expected["metadata"].items())
    ]
    if not matches:
        pytest.xfail(limitation["reason"])
    assert matches
