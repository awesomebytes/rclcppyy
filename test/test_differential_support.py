"""Unit tests for the differential subprocess foundation."""

import json
import os
import sys

import pytest

from differential.domains import acquire_domain
from differential.process import run_owned_process
from differential.protocol import (
    ProtocolError,
    encode_result,
    parse_result_lines,
    verify_backend_expectations,
)


def _result(mode="stock"):
    return {
        "schema": "rclcppyy.differential/v1",
        "scenario": "unit_test",
        "mode": mode,
        "outcome": "pass",
        "observations": {},
        "backend_expectations": [],
        "backend_status": None,
        "backend_verified": True,
        "cleanup": {},
        "error": None,
    }


def test_result_protocol_round_trip_is_strict_json():
    encoded = encode_result(_result())
    assert parse_result_lines("diagnostic\n" + encoded + "\n") == _result()
    json.loads(encoded.partition("=")[2], parse_constant=lambda value: pytest.fail(value))


def test_result_protocol_rejects_extra_keys_and_duplicate_markers():
    invalid = _result()
    invalid["unexpected"] = True
    with pytest.raises(ProtocolError, match="extra"):
        encode_result(invalid)
    marker = encode_result(_result())
    with pytest.raises(ProtocolError, match="exactly one"):
        parse_result_lines(marker + "\n" + marker)


def test_backend_expectations_filter_status_records():
    status = {
        "schema_version": 1,
        "nodes": [],
        "entities": [],
        "operations": [{
            "backend": "cpp",
            "metadata": {"operation": "create_node", "detail": "retained"},
        }],
    }
    expected = [{
        "kind": "operations",
        "backend": "cpp",
        "minimum": 1,
        "metadata": {"operation": "create_node"},
    }]
    assert verify_backend_expectations(status, expected) == []
    expected[0]["minimum"] = 2
    assert verify_backend_expectations(status, expected) == [{
        "expectation": expected[0],
        "observed": 1,
    }]


def test_result_protocol_recomputes_activated_backend_evidence():
    result = _result("activated")
    result["backend_expectations"] = [{
        "kind": "operations",
        "backend": "cpp",
        "minimum": 1,
        "metadata": {"operation": "enable_cpp_acceleration"},
    }]
    result["backend_status"] = {
        "schema_version": 1,
        "nodes": [],
        "entities": [],
        "operations": [{
            "backend": "python",
            "metadata": {"operation": "enable_cpp_acceleration"},
        }],
    }
    with pytest.raises(ProtocolError, match="disagrees"):
        encode_result(result)


def test_domain_leases_are_unique_and_supply_child_environment(tmp_path):
    with acquire_domain(tmp_path) as first, acquire_domain(tmp_path) as second:
        assert first.domain_id != second.domain_id
        assert first.environment({"BASE": "1"}) == {
            "BASE": "1",
            "ROS_DOMAIN_ID": str(first.domain_id),
        }
        assert first.lock_file.exists()


def test_owned_process_has_its_own_group_and_writes_diagnostics(tmp_path):
    artifact = tmp_path / "owned.json"
    code = "import os,sys; print('%d %d' % (os.getpid(), os.getpgrp())); print('err', file=sys.stderr)"
    result = run_owned_process(
        [sys.executable, "-c", code],
        timeout_s=5,
        require_protocol=False,
        artifact_path=artifact,
    )
    child_pid, child_group = [int(value) for value in result.stdout.split()]
    assert result.returncode == 0
    assert child_pid == child_group
    assert child_group != os.getpgrp()
    saved = json.loads(artifact.read_text(encoding="utf-8"))
    assert saved["schema"] == "rclcppyy.subprocess-artifact/v1"
    assert saved["stdout"] == result.stdout
    assert saved["stderr"] == "err\n"


def test_owned_process_times_out_and_kills_descendant_group():
    grandchild = (
        "import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); "
        "time.sleep(60)"
    )
    child = (
        "import subprocess,sys,time; "
        "subprocess.Popen([sys.executable, '-c', %r]); time.sleep(60)" % grandchild
    )
    result = run_owned_process(
        [sys.executable, "-c", child],
        timeout_s=0.2,
        terminate_grace_s=0.2,
        require_protocol=False,
    )
    assert result.timed_out
    assert result.duration_s < 5
    assert result.termination_signals == ["SIGTERM", "SIGKILL"]
