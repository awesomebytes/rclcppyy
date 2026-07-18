import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import scripts.ci.stress_runtime as stress_runtime
import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "ci" / "stress_runtime.py"


def test_scheduled_hardening_overrides_rmw_after_pixi_activation():
    workflow = yaml.safe_load(
        (ROOT / ".github" / "workflows" / "scheduled.yml").read_text())
    steps = workflow["jobs"]["native-safety"]["steps"]
    commands = "\n".join(step.get("run", "") for step in steps)
    assert 'pixi run env RMW_IMPLEMENTATION="$rmw" pytest' in commands
    assert 'pixi run env RMW_IMPLEMENTATION="$rmw" \\' in commands
    assert "python scripts/ci/stress_runtime.py" in commands
    assert 'RMW_IMPLEMENTATION="$rmw" pixi run' not in commands


def test_runtime_stress_emits_repeated_structured_evidence(tmp_path):
    output = tmp_path / "runtime-stress.json"
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        path for path in sys.path if path)
    environment["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
    environment["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--cycles", "2",
            "--threads", "4",
            "--messages-per-thread", "250",
            "--repetitions", "2",
            "--signal-repetitions", "5",
            "--seed", "314159",
            "--output", str(output),
        ],
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
        env=environment,
    )
    assert result.returncode == 0, result.stdout + "\n" + result.stderr
    assert "RUNTIME_STRESS_OK rounds=2" in result.stdout

    evidence = json.loads(output.read_text(encoding="utf-8"))
    assert evidence["schema"] == "rclcppyy.runtime-stress/v3"
    assert evidence["architecture"]
    assert evidence["rmw_implementation"]
    assert evidence["parameters"]["seed"] == 314159
    assert evidence["source"]["product"]["commit"]
    assert isinstance(evidence["source"]["product"]["dirty"], bool)
    assert evidence["source"]["suite"]["active_commit"] == (
        evidence["source"]["suite"]["locked_commit"])
    assert evidence["performance_claims_allowed"] is False
    assert [item["seed"] for item in evidence["rounds"]] == [314159, 314160]
    assert evidence["summary"]["rounds"] == 2
    assert evidence["summary"]["result"] == "pass"
    assert evidence["failures"] == []
    assert evidence["summary"]["entity_cycles"] == 4
    assert evidence["summary"]["messages_expected"] == 2000
    assert evidence["summary"]["messages_received"] == 2000
    assert evidence["summary"]["clean_signal_shutdowns"] == 10
    assert all(
        item["concurrent_publish"]["qos_depth"] == 1000
        for item in evidence["rounds"])
    assert evidence["summary"]["peak_rss_growth_kib"] >= 0
    assert not list(tmp_path.glob(".*.tmp"))


def test_runtime_stress_retains_partial_evidence_after_signal_failure(monkeypatch):
    monkeypatch.setattr(
        stress_runtime,
        "entity_churn",
        lambda cycles, seed: {"cycles": cycles, "peak_rss_growth_kib": 0},
    )
    monkeypatch.setattr(
        stress_runtime,
        "concurrent_publish",
        lambda threads, messages: {
            "threads": threads,
            "messages_per_thread": messages,
            "qos_depth": threads * messages,
            "messages_expected": threads * messages,
            "messages_received": threads * messages,
        },
    )

    def fail_signal(_timeout):
        raise AssertionError("captured signal timeout")

    monkeypatch.setattr(stress_runtime, "signal_shutdown", fail_signal)
    arguments = SimpleNamespace(
        cycles=2,
        threads=4,
        messages_per_thread=250,
        timeout=30.0,
        repetitions=2,
        signal_repetitions=5,
        min_duration_seconds=0.0,
        seed=17,
        max_rss_growth_kib=0,
    )

    evidence = stress_runtime.run_stress(
        arguments,
        enable_acceleration=lambda: None,
        rmw_identifier=lambda: "test_rmw",
    )

    assert evidence["summary"] == {
        "result": "fail",
        "rounds": 2,
        "entity_cycles": 4,
        "messages_expected": 2000,
        "messages_received": 2000,
        "clean_signal_shutdowns": 0,
        "failures": 1,
        "duration_s": evidence["summary"]["duration_s"],
        "peak_rss_growth_kib": evidence["summary"]["peak_rss_growth_kib"],
    }
    assert evidence["failures"] == [{
        "round": 0,
        "probe": "signal_shutdown",
        "exception_type": "AssertionError",
        "error": "captured signal timeout",
        "repetition": 0,
        "accelerated": True,
    }]
    assert evidence["rounds"][0]["signal_probe_disabled_after_failure"] is True
    assert evidence["rounds"][1]["signal_probe_skipped_after_failure"] is True


def test_signal_only_stress_retains_backend_specific_failure(monkeypatch):
    attempts = []

    def signal_probe(_timeout, accelerated):
        attempts.append(accelerated)
        if len(attempts) == 3:
            raise AssertionError("executor remained blocked")
        return {
            "returncode": 0,
            "accelerated": accelerated,
            "clean_marker": True,
            "duration_s": 0.1,
        }

    monkeypatch.setattr(stress_runtime, "signal_shutdown", signal_probe)
    evidence = stress_runtime.run_signal_stress(
        repetitions=20, timeout=30.0, accelerated=False)

    assert evidence["schema"] == "rclcppyy.signal-stress/v2"
    assert evidence["backend"] == "stock"
    assert evidence["performance_claims_allowed"] is False
    assert evidence["summary"]["result"] == "fail"
    assert evidence["summary"]["attempts"] == 3
    assert evidence["summary"]["clean_shutdowns"] == 2
    assert evidence["failures"] == [{
        "attempt": 2,
        "probe": "signal_shutdown",
        "exception_type": "AssertionError",
        "error": "executor remained blocked",
    }]
    assert attempts == [False, False, False]
