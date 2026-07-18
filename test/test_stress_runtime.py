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


def test_scheduled_hardening_is_independent_and_evidence_based():
    workflow = yaml.safe_load(
        (ROOT / ".github" / "workflows" / "scheduled.yml").read_text())
    job = workflow["jobs"]["native-safety"]
    matrix = job["strategy"]["matrix"]["include"]
    assert {
        (item["platform"], item["machine"], item["rmw"])
        for item in matrix
    } == {
        ("linux-64", "x86_64", "rmw_cyclonedds_cpp"),
        ("linux-64", "x86_64", "rmw_fastrtps_cpp"),
        ("linux-aarch64", "aarch64", "rmw_cyclonedds_cpp"),
        ("linux-aarch64", "aarch64", "rmw_fastrtps_cpp"),
    }
    steps = job["steps"]
    commands = "\n".join(step.get("run", "") for step in steps)
    assert 'pixi run env RMW_IMPLEMENTATION="${{ matrix.rmw }}" pytest' in commands
    assert 'RMW_IMPLEMENTATION="${{ matrix.rmw }}" pixi run' not in commands
    assert "for rmw in" not in commands
    assert "python scripts/ci/stress_runtime.py" in commands
    assert "--profile publisher_cpp" in commands
    assert "--round-period-seconds 15" in commands
    assert "--repetitions 3 --signal-repetitions 0" in commands
    assert "--profile stock --cycles 100 --repetitions 50" in commands
    assert "--profile compatible --cycles 100 --repetitions 50" in commands
    assert "compare_stress_memory.py" in commands
    assert "--profile optimized" in commands
    assert "--signal-settle-seconds 0 --signal-repetitions 50" in commands

    stock_diagnostic = next(
        step for step in steps
        if step.get("name") == "Record stock immediate signal boundary diagnostic")
    assert stock_diagnostic["continue-on-error"] is True
    compatible_diagnostic = next(
        step for step in steps
        if step.get("name") == "Record compatible settled signal diagnostic")
    assert compatible_diagnostic["continue-on-error"] is True
    assert "--signal-settle-seconds 0.05" in compatible_diagnostic["run"]
    cache = next(
        step for step in steps
        if step.get("name") == "Prove isolated cold, warm, and corrupt generated caches")
    assert cache["if"] == (
        "${{ always() && matrix.rmw == 'rmw_cyclonedds_cpp' }}")
    upload = next(
        step for step in steps
        if step.get("name") == "Upload scheduled evidence")
    assert upload["if"] == "always()"
    assert upload["with"]["name"] == (
        "scheduled-${{ matrix.platform }}-${{ matrix.rmw_label }}")


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
    assert evidence["schema"] == "rclcppyy.runtime-stress/v4"
    assert evidence["architecture"]
    assert evidence["rmw_implementation"]
    assert evidence["profile"] == "compatible"
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
    assert isinstance(evidence["summary"]["current_rss_growth_kib"], int)
    assert evidence["memory"]["warmup_rounds_excluded"] == 1
    assert evidence["memory"]["post_warmup_entity_cycles"] == 2
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

    def fail_signal(_timeout, profile, settle_seconds):
        assert profile == "compatible"
        assert settle_seconds == 0.05
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
        round_period_seconds=0.0,
        signal_settle_seconds=0.05,
        seed=17,
        max_rss_growth_kib=0,
        profile="compatible",
    )

    evidence = stress_runtime.run_stress(
        arguments,
        configure_profile=lambda profile: None,
        rmw_identifier=lambda: "test_rmw",
    )

    assert evidence["summary"]["result"] == "fail"
    assert evidence["summary"]["rounds"] == 2
    assert evidence["summary"]["entity_cycles"] == 4
    assert evidence["summary"]["messages_expected"] == 2000
    assert evidence["summary"]["messages_received"] == 2000
    assert evidence["summary"]["clean_signal_shutdowns"] == 0
    assert evidence["summary"]["failures"] == 1
    assert evidence["summary"]["active_duration_s"] >= 0
    assert evidence["summary"]["sleep_s"] == 0
    assert evidence["failures"] == [{
        "round": 0,
        "probe": "signal_shutdown",
        "exception_type": "AssertionError",
        "error": "captured signal timeout",
        "repetition": 0,
        "profile": "compatible",
    }]
    assert evidence["rounds"][0]["signal_probe_disabled_after_failure"] is True
    assert evidence["rounds"][1]["signal_probe_skipped_after_failure"] is True


def test_duration_soak_is_paced_and_reports_post_warm_memory(monkeypatch):
    clock = [0.0]

    def advance(seconds):
        clock[0] += seconds

    def churn(cycles, seed):
        del seed
        advance(0.01)
        return {"cycles": cycles, "peak_rss_growth_kib": 0}

    def publish(threads, messages):
        advance(0.01)
        return {
            "threads": threads,
            "messages_per_thread": messages,
            "qos_depth": threads * messages,
            "messages_expected": threads * messages,
            "messages_received": threads * messages,
        }

    snapshots = iter([
        {"current_rss_kib": 100, "anonymous_rss_kib": 80, "peak_rss_kib": 100},
        {"current_rss_kib": 101, "anonymous_rss_kib": 81, "peak_rss_kib": 101},
        {"current_rss_kib": 102, "anonymous_rss_kib": 82, "peak_rss_kib": 102},
        {"current_rss_kib": 103, "anonymous_rss_kib": 83, "peak_rss_kib": 103},
    ])
    monkeypatch.setattr(stress_runtime, "entity_churn", churn)
    monkeypatch.setattr(stress_runtime, "concurrent_publish", publish)
    monkeypatch.setattr(stress_runtime, "_memory_kib", lambda: next(snapshots))
    monkeypatch.setattr(stress_runtime, "_source_metadata", lambda: {})
    monkeypatch.setattr(stress_runtime, "_rss_kib", lambda: 100)
    monkeypatch.setattr(stress_runtime.gc, "collect", lambda: None)
    monkeypatch.setattr(stress_runtime.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(stress_runtime.time, "sleep", advance)
    arguments = SimpleNamespace(
        cycles=2,
        threads=4,
        messages_per_thread=10,
        timeout=30.0,
        repetitions=1,
        signal_repetitions=0,
        min_duration_seconds=0.25,
        round_period_seconds=0.1,
        signal_settle_seconds=0.05,
        seed=17,
        max_rss_growth_kib=0,
        profile="stock",
    )

    evidence = stress_runtime.run_stress(
        arguments,
        configure_profile=lambda profile: None,
        rmw_identifier=lambda: "test_rmw",
    )

    assert evidence["summary"]["rounds"] == 3
    assert evidence["summary"]["entity_cycles"] == 6
    assert evidence["summary"]["duration_s"] == 0.25
    assert evidence["summary"]["active_duration_s"] == 0.06
    assert evidence["summary"]["sleep_s"] == 0.19
    assert evidence["memory"]["post_warmup_entity_cycles"] == 4
    assert (
        evidence["memory"]["post_warmup_anonymous_rss_kib_per_1000_cycles"]
        == 500.0
    )


def test_signal_only_stress_retains_backend_specific_failure(monkeypatch):
    attempts = []

    def signal_probe(_timeout, profile, settle_seconds):
        attempts.append((profile, settle_seconds))
        if len(attempts) == 3:
            raise AssertionError("executor remained blocked")
        return {
            "returncode": 0,
            "profile": profile,
            "accelerated": profile != "stock",
            "settle_seconds": settle_seconds,
            "clean_marker": True,
            "duration_s": 0.1,
        }

    monkeypatch.setattr(stress_runtime, "signal_shutdown", signal_probe)
    evidence = stress_runtime.run_signal_stress(
        repetitions=20, timeout=30.0, profile="stock", settle_seconds=0.0)

    assert evidence["schema"] == "rclcppyy.signal-stress/v3"
    assert evidence["profile"] == "stock"
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
    assert attempts == [("stock", 0.0)] * 3
