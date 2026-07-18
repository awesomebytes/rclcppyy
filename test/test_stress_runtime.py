import json
import os
from pathlib import Path
import subprocess
import sys

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
            "--signal-repetitions", "2",
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
    assert evidence["schema"] == "rclcppyy.runtime-stress/v1"
    assert evidence["architecture"]
    assert evidence["rmw_implementation"]
    assert evidence["parameters"]["seed"] == 314159
    assert [item["seed"] for item in evidence["rounds"]] == [314159, 314160]
    assert evidence["summary"]["rounds"] == 2
    assert evidence["summary"]["entity_cycles"] == 4
    assert evidence["summary"]["messages_expected"] == 2000
    assert evidence["summary"]["messages_received"] == 2000
    assert evidence["summary"]["clean_signal_shutdowns"] == 4
    assert all(
        item["concurrent_publish"]["qos_depth"] == 1000
        for item in evidence["rounds"])
    assert evidence["summary"]["peak_rss_growth_kib"] >= 0
    assert not list(tmp_path.glob(".*.tmp"))
