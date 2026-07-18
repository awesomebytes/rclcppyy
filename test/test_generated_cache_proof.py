import json
import os
from pathlib import Path
import subprocess
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "ci" / "prove_generated_cache.py"


def test_generated_rclcpp_factory_cache_is_cold_then_warm(tmp_path):
    cache_root = tmp_path / "isolated-cache"
    output = tmp_path / "evidence.json"
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        path for path in sys.path if path)
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--cache-root",
            str(cache_root),
            "--output",
            str(output),
        ],
        capture_output=True,
        text=True,
        timeout=180,
        check=False,
        env=environment,
    )
    assert result.returncode == 0, result.stdout + "\n" + result.stderr
    assert "GENERATED_CACHE_OK cold=miss-built warm=hit" in result.stdout

    evidence = json.loads(output.read_text(encoding="utf-8"))
    assert evidence["schema"] == "rclcppyy.generated-cache-proof/v1"
    assert evidence["policy"] == {
        "isolated": True,
        "autopch_disabled": True,
        "fresh_process_per_phase": True,
    }
    assert evidence["phases"]["cold"]["result"]["cached"] is False
    assert evidence["phases"]["cold"]["result"]["reason"] == "miss-built"
    assert evidence["phases"]["warm"]["result"]["cached"] is True
    assert len(evidence["artifact"]["sha256"]) == 64
    assert evidence["artifact"]["size_bytes"] > 0


def test_review_and_scheduled_workflows_enforce_cache_and_default_policy():
    owners = (ROOT / ".github" / "CODEOWNERS").read_text(encoding="utf-8")
    assert "/rclcppyy/" in owners

    workflow = yaml.safe_load(
        (ROOT / ".github" / "workflows" / "scheduled.yml").read_text(
            encoding="utf-8"))
    safety_commands = "\n".join(
        step.get("run", "") for step in workflow["jobs"]["native-safety"]["steps"])
    assert "prove_generated_cache.py" in safety_commands
    assert "phase0_same_handle_publisher.py" not in safety_commands

    benchmark = workflow["jobs"]["dedicated-benchmark"]
    assert benchmark["env"]["CPPYY_KIT_NO_AUTOPCH"] == "1"
    assert "XDG_CACHE_HOME" in benchmark["env"]
    benchmark_commands = "\n".join(
        step.get("run", "") for step in benchmark["steps"])
    assert "prove_generated_cache.py" in benchmark_commands
