import importlib.util
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parent.parent
SKILL = ROOT / "skills" / "rclcppyy-accelerate"
SCANNER = SKILL / "scripts" / "scan_project.py"
FIXTURE = SKILL / "fixtures" / "tutorial_node"
SPEC = importlib.util.spec_from_file_location("rclcppyy_scan_project", SCANNER)
scanner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(scanner)


def test_scanner_inventory_and_recommendation_inputs_are_deterministic():
    first = scanner.scan(FIXTURE)
    second = scanner.scan(FIXTURE)
    assert first == second
    assert first["schema"] == "rclcppyy.project-scan/v1"
    assert first["files_scanned"] == 2
    assert first["parse_errors"] == []
    assert {item["method"] for item in first["observations"]["entities"]} == {
        "create_publisher", "create_subscription",
    }
    assert first["signals"]["known_large_messages"] == ["Image"]
    assert first["signals"]["native_library_imports"] == [
        {"import": "cv2", "capability": "opencv"},
    ]
    assert first["signals"]["launch_files"] == ["relay.launch.py"]
    assert [item["tier"] for item in first["recommendation_inputs"]] == [0, 2, 3, 4]


def test_scanner_cli_writes_strict_json(tmp_path):
    output = tmp_path / "nested" / "scan.json"
    proc = subprocess.run(
        [sys.executable, str(SCANNER), str(FIXTURE), "--output", str(output)],
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert proc.returncode == 0, proc.stderr
    assert proc.stdout == ""
    result = json.loads(output.read_text(encoding="utf-8"))
    assert result["schema"] == "rclcppyy.project-scan/v1"
    assert result["warnings"]
