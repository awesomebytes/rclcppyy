import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parent.parent
SKILL = ROOT / "skills" / "rclcppyy-accelerate"
SCANNER = SKILL / "scripts" / "scan_project.py"
FIXTURE = SKILL / "fixtures" / "tutorial_node"
SPEC = importlib.util.spec_from_file_location("rclcppyy_scan_project", SCANNER)
scanner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(scanner)


def _by_method(result, method):
    return next(
        item for item in result["observations"]["entities"]
        if item["method"] == method)


def test_scanner_inventory_and_recommendation_inputs_are_evidence_backed():
    first = scanner.scan(FIXTURE)
    second = scanner.scan(FIXTURE)

    assert first == second
    assert first["schema"] == "rclcppyy.project-scan/v2"
    assert first["root"] == "."
    assert str(FIXTURE) not in json.dumps(first, sort_keys=True)
    assert first["files_scanned"] == 2
    assert first["parse_errors"] == []
    assert first["observations"]["nodes"] == [{
        "file": "talker.py",
        "line": 12,
        "kind": "subclass",
        "class_name": "CameraRelay",
        "bases": ["Node"],
        "name": "camera_relay",
    }]
    assert {item["method"] for item in first["observations"]["entities"]} == {
        "create_publisher", "create_subscription",
    }
    assert _by_method(first, "create_publisher")["qos"] == {
        "expression": "10", "value": 10}
    subscription = _by_method(first, "create_subscription")
    assert subscription["qos"] == {"expression": "10", "value": 10}
    assert subscription["callback_group"] == {"expression": "self.group"}
    assert first["observations"]["executors"][0]["inputs"]["num_threads"] == {
        "expression": "2", "value": 2}

    callback = first["observations"]["callbacks"][0]
    assert callback["definition_line"] == 20
    assert callback["has_loop"] is True
    assert callback["ast_nodes"] > 0
    assert callback["message_type"] == "Image"
    assert callback["native_library_uses"] == []
    assert first["signals"]["known_large_messages"] == ["Image"]
    assert first["signals"]["native_library_imports"] == [
        {"import": "cv2", "capability": "opencv"},
    ]
    assert first["signals"]["callback_native_library_uses"] == []
    assert first["signals"]["launch_files"] == ["relay.launch.py"]

    evidence = first["signals"]["evidence"]
    assert evidence["scanner_only"] is True
    assert evidence["test_files"] == []
    assert evidence["benchmark_files"] == []
    assert len(evidence["blockers"]) == 3
    assert [item["tier"] for item in first["recommendation_inputs"]] == [0, 2, 3]
    assert all(item["confidence"] in ("high", "medium", "low")
               for item in first["recommendation_inputs"])
    assert all(item["evidence"] for item in first["recommendation_inputs"])
    assert all(item["blockers"] for item in first["recommendation_inputs"])


def test_scanner_output_is_relocation_stable(tmp_path):
    first_root = tmp_path / "first" / "project"
    second_root = tmp_path / "second" / "project"
    shutil.copytree(FIXTURE, first_root)
    shutil.copytree(FIXTURE, second_root)

    assert scanner.scan(first_root) == scanner.scan(second_root)


def test_domain_tier_requires_callback_level_native_library_use(tmp_path):
    source = tmp_path / "native_relay.py"
    source.write_text(
        "import cv2\n"
        "from rclpy.node import Node\n"
        "from sensor_msgs.msg import Image\n"
        "class NativeRelay(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('native_relay')\n"
        "        self.pub = self.create_publisher(Image, 'out', 10)\n"
        "        self.sub = self.create_subscription(Image, 'in', self.convert, 10)\n"
        "    def convert(self, message):\n"
        "        cv2.mean(message.data)\n"
        "        self.pub.publish(message)\n",
        encoding="utf-8",
    )

    result = scanner.scan(tmp_path)

    assert [item["tier"] for item in result["recommendation_inputs"]] == [0, 3, 4]
    domain = result["signals"]["callback_native_library_uses"]
    assert len(domain) == 1
    assert domain[0]["callback"] == "self.convert"
    assert domain[0]["native_library_uses"] == [{
        "binding": "cv2", "import": "cv2", "capability": "opencv"}]
    tier = result["recommendation_inputs"][-1]
    assert tier["confidence"] == "low"
    assert tier["evidence"][0]["kind"] == "callback_native_library_use"
    assert any("unmeasured" in blocker for blocker in tier["blockers"])


def test_scanner_cli_writes_strict_json(tmp_path):
    output = tmp_path / "nested" / "scan.json"
    proc = subprocess.run(
        [sys.executable, str(SCANNER), str(FIXTURE), "--strict", "--output", str(output)],
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert proc.returncode == 0, proc.stderr
    assert proc.stdout == ""
    result = json.loads(output.read_text(encoding="utf-8"))
    assert result["schema"] == "rclcppyy.project-scan/v2"
    assert result["warnings"]


def test_strict_cli_preserves_incomplete_scan_and_fails(tmp_path):
    (tmp_path / "broken.py").write_text("def broken(:\n", encoding="utf-8")
    output = tmp_path / "scan.json"

    proc = subprocess.run(
        [sys.executable, str(SCANNER), str(tmp_path), "--strict", "--output", str(output)],
        capture_output=True,
        text=True,
        timeout=10,
    )

    assert proc.returncode == 2
    assert "scan is incomplete" in proc.stderr
    result = json.loads(output.read_text(encoding="utf-8"))
    assert result["parse_errors"]
    assert any(
        blocker.startswith("scan contains parse errors")
        for blocker in result["signals"]["evidence"]["blockers"])
