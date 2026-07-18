"""Contract tests for the machine-readable compatibility inventory."""

import importlib.util
import json
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parent.parent
CHECKER_PATH = REPO_ROOT / "scripts" / "check_compatibility_manifest.py"
MANIFEST_PATH = REPO_ROOT / "compatibility" / "jazzy.json"
SPEC = importlib.util.spec_from_file_location("rclcppyy_manifest_checker", CHECKER_PATH)
checker = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(checker)


def test_jazzy_manifest_is_valid_and_inventory_is_explicit():
    summary = checker.load_and_validate(MANIFEST_PATH, REPO_ROOT)

    assert summary["schema"] == "rclcppyy.compatibility/v1"
    assert summary["ros_distribution"] == "jazzy"
    assert summary["entries"] >= 20
    assert summary["support"]["certified"] >= 8
    assert summary["support"]["unsupported"] >= 1
    assert set(summary["areas"]) >= {
        "activation", "node", "message", "publisher", "subscription",
        "timer", "executor", "service", "action", "parameter", "lifecycle",
        "context", "serialization",
    }


def test_every_certified_entry_references_existing_evidence():
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))

    for entry in manifest["entries"]:
        if entry["support"] != "certified":
            continue
        assert entry["evidence"], entry["id"]
        assert all((REPO_ROOT / path).exists() for path in entry["evidence"]), entry["id"]


def test_duplicate_ids_are_rejected():
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    manifest["entries"].append(dict(manifest["entries"][0]))

    with pytest.raises(checker.ManifestError, match="duplicate entry id"):
        checker.validate(manifest, REPO_ROOT)


def test_certified_unsafe_fallback_is_rejected():
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    manifest["entries"][0]["fallback"] = "unsafe"

    with pytest.raises(checker.ManifestError, match="unsafe fallback"):
        checker.validate(manifest, REPO_ROOT)
