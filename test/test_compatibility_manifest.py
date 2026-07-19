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


def _manifest():
    return json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))


def test_jazzy_manifest_is_valid_and_inventory_is_explicit():
    summary = checker.load_and_validate(MANIFEST_PATH, REPO_ROOT)

    assert summary["schema"] == "rclcppyy.compatibility/v2"
    assert summary["ros_distribution"] == "jazzy"
    assert summary["entries"] == 70
    assert summary["support"] == {
        "certified": 36,
        "experimental": 15,
        "stock_authoritative": 16,
        "unassessed": 3,
    }
    assert summary["backend"] == {
        "cpp": 13,
        "mixed": 8,
        "none": 3,
        "python": 46,
    }
    assert summary["upstream_contract"] == {
        "manifest": "compatibility/upstream-rclpy-contract.json",
        "selected": 49,
        "reviewed_exclusions": 3,
        "total": 52,
    }
    assert set(summary["areas"]) >= {
        "activation", "node", "message", "publisher", "subscription",
        "timer", "executor", "service", "action", "parameter", "lifecycle",
        "context", "serialization", "logging", "qos", "time", "graph",
        "utility", "type_support",
    }


def test_every_supported_entry_references_existing_evidence():
    manifest = _manifest()

    for entry in manifest["entries"]:
        if entry["support"] not in {"certified", "stock_authoritative"}:
            continue
        assert entry["evidence"], entry["id"]
        assert all((REPO_ROOT / path).exists() for path in entry["evidence"]), entry["id"]


def test_duplicate_ids_are_rejected():
    manifest = _manifest()
    manifest["entries"].append(dict(manifest["entries"][0]))

    with pytest.raises(checker.ManifestError, match="duplicate entry id"):
        checker.validate(manifest, REPO_ROOT)


def test_certified_unsafe_fallback_is_rejected():
    manifest = _manifest()
    manifest["entries"][0]["fallback"] = "unsafe"

    with pytest.raises(checker.ManifestError, match="unsafe fallback"):
        checker.validate(manifest, REPO_ROOT)


def test_missing_selected_upstream_mapping_is_rejected():
    manifest = _manifest()
    del manifest["upstream_contract"]["coverage"][0]

    with pytest.raises(checker.ManifestError, match="missing selected upstream coverage"):
        checker.validate(manifest, REPO_ROOT)


def test_missing_reviewed_exclusion_mapping_is_rejected():
    manifest = _manifest()
    coverage = manifest["upstream_contract"]["coverage"]
    index = next(
        index for index, mapping in enumerate(coverage)
        if mapping["disposition"] == "reviewed_exclusion"
    )
    del coverage[index]

    with pytest.raises(
        checker.ManifestError,
        match="missing reviewed_exclusion upstream coverage",
    ):
        checker.validate(manifest, REPO_ROOT)


def test_duplicate_upstream_mapping_is_rejected():
    manifest = _manifest()
    duplicate = dict(manifest["upstream_contract"]["coverage"][0])
    manifest["upstream_contract"]["coverage"].append(duplicate)

    with pytest.raises(checker.ManifestError, match="duplicate upstream coverage path"):
        checker.validate(manifest, REPO_ROOT)


def test_unknown_upstream_path_mapping_is_rejected():
    manifest = _manifest()
    manifest["upstream_contract"]["coverage"][0]["path"] = "test_unknown.py"

    with pytest.raises(checker.ManifestError, match="unknown selected upstream coverage path"):
        checker.validate(manifest, REPO_ROOT)


def test_unknown_upstream_entry_mapping_is_rejected():
    manifest = _manifest()
    manifest["upstream_contract"]["coverage"][0]["entry_id"] = "action.unknown"

    with pytest.raises(checker.ManifestError, match="unknown compatibility entry"):
        checker.validate(manifest, REPO_ROOT)


def test_reviewed_exclusion_must_map_to_unassessed_entry():
    manifest = _manifest()
    exclusion = next(
        mapping for mapping in manifest["upstream_contract"]["coverage"]
        if mapping["disposition"] == "reviewed_exclusion"
    )
    exclusion["entry_id"] = "node.create_node"

    with pytest.raises(checker.ManifestError, match="must map to unassessed behavior"):
        checker.validate(manifest, REPO_ROOT)
