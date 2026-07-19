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
    assert summary["entries"] == 80
    assert summary["support"] == {
        "certified": 36,
        "experimental": 23,
        "stock_authoritative": 16,
        "unassessed": 3,
        "unsupported": 2,
    }
    assert summary["backend"] == {
        "cpp": 20,
        "mixed": 9,
        "none": 5,
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


def test_direct_parameter_note_locks_pre_through_phase_three_cpu_evidence():
    entry = next(
        item for item in _manifest()["entries"]
        if item["id"] == "parameter.direct_cpp_local_first_slice"
    )
    assert (
        "The clean pre-optimization artifact at product a8c2cb6 with suite 852e48b "
        "records "
        "direct/stock process-CPU ratios of 0.580 for declare, 38.682 for "
        "get-native, 30.950 for get-value-snapshot, and 0.446 for atomic set."
    ) in entry["notes"]
    assert (
        "The clean Phase-1 artifact at product 6da8193 with suite 0518f37 records "
        "0.474 for declare, 10.316 for get-native, 8.712 for get-value-snapshot, "
        "and 0.349 for atomic set; direct get CPU fell 73.5% to 1,223 ns/op but "
        "still failed the stock CPU priority."
    ) in entry["notes"]
    assert (
        "The clean Phase-2 artifact at product 72e28b5 with suite 0518f37 records "
        "0.6857 for declare, 0.5633 for get-native, 1.3433 for "
        "get-value-snapshot, and 0.5205 for atomic set; snapshot median CPU is "
        "207 ns/op versus 157 ns/op stock."
    ) in entry["notes"]
    assert (
        "The clean Phase-3 artifact at product 0faa435 with suite 941d6ce records "
        "0.6943 for declare, 0.6090 for get-native, 1.1073 for "
        "get-value-snapshot, and 0.5148 for atomic set; snapshot median CPU is "
        "167 ns/op versus 152 ns/op stock."
    ) in entry["notes"]
    assert "Performance claims remain disabled" in entry["notes"]
    assert "explicit snapshot CPU still misses the stock priority" in entry["notes"]


def test_direct_remote_parameter_note_locks_clean_cpu_evidence():
    entry = next(
        item for item in _manifest()["entries"]
        if item["id"] == "parameter.direct_cpp_remote_client"
    )
    assert (
        "Clean five-repetition artifact "
        "build/remote-parameter-cyclone-be024c2.json records direct/stock "
        "combined process-CPU ratios of 0.1583 for get-one and 0.2492 for "
        "set-atomically-one, with 6.3175x and 4.0148x throughput respectively."
    ) in entry["notes"]
    assert "boundary counters at zero" in entry["notes"]
    assert "claims remain disabled" in entry["notes"]


def test_direct_action_notes_lock_corrected_cpu_evidence():
    entries = {item["id"]: item for item in _manifest()["entries"]}
    client = entries["action.direct_cpp_client_first_slice"]["notes"]
    server = entries["action.direct_cpp_server_first_slice"]["notes"]
    assert (
        "build/action-client-cyclone-b2f0f2d-corrected.json uses identical "
        "public QoS and source/executor shape and records median paired "
        "direct/stock ratios of 0.2606 CPU"
    ) in client
    assert "record zero conversion/serialization/CDR calls" in client
    assert (
        "build/action-server-cyclone-b2f0f2d-corrected.json uses one common "
        "AOT client plus identical public QoS and records median paired "
        "direct/stock ratios of 0.6475 server CPU"
    ) in server
    assert "2,080 counted exact-C++ adapter deep copies" in server
    assert "representation conversion" in server
    assert "create_feedback_shared/create_result_shared" in server
    assert "zero adapter deep copies" in server
    assert "105 feedback shared handoffs" in server
    assert "35 result shared handoffs" in server
    assert "5.09% server-CPU reduction" in server
    assert "one of three paired repetitions regresses" in server
    assert "not a performance claim" in server


def test_direct_wait_for_message_is_distinct_from_stock_authority():
    entries = {item["id"]: item for item in _manifest()["entries"]}
    stock = entries["subscription.wait_for_message"]
    direct = entries["subscription.direct_cpp_wait_for_message"]

    assert stock["support"] == "stock_authoritative"
    assert stock["backend"] == "python"
    assert stock["default"] is True
    assert direct["support"] == "experimental"
    assert direct["backend"] == "cpp"
    assert direct["default"] is False
    assert "actual generated C++ message" in direct["notes"]
    assert "zero conversion, serialization, and CDR poison calls" in direct["notes"]
    assert "no CPU result or performance claim" in direct["notes"]


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
