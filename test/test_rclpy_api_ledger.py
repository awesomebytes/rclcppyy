"""Focused contract tests for the public API ledger extractor."""

import enum
import importlib.util
import json
import types
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parent.parent
SCRIPT = REPO_ROOT / "scripts" / "generate_rclpy_api_ledger.py"
SPEC = importlib.util.spec_from_file_location("rclpy_api_ledger", SCRIPT)
ledger = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ledger)


def _manifest():
    return {
        "entries": [
            {"id": "node.direct", "area": "node"},
            {"id": "utility.stock", "area": "utility"},
        ]
    }


def _annotations(entries=()):
    return {
        "schema": ledger.ANNOTATION_SCHEMA,
        "source_manifest": "compatibility/jazzy.json",
        "entries": list(entries),
    }


def _observation(mode, symbols):
    return {
        "schema": ledger.OBSERVATION_SCHEMA,
        "mode": mode,
        "runtime": {},
        "modules": [{
            "name": "rclpy.node",
            "import": {"state": "ok"},
            "symbols": symbols,
        }],
        "generated_interfaces": [],
    }


def _symbol(path, signature="(value)", members=()):
    return {
        "name": path.rsplit(".", 1)[-1],
        "path": path,
        "kind": "function",
        "origin": "rclpy.node",
        "qualname": path.rsplit(".", 1)[-1],
        "implementation": "python",
        "signature": {"state": "inspectable", "value": signature},
        "members": list(members),
    }


def _member(path, signature="(self)"):
    return {
        "name": path.rsplit(".", 1)[-1],
        "path": path,
        "kind": "method",
        "owner": path.rsplit(".", 1)[0],
        "origin": "rclpy.action.client",
        "implementation": "python",
        "signature": {"state": "inspectable", "value": signature},
    }


def test_extractor_keeps_public_members_required_dunders_and_constants():
    module = types.ModuleType("rclpy.fixture")

    class Fixture:
        PUBLIC = 3

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, traceback):
            return None

        @property
        def value(self):
            return 1

        def method(self, value=1):
            return value

    Fixture.__module__ = module.__name__

    def operation(value, *, enabled=True):
        return value, enabled

    operation.__module__ = module.__name__
    module.Fixture = Fixture
    module.operation = operation
    module.PUBLIC_CONSTANT = 4
    module.imported_noise = pytest

    symbols = [
        ledger._symbol_descriptor(module, name, value)
        for name, value in sorted(vars(module).items())
        if ledger._is_public_symbol(module, name, value)
    ]
    assert [symbol["name"] for symbol in symbols] == [
        "Fixture", "PUBLIC_CONSTANT", "operation"]
    fixture = symbols[0]
    members = {member["name"]: member for member in fixture["members"]}
    assert {"__enter__", "__exit__", "method", "value"} <= set(members)
    assert "__class__" not in members
    assert members["value"]["kind"] == "property"
    assert symbols[-1]["signature"] == {
        "state": "inspectable", "value": "(value, *, enabled=True)"}


def test_uninspectable_signature_is_explicit(monkeypatch):
    def operation():
        return None

    def fail(_value):
        raise ValueError("not inspectable")

    monkeypatch.setattr(ledger.inspect, "signature", fail)
    assert ledger._signature(operation) == {
        "state": "uninspectable",
        "reason": "inspect_signature_unsupported",
    }


def test_matching_name_and_signature_remain_unassessed():
    stock = _observation("stock", [_symbol("rclpy.node.operation")])
    direct = _observation("direct", [_symbol("rclpy.node.operation")])
    document = ledger.build_ledger(stock, direct, _annotations(), _manifest())

    assert document["entries"][0]["comparison"]["signature"] == "equal"
    assert document["entries"][0]["status"] == "unassessed"


@pytest.mark.parametrize(
    ("direct_symbols", "signature"),
    [([], "missing"), ([_symbol("rclpy.node.operation", "(other)")], "different")],
)
def test_missing_or_signature_drift_is_fail_visible(direct_symbols, signature):
    stock = _observation("stock", [_symbol("rclpy.node.operation")])
    direct = _observation("direct", direct_symbols)
    document = ledger.build_ledger(stock, direct, _annotations(), _manifest())

    entry = document["entries"][0]
    assert entry["comparison"]["signature"] == signature
    assert entry["status"] == "missing_mismatch"


def test_explicit_annotation_assigns_authority_without_hiding_drift():
    annotation = {
        "path": "rclpy.node.operation",
        "status": "exact_direct_cpp_authority",
        "manifest_entry_ids": ["node.direct"],
        "rationale": "Focused native authority proof.",
    }
    stock = _observation("stock", [_symbol("rclpy.node.operation")])
    direct = _observation("direct", [_symbol("rclpy.node.operation")])
    document = ledger.build_ledger(
        stock, direct, _annotations([annotation]), _manifest())
    assert document["entries"][0]["status"] == "exact_direct_cpp_authority"

    direct = _observation("direct", [_symbol("rclpy.node.operation", "(other)")])
    document = ledger.build_ledger(
        stock, direct, _annotations([annotation]), _manifest())
    assert document["entries"][0]["status"] == "missing_mismatch"


def test_annotation_rejects_unknown_manifest_reference():
    annotations = _annotations([{
        "path": "rclpy.node.operation",
        "status": "unassessed",
        "manifest_entry_ids": ["node.unknown"],
        "rationale": "Unknown evidence must not pass.",
    }])
    with pytest.raises(ledger.LedgerError, match="unknown manifest entry id"):
        ledger.validate_annotations(annotations, _manifest())


def test_two_level_module_symbol_is_symbol_level():
    def _action_client():
        return _symbol(
            "rclpy.action.client.ActionClient",
            members=[_member("rclpy.action.client.ActionClient.send_goal")],
        )

    stock = _observation("stock", [_action_client()])
    direct = _observation("direct", [_action_client()])
    document = ledger.build_ledger(stock, direct, _annotations(), _manifest())

    entries_by_path = {entry["path"]: entry for entry in document["entries"]}
    assert entries_by_path["rclpy.action.client.ActionClient"]["level"] == "symbol"
    assert entries_by_path["rclpy.action.client.ActionClient.send_goal"]["level"] == "member"


def test_signature_addresses_are_normalized(monkeypatch):
    volatile = "(goal_service_qos_profile=<rclpy.qos.QoSProfile object at 0x7f1234567890>)"

    def fake_signature(_value):
        return volatile

    monkeypatch.setattr(ledger.inspect, "signature", fake_signature)

    def operation():
        return None

    result = ledger._signature(operation)
    assert result == {
        "state": "inspectable",
        "value": "(goal_service_qos_profile=<rclpy.qos.QoSProfile object>)",
    }
    assert "0x" not in result["value"]


def test_validate_ledger_accepts_built_document_and_rejects_summary_drift():
    stock = _observation("stock", [_symbol("rclpy.node.operation")])
    direct = _observation("direct", [_symbol("rclpy.node.operation")])
    document = ledger.build_ledger(stock, direct, _annotations(), _manifest())

    summary = ledger.validate_ledger(document)
    assert summary == document["summary"]

    document["summary"]["ledger_entries"] += 1
    with pytest.raises(ledger.LedgerError, match="entry summary drift"):
        ledger.validate_ledger(document)


def test_enum_with_unrepresentable_value_degrades_to_type_only():
    class Fixture(enum.Enum):
        MEMBER = object()

    class Owner:
        ATTR = Fixture.MEMBER

    descriptor = ledger._member_descriptor("rclpy.fixture.Owner", Owner, "ATTR")

    assert descriptor["value"] == {
        "state": "type_only",
        "type": f"{Fixture.__module__}.{Fixture.__qualname__}",
    }
    json.dumps(descriptor)  # must not raise: the sentinel must never reach serialization


def test_baseline_matches_hardened_schema():
    jsonschema = pytest.importorskip("jsonschema")
    baseline_path = REPO_ROOT / "compatibility" / "rclpy-api-ledger-jazzy.json"
    if not baseline_path.exists():
        pytest.skip("baseline not generated yet")
    schema_path = REPO_ROOT / "schemas" / "rclpy-api-ledger-v1.schema.json"
    baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    jsonschema.validate(baseline, schema)
