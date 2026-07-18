"""Tests for the versioned benchmark result envelope."""

import importlib.util
import json
import math
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parent.parent
SCHEMA_PATH = REPO_ROOT / "scripts" / "benchmarks" / "_result_schema.py"
SPEC = importlib.util.spec_from_file_location("rclcppyy_benchmark_schema", SCHEMA_PATH)
schema = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(schema)


def _document(**overrides):
    values = {
        "repo_root": REPO_ROOT,
        "benchmark_name": "unit_test",
        "mode": "measurement",
        "matrix": {"duration_s": 1.0},
        "results": [{
            "case_id": "example_case",
            "backend": "example",
            "workload": "small-string",
            "target_rate_hz": 1000,
            "payload_bytes": 0,
            "backend_verified": True,
            "avg_latency_us": math.nan,
        }],
        "failures": [],
        "command": ["benchmark", "--json"],
    }
    values.update(overrides)
    return schema.build_document(**values)


def test_document_is_versioned_flat_and_strict_json():
    document = _document()

    assert document["schema"] == "rclcppyy.benchmark/v2"
    assert document["benchmark"]["name"] == "unit_test"
    assert document["results"] == [{
        "case_id": "example_case",
        "backend": "example",
        "workload": "small-string",
        "target_rate_hz": 1000,
        "payload_bytes": 0,
        "backend_verified": True,
        "avg_latency_us": None,
    }]
    assert document["environment"]["host"]["architecture"]
    assert document["environment"]["source"]["commit"]
    assert json.loads(schema.dumps(document)) == document


def test_document_records_structured_failures():
    failures = [{"variant": "example", "target_rate_hz": 10, "error": "failed"}]
    document = _document(results=[], failures=failures)

    assert document["results"] == []
    assert document["failures"] == failures


def test_write_is_atomic_and_round_trips(tmp_path):
    document = _document()
    output = tmp_path / "nested" / "result.json"

    schema.write(document, output)

    assert json.loads(output.read_text(encoding="utf-8")) == document
    assert not output.with_name(output.name + ".tmp").exists()


@pytest.mark.parametrize(
    "field,value",
    [
        ("schema", "other/v1"),
        ("results", {}),
        ("failures", {}),
    ],
)
def test_validate_rejects_invalid_document(field, value):
    document = _document()
    document[field] = value

    with pytest.raises(ValueError):
        schema.validate_document(document)


def test_smoke_document_forbids_performance_claims():
    document = _document(mode="smoke")

    assert document["benchmark"]["mode"] == "smoke"
    assert document["benchmark"]["performance_claims_allowed"] is False

    document["benchmark"]["performance_claims_allowed"] = True
    with pytest.raises(ValueError, match="smoke results cannot"):
        schema.validate_document(document)
