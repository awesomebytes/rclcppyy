"""Tests for benchmark child backend evidence and parent enforcement."""

import importlib.util
import json
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parent.parent
BENCH_DIR = REPO_ROOT / "scripts" / "benchmarks"
sys.path.insert(0, str(BENCH_DIR))
SPEC = importlib.util.spec_from_file_location(
    "rclcppyy_benchmark_runner", BENCH_DIR / "run_benchmarks.py")
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)


def _marker(role="publisher", backend="cpp"):
    return {
        "schema": runner.BACKEND_SCHEMA,
        "role": role,
        "backend": backend,
        "evidence": "unit_test",
        "metadata": {},
    }


class _Child:
    def __init__(self, markers=(), errors=()):
        self._markers = list(markers)
        self._errors = list(errors)

    def backend_snapshot(self):
        return self._markers, self._errors


def test_valid_marker_is_accepted():
    marker = _marker()
    runner.validate_backend_marker(marker)
    assert runner.require_backend_marker(_Child([marker]), "publisher", "cpp") == marker


@pytest.mark.parametrize(
    "markers,errors,message",
    [
        ([], [], "no backend evidence"),
        ([_marker(backend="python")], [], "backend mismatch"),
        ([], ["bad JSON"], "invalid backend evidence"),
    ],
)
def test_required_backend_rejects_unproven_or_wrong_route(markers, errors, message):
    with pytest.raises(RuntimeError, match=message):
        runner.require_backend_marker(_Child(markers, errors), "publisher", "cpp")


def test_marker_protocol_is_strict_json(capsys):
    import _backend_marker

    _backend_marker._emit("subscriber", "python", "unit_test", entity="example")
    line = capsys.readouterr().out.rstrip()
    assert line.startswith(runner.BACKEND_PREFIX)
    marker = json.loads(line[len(runner.BACKEND_PREFIX):])
    runner.validate_backend_marker(marker)
    assert marker["metadata"] == {"entity": "example"}
