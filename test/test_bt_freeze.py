#!/usr/bin/env python3
"""Frozen-path (L1) tests for bt_kit.

The freeze bakes behaviortree_cpp/bt_factory.h into a Cling PCH so the ~0.83 s
header JIT-parse becomes a few-ms PCH load. These tests run a worker subprocess
with the frozen PCH active (CLING_STANDARD_PCH must be set before the interpreter
starts, hence a subprocess) and assert two things:

  * the header parse is eliminated -- include(bt_factory.h) is well under the
    ~0.83 s JIT cost (a large margin keeps it non-flaky on a shared machine);
  * the frozen path is behaviourally correct, including the typed-port glue that
    only links once the frozen force-symbol fix resolves BT::UndefinedAnyType.

Skips cleanly when behaviortree_cpp is absent (default env) or the artifact has
not been built (`pixi run -e bt freeze-bt-build`). The rest of the suite proves
the SAME 16 tests pass frozen -- see `pixi run -e bt test-bt-frozen`.
"""
import json
import os
import subprocess
import sys

import pytest

try:
    from ament_index_python.packages import get_package_prefix
    get_package_prefix("behaviortree_cpp")
    _HAVE_BT = True
except Exception:
    _HAVE_BT = False

_HELPER = os.path.join(os.path.dirname(__file__), "_freeze_helper.py")


def _artifact():
    """Frozen PCH path via rclcppyy.kits.freeze, or None."""
    if not _HAVE_BT:
        return None
    from rclcppyy.kits import freeze
    p = freeze.artifact_path("bt")
    return p if os.path.exists(p) else None


pytestmark = pytest.mark.skipif(
    not _HAVE_BT or _artifact() is None,
    reason="behaviortree_cpp missing or frozen PCH not built (freeze-bt-build)")


def _run_frozen():
    env = dict(os.environ)
    env["CLING_STANDARD_PCH"] = _artifact()
    env["RCLCPPYY_FROZEN"] = "1"
    proc = subprocess.run([sys.executable, _HELPER], capture_output=True, text=True, env=env)
    for line in proc.stdout.splitlines():
        if line.startswith("JSON:"):
            return json.loads(line[5:])
    raise AssertionError("frozen worker produced no result:\n%s\n%s"
                         % (proc.stdout, proc.stderr))


def test_frozen_path_is_active_and_correct():
    r = _run_frozen()
    assert r["frozen"] and r["kit_frozen"], "frozen PCH was not active in the worker"
    assert r["status"] == 2, "frozen tree did not reach SUCCESS"
    # Guards the force-symbol fix: getInput<int>/setOutput<int> resolved frozen.
    assert r["sum"] == 107, "typed-port roundtrip wrong on frozen path: %r" % r["sum"]


def test_frozen_eliminates_header_parse():
    r = _run_frozen()
    # L0 JIT-parses this header in ~0.83 s; frozen loads it from the PCH in a few
    # ms. 200 ms is a deliberately loose ceiling so a shared machine won't flake.
    assert r["include_ms"] < 200.0, (
        "include(bt_factory.h) took %.1f ms frozen -- parse not eliminated"
        % r["include_ms"])
