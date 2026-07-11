#!/usr/bin/env python3
"""Unit tests for scripts/vision/vision_viz.py -- the spawn-vs-headless decision
that makes the live Rerun viewer the interactive default while keeping tests/CI
headless, plus native-viewer resolution.

These are pure-logic tests (``should_spawn`` takes an injected env dict and an
``in_pytest`` flag, so every branch is exercised without touching the real
environment). The module imports ``rerun``, so the whole file auto-skips when
rerun-sdk is absent -- the default ``pixi run test`` is unaffected. Run under the
vision env: ``pixi run -e vision test-vision``.
"""
import importlib.util
import os
import sys

import pytest

_HAVE_RERUN = importlib.util.find_spec("rerun") is not None
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(_HERE)

pytestmark = pytest.mark.skipif(
    not _HAVE_RERUN,
    reason="rerun-sdk not present (use the vision env: pixi run -e vision test-vision)")

if _HAVE_RERUN:
    sys.path.insert(0, os.path.join(_REPO, "scripts", "vision"))
    import vision_viz


def test_force_on_beats_everything():
    # RCLCPPYY_RERUN_SPAWN=1 forces the live viewer even under pytest / no display.
    assert vision_viz.should_spawn({"RCLCPPYY_RERUN_SPAWN": "1"}, in_pytest=True) is True
    assert vision_viz.should_spawn({"RCLCPPYY_RERUN_SPAWN": "1"}, in_pytest=False) is True


def test_force_off_beats_display():
    # Explicit 0 forces headless even with a display and outside pytest (e.g. CI).
    assert vision_viz.should_spawn({"RCLCPPYY_RERUN_SPAWN": "0", "DISPLAY": ":1"},
                                   in_pytest=False) is False


def test_interactive_display_spawns():
    assert vision_viz.should_spawn({"DISPLAY": ":1"}, in_pytest=False) is True
    assert vision_viz.should_spawn({"WAYLAND_DISPLAY": "wayland-0"}, in_pytest=False) is True


def test_no_display_is_headless():
    assert vision_viz.should_spawn({}, in_pytest=False) is False


def test_under_pytest_is_headless():
    # A display present but running under pytest must stay headless (protects CI).
    assert vision_viz.should_spawn({"DISPLAY": ":1"}, in_pytest=True) is False


def test_auto_pytest_detection_is_headless():
    # The real auto path: we ARE under pytest now, so even with a display the
    # auto-detected in_pytest must force headless.
    assert vision_viz.should_spawn({"DISPLAY": ":1"}) is False


def test_empty_env_var_falls_through_to_heuristic():
    # Empty / whitespace value is treated as unset -> heuristic applies.
    assert vision_viz.should_spawn({"RCLCPPYY_RERUN_SPAWN": "", "DISPLAY": ":1"},
                                   in_pytest=False) is True
    assert vision_viz.should_spawn({"RCLCPPYY_RERUN_SPAWN": "  ", "DISPLAY": ":1"},
                                   in_pytest=False) is True


def test_native_viewer_path_resolves():
    # In the vision env the native viewer binary must resolve (the executable_path
    # rr.spawn needs, since the `rerun` console shim is broken here).
    p = vision_viz.native_viewer_path()
    assert p is not None and os.path.isfile(p) and os.access(p, os.X_OK)
