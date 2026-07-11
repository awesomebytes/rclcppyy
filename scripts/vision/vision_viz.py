"""
vision_viz -- shared Rerun setup for the vision demos.

The interactive default is a **live viewer**: when you run a demo by hand, a Rerun
window opens and you watch the pipeline work -- the camera stream, the ORB
keypoints, the per-frame processing time, and (M3/M4) the loops closing and the
trajectory snapping back. Headless (writing a ``.rrd`` you open later) is the
fallback for tests/CI and displayless shells, so ``pixi run -e vision test-vision``
and any headless run are unchanged.

One place decides, so every demo behaves the same:

Decision (:func:`should_spawn`):
  * ``RCLCPPYY_RERUN_SPAWN=1`` -> force the live viewer;
  * ``RCLCPPYY_RERUN_SPAWN=0`` -> force headless ``.rrd`` (e.g. CI on a display);
  * unset -> live **iff** a display is present (``DISPLAY`` or ``WAYLAND_DISPLAY``)
    and we are **not** under pytest.

Spawning the viewer (:func:`native_viewer_path`): rerun-sdk's console entry point
(the ``rerun`` on ``PATH``) is, in this env, a thin Python shim that fails to import
its native bindings (a ``rerun_bindings`` symbol mismatch), so ``rr.spawn()`` with
the default executable never binds its port. We therefore spawn the **native**
viewer binary that ships inside the wheel (``rerun_sdk/rerun_cli/rerun``) via
``rr.spawn(executable_path=...)``, falling back to the default name if we can't find
it, and finally degrading to a headless ``.rrd`` if the viewer won't come up (so a
demo never dies just because it couldn't open a window).

Tidy tree: demos log under a small set of stable entity roots (``camera/``,
``perf/``, ``loop/``, ``world/``) and pass a :mod:`rerun.blueprint` layout so the
viewer opens already arranged into comprehensible panels instead of an
auto-generated pile.
"""
import os
import sys
from collections import namedtuple

import rerun as rr

# What init_rerun returns so the caller can print the right "how to view" line.
#   mode: "spawn" (live viewer) or "headless" (.rrd on disk)
#   rrd:  the .rrd path in headless mode, else None
VizSession = namedtuple("VizSession", "mode rrd")


def under_pytest():
    """True if we are running inside pytest (so demos invoked by a test never try
    to pop a window)."""
    return "PYTEST_CURRENT_TEST" in os.environ or "pytest" in sys.modules


def should_spawn(env=None, in_pytest=None):
    """Decide live viewer (True) vs headless .rrd (False).

    ``RCLCPPYY_RERUN_SPAWN`` forces it when set (``1`` -> live, ``0`` -> headless);
    otherwise go live only when interactive: a display is present and we are not
    under pytest. Pure and side-effect-free so it is unit-testable -- pass ``env``
    (a dict) and ``in_pytest`` to exercise every branch.
    """
    env = os.environ if env is None else env
    forced = env.get("RCLCPPYY_RERUN_SPAWN")
    if forced is not None and forced.strip() != "":
        return forced.strip() != "0"
    if in_pytest is None:
        in_pytest = under_pytest()
    if in_pytest:
        return False
    return bool(env.get("DISPLAY") or env.get("WAYLAND_DISPLAY"))


def native_viewer_path():
    """Absolute path to the native Rerun viewer binary bundled in the rerun_sdk
    wheel (``rerun_sdk/rerun_cli/rerun``), or ``None`` if not found.

    Preferred over the ``rerun`` console script on ``PATH``, which may be a Python
    shim that fails to import its bindings in this env."""
    try:
        pkg = os.path.dirname(os.path.abspath(rr.__file__))          # .../rerun_sdk/rerun
        cand = os.path.join(os.path.dirname(pkg), "rerun_cli", "rerun")
        if os.path.isfile(cand) and os.access(cand, os.X_OK):
            return cand
    except Exception:
        pass
    # Fallback: search under the active env prefix.
    import glob
    prefix = os.environ.get("CONDA_PREFIX", sys.prefix)
    for cand in glob.glob(os.path.join(prefix, "lib", "python*", "site-packages",
                                       "rerun_sdk", "rerun_cli", "rerun")):
        if os.access(cand, os.X_OK):
            return cand
    return None


def init_rerun(app_id, rrd_path, blueprint=None, env=None):
    """One-call Rerun setup for a demo. Returns a :class:`VizSession`.

    Live (interactive) by default: spawns the native viewer and streams to it.
    Headless otherwise: writes ``rrd_path`` (a ``.rrd`` recording). ``blueprint`` (a
    :mod:`rerun.blueprint` layout) is applied in both modes, so the recording opens
    with the same tidy panels the live viewer shows. If a live viewer is wanted but
    won't come up, degrade to headless rather than fail.
    """
    env = os.environ if env is None else env
    spawn = should_spawn(env)
    rr.init(app_id, default_blueprint=blueprint)
    if spawn:
        exe = native_viewer_path()
        try:
            if exe is not None:
                rr.spawn(executable_path=exe)
            else:
                rr.spawn()
            return VizSession("spawn", None)
        except Exception as exc:  # pragma: no cover - environment dependent
            sys.stderr.write(
                "[vision_viz] could not open the live Rerun viewer (%s); "
                "falling back to a headless .rrd. Force headless with "
                "RCLCPPYY_RERUN_SPAWN=0.\n" % exc)
    os.makedirs(os.path.dirname(rrd_path), exist_ok=True)
    rr.save(rrd_path)
    return VizSession("headless", rrd_path)


def announce(session):
    """Print a one-line, honest note about where the visualization went, matching
    the mode init_rerun actually chose."""
    if session.mode == "spawn":
        print("Rerun: live viewer opened -- watch it stream. "
              "(headless instead: RCLCPPYY_RERUN_SPAWN=0)", flush=True)
    else:
        print("Rerun recording saved: %s  (open with: rerun %s)"
              % (session.rrd, session.rrd), flush=True)


# --- Blueprints: a comprehensible default layout per demo --------------------
# Kept here (not in the demos) because "how the viewer is arranged" is a viz
# concern, and sharing the builders keeps the entity-path vocabulary consistent.

def _rrb():
    import rerun.blueprint as rrb
    return rrb


def blueprint_camera_perf(perf_title="processing time (ms/frame)"):
    """Spine/features layout: the camera image (with any keypoint overlay) beside a
    live per-frame processing-time plot."""
    rrb = _rrb()
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="/camera", name="camera + keypoints"),
            rrb.TimeSeriesView(origin="/perf", name=perf_title),
            column_shares=[3, 2],
        ),
        collapse_panels=True,
    )


def blueprint_loop():
    """M3 layout: the live camera on the left; on the right a stack of the
    processing-time / loop-score plots, the confirmed loop image pair, and the loop
    event log -- so a closing loop is visible from three angles at once."""
    rrb = _rrb()
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="/camera", name="live camera + ORB"),
            rrb.Vertical(
                rrb.TimeSeriesView(origin="/perf", name="processing time (ms/frame)"),
                rrb.Horizontal(
                    rrb.Spatial2DView(origin="/loop/pair/query", name="loop: current"),
                    rrb.Spatial2DView(origin="/loop/pair/match", name="loop: revisited"),
                ),
                rrb.TimeSeriesView(origin="/loop/score", name="confirmed loop score"),
                rrb.TextLogView(origin="/loop/events", name="loop events"),
                row_shares=[2, 2, 1, 1],
            ),
            column_shares=[3, 3],
        ),
        collapse_panels=True,
    )


def blueprint_posegraph():
    """M4 layout: the trajectories in a 3D view (drift vs corrected vs ground truth
    + loop edges) beside the mean-error-over-time plot that drops when the optimizer
    runs."""
    rrb = _rrb()
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="/world", name="trajectories + loop edges"),
            rrb.Vertical(
                rrb.TimeSeriesView(origin="/error", name="mean position error (m)"),
                rrb.TextLogView(origin="/log", name="events"),
                row_shares=[3, 1],
            ),
            column_shares=[3, 2],
        ),
        collapse_panels=True,
    )
