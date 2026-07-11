#!/usr/bin/env python
"""
vision demo M4 (STRETCH) -- POSE-GRAPH loop-closure correction with GTSAM.

Closing the loop is only half the story; the payoff is *correcting the trajectory*.
This demo builds a 2D pose graph over the synthetic sequence: the camera's
rectangular circuit is the ground truth, the odometry is that circuit corrupted by
an accumulating heading drift (so the open-loop trajectory spirals away from the
start), and each loop closure confirmed by the M3 detector (loop_detector.py) adds a
BetweenFactor tying the revisiting pose back to the earlier one. GTSAM's
Levenberg-Marquardt optimizer then pulls the drifted trajectory back onto itself.

Live by default (a Rerun window opens; headless .rrd under pytest/CI or no display;
force with RCLCPPYY_RERUN_SPAWN=1/0). What you watch, on the "step" timeline:
  * the DRIFTED open-loop trajectory (red) spiralling away from the GROUND TRUTH
    (green) as the camera drives, with the mean-error plot climbing;
  * yellow LOOP EDGES snapping in as the detector confirms each revisit;
  * then, when the optimizer runs (last step), the CORRECTED trajectory (blue)
    appearing pulled back onto the ground truth and the error plot dropping ~15x.
Scrub or play the timeline to see the correction snap into place.

NOTE on GTSAM + cppyy: we use gtsam's own **Python binding** here, not cppyy. With
libboost-headers now in the env the old boost/optional.hpp wall is gone, but gtsam
still does not JIT+run under cppyy in this env: its conda build's config.h sets
GTSAM_USE_TBB (headers #include <tbb/...>, absent from the env), and -- even with
tbb headers supplied -- the Cling JIT fails to materialize the static-initializer of
gtsam's namespace-scope `static const KeyFormatter DefaultKeyFormatter` in Key.h.
That is fine: pose-graph optimization is a one-shot *batch* step, not a hot ROS loop,
so the "keep it in C++" argument does not apply and the binding is the honest choice.
See docs/vision/REPORT.md (GTSAM/cppyy probe) for the full evidence.

    pixi run -e vision demo-vision-posegraph
"""
import argparse
import math
import os
import sys

import numpy as np
import rerun as rr

os.environ.setdefault("ROS_DOMAIN_ID", "50")

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, "scripts", "datasets"))
sys.path.insert(0, HERE)

import synthetic_loop  # noqa: E402
import train_vocab  # noqa: E402
import vision_viz  # noqa: E402
from loop_detector import LoopDetector  # noqa: E402
from rclcppyy.kits import cv_kit  # noqa: E402

# Trajectory colors, shared by the 3D lines and their labels.
C_GT = (0, 200, 0)         # ground truth  -- green
C_DRIFT = (220, 60, 60)    # open-loop odometry -- red
C_OPT = (60, 120, 255)     # after pose-graph  -- blue
C_LOOP = (255, 200, 0)     # loop closure edges -- yellow


def ground_truth_poses(n, scale=0.01):
    """2D (x, y, theta) ground-truth poses from the synthetic circuit positions;
    theta faces the direction of travel."""
    pos = np.array(synthetic_loop.positions(n), dtype=float) * scale
    poses = []
    for i in range(len(pos)):
        j = min(i + 1, len(pos) - 1)
        d = pos[j] - pos[i]
        theta = math.atan2(d[1], d[0]) if np.linalg.norm(d) > 1e-9 else (poses[-1][2] if poses else 0.0)
        poses.append((pos[i][0], pos[i][1], theta))
    return poses


def detect_loops(n, nfeatures=1000):
    """Run the M3 front-end on the synthetic sequence; return confirmed (q, m)."""
    orb = cv_kit.create_orb(nfeatures)
    descs = [orb.detect_and_compute(cv_kit.numpy_to_mat(f))[1]
             for _, f in synthetic_loop.frames(n)]
    voc = train_vocab.train("synthetic", None, k=10, L=4, nfeatures=nfeatures, n_frames=n)
    det = LoopDetector(voc, min_score=0.4, consistency_k=3, ignore_recent=25)
    loops = []
    for d in descs:
        lc = det.add_and_query(d)
        if lc:
            loops.append((lc.query_id, lc.match_id))
    return loops


def relative(a, b):
    """Pose2 measurement a->b in a's frame (gtsam Pose2.between)."""
    import gtsam
    return gtsam.Pose2(*a).between(gtsam.Pose2(*b))


def strip(xy):
    """(N,2) or list of (x,y) -> (N,3) float32 for a Rerun LineStrips3D on z=0."""
    return np.array([[x, y, 0.0] for x, y in xy], dtype=np.float32)


def mean_err(traj, gt_xy):
    if not traj:
        return 0.0
    return float(np.mean([np.hypot(a[0] - b[0], a[1] - b[1])
                          for a, b in zip(traj, gt_xy[:len(traj)])]))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--drift", type=float, default=0.004, help="per-step heading drift (rad)")
    ap.add_argument("--rrd", default=os.path.join(REPO, "build", "vision", "posegraph.rrd"))
    args = ap.parse_args()

    try:
        import gtsam
    except ImportError:
        sys.exit("gtsam Python binding not found (install the vision env)")

    session = vision_viz.init_rerun("rclcppyy_vision_posegraph", args.rrd,
                                    blueprint=vision_viz.blueprint_posegraph())
    # Style the error time series (two named lines: open-loop vs corrected).
    rr.log("error/open_loop", rr.SeriesLines(names=["open-loop"], colors=[C_DRIFT],
                                             widths=[2.0]), static=True)
    rr.log("error/corrected", rr.SeriesLines(names=["after pose-graph"], colors=[C_OPT],
                                             widths=[2.0]), static=True)

    gt = ground_truth_poses(args.n)
    gt_xy = [(p[0], p[1]) for p in gt]
    loops = detect_loops(args.n)
    print("confirmed loops: %d (e.g. %s)" % (len(loops), loops[:3]))

    # Open-loop odometry: ground-truth relative motion + accumulating heading drift.
    odom = []
    for i in range(1, len(gt)):
        rel = relative(gt[i - 1], gt[i])
        odom.append(gtsam.Pose2(rel.x(), rel.y(), rel.theta() + args.drift))

    # Integrate odometry to get the drifted (open-loop) initial estimate + the graph.
    graph = gtsam.NonlinearFactorGraph()
    values = gtsam.Values()
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.005]))
    odom_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.05, 0.02]))
    loop_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))

    cur = gtsam.Pose2(*gt[0])
    values.insert(0, cur)
    graph.add(gtsam.PriorFactorPose2(0, cur, prior_noise))
    drifted = [(cur.x(), cur.y())]
    for i, o in enumerate(odom, start=1):
        graph.add(gtsam.BetweenFactorPose2(i - 1, i, o, odom_noise))
        cur = cur.compose(o)
        values.insert(i, cur)
        drifted.append((cur.x(), cur.y()))

    # Loop-closure factors: revisiting pose q ties back to earlier pose m.
    for q, m in loops:
        graph.add(gtsam.BetweenFactorPose2(m, q, relative(gt[m], gt[q]), loop_noise))

    result = gtsam.LevenbergMarquardtOptimizer(graph, values).optimize()
    optimized = [(result.atPose2(i).x(), result.atPose2(i).y()) for i in range(len(gt))]

    # --- Stream it so you SEE the drift accumulate, the loops confirm, then the
    # correction snap in. The full ground truth is a faint static reference; the
    # drifted path and loop edges are revealed frame-by-frame on the "step" timeline.
    rr.log("world/ground_truth", rr.LineStrips3D([strip(gt_xy)], colors=[C_GT],
                                                 labels=["ground truth"]), static=True)
    loops_by_q = {}
    for q, m in loops:
        loops_by_q.setdefault(q, []).append(m)
    revealed_edges = []
    for i in range(len(gt)):
        rr.set_time("step", sequence=i)
        rr.log("world/odometry_drift",
               rr.LineStrips3D([strip(drifted[:i + 1])], colors=[C_DRIFT], labels=["odometry (drift)"]))
        rr.log("world/current", rr.Points3D([strip([drifted[i]])[0]], colors=[C_DRIFT], radii=0.05))
        for q in [q for q in loops_by_q if q == i]:
            for m in loops_by_q[q]:
                revealed_edges.append(strip([drifted[m], drifted[q]]))
                rr.log("log/events", rr.TextLog(
                    "loop confirmed: frame %d revisits frame %d" % (q, m)))
        if revealed_edges:
            rr.log("world/loop_edges", rr.LineStrips3D(revealed_edges, colors=[C_LOOP]))
        rr.log("error/open_loop", rr.Scalars(mean_err(drifted[:i + 1], gt_xy)))

    # Optimizer runs: on the next step the corrected (blue) trajectory snaps in onto
    # the ground truth, the loop edges move to their corrected positions, and the
    # error drops. Held for a few steps so it is unmistakable when playing.
    opt_err = mean_err(optimized, gt_xy)
    drift_err = mean_err(drifted, gt_xy)
    rr.log("log/events", rr.TextLog(
        "GTSAM LevenbergMarquardt optimize(): %.3f m -> %.3f m mean error"
        % (drift_err, opt_err)))
    for k in range(1, 6):
        rr.set_time("step", sequence=len(gt) - 1 + k)
        rr.log("world/optimized", rr.LineStrips3D([strip(optimized)], colors=[C_OPT],
                                                  labels=["after pose-graph"]))
        rr.log("world/loop_edges",
               rr.LineStrips3D([strip([optimized[m], optimized[q]]) for q, m in loops],
                               colors=[C_LOOP]))
        rr.log("error/open_loop", rr.Scalars(drift_err))
        rr.log("error/corrected", rr.Scalars(opt_err))

    print("mean position error vs ground truth:")
    print("  open-loop odometry : %.3f m" % drift_err)
    print("  after pose-graph   : %.3f m" % opt_err)
    print("SUMMARY loops=%d drift_err=%.3f optimized_err=%.3f"
          % (len(loops), drift_err, opt_err))
    vision_viz.announce(session)


if __name__ == "__main__":
    main()
