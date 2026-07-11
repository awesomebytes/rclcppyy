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
Before/after trajectories + loop edges are logged to Rerun as LineStrips3D.

NOTE on GTSAM + cppyy: the conda gtsam 4.2 C++ headers need boost/optional.hpp,
which is not in the env, so the header-heavy boost-coupled gtsam does NOT JIT under
cppyy (the fragile case the microplan flagged). We therefore use gtsam's own Python
binding here -- pose-graph optimization is a batch step, not a hot ROS loop, so the
"keep it in C++" argument does not apply; a gtsam_kit cppyy wrapper would not earn
its existence.

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
from loop_detector import LoopDetector  # noqa: E402
from rclcppyy.kits import cv_kit  # noqa: E402


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

    rr.init("rclcppyy_vision_posegraph")
    rrd = None
    if os.environ.get("RCLCPPYY_RERUN_SPAWN") == "1":
        rr.spawn()
    else:
        os.makedirs(os.path.dirname(args.rrd), exist_ok=True)
        rr.save(args.rrd)
        rrd = args.rrd

    gt = ground_truth_poses(args.n)
    loops = detect_loops(args.n)
    print("confirmed loops: %d (e.g. %s)" % (len(loops), loops[:3]))

    # Open-loop odometry: ground-truth relative motion + accumulating heading drift.
    odom = []
    for i in range(1, len(gt)):
        rel = relative(gt[i - 1], gt[i])
        odom.append(gtsam.Pose2(rel.x(), rel.y(), rel.theta() + args.drift))

    # Integrate odometry to get the drifted (open-loop) initial estimate.
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

    # Loop-closure factors: revisiting pose q ties back to earlier pose m, with the
    # (near-identity) ground-truth relative pose as the measurement.
    for q, m in loops:
        graph.add(gtsam.BetweenFactorPose2(m, q, relative(gt[m], gt[q]), loop_noise))

    result = gtsam.LevenbergMarquardtOptimizer(graph, values).optimize()
    optimized = [(result.atPose2(i).x(), result.atPose2(i).y()) for i in range(len(gt))]

    gt_xy = [(p[0], p[1]) for p in gt]

    def strip(xy):
        return np.array([[x, y, 0.0] for x, y in xy], dtype=np.float32)

    rr.log("trajectory/ground_truth", rr.LineStrips3D([strip(gt_xy)], colors=[(0, 200, 0)]))
    rr.log("trajectory/odometry_drift", rr.LineStrips3D([strip(drifted)], colors=[(220, 60, 60)]))
    rr.log("trajectory/optimized", rr.LineStrips3D([strip(optimized)], colors=[(60, 120, 255)]))
    loop_edges = [strip([optimized[m], optimized[q]]) for q, m in loops]
    if loop_edges:
        rr.log("trajectory/loop_edges", rr.LineStrips3D(loop_edges, colors=[(255, 200, 0)]))

    def err(traj):
        return float(np.mean([np.hypot(a[0] - b[0], a[1] - b[1])
                              for a, b in zip(traj, gt_xy)]))

    print("mean position error vs ground truth:")
    print("  open-loop odometry : %.3f m" % err(drifted))
    print("  after pose-graph   : %.3f m" % err(optimized))
    print("SUMMARY loops=%d drift_err=%.3f optimized_err=%.3f"
          % (len(loops), err(drifted), err(optimized)))
    if rrd:
        print("Rerun recording saved: %s  (open with: rerun %s)" % (rrd, rrd))


if __name__ == "__main__":
    main()
