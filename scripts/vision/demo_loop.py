#!/usr/bin/env python
"""
vision demo M3 -- PLACE RECOGNITION / LOOP CLOSURE: the heart.

The full front-end as a single single-screen node. A vocabulary (trained on the
sequence, or the real ORBvoc via --vocab PATH) backs a DBoW2 OrbDatabase. Each
incoming C++ sensor_msgs/Image is wrapped zero-copy, ORB'd, added to the database,
and queried for a revisit; a DLoopDetector-style temporal-consistency gate
(loop_detector.py) confirms a loop only once a candidate persists over k frames.
Confirmed loops are logged to Rerun (query frame, matched frame, score) and printed.

  * synthetic (default): detects the constructed revisit (frame ~180+j -> frame j).
  * TUM (--tum DIR, ideally with the real --vocab data/ORBvoc.txt): qualitative
    detection at the sequence's genuine revisit.

Citable basis: Mur-Artal & Tardos (ORB-SLAM); Galvez-Lopez & Tardos (Bags of Binary
Words / DLoopDetector).

    pixi run -e vision demo-vision-loop
    pixi run -e vision demo-vision-loop --tum data/<seq> --vocab data/ORBvoc.txt
"""
import argparse
import os
import sys
import time

import rerun as rr

os.environ.setdefault("ROS_DOMAIN_ID", "50")

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, "scripts", "datasets"))
sys.path.insert(0, HERE)

import dataset_publisher as DP  # noqa: E402
import train_vocab  # noqa: E402
from loop_detector import LoopDetector  # noqa: E402
from rclcppyy.bringup_rclcpp import bringup_rclcpp  # noqa: E402
from rclcppyy.kits import cv_kit, dbow_kit  # noqa: E402

TOPIC = "vision/image"
FX, FY, CX, CY = 525.0, 525.0, 319.5, 239.5


def init_rerun(default_rrd):
    rr.init("rclcppyy_vision_loop")
    if os.environ.get("RCLCPPYY_RERUN_SPAWN") == "1":
        rr.spawn()
        return None
    os.makedirs(os.path.dirname(default_rrd), exist_ok=True)
    rr.save(default_rrd)
    return default_rrd


def get_vocabulary(args, kind, path):
    if args.vocab == "train":
        print("Training vocabulary on the sequence (offline pass) ...")
        return train_vocab.train(kind, path, k=args.k, L=args.L,
                                 nfeatures=args.nfeatures, n_frames=args.n, limit=args.limit)
    print("Loading vocabulary: %s" % args.vocab)
    t0 = time.perf_counter()
    voc = dbow_kit.load_vocabulary(args.vocab)
    print("  %d words, %.1f s" % (int(voc.size()), time.perf_counter() - t0))
    return voc


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tum", metavar="DIR")
    ap.add_argument("--folder", metavar="DIR")
    ap.add_argument("--vocab", default="train",
                    help="'train' (default) or a vocabulary path (.txt/.dbow2/.yml)")
    ap.add_argument("--rate", type=float, default=15.0)
    ap.add_argument("--n", type=int, default=200, help="synthetic frame count")
    ap.add_argument("--limit", type=int, help="max frames (tum/folder)")
    ap.add_argument("--nfeatures", type=int, default=1000)
    ap.add_argument("--k", type=int, default=10)
    ap.add_argument("--L", type=int, default=4)
    ap.add_argument("--min-score", type=float, default=0.4)
    ap.add_argument("--consistency", type=int, default=3)
    ap.add_argument("--ignore-recent", type=int, default=25)
    ap.add_argument("--rrd", default=os.path.join(REPO, "build", "vision", "loop.rrd"))
    args = ap.parse_args()

    kind = "tum" if args.tum else "folder" if args.folder else "synthetic"
    path = args.tum or args.folder
    rrd = init_rerun(args.rrd)

    rclcpp = bringup_rclcpp()
    cv_kit.warmup(args.nfeatures)
    dbow_kit.warmup()
    orb = cv_kit.create_orb(args.nfeatures)
    voc = get_vocabulary(args, kind, path)
    detector = LoopDetector(voc, min_score=args.min_score, consistency_k=args.consistency,
                            ignore_recent=args.ignore_recent)

    import cppyy
    cppyy.include("sensor_msgs/msg/image.hpp")
    Image = cppyy.gbl.sensor_msgs.msg.Image
    if not rclcpp.ok():
        rclcpp.init()
    rr.log("camera", rr.Pinhole(resolution=[640, 480], focal_length=[FX, FY],
                                principal_point=[CX, CY]), static=True)

    state = {"n": 0, "loops": [], "thumbs": {}}

    def on_image(msg):
        idx = state["n"]
        rr.set_time("frame", sequence=idx)
        mat = cv_kit.msg_to_mat(msg)
        gray = cv_kit.to_gray(mat)
        kps, desc = orb.detect_and_compute(gray)
        arr = cv_kit.mat_to_numpy(mat, copy=True)   # copy: kept for later match display
        state["thumbs"][idx] = arr
        if arr.ndim == 3:
            rr.log("camera/image", rr.Image(arr, color_model="BGR"))
        else:
            rr.log("camera/image", rr.Image(arr))
        rr.log("camera/image/keypoints",
               rr.Points2D(cv_kit.keypoints_to_numpy(kps), radii=2.0))
        loop = detector.add_and_query(desc)
        if loop:
            state["loops"].append((loop.query_id, loop.match_id, round(loop.score, 4)))
            rr.log("loop/score", rr.Scalars(loop.score))
            q = state["thumbs"].get(loop.query_id)
            m = state["thumbs"].get(loop.match_id)
            if q is not None:
                rr.log("loop/query", rr.Image(q) if q.ndim == 2 else rr.Image(q, color_model="BGR"))
            if m is not None:
                rr.log("loop/match", rr.Image(m) if m.ndim == 2 else rr.Image(m, color_model="BGR"))
            rr.log("loop/event", rr.TextLog(
                "LOOP: frame %d revisits frame %d (score %.3f)"
                % (loop.query_id, loop.match_id, loop.score)))
            print("  LOOP  frame %d revisits frame %d  score=%.3f"
                  % (loop.query_id, loop.match_id, loop.score), flush=True)
        state["n"] += 1

    pub_node = rclcpp.Node("vision_publisher")
    sub_node = rclcpp.Node("vision_loop")
    pub = pub_node.create_publisher(Image, TOPIC, 10)
    sub = sub_node.create_subscription(Image, TOPIC, on_image, 10)  # noqa: F841
    executor = rclcpp.executors.SingleThreadedExecutor()
    executor.add_node(pub_node)
    executor.add_node(sub_node)

    deadline = time.monotonic() + 10.0
    while pub.get_subscription_count() < 1 and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.02)

    print("Streaming %s frames for loop detection ..." % kind)
    period = 1.0 / args.rate
    next_pub = time.monotonic()
    idx = -1
    for idx, frame, enc in DP.frame_source(kind, path, n_frames=args.n, gray=True, limit=args.limit):
        now = time.monotonic()
        if now < next_pub:
            time.sleep(max(0.0, next_pub - now))
        pub.publish(DP.build_image_msg(Image, frame, enc, seq=idx))
        next_pub += period
        for _ in range(3):
            executor.spin_some()
            time.sleep(0.001)
    end = time.monotonic() + 2.0
    while state["n"] < idx + 1 and time.monotonic() < end:
        executor.spin_some()
        time.sleep(0.005)

    loops = state["loops"]
    print("\nSUMMARY frames=%d confirmed_loops=%d" % (state["n"], len(loops)))
    if kind == "synthetic":
        import synthetic_loop
        s, e = synthetic_loop.loop_segment(args.n)
        print("  synthetic loop segment (ground truth): frames [%d,%d) revisit [0,%d)"
              % (s, e, e - s))
    for q, m, sc in loops:
        print("  frame %d -> frame %d  score=%.4f" % (q, m, sc))
    if rrd:
        print("Rerun recording saved: %s  (open with: rerun %s)" % (rrd, rrd))
    return loops


if __name__ == "__main__":
    main()
