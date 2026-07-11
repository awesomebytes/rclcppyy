#!/usr/bin/env python
"""
bench_vision -- the numbers behind the visual loop-closure tutorial (M5).

Reports, on the deterministic synthetic sequence (zero download):
  * ingest -- rclcppyy zero-copy (cv_kit.msg_to_mat wraps the C++ message buffer)
    vs the standard rclpy path (materialize msg.data as bytes, np.frombuffer +
    reshape + copy into a usable NumPy image), per-frame time + CPU;
  * ORB throughput (frames/s, CPU cv::ORB);
  * vocabulary: small-vocab train time; ORBvoc text-parse vs binary-cache load
    (if data/ORBvoc.txt is present);
  * DBoW2 query latency per frame;
  * loop precision/recall on the synthetic sequence.

Shared machine: run in a quiet window; treat as directional, not exact.

    pixi run -e vision bench-vision
"""
import argparse
import os
import sys
import time

import numpy as np

os.environ.setdefault("ROS_DOMAIN_ID", "50")

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, "scripts", "datasets"))
sys.path.insert(0, HERE)

import synthetic_loop  # noqa: E402
import dataset_publisher as DP  # noqa: E402
from loop_detector import LoopDetector  # noqa: E402
from rclcppyy.bringup_rclcpp import bringup_rclcpp  # noqa: E402
from rclcppyy.kits import cv_kit, dbow_kit  # noqa: E402


def _cpu_percent(fn, iters):
    """Run fn() iters times; return (avg_ms, cpu_percent) using psutil if present."""
    try:
        import psutil
        proc = psutil.Process()
        proc.cpu_percent(None)
    except Exception:
        proc = None
    t0 = time.perf_counter()
    for _ in range(iters):
        fn()
    wall = time.perf_counter() - t0
    cpu = proc.cpu_percent(None) if proc else float("nan")
    return wall / iters * 1e3, cpu


def bench_ingest_one(w, h, iters):
    """One resolution: rclcppyy ingest (msg_to_mat -> a cv::Mat ready for ORB, no
    per-frame image copy at any size) vs the standard rclpy path (buffer copy +
    frombuffer + reshape + mutable copy)."""
    import cppyy
    cppyy.include("sensor_msgs/msg/image.hpp")
    Image = cppyy.gbl.sensor_msgs.msg.Image
    frame = (np.arange(h * w, dtype=np.uint8).reshape(h, w))
    msg = DP.build_image_msg(Image, frame, "mono8")
    source = bytearray(frame.tobytes())

    def rclcppyy_path():
        mat = cv_kit.msg_to_mat(msg)   # pointer wrap; the Mat feeds ORB directly
        return int(mat.rows)

    def rclpy_path():
        raw = bytes(source)                                     # buffer copy (deserialization)
        arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w)  # view
        arr = arr.copy()                                        # mutable image to work on
        return int(arr[0, 0])

    a_ms, a_cpu = _cpu_percent(rclcppyy_path, iters)
    b_ms, b_cpu = _cpu_percent(rclpy_path, iters)
    return {"w": w, "h": h, "rclcppyy_ms": a_ms, "rclcppyy_cpu": a_cpu,
            "rclpy_ms": b_ms, "rclpy_cpu": b_cpu}


def bench_ingest(iters=2000):
    """Ingest at two resolutions -- the zero-copy pointer wrap is flat in image
    size while the rclpy copy grows with pixels (and understates rclpy, which also
    deserializes the whole Image message rclcppyy never materializes in Python)."""
    bringup_rclcpp()
    return [bench_ingest_one(640, 480, iters), bench_ingest_one(1920, 1080, iters)]


def bench_orb(n=200, nfeatures=1000):
    orb = cv_kit.create_orb(nfeatures)
    frames = [f for _, f in synthetic_loop.frames(n)]
    orb.detect_and_compute(cv_kit.numpy_to_mat(frames[0]))  # warm
    t0 = time.perf_counter()
    kps_total = 0
    for f in frames:
        kps, _ = orb.detect_and_compute(cv_kit.numpy_to_mat(f))
        kps_total += int(kps.size())
    dt = time.perf_counter() - t0
    return {"fps": n / dt, "ms": dt / n * 1e3, "avg_kps": kps_total / n}


def bench_vocab_and_loop(n=200, nfeatures=1000):
    orb = cv_kit.create_orb(nfeatures)
    descs = [orb.detect_and_compute(cv_kit.numpy_to_mat(f))[1]
             for _, f in synthetic_loop.frames(n)]
    t0 = time.perf_counter()
    voc = dbow_kit.train_vocabulary(descs, k=10, L=4, seed=0)
    train_ms = (time.perf_counter() - t0) * 1e3

    # Query latency (add all, then query each).
    db = dbow_kit.make_database(voc)
    for d in descs:
        dbow_kit.add_image(db, d)
    t0 = time.perf_counter()
    for d in descs:
        dbow_kit.query(db, d, max_results=5)
    query_ms = (time.perf_counter() - t0) / n * 1e3

    # Precision / recall vs ground truth.
    det = LoopDetector(voc, min_score=0.4, consistency_k=3, ignore_recent=25)
    detected = []
    for d in descs:
        lc = det.add_and_query(d)
        if lc:
            detected.append((lc.query_id, lc.match_id))
    seg_start, seg_end = synthetic_loop.loop_segment(n)

    def is_true(q, m):
        return seg_start <= q < seg_end and abs(m - (q - seg_start)) <= 3
    tp = sum(1 for q, m in detected if is_true(q, m))
    fp = len(detected) - tp
    precision = tp / len(detected) if detected else float("nan")
    # Recall over the revisit segment (the first ~k-1 frames can't be confirmed by
    # the temporal gate, so recall tops out just under 1.0 -- by construction).
    recall = tp / (seg_end - seg_start)
    return {"train_ms": train_ms, "words": int(voc.size()), "query_ms": query_ms,
            "loops": len(detected), "tp": tp, "fp": fp,
            "precision": precision, "recall": recall}


def bench_orbvoc():
    txt = os.path.join(REPO, "data", "ORBvoc.txt")
    if not os.path.isfile(txt):
        return None
    cache = txt + ".dbow2"
    if os.path.isfile(cache):
        os.remove(cache)
    dbow_kit.bringup_dbow()
    t0 = time.perf_counter()
    voc = dbow_kit.load_vocabulary(txt)             # text parse (+ writes cache)
    text_s = time.perf_counter() - t0
    t0 = time.perf_counter()
    dbow_kit.load_vocabulary(txt)                   # binary reload
    bin_s = time.perf_counter() - t0
    return {"words": int(voc.size()), "text_s": text_s, "bin_s": bin_s}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--iters", type=int, default=2000)
    ap.add_argument("--orbvoc", action="store_true", help="also bench ORBvoc load (slow)")
    args = ap.parse_args()

    cv_kit.warmup()
    print("== Ingest per frame (%d iters); rclcppyy = msg_to_mat (Mat ready for ORB, "
          "no image copy), rclpy = buffer copy + frombuffer + reshape + copy ==" % args.iters)
    for ing in bench_ingest(args.iters):
        print("  %dx%d: rclcppyy %.4f ms  |  rclpy %.4f ms  |  %.1fx  (rclpy also "
              "deserializes the whole msg, not counted)"
              % (ing["w"], ing["h"], ing["rclcppyy_ms"], ing["rclpy_ms"],
                 ing["rclpy_ms"] / ing["rclcppyy_ms"]))

    print("== ORB (CPU cv::ORB) ==")
    orb = bench_orb(args.n)
    print("  %.1f fps  (%.2f ms/frame, %.0f keypoints)" % (orb["fps"], orb["ms"], orb["avg_kps"]))

    print("== Vocabulary + loop detection (synthetic, %d frames) ==" % args.n)
    v = bench_vocab_and_loop(args.n)
    print("  small-vocab train  : %.0f ms (%d words, k=10 L=4)" % (v["train_ms"], v["words"]))
    print("  query latency      : %.3f ms/frame" % v["query_ms"])
    print("  loops confirmed    : %d (tp=%d fp=%d)" % (v["loops"], v["tp"], v["fp"]))
    print("  precision / recall : %.2f / %.2f  (recall over the revisit segment; the "
          "first ~k-1 frames are unconfirmable by the temporal gate)"
          % (v["precision"], v["recall"]))

    if args.orbvoc:
        ov = bench_orbvoc()
        if ov:
            print("== ORBvoc (real, %d words) ==" % ov["words"])
            print("  text parse : %.1f s ;  binary cache reload : %.2f s  (%.0fx)"
                  % (ov["text_s"], ov["bin_s"], ov["text_s"] / ov["bin_s"]))
        else:
            print("== ORBvoc: data/ORBvoc.txt not present (run dataset-orbvoc) ==")


if __name__ == "__main__":
    main()
