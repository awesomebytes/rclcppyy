#!/usr/bin/env python
"""
vision demo M1 -- THE SPINE: the zero-copy image path, end to end.

One process, two rclcpp nodes (like the pcl showcase): a dataset publisher emits a
sequence as ``sensor_msgs/Image``, and a subscriber node -- subscribing via
rclcppyy, so its callback receives the **C++** message -- wraps each frame as a
``cv::Mat`` with cv_kit.msg_to_mat (**no copy**: the Mat's storage IS the message's
data buffer) and logs it to Rerun (with a plausible pinhole camera). Python never
copies a pixel on the ingest path. Per-frame ingest latency is reported.

Rerun is LIVE by default when you run this interactively: a viewer window opens
and you watch the camera stream in real time. Under pytest/CI or with no display it
is headless (writes a .rrd you open later with ``rerun <file>``). Force either way
with RCLCPPYY_RERUN_SPAWN=1 (live) / =0 (headless). See scripts/vision/vision_viz.py.

    pixi run -e vision demo-vision-spine
    RCLCPPYY_RERUN_SPAWN=1 pixi run -e vision demo-vision-spine --tum data/rgbd_dataset_freiburg3_long_office_household
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
import vision_viz  # noqa: E402
from rclcppyy.bringup_rclcpp import bringup_rclcpp  # noqa: E402
from rclcppyy.kits import cv_kit  # noqa: E402

TOPIC = "vision/image"
# Plausible pinhole intrinsics for a 640x480 camera (TUM fr3 is ~535/320/248).
FX, FY, CX, CY = 525.0, 525.0, 319.5, 239.5


def log_frame(entity, mat, encoding, idx):
    rr.set_time("frame", sequence=idx)
    # Zero-copy view of the C++ Mat (aliases the message buffer, valid within this
    # callback) handed straight to Rerun -- no intermediate copy on the hot path.
    arr = cv_kit.mat_to_numpy(mat, copy=False)
    if arr.ndim == 3:
        rr.log(entity, rr.Image(arr, color_model="BGR"))
    else:
        rr.log(entity, rr.Image(arr))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tum", metavar="DIR", help="TUM sequence dir (default: synthetic)")
    ap.add_argument("--folder", metavar="DIR")
    ap.add_argument("--rate", type=float, default=15.0)
    ap.add_argument("--n", type=int, default=200, help="synthetic frame count")
    ap.add_argument("--rrd", default=os.path.join(REPO, "build", "vision", "spine.rrd"))
    args = ap.parse_args()

    kind = "tum" if args.tum else "folder" if args.folder else "synthetic"
    path = args.tum or args.folder
    session = vision_viz.init_rerun("rclcppyy_vision_spine", args.rrd,
                                    blueprint=vision_viz.blueprint_camera_perf("ingest (ms/frame)"))

    rclcpp = bringup_rclcpp()
    cv_kit.warmup()
    import cppyy
    cppyy.include("sensor_msgs/msg/image.hpp")
    Image = cppyy.gbl.sensor_msgs.msg.Image
    if not rclcpp.ok():
        rclcpp.init()

    rr.log("camera", rr.Pinhole(resolution=[640, 480], focal_length=[FX, FY],
                                principal_point=[CX, CY]), static=True)
    # Style the "how fast" plot once (static): a named, colored time series.
    rr.log("perf/ingest_ms", rr.SeriesLines(names=["ingest ms"], colors=[(80, 170, 255)],
                                            widths=[2.0]), static=True)

    stats = {"n": 0, "ingest_ms": []}

    def on_image(msg):
        t0 = time.perf_counter()
        mat = cv_kit.msg_to_mat(msg)          # zero-copy wrap
        gray = cv_kit.to_gray(mat)            # (spine just measures ingest+decode)
        _ = int(gray.rows)
        dt = (time.perf_counter() - t0) * 1e3
        log_frame("camera/image", mat, str(msg.encoding), stats["n"])
        rr.log("perf/ingest_ms", rr.Scalars(dt))   # per-frame "how fast", live
        stats["n"] += 1
        stats["ingest_ms"].append(dt)
        if stats["n"] % 25 == 0:
            print("  frame %d ingest=%.3f ms" % (stats["n"], dt), flush=True)

    pub_node = rclcpp.Node("vision_publisher")
    sub_node = rclcpp.Node("vision_spine")
    pub = pub_node.create_publisher(Image, TOPIC, 10)
    sub = sub_node.create_subscription(Image, TOPIC, on_image, 10)  # noqa: F841

    executor = rclcpp.executors.SingleThreadedExecutor()
    executor.add_node(pub_node)
    executor.add_node(sub_node)

    # Wait for discovery so no frames are dropped.
    deadline = time.monotonic() + 10.0
    while pub.get_subscription_count() < 1 and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.02)

    frames = DP.frame_source(kind, path, n_frames=args.n)
    period = 1.0 / args.rate
    next_pub = time.monotonic()
    for idx, frame, enc in frames:
        now = time.monotonic()
        if now < next_pub:
            time.sleep(max(0.0, next_pub - now))
        pub.publish(DP.build_image_msg(Image, frame, enc, seq=idx))
        next_pub += period
        for _ in range(3):
            executor.spin_some()
            time.sleep(0.001)
    # Drain remaining callbacks.
    end = time.monotonic() + 1.0
    while stats["n"] < idx + 1 and time.monotonic() < end:
        executor.spin_some()
        time.sleep(0.005)

    ms = stats["ingest_ms"] or [float("nan")]
    ms_sorted = sorted(ms)
    print("\nSUMMARY frames=%d ingest_avg_ms=%.3f ingest_p50_ms=%.3f ingest_max_ms=%.3f"
          % (stats["n"], sum(ms) / len(ms), ms_sorted[len(ms_sorted) // 2], max(ms)))
    vision_viz.announce(session)


if __name__ == "__main__":
    main()
