#!/usr/bin/env python
"""
vision demo M2 -- FEATURES: C++ cv::ORB on every frame, keypoints overlaid in Rerun.

Extends the spine: each C++ ``sensor_msgs/Image`` is wrapped zero-copy
(cv_kit.msg_to_mat), converted to gray, and run through C++ ``cv::ORB`` (cv_kit's
OrbDetector -- CPU here; the same call switches to ``cv::cuda::ORB`` when a CUDA
OpenCV build is present, see docs/vision/CUDA_OPENCV.md). Keypoints are logged as a
Rerun Points2D overlay on the image; the ORB descriptor is the Nx32 CV_8U matrix
DBoW2 will consume in M3. Reports ORB throughput (frames/s).

Honest note: cv2.ORB would give similar per-frame numbers -- the win here is not the
detector call, it is *composition*: the frame never leaves C++ between the rclcppyy
subscription, the Mat, ORB, and (M3) the DBoW2 query.

    pixi run -e vision demo-vision-features
    RCLCPPYY_RERUN_SPAWN=1 pixi run -e vision demo-vision-features --tum data/<tum-seq>
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

import dataset_publisher as DP  # noqa: E402
from rclcppyy.bringup_rclcpp import bringup_rclcpp  # noqa: E402
from rclcppyy.kits import cv_kit  # noqa: E402

TOPIC = "vision/image"
FX, FY, CX, CY = 525.0, 525.0, 319.5, 239.5


def init_rerun(default_rrd):
    rr.init("rclcppyy_vision_features")
    if os.environ.get("RCLCPPYY_RERUN_SPAWN") == "1":
        rr.spawn()
        return None
    os.makedirs(os.path.dirname(default_rrd), exist_ok=True)
    rr.save(default_rrd)
    return default_rrd


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tum", metavar="DIR")
    ap.add_argument("--folder", metavar="DIR")
    ap.add_argument("--rate", type=float, default=15.0)
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--nfeatures", type=int, default=1000)
    ap.add_argument("--rrd", default=os.path.join(REPO, "build", "vision", "features.rrd"))
    args = ap.parse_args()

    kind = "tum" if args.tum else "folder" if args.folder else "synthetic"
    path = args.tum or args.folder
    rrd = init_rerun(args.rrd)

    rclcpp = bringup_rclcpp()
    cv_kit.warmup(args.nfeatures)
    orb = cv_kit.create_orb(args.nfeatures)
    print("ORB backend: %s" % ("cv::cuda::ORB (GPU)" if orb.use_cuda else "cv::ORB (CPU)"))
    import cppyy
    cppyy.include("sensor_msgs/msg/image.hpp")
    Image = cppyy.gbl.sensor_msgs.msg.Image
    if not rclcpp.ok():
        rclcpp.init()

    rr.log("camera", rr.Pinhole(resolution=[640, 480], focal_length=[FX, FY],
                                principal_point=[CX, CY]), static=True)

    stats = {"n": 0, "orb_ms": [], "kps": []}

    def on_image(msg):
        rr.set_time("frame", sequence=stats["n"])
        mat = cv_kit.msg_to_mat(msg)
        gray = cv_kit.to_gray(mat)
        t0 = time.perf_counter()
        kps, desc = orb.detect_and_compute(gray)
        stats["orb_ms"].append((time.perf_counter() - t0) * 1e3)
        xy = cv_kit.keypoints_to_numpy(kps)
        arr = cv_kit.mat_to_numpy(mat, copy=False)
        if arr.ndim == 3:
            rr.log("camera/image", rr.Image(arr, color_model="BGR"))
        else:
            rr.log("camera/image", rr.Image(arr))
        rr.log("camera/image/keypoints", rr.Points2D(xy, radii=2.0))
        stats["kps"].append(int(kps.size()))
        stats["n"] += 1
        if stats["n"] % 25 == 0:
            print("  frame %d: %d keypoints, desc %dx%d, orb=%.2f ms"
                  % (stats["n"], int(kps.size()), int(desc.rows), int(desc.cols),
                     stats["orb_ms"][-1]), flush=True)

    pub_node = rclcpp.Node("vision_publisher")
    sub_node = rclcpp.Node("vision_features")
    pub = pub_node.create_publisher(Image, TOPIC, 10)
    sub = sub_node.create_subscription(Image, TOPIC, on_image, 10)  # noqa: F841
    executor = rclcpp.executors.SingleThreadedExecutor()
    executor.add_node(pub_node)
    executor.add_node(sub_node)

    deadline = time.monotonic() + 10.0
    while pub.get_subscription_count() < 1 and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.02)

    period = 1.0 / args.rate
    next_pub = time.monotonic()
    idx = -1
    for idx, frame, enc in DP.frame_source(kind, path, n_frames=args.n):
        now = time.monotonic()
        if now < next_pub:
            time.sleep(max(0.0, next_pub - now))
        pub.publish(DP.build_image_msg(Image, frame, enc, seq=idx))
        next_pub += period
        for _ in range(3):
            executor.spin_some()
            time.sleep(0.001)
    end = time.monotonic() + 1.0
    while stats["n"] < idx + 1 and time.monotonic() < end:
        executor.spin_some()
        time.sleep(0.005)

    ms = stats["orb_ms"] or [float("nan")]
    avg_ms = sum(ms) / len(ms)
    avg_kps = sum(stats["kps"]) / max(1, len(stats["kps"]))
    print("\nSUMMARY frames=%d orb_avg_ms=%.2f orb_fps=%.1f avg_keypoints=%.0f backend=%s"
          % (stats["n"], avg_ms, 1000.0 / avg_ms, avg_kps,
             "GPU" if orb.use_cuda else "CPU"))
    if rrd:
        print("Rerun recording saved: %s  (open with: rerun %s)" % (rrd, rrd))


if __name__ == "__main__":
    main()
