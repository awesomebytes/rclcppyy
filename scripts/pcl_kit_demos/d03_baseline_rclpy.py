#!/usr/bin/env python
"""
pcl_kit demo 3 -- the honest baseline: the SAME point-cloud pipeline as d02, but
in plain rclpy + NumPy, with no PCL and no C++.

One process, two rclpy nodes (mirrors d02):
  * a synthetic publisher -- one 100k-point ``sensor_msgs/PointCloud2`` built once
    (via ``sensor_msgs_py``) and republished at ``--rate`` Hz;
  * a pipeline node -- subscribes, reads the points into NumPy
    (``read_points_numpy``), voxel-downsamples them in NumPy (centroid per
    0.05 m cell, the same semantics as PCL's VoxelGrid), rebuilds a PointCloud2
    (``create_cloud_xyz32``), and republishes -- reporting the same per-frame stats
    as d02 so bench_pcl_pipeline.py can compare them apples to apples.

This is what you would write without the kit: every message is deserialized into
Python/NumPy, processed in Python, and re-serialized. Compare the latency, CPU,
and line count against d02.

Run: python scripts/pcl_kit_demos/d03_baseline_rclpy.py   (or via bench-pcl)
"""
import argparse
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

os.environ.setdefault("ROS_DOMAIN_ID", "43")

LEAF = 0.05
N_POINTS = 100_000
TOPIC_IN = "pcl_base/points_in"
TOPIC_OUT = "pcl_base/points_out"


def voxel_downsample(points, leaf):
    """Centroid-per-voxel downsample in NumPy -- the same semantics as PCL's
    VoxelGrid (each occupied cell collapses to the mean of its points)."""
    keys = np.floor(points / leaf).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    inverse = inverse.reshape(-1)
    n_vox = int(inverse.max()) + 1 if inverse.size else 0
    sums = np.zeros((n_vox, 3), dtype=np.float64)
    np.add.at(sums, inverse, points)
    counts = np.bincount(inverse, minlength=n_vox)
    return (sums / counts[:, None]).astype(np.float32)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--rate", type=float, default=10.0, help="publish rate (Hz)")
    ap.add_argument("--duration", type=float, default=5.0, help="run window (s)")
    args = ap.parse_args()

    rclpy.init()

    # Build the synthetic cloud ONCE and reuse it (matches d02).
    rng = np.random.default_rng(0)
    in_msg = pc2.create_cloud_xyz32(
        PointCloud2().header, rng.random((N_POINTS, 3), dtype=np.float32))

    pub_node = Node("pcl_base_publisher")
    pipe_node = Node("pcl_base_pipeline")
    out_pub = pipe_node.create_publisher(PointCloud2, TOPIC_OUT, 10)

    stats = {"frames": 0, "lat": []}

    def process(msg):
        t0 = time.perf_counter()
        pts = pc2.read_points_numpy(msg, field_names=("x", "y", "z"))
        down = voxel_downsample(pts, LEAF)
        out_msg = pc2.create_cloud_xyz32(msg.header, down)
        out_pub.publish(out_msg)
        return pts.shape[0], down.shape[0], (time.perf_counter() - t0) * 1e3

    def on_cloud(msg):
        n_in, n_out, lat = process(msg)
        stats["frames"] += 1
        stats["lat"].append(lat)
        print(f"STAT seq={stats['frames']} in={n_in} out={n_out} lat_ms={lat:.3f}", flush=True)

    sub = pipe_node.create_subscription(PointCloud2, TOPIC_IN, on_cloud, 10)  # noqa: F841
    in_pub = pub_node.create_publisher(PointCloud2, TOPIC_IN, 10)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(pub_node)
    executor.add_node(pipe_node)

    # Warm up NumPy machinery once for parity with d02's JIT warm-up.
    process(in_msg)

    deadline = time.monotonic() + 15.0
    while in_pub.get_subscription_count() < 1 and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)

    period = 1.0 / args.rate
    next_pub = time.monotonic()
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        now = time.monotonic()
        if now >= next_pub:
            in_pub.publish(in_msg)
            next_pub += period
        executor.spin_once(timeout_sec=0.001)

    lat = stats["lat"] or [float("nan")]
    avg = sum(lat) / len(lat)
    p_lat = sorted(lat)
    p99 = p_lat[min(len(p_lat) - 1, int(0.99 * len(p_lat)))]
    thru = stats["frames"] / args.duration
    print(f"SUMMARY frames={stats['frames']} rate_hz={args.rate} thru_msgs_s={thru:.1f} "
          f"avg_lat_ms={avg:.3f} p99_lat_ms={p99:.3f}", flush=True)
    # Plain rclpy + NumPy: a normal return exits cleanly (this baseline never
    # needed the os._exit dodge the C++/cppyy demos carried).


if __name__ == "__main__":
    main()
