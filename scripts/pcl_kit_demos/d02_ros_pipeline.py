#!/usr/bin/env python
"""
pcl_kit demo 2 -- THE SHOWCASE: a self-contained ROS 2 point-cloud pipeline where
the data stays in C++ end to end and Python only orchestrates.

One process, two rclcpp nodes (no external bag):
  * a synthetic publisher -- one 100k-point ``sensor_msgs/PointCloud2`` built once
    (NumPy -> PCL -> ROS msg) and republished at ``--rate`` Hz;
  * a pipeline node -- subscribes via rclcppyy (the callback gets the **C++**
    message, not a Python copy), runs a pcl_kit VoxelGrid (0.05 m leaf),
    republishes the filtered cloud, and reports per-frame processing latency and
    input/output point counts.

Every point cloud lives in C++ the whole way: the message is a C++
``sensor_msgs::msg::PointCloud2``, ``pcl::fromROSMsg`` / the VoxelGrid /
``pcl::toROSMsg`` all run in C++, and Python never touches a point. Compare with
d03 (the same pipeline in plain rclpy + NumPy) and bench_pcl_pipeline.py.

Run: pixi run -e pcl demo-pcl-pipeline
"""
import argparse
import os
import time

import numpy as np

os.environ.setdefault("ROS_DOMAIN_ID", "43")

from rclcppyy.bringup_rclcpp import bringup_rclcpp  # noqa: E402
from rclcppyy.kits import pcl_kit                    # noqa: E402

LEAF = 0.05
N_POINTS = 100_000
TOPIC_IN = "pcl_kit/points_in"
TOPIC_OUT = "pcl_kit/points_out"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--rate", type=float, default=10.0, help="publish rate (Hz)")
    ap.add_argument("--duration", type=float, default=5.0, help="run window (s)")
    args = ap.parse_args()

    rclcpp = bringup_rclcpp()
    pcl = pcl_kit.bringup_pcl()                     # with_ros: pulls pcl_conversions
    from sensor_msgs.msg import PointCloud2

    if not rclcpp.ok():
        rclcpp.init()

    # Build the synthetic cloud ONCE (NumPy -> PCL -> C++ PointCloud2) and reuse.
    rng = np.random.default_rng(0)
    base_cloud = pcl_kit.cloud_from_numpy(rng.random((N_POINTS, 3), dtype=np.float32))
    in_msg = pcl_kit.msg_from_cloud(base_cloud)

    pub_node = rclcpp.Node("pcl_kit_publisher")
    pipe_node = rclcpp.Node("pcl_kit_pipeline")
    out_pub = pipe_node.create_publisher(PointCloud2, TOPIC_OUT, 10)

    stats = {"frames": 0, "lat": []}

    def process(msg):
        """The pipeline: C++ msg -> PCL cloud -> VoxelGrid -> C++ msg. No Python
        per-point touch; the timer is just to measure the work."""
        t0 = time.perf_counter()
        cloud = pcl_kit.cloud_from_msg(msg)
        vox = pcl.VoxelGrid[pcl.PointXYZ]()
        vox.setInputCloud(cloud.makeShared())
        vox.setLeafSize(LEAF, LEAF, LEAF)
        out = pcl.PointCloud[pcl.PointXYZ]()
        vox.filter(out)
        out_msg = pcl_kit.msg_from_cloud(out)
        out_pub.publish(out_msg)
        return int(cloud.size()), int(out.size()), (time.perf_counter() - t0) * 1e3

    def on_cloud(msg):
        n_in, n_out, lat = process(msg)
        stats["frames"] += 1
        stats["lat"].append(lat)
        print(f"STAT seq={stats['frames']} in={n_in} out={n_out} lat_ms={lat:.3f}", flush=True)

    sub = pipe_node.create_subscription(PointCloud2, TOPIC_IN, on_cloud, 10)  # noqa: F841
    in_pub = pub_node.create_publisher(PointCloud2, TOPIC_IN, 10)

    executor = rclcpp.executors.SingleThreadedExecutor()
    executor.add_node(pub_node)
    executor.add_node(pipe_node)

    # Warm up the JIT-instantiated templates once so the first measured frame is
    # steady-state (the very first filter call compiles VoxelGrid<PointXYZ> etc).
    process(in_msg)

    # Wait for the subscription to match (volatile QoS drops pre-discovery msgs).
    deadline = time.monotonic() + 15.0
    while in_pub.get_subscription_count() < 1 and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.02)

    period = 1.0 / args.rate
    next_pub = time.monotonic()
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        now = time.monotonic()
        if now >= next_pub:
            in_pub.publish(in_msg)
            next_pub += period
        executor.spin_some()
        time.sleep(0.001)

    lat = stats["lat"] or [float("nan")]
    avg = sum(lat) / len(lat)
    p_lat = sorted(lat)
    p99 = p_lat[min(len(p_lat) - 1, int(0.99 * len(p_lat)))]
    thru = stats["frames"] / args.duration
    print(f"SUMMARY frames={stats['frames']} rate_hz={args.rate} thru_msgs_s={thru:.1f} "
          f"avg_lat_ms={avg:.3f} p99_lat_ms={p99:.3f}", flush=True)
    # A normal return is clean: rclcppyy registers an ordered teardown that
    # brings the rclcpp context / DDS layer down before interpreter finalization,
    # so no os._exit dodge is needed.


if __name__ == "__main__":
    main()
