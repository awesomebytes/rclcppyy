#!/usr/bin/env python3
"""Synthetic TF-storm publisher (rclcppyy / C++ TFMessage).

Publishes a chain of ``--frames`` transforms (world -> link_0 -> ... -> link_{N-1})
as a single ``tf2_msgs/TFMessage`` on ``/tf`` at ``--rate`` Hz, so the aggregate
ingest load is ``frames * rate`` transforms/second. Each link is offset ``+0.1 m`` in
x from its parent, so ``lookup_transform("world", "link_{N-1}")`` should give
``x = 0.1 * N`` -- a cheap correctness anchor for the listeners.

The message is built once and only its timestamps are advanced each tick (in a C++
helper), so the publisher stays lean and is never the bottleneck. Publishing goes
through rclcppyy so the whole publish path is C++ too.

    python tf_storm_publisher.py --frames 50 --rate 100 --duration 8
"""
import argparse
import os
import time

import cppyy

from rclcppyy.bringup_rclcpp import bringup_rclcpp, add_ros2_include_paths
from rclcppyy.kits import cppyy_kit


def build_message(frames):
    gm = cppyy.gbl.geometry_msgs.msg
    msg = cppyy.gbl.tf2_msgs.msg.TFMessage()
    for i in range(frames):
        ts = gm.TransformStamped()
        ts.header.frame_id = "world" if i == 0 else "link_%d" % (i - 1)
        ts.child_frame_id = "link_%d" % i
        ts.transform.translation.x = 0.1
        ts.transform.rotation.w = 1.0
        msg.transforms.push_back(ts)
    return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=50)
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--duration", type=float, default=0.0, help="0 = run until killed")
    ap.add_argument("--topic", default="/tf")
    args = ap.parse_args()

    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    add_ros2_include_paths()
    conda = os.environ["CONDA_PREFIX"]
    cppyy.add_include_path(os.path.join(conda, "include"))
    cppyy.include("tf2_msgs/msg/tf_message.hpp")
    cppyy_kit.load_libraries(
        ["libtf2_msgs__rosidl_typesupport_cpp.so"], [os.path.join(conda, "lib")])
    # Advance all header stamps in one C++ pass (keeps the hot loop off Python).
    cppyy.cppdef(r"""
    namespace rclcppyy_tf_storm {
    inline void stamp_all(tf2_msgs::msg::TFMessage& m, int sec, unsigned nsec) {
      for (auto& t : m.transforms) { t.header.stamp.sec = sec; t.header.stamp.nanosec = nsec; }
    }}
    """)
    stamp_all = cppyy.gbl.rclcppyy_tf_storm.stamp_all

    node = rclcpp.Node("tf_storm_publisher")
    clock = node.get_clock()
    pub = node.create_publisher[cppyy.gbl.tf2_msgs.msg.TFMessage](args.topic, 100)
    msg = build_message(args.frames)

    period = 1.0 / args.rate
    agg = args.frames * args.rate
    print("[storm] publishing %d frames @ %.0f Hz = %.0f transforms/s on %s"
          % (args.frames, args.rate, agg, args.topic), flush=True)

    t0 = time.perf_counter()
    next_tick = t0
    ticks = 0
    last_report = t0
    while True:
        now = clock.now().nanoseconds()
        stamp_all(msg, int(now // 1_000_000_000), int(now % 1_000_000_000))
        pub.publish(msg)
        ticks += 1
        next_tick += period
        sleep = next_tick - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)
        wall = time.perf_counter()
        if wall - last_report >= 1.0:
            print("[storm] %d ticks (%.0f transforms) in %.1fs"
                  % (ticks, ticks * args.frames, wall - t0), flush=True)
            last_report = wall
        if args.duration and (wall - t0) >= args.duration:
            break
    print("[storm] done", flush=True)


if __name__ == "__main__":
    main()
