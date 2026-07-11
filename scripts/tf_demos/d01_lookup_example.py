#!/usr/bin/env python3
"""Minimal rclcppyy.tf lookup example.

Publishes a tiny transform tree (world -> base -> sensor) on ``/tf`` while a C++
``tf2_ros::TransformListener`` (running on its own C++ thread, via rclcppyy.tf)
ingests it, then looks the composed transform up from Python and prints it. The
listener never runs a Python callback -- ingest is entirely in C++.

    pixi run demo-tf-lookup
"""
import time

import cppyy

import rclcppyy
from rclcppyy import tf


def make_ts(parent, child, x, y, z):
    ts = cppyy.gbl.geometry_msgs.msg.TransformStamped()
    ts.header.frame_id = parent
    ts.child_frame_id = child
    ts.transform.translation.x = float(x)
    ts.transform.translation.y = float(y)
    ts.transform.translation.z = float(z)
    ts.transform.rotation.w = 1.0
    return ts


def main():
    rclcpp = rclcppyy.bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()

    listener = tf.TransformListener()          # own node + own C++ spin thread
    node = rclcpp.Node("tf_lookup_example_pub")
    pub = node.create_publisher[cppyy.gbl.tf2_msgs.msg.TFMessage]("/tf", 100)

    msg = cppyy.gbl.tf2_msgs.msg.TFMessage()
    msg.transforms.push_back(make_ts("world", "base", 1.0, 0.0, 0.0))
    msg.transforms.push_back(make_ts("base", "sensor", 0.0, 2.0, 0.0))
    for _ in range(10):
        pub.publish(msg)
        time.sleep(0.02)

    ts = listener.lookup_transform("world", "sensor", timeout=2.0)
    t = ts.transform.translation
    print("world <- sensor : translation = (%.3f, %.3f, %.3f)  [expect (1, 2, 0)]"
          % (t.x, t.y, t.z))
    print("frames in buffer:", listener.get_frame_names())
    assert abs(t.x - 1.0) < 1e-6 and abs(t.y - 2.0) < 1e-6, "unexpected transform"
    print("OK -- ingested by the C++ listener, looked up from Python.")


if __name__ == "__main__":
    main()
