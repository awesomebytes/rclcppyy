#!/usr/bin/env python3
"""``ros2 topic hz``-equivalent subscriber for the heavy-topic demo.

Subscribes to the heavy image topic and reports, once per second: the received
rate (hz), received-vs-expected for the window, dropped count (from gaps in the
header sequence number), and average end-to-end latency. It reads only the small
header fields -- never the multi-megabyte ``data`` payload -- so the cost
difference between the two runs is purely the framework's per-message handling:

  * plain rclpy deserialises the *entire* message (including the whole data array)
    into a Python object before the callback, every message;
  * with acceleration the message stays a C++ object and only the header fields
    the callback touches are read -- the payload is never copied into Python.

This is the SAME script for both runs. The only difference is whether this line
near the top executes (gated on HEAVY_HZ_ACCEL)::

    import rclcppyy; rclcppyy.enable_cpp_acceleration()

Usage: hz_subscriber.py [target_rate_hz]   (target rate only feeds the
expected-count column; the measured hz is independent of it).
"""
import os
import sys
import time as _time

# Wall clock at the very first line of the process, so "time to first message"
# below includes import + rclcpp bringup -- the cost the cold-vs-warm story is about.
_PROC_START = _time.perf_counter()

# --- the one line the whole demo is about (gated so one script runs both ways) ---
if os.environ.get("HEAVY_HZ_ACCEL") == "1":
    import rclcppyy; rclcppyy.enable_cpp_acceleration()

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import _hz_common as hz


class HzSubscriber(Node):
    def __init__(self, target_rate_hz):
        super().__init__("hz_subscriber")
        self.accel = hz.accel_enabled()
        self.target_rate_hz = target_rate_hz
        self.subscription = self.create_subscription(
            Image, hz.DEFAULT_TOPIC, self.on_image, hz.sensor_qos(hz.DEFAULT_DEPTH)
        )
        # Per-window accumulators, reset each time a stat line is printed.
        self.window_start = time.monotonic()
        self.window_count = 0
        self.window_dropped = 0
        self.window_lat_sum_ms = 0.0
        self.last_seq = None
        self.payload_mb = 0.0
        self.first_msg_reported = False
        print(
            "hz subscriber up: variant=%s topic=%s (reads header only, not data)"
            % (hz.variant_label(self.accel), hz.DEFAULT_TOPIC),
            flush=True,
        )

    def on_image(self, msg):
        recv_ns = time.time_ns()
        if not self.first_msg_reported:
            self.first_msg_reported = True
            print("FIRST_MSG elapsed_s=%.2f" % (time.perf_counter() - _PROC_START),
                  flush=True)
        # Header-only reads: sequence (for drop detection) + stamp (for latency).
        # str() normalises the C++ std::string (accel) and the Python str (rclpy).
        frame_id = str(msg.header.frame_id)
        seq = int(frame_id) if frame_id else 0
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.window_lat_sum_ms += (recv_ns - stamp_ns) / 1e6

        if self.last_seq is not None and seq > self.last_seq + 1:
            self.window_dropped += seq - self.last_seq - 1
        self.last_seq = seq
        self.window_count += 1
        if self.payload_mb == 0.0:
            # width/height are small header fields; derive MB without touching data.
            self.payload_mb = (msg.height * msg.step) / (1024.0 * 1024.0)

        elapsed = time.monotonic() - self.window_start
        if elapsed >= 1.0:
            rate = self.window_count / elapsed
            expected = self.target_rate_hz * elapsed
            avg_lat = self.window_lat_sum_ms / self.window_count
            print(
                "STAT hz=%.2f recv=%d expected=%d dropped=%d avg_lat_ms=%.1f "
                "window_s=%.2f payload_mb=%.2f"
                % (rate, self.window_count, round(expected), self.window_dropped,
                   avg_lat, elapsed, self.payload_mb),
                flush=True,
            )
            self.window_start = time.monotonic()
            self.window_count = 0
            self.window_dropped = 0
            self.window_lat_sum_ms = 0.0


def main():
    target_rate_hz = float(sys.argv[1]) if len(sys.argv) > 1 else hz.DEFAULT_RATE
    rclpy.init()
    node = HzSubscriber(target_rate_hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
