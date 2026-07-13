#!/usr/bin/env python3
"""Heavy-topic publisher for the ``ros2 topic hz`` demo.

Publishes a large ``sensor_msgs/Image`` (default 1024x1024 bgr8 = 3.0 MB) at a
demanding rate. The payload is built once and re-published every tick -- only the
header's sequence number (carried in ``frame_id``) and timestamp change -- so the
publisher's per-message cost is tiny and it is never the bottleneck under test.

Acceleration is toggled by the ``HEAVY_HZ_ACCEL`` env var (see _hz_common). The
demo runs the publisher accelerated so the payload lives as a C++ object end to
end; the same script also runs on plain rclpy (used by the headless test).

Usage: heavy_publisher.py [rate_hz] [width] [height]   (each optional; env-backed)
"""
import os
import sys

# --- the one line the whole demo is about (gated so one script runs both ways) ---
if os.environ.get("HEAVY_HZ_ACCEL") == "1":
    import rclcppyy; rclcppyy.enable_cpp_acceleration()

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import _hz_common as hz


class HeavyImagePublisher(Node):
    def __init__(self, rate_hz, width, height):
        super().__init__("heavy_image_publisher")
        self.rate_hz = rate_hz
        self.accel = hz.accel_enabled()
        self.publisher = self.create_publisher(
            Image, hz.DEFAULT_TOPIC, hz.sensor_qos(hz.DEFAULT_DEPTH)
        )
        # Build the big payload exactly once; every tick only re-stamps + republishes.
        self.msg = hz.build_image(width, height, self.accel)
        self.seq = 0
        # Independent, publisher-side achieved-rate report (so the publisher's own
        # output confirms it hits its target, separate from any subscriber's view).
        self._window_count = 0
        self._window_t0 = time.monotonic()
        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)
        print(
            "Heavy publisher up: variant=%s topic=%s %dx%d (%.2f MB) target=%.0f Hz"
            % (hz.variant_label(self.accel), hz.DEFAULT_TOPIC, width, height,
               hz.payload_mb(width, height), rate_hz),
            flush=True,
        )

    def timer_callback(self):
        now = time.time_ns()
        self.msg.header.stamp.sec = now // 1_000_000_000
        self.msg.header.stamp.nanosec = now % 1_000_000_000
        self.msg.header.frame_id = str(self.seq)
        self.publisher.publish(self.msg)
        self.seq += 1
        self._window_count += 1
        elapsed = time.monotonic() - self._window_t0
        if elapsed >= 1.0:
            print("PUB achieved_hz=%.1f published=%d target=%.0f"
                  % (self._window_count / elapsed, self.seq, self.rate_hz), flush=True)
            self._window_count = 0
            self._window_t0 = time.monotonic()


def main():
    rate_hz = float(sys.argv[1]) if len(sys.argv) > 1 else hz.DEFAULT_RATE
    width = int(sys.argv[2]) if len(sys.argv) > 2 else hz.DEFAULT_WIDTH
    height = int(sys.argv[3]) if len(sys.argv) > 3 else hz.DEFAULT_HEIGHT
    rclpy.init()
    node = HeavyImagePublisher(rate_hz, width, height)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
