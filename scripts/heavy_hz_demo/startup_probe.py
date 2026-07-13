#!/usr/bin/env python3
"""Single-process rclcppyy bringup probe for the cold-vs-warm startup story.

Measures the two numbers the story is about, in one deterministic process (no
publisher, subscriber traffic, executor, or cross-process discovery -- so there is
nothing to hang or race):

  * ``rclcpp C++ headers loaded (X.Xs)`` -- printed by bringup; the header parse
    cost the Cling PCH eliminates (cold ~1.7s, warm ~0.0s);
  * ``READY elapsed_s=Y.YY`` -- wall time from process start through
    ``enable_cpp_acceleration()`` + ``rclpy.init()`` + node creation + creating a
    subscription on the heavy topic, i.e. how long until an accelerated subscriber
    node is up and subscribed.

On a cold run the auto-PCH build is scheduled at this process's exit.

Usage: startup_probe.py   (no args; the run_heavy_hz orchestrator pins
XDG_CACHE_HOME per run to control cold vs warm).
"""
import os
import time as _time

_PROC_START = _time.perf_counter()

# --- the one line the whole demo is about (always on for the probe) ---
import rclcppyy; rclcppyy.enable_cpp_acceleration()

import time  # noqa: E402

import rclpy  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

import _hz_common as hz  # noqa: E402


def main():
    topic = os.environ.get("HEAVY_HZ_TOPIC", "heavy_startup_probe")
    rclpy.init()
    node = rclpy.create_node("startup_probe")
    node.create_subscription(Image, topic, lambda _msg: None, hz.sensor_qos(hz.DEFAULT_DEPTH))
    print("READY elapsed_s=%.2f" % (time.perf_counter() - _PROC_START), flush=True)
    # No spin: the point is time-to-ready, not traffic. Shut the context down
    # cleanly (this is also when the auto-PCH build is scheduled on a cold run).
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
