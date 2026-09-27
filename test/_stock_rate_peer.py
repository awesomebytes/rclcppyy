#!/usr/bin/env python3
"""Unchanged stock rclpy Rate scenario, for the direct_cpp DirectRate differential.

Deliberately does not import rclcppyy: this proves what stock rclpy's own
Rate/Timer/Executor combination does on its own, as the reference the
direct_cpp DirectRate scenario is compared against. Stock's Rate wraps a
Timer whose callback sets a threading.Event, so it needs a spinning executor
in a background thread while sleep() blocks on the main thread.
"""

import json
import os
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


RATE_HZ = 10.0
ITERATIONS = 5

rclpy.init(args=[])
node = Node("stock_rate_peer_%d" % os.getpid())
executor = SingleThreadedExecutor()
executor.add_node(node)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()

rate = node.create_rate(RATE_HZ)
start = time.monotonic()
timestamps = []
for _ in range(ITERATIONS):
    rate.sleep()
    timestamps.append(time.monotonic())
elapsed = time.monotonic() - start

node.destroy_rate(rate)
executor.shutdown(timeout_sec=2.0)
node.destroy_node()
rclpy.shutdown()
spin_thread.join(timeout=2.0)

print("STOCK_RATE_OBSERVATIONS " + json.dumps({
    "elapsed_s": elapsed,
    "timestamps": timestamps,
}))
