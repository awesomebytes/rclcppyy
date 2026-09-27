#!/usr/bin/env python3
"""Unchanged stock rclpy subscriber for direct native rosout proof."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import time

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node


parser = argparse.ArgumentParser()
parser.add_argument("--logger-name", required=True)
parser.add_argument("--node-name", required=True)
parser.add_argument("--node-namespace", required=True)
parser.add_argument("--expected-json", required=True)
parser.add_argument("--window-sec", type=float, default=1.0)
args = parser.parse_args()

expected = {
    (item["name"], item["message"])
    for item in json.loads(args.expected_json)
}
observed = {}


def callback(message):
    key = (message.name, message.msg)
    if key not in expected or key in observed:
        return
    observed[key] = {
        "name": message.name,
        "message": message.msg,
        "level": int(message.level),
        "file": Path(message.file).name,
        "function": message.function,
        "line": int(message.line),
    }


def server_rosout_present(node):
    for endpoint in node.get_publishers_info_by_topic("/rosout"):
        if (
            endpoint.node_name == args.node_name
            and endpoint.node_namespace == args.node_namespace
        ):
            return True
    return False


rclpy.init(args=[
    "--ros-args",
    "--disable-stdout-logs",
    "--disable-external-lib-logs",
])
node = Node(
    "stock_logging_rosout_peer_%d" % os.getpid(),
    enable_rosout=False,
)
node.create_subscription(
    Log, "/rosout", callback, 10)
try:
    print("RCLCPPYY_LOGGING_ROSOUT_READY", flush=True)
    deadline = time.monotonic() + args.window_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)

    records = [observed[key] for key in sorted(observed)]
    print(
        "RCLCPPYY_LOGGING_ROSOUT_OBSERVED "
        + json.dumps(records, sort_keys=True),
        flush=True,
    )
    if sys.stdin.readline().strip() != "node-destroyed":
        raise RuntimeError("rosout peer did not receive teardown control")

    teardown_deadline = time.monotonic() + 3.0
    while server_rosout_present(node) and time.monotonic() < teardown_deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
    print(
        "RCLCPPYY_LOGGING_ROSOUT_RESULT "
        + json.dumps({
            "records": records,
            "server_publisher_removed": not server_rosout_present(node),
        }, sort_keys=True),
        flush=True,
    )
finally:
    node.destroy_node()
    rclpy.shutdown()
