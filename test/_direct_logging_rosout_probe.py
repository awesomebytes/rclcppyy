#!/usr/bin/env python3
"""Stock/direct node logging against an external unchanged stock subscriber."""

from __future__ import annotations

import argparse
import gc
import importlib
import json
from pathlib import Path
import select
import subprocess
import sys
import time


parser = argparse.ArgumentParser()
parser.add_argument("--backend", choices=("stock", "direct"), required=True)
parser.add_argument("--enable-rosout", choices=("yes", "no"), required=True)
args = parser.parse_args()
enabled = args.enable_rosout == "yes"

if args.backend == "direct":
    import rclcppyy

    rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402


def poison_direct_boundaries():
    if args.backend != "direct":
        return

    def forbidden(*_args, **_kwargs):
        raise AssertionError("conversion or serialization entered rosout proof")

    kit = importlib.import_module("rclcpp_kit")
    bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
    serialization = importlib.import_module("rclcpp_kit.serialization")
    rclpy_serialization = importlib.import_module("rclpy.serialization")
    kit.convert_python_msg_to_cpp = forbidden
    bringup.convert_python_msg_to_cpp = forbidden
    serialization.serialize_message = forbidden
    serialization.deserialize_message = forbidden
    serialization.serialized_message_from_bytes = forbidden
    serialization.serialized_message_to_bytes = forbidden
    rclpy_serialization.serialize_message = forbidden
    rclpy_serialization.deserialize_message = forbidden


def read_line(process, deadline, prefix):
    while time.monotonic() < deadline:
        ready, _, _ = select.select([process.stdout], [], [], 0.05)
        if not ready:
            if process.poll() is not None:
                break
            continue
        line = process.stdout.readline().strip()
        if line.startswith(prefix):
            return line
    raise RuntimeError("timed out waiting for %s" % prefix)


def emit(logger, message):
    return logger.info(message)


poison_direct_boundaries()
rclpy.init(args=[
    "--ros-args",
    "--disable-stdout-logs",
    "--disable-external-lib-logs",
])
node = None
peer = None
try:
    suffix = "case"
    node = Node(
        "logging_rosout_" + suffix,
        namespace="/logging_audit",
        enable_rosout=enabled,
    )
    logger = node.get_logger()
    first_child = logger.get_child("child")
    surviving_child = logger.get_child("child")
    grandchild = logger.get_child("child").get_child("grandchild")
    expected = [
        {"name": logger.name, "message": "parent_" + suffix},
        {"name": first_child.name, "message": "child_before_drop_" + suffix},
        {"name": surviving_child.name, "message": "child_after_drop_" + suffix},
        {"name": grandchild.name, "message": "grandchild_" + suffix},
    ]

    peer_path = Path(__file__).with_name("_stock_logging_rosout_peer.py")
    peer = subprocess.Popen(
        [
            sys.executable,
            str(peer_path),
            "--logger-name", logger.name,
            "--node-name", node.get_name(),
            "--node-namespace", node.get_namespace(),
            "--expected-json", json.dumps(expected),
        ],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    read_line(
        peer,
        time.monotonic() + 10.0,
        "RCLCPPYY_LOGGING_ROSOUT_READY",
    )
    discovery_deadline = time.monotonic() + 3.0
    while (
        node.count_subscribers("/rosout") == 0
        and time.monotonic() < discovery_deadline
    ):
        time.sleep(0.02)
    if node.count_subscribers("/rosout") == 0:
        raise RuntimeError("direct logger did not discover stock rosout subscriber")
    time.sleep(0.1)

    emit(first_child, "child_before_drop_" + suffix)
    first_child = None
    gc.collect()
    emission_deadline = time.monotonic() + 1.1
    while time.monotonic() < emission_deadline:
        emit(logger, "parent_" + suffix)
        emit(surviving_child, "child_after_drop_" + suffix)
        emit(grandchild, "grandchild_" + suffix)
        time.sleep(0.02)

    observed_line = read_line(
        peer,
        time.monotonic() + 5.0,
        "RCLCPPYY_LOGGING_ROSOUT_OBSERVED ",
    )
    node.destroy_node()
    node = None
    peer.stdin.write("node-destroyed\n")
    peer.stdin.flush()
    result_line = read_line(
        peer,
        time.monotonic() + 5.0,
        "RCLCPPYY_LOGGING_ROSOUT_RESULT ",
    )
    stdout, stderr = peer.communicate(timeout=5.0)
    if peer.returncode != 0:
        raise RuntimeError(
            "stock rosout peer failed\nstdout:\n%s\nstderr:\n%s" %
            (stdout, stderr))
    result = json.loads(
        result_line[len("RCLCPPYY_LOGGING_ROSOUT_RESULT "):])
    result.update({
        "enabled": enabled,
        "expected": expected,
        "observed_before_teardown": json.loads(
            observed_line[len("RCLCPPYY_LOGGING_ROSOUT_OBSERVED "):]),
    })
    print("RCLCPPYY_LOGGING_ROSOUT_PROBE " + json.dumps({
        "backend": args.backend,
        "result": result,
        "boundary": {
            "conversion_forbidden": args.backend == "direct",
            "serialization_forbidden": args.backend == "direct",
        },
    }, sort_keys=True), flush=True)
finally:
    if node is not None:
        node.destroy_node()
    if peer is not None and peer.poll() is None:
        peer.kill()
        peer.communicate()
    rclpy.shutdown()
