#!/usr/bin/env python3
"""Generic isolated publisher/subscriber for the declarative benchmark matrix."""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time

from _backend_marker import emit_native_backend, emit_status_backend, emit_stock_backend
from _benchmark_matrix import WORKLOADS
from _benchmark_protocol import (
    READY_PREFIX,
    RESULT_PREFIX,
    STARTED_PREFIX,
    MeasurementState,
    emit,
    event_document,
    validate_control,
)


def _padding(size):
    return "x" * size


class PythonCodec:
    def __init__(self, workload, payload_bytes):
        self.workload = workload
        self.padding = _padding(payload_bytes)
        if workload == "small-string":
            from std_msgs.msg import String
            self.message_type = String
        elif workload == "nested-header":
            from std_msgs.msg import Header
            self.message_type = Header
        else:
            raise ValueError(f"unsupported workload: {workload}")

    def make(self, sequence, timestamp_ns):
        message = self.message_type()
        if self.workload == "small-string":
            message.data = f"{sequence}:{timestamp_ns}:{self.padding}"
        else:
            message.stamp.sec = timestamp_ns // 1_000_000_000
            message.stamp.nanosec = timestamp_ns % 1_000_000_000
            message.frame_id = f"{sequence}:{self.padding}"
        return message

    def decode(self, message):
        if self.workload == "small-string":
            sequence, timestamp_ns, _ = message.data.split(":", 2)
            return int(sequence), int(timestamp_ns)
        sequence, _ = message.frame_id.split(":", 1)
        timestamp_ns = message.stamp.sec * 1_000_000_000 + message.stamp.nanosec
        return int(sequence), int(timestamp_ns)


class NativeCodec:
    def __init__(self, cppyy, workload, payload_bytes):
        self.workload = workload
        self.padding = _padding(payload_bytes)
        if workload == "small-string":
            cppyy.include("std_msgs/msg/string.hpp")
            self.message_type = cppyy.gbl.std_msgs.msg.String
            self.cpp_type = "std_msgs::msg::String"
        elif workload == "nested-header":
            cppyy.include("std_msgs/msg/header.hpp")
            self.message_type = cppyy.gbl.std_msgs.msg.Header
            self.cpp_type = "std_msgs::msg::Header"
        else:
            raise ValueError(f"unsupported workload: {workload}")

    def make(self, sequence, timestamp_ns):
        message = self.message_type()
        if self.workload == "small-string":
            message.data = f"{sequence}:{timestamp_ns}:{self.padding}"
        else:
            message.stamp.sec = timestamp_ns // 1_000_000_000
            message.stamp.nanosec = timestamp_ns % 1_000_000_000
            message.frame_id = f"{sequence}:{self.padding}"
        return message

    def decode(self, message):
        if self.workload == "small-string":
            sequence, timestamp_ns, _ = str(message.data).split(":", 2)
            return int(sequence), int(timestamp_ns)
        sequence, _ = str(message.frame_id).split(":", 1)
        timestamp_ns = int(message.stamp.sec) * 1_000_000_000 + int(message.stamp.nanosec)
        return int(sequence), timestamp_ns


def _emit_backend(backend, role, entity):
    if backend == "stock":
        emit_stock_backend(role, entity)
    elif backend == "compatibility":
        emit_status_backend(role, "publisher" if role == "publisher" else "subscription")
    else:
        emit_native_backend(role, entity)


def _control_loop(state):
    for line in sys.stdin:
        try:
            document = json.loads(line)
            validate_control(document)
            run_id = document["run_id"]
            if document["command"] == "start":
                state.start(run_id)
                emit(STARTED_PREFIX, event_document("started", run_id=run_id))
            else:
                emit(RESULT_PREFIX, state.stop(run_id))
        except Exception as exc:
            print(f"benchmark control error: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)


def _subscriber_callback(codec, state):
    ready = False

    def callback(message):
        nonlocal ready
        sequence, published_ns = codec.decode(message)
        latency_us = (time.monotonic_ns() - published_ns) / 1000.0
        state.observe(sequence, latency_us)
        if not ready:
            ready = True
            emit(READY_PREFIX, event_document("ready", role="subscriber"))

    return callback


def _run_python(args):
    if args.backend == "compatibility":
        import rclcppyy
        rclcppyy.enable_cpp_acceleration()

    import rclpy

    codec = PythonCodec(args.workload, args.payload_bytes)
    rclpy.init(args=[])
    node = rclpy.create_node(args.node_name)

    if args.role == "publisher":
        publisher = node.create_publisher(codec.message_type, args.topic, 10)
        _emit_backend(args.backend, "publisher", publisher)
        sequence = 0

        def publish_one():
            nonlocal sequence
            publisher.publish(codec.make(sequence, time.monotonic_ns()))
            sequence += 1

        timer = node.create_timer(1.0 / args.rate_hz, publish_one)  # noqa: F841 - node owns timer
    else:
        state = MeasurementState()
        callback = _subscriber_callback(codec, state)
        subscription = node.create_subscription(codec.message_type, args.topic, callback, 500)
        _emit_backend(args.backend, "subscriber", subscription)
        threading.Thread(target=_control_loop, args=(state,), daemon=True).start()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def _run_native(args):
    from rclcppyy import bringup_rclcpp
    import cppyy

    rclcpp = bringup_rclcpp()
    codec = NativeCodec(cppyy, args.workload, args.payload_bytes)
    if not rclcpp.ok():
        rclcpp.init()
    node = rclcpp.Node(args.node_name)
    kept_alive = []

    if args.role == "publisher":
        publisher = node.create_publisher[codec.message_type](args.topic, 10)
        _emit_backend(args.backend, "publisher", publisher)
        sequence = 0

        def publish_one():
            nonlocal sequence
            publisher.publish(codec.make(sequence, time.monotonic_ns()))
            sequence += 1

        callback = cppyy.gbl.std.function["void()"](publish_one)
        timer = node.create_wall_timer(
            cppyy.gbl.std.chrono.nanoseconds(int(1e9 / args.rate_hz)), callback)
        kept_alive.extend((publish_one, callback, timer))
    else:
        state = MeasurementState()
        on_message = _subscriber_callback(codec, state)
        callback = cppyy.gbl.std.function[
            f"void(std::shared_ptr<const {codec.cpp_type}>)"](on_message)
        subscription = node.create_subscription[codec.message_type](
            args.topic, 500, callback)
        _emit_backend(args.backend, "subscriber", subscription)
        kept_alive.extend((on_message, callback, subscription))
        threading.Thread(target=_control_loop, args=(state,), daemon=True).start()

    try:
        rclcpp.spin(node)
    finally:
        kept_alive.clear()
        node = None
        if rclcpp.ok():
            rclcpp.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("stock", "compatibility", "native"), required=True)
    parser.add_argument("--role", choices=("publisher", "subscriber"), required=True)
    parser.add_argument("--workload", choices=tuple(WORKLOADS), required=True)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--node-name", required=True)
    parser.add_argument("--rate-hz", type=int, required=True)
    parser.add_argument("--payload-bytes", type=int, required=True)
    args = parser.parse_args()

    if args.rate_hz <= 0:
        parser.error("--rate-hz must be positive")
    if args.payload_bytes < 0:
        parser.error("--payload-bytes must be non-negative")

    if args.backend == "native":
        _run_native(args)
    else:
        _run_python(args)


if __name__ == "__main__":
    main()
