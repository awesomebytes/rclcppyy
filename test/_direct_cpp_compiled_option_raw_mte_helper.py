#!/usr/bin/env python3
"""Prove compiled primary dispatch for option-only typed and raw subscriptions."""
import os
import threading
import time

os.environ.setdefault("ROS_DOMAIN_ID", "77")

import rclcppyy

rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos_overriding_options import QoSOverridingOptions  # noqa: E402
from std_msgs.msg import UInt64  # noqa: E402


def wait_for(predicate, description, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("timed out waiting for %s" % description)


def main():
    rclpy.init(args=[])
    node = Node("compiled_option_raw_mte_%d" % os.getpid())
    # ROS topic tokens may not begin with a digit.
    suffix = "p%d" % os.getpid()
    option_topic = "/compiled_option_raw_mte/%s/option" % suffix
    raw_topic = "/compiled_option_raw_mte/%s/raw" % suffix
    option_seen = threading.Event()
    raw_seen = []

    qos = 10
    option_subscription = node.create_subscription(
        UInt64, option_topic, lambda _msg: option_seen.set(), qos,
        qos_overriding_options=QoSOverridingOptions.with_default_policies())
    raw_subscription = node.create_subscription(
        UInt64, raw_topic, raw_seen.append, qos, raw=True)

    for label, subscription in (
            ("option", option_subscription), ("raw", raw_subscription)):
        bridge = subscription._native
        assert bridge.callback_handoff == "compiled_python_callback", label
        assert bridge.source_id, label
        assert not bridge.event_callback_handoffs, label

    option_publisher = node.create_publisher(UInt64, option_topic, qos)
    raw_publisher = node.create_publisher(UInt64, raw_topic, qos)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_errors = []

    def spin():
        try:
            executor.spin()
        except BaseException as exc:  # noqa: BLE001
            spin_errors.append(exc)

    thread = threading.Thread(target=spin, name="compiled-option-raw-mte")
    thread.start()
    try:
        wait_for(
            lambda: option_publisher.get_subscription_count() == 1,
            "option-only subscription discovery")
        wait_for(
            lambda: raw_publisher.get_subscription_count() == 1,
            "raw subscription discovery")
        option_publisher.publish(UInt64(data=731))
        raw_publisher.publish(UInt64(data=947))
        assert option_seen.wait(5.0), "option-only callback did not run"
        wait_for(lambda: bool(raw_seen), "raw callback")
        assert isinstance(raw_seen[0], bytes)
        assert len(raw_seen[0]) > 4
        assert not spin_errors, "executor errors: %r" % (spin_errors,)
        print("COMPILED_OPTION_RAW_MTE_OK", flush=True)
    finally:
        executor.shutdown(timeout_sec=5.0)
        thread.join(timeout=5.0)
        assert not thread.is_alive(), "executor spin thread failed to stop"
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
