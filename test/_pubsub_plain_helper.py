#!/usr/bin/env python3
"""Helper process for test_pubsub_roundtrip.py.

Exercises the plain-bringup, rclpy-style pub/sub path (no monkeypatching):
bring rclcpp up, create an rclcpp.Node, and drive create_publisher /
create_subscription with the rclpy calling convention and *Python* std_msgs
messages, publishing through the descriptor-based adapters into the C++
backend and reading them back out of a C++ subscription callback.

Run as a subprocess (not collected by pytest); it prints ``PLAIN_PUBSUB_OK``
as its last line on success. See the sibling test for why this runs in its own
interpreter and why we exit via os._exit.
"""
import os
import sys
import time

from rclcppyy.bringup_rclcpp import bringup_rclcpp

# Deadline for the bounded spin loop, measured *after* the (separate) rclcpp
# JIT bringup so a slow cold compile never eats into it. Generous for CI.
SPIN_DEADLINE_S = 15.0
N_MESSAGES = 5
TOPIC = "rclcppyy_plain_roundtrip"


def main():
    rclcpp = bringup_rclcpp()

    from std_msgs.msg import String
    # This path must work with the *unpatched* Python message class; assert we
    # really are exercising that (no import-hook redirect to a cppyy type).
    assert not hasattr(String, "__smartptr__"), (
        f"expected plain Python String, got a C++ type: {String.__module__}")
    print("PLAIN_MSG_OK", flush=True)

    if not rclcpp.ok():
        rclcpp.init()

    node = rclcpp.Node("rclcppyy_plain_helper")
    received = []

    def on_msg(msg):
        # msg is a C++ std_msgs::msg::String proxy; str() its std::string data.
        received.append(str(msg.data))

    # rclpy-style calls on a plain rclcpp.Node, backed by the C++ adapters.
    sub = node.create_subscription(String, TOPIC, on_msg, 10)
    pub = node.create_publisher(String, TOPIC, 10)

    executor = rclcpp.executors.SingleThreadedExecutor()
    executor.add_node(node)

    deadline = time.monotonic() + SPIN_DEADLINE_S

    # Wait for the subscription to be matched before publishing: with volatile
    # QoS, anything sent before discovery completes is silently dropped.
    while pub.get_subscription_count() < 1 and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.05)
    assert pub.get_subscription_count() >= 1, "subscription never matched publisher"

    expected = [f"plain hello {i}" for i in range(N_MESSAGES)]
    for payload in expected:
        msg = String()
        msg.data = payload
        pub.publish(msg)

    # rclcpp's spin_some does not block/sleep, so poll it with small sleeps
    # until every payload has arrived or we hit the deadline.
    while len(received) < N_MESSAGES and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.01)

    assert received == expected, f"payload mismatch: {received!r} != {expected!r}"

    print("PLAIN_PUBSUB_OK", flush=True)
    sys.stdout.flush()
    sys.stderr.flush()
    # cppyy/rclcpp C++ static destructors can segfault at interpreter shutdown
    # (documented teardown wart). The work above is done and verified, so exit
    # hard to keep the subprocess return code deterministic for the test.
    os._exit(0)


if __name__ == "__main__":
    main()
