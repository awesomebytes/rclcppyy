#!/usr/bin/env python3
"""Helper process for test_pubsub_roundtrip.py (nested-message case).

Same plain-bringup, rclpy-style pub/sub path as _pubsub_plain_helper.py, but
with a *nested* message: std_msgs/Header, which carries a builtin_interfaces/Time
``stamp`` sub-message plus a ``frame_id`` string. This exercises the recursive
Python->C++ conversion on the plain-bringup publish path (a shallow field copy
would fail to assign the nested Time).

Run as a subprocess (not collected by pytest); prints ``PLAIN_NESTED_OK`` as its
last line on success. See the sibling test for why this runs in its own
interpreter.
"""
import time

from rclcppyy.bringup_rclcpp import bringup_rclcpp

# Deadline for the bounded spin loop, measured after the (separate) rclcpp JIT
# bringup so a slow cold compile never eats into it. Generous for CI.
SPIN_DEADLINE_S = 15.0
N_MESSAGES = 5
TOPIC = "rclcppyy_plain_nested_roundtrip"


def main():
    rclcpp = bringup_rclcpp()

    from std_msgs.msg import Header
    # Must work with the unpatched Python message class (no import-hook redirect).
    assert not hasattr(Header, "__smartptr__"), (
        f"expected plain Python Header, got a C++ type: {Header.__module__}")
    print("PLAIN_NESTED_MSG_OK", flush=True)

    if not rclcpp.ok():
        rclcpp.init()

    node = rclcpp.Node("rclcppyy_plain_nested_helper")
    received = []

    def on_msg(msg):
        # msg is a C++ std_msgs::msg::Header proxy: read the nested Time stamp
        # and the frame_id back out.
        received.append((int(msg.stamp.sec), int(msg.stamp.nanosec), str(msg.frame_id)))

    sub = node.create_subscription(Header, TOPIC, on_msg, 10)  # noqa: F841 - keep alive
    pub = node.create_publisher(Header, TOPIC, 10)

    executor = rclcpp.executors.SingleThreadedExecutor()
    executor.add_node(node)

    deadline = time.monotonic() + SPIN_DEADLINE_S

    # Wait for discovery before publishing (volatile QoS drops early messages).
    while pub.get_subscription_count() < 1 and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.05)
    assert pub.get_subscription_count() >= 1, "subscription never matched publisher"

    expected = [(100 + i, 200 + i, f"frame_{i}") for i in range(N_MESSAGES)]
    for sec, nanosec, frame_id in expected:
        msg = Header()
        msg.stamp.sec = sec
        msg.stamp.nanosec = nanosec
        msg.frame_id = frame_id
        pub.publish(msg)

    while len(received) < N_MESSAGES and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.01)

    assert received == expected, f"nested payload mismatch: {received!r} != {expected!r}"

    print("PLAIN_NESTED_OK", flush=True)
    # A normal return exits cleanly (see _pubsub_plain_helper.py): rclcppyy's
    # ordered teardown brings the rclcpp context down before interpreter
    # finalization, so the return code is deterministic without os._exit.


if __name__ == "__main__":
    main()
