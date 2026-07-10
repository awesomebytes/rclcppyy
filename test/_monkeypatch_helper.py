#!/usr/bin/env python3
"""Helper process for test_monkeypatch.py.

Exercises the ``enable_cpp_acceleration()`` monkeypatch path end to end. This
patch mutates rclpy process-globally and irreversibly, so it must run in a
throwaway interpreter -- hence a subprocess helper rather than an in-process
test.

Prints one marker per checkpoint and ``MONKEYPATCH_ALL_OK`` last on success:
  CREATE_NODE_OK   - rclpy.create_node returns an RclcppyyNode
  MSG_REDIRECT_OK  - an imported message class is now the cppyy C++ type
  ROUNDTRIP_OK     - pub/sub roundtrip through the patched rclpy API
"""
import os
import sys
import time

import rclcppyy
rclcppyy.enable_cpp_acceleration()

# Must be imported after enable_cpp_acceleration() installs the message/node
# patches, so the ordering below is intentional, not an oversight.
import rclpy  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from rclcppyy.node import RclcppyyNode  # noqa: E402

SPIN_DEADLINE_S = 15.0
N_MESSAGES = 5
TOPIC = "rclcppyy_monkey_roundtrip"


def main():
    # Imported message class must have been redirected to the C++ (cppyy) type
    # by the import hook that enable_cpp_acceleration() installs.
    assert hasattr(String, "__smartptr__"), (
        f"String was not redirected to a C++ type: {String.__module__}")
    assert String.__module__.startswith("cppyy"), String.__module__
    print("MSG_REDIRECT_OK", flush=True)

    rclpy.init()

    # Patched rclpy.create_node must hand back our C++-backed node.
    node = rclpy.create_node("rclcppyy_monkey_helper")
    assert isinstance(node, RclcppyyNode), f"got {type(node)!r}"
    print("CREATE_NODE_OK", flush=True)

    received = []

    def on_msg(msg):
        received.append(str(msg.data))

    sub = node.create_subscription(String, TOPIC, on_msg, 10)  # noqa: F841 - keep alive
    pub = node.create_publisher(String, TOPIC, 10)

    rclcpp = rclcppyy.bringup_rclcpp()
    executor = rclcpp.executors.SingleThreadedExecutor()
    executor.add_node(node._rclcpp_node)

    deadline = time.monotonic() + SPIN_DEADLINE_S
    while pub.get_subscription_count() < 1 and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.05)
    assert pub.get_subscription_count() >= 1, "subscription never matched publisher"

    expected = [f"monkey hello {i}" for i in range(N_MESSAGES)]
    for payload in expected:
        # String is now the C++ class with kwargs enabled by the patch.
        pub.publish(String(data=payload))

    while len(received) < N_MESSAGES and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.01)

    assert received == expected, f"payload mismatch: {received!r} != {expected!r}"
    print("ROUNDTRIP_OK", flush=True)

    print("MONKEYPATCH_ALL_OK", flush=True)
    sys.stdout.flush()
    sys.stderr.flush()
    # See _pubsub_plain_helper.py: hard-exit to dodge the shutdown segfault wart.
    os._exit(0)


if __name__ == "__main__":
    main()
