#!/usr/bin/env python3
"""Live proof: the direct_cpp publisher's hot-path ``publish`` binding
releases the GIL for the duration of the native call.

Under RELIABLE QoS backpressure ``rcl_publish`` can block the calling
thread waiting on the DDS write (observed up to ~100ms); without
``__release_gil__`` that would freeze the whole interpreter for the
duration, exactly like an unmarked blocking ``rcl_wait``/``spin_once``
would (see direct_executors.py and direct_guard_condition.py). This proof
checks the marker is present on both ``DirectPublisher`` (plain publisher)
and ``DirectLifecyclePublisher`` (both funnel through
``DirectPublisher.__init__``), then confirms an ordinary publish/subscribe
round trip still works with the marker set.
"""
import inspect
import os
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclpy.lifecycle import LifecycleNode  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from rclcppyy.direct_lifecycle import DirectLifecyclePublisher  # noqa: E402


def spin_until(node, condition, timeout=10.0):
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert condition()


def main():
    rclpy.init(args=None)
    pid = os.getpid()

    node = Node("direct_cpp_publish_release_gil_%d" % pid)
    topic = "/direct_cpp/publish_release_gil/p%d" % pid

    received = []
    publisher = node.create_publisher(String, topic, 10)
    node.create_subscription(String, topic, received.append, 10)

    # The hot-path publish binding is the raw cppyy-bound
    # ManagedPublisher<MessageT>::publish overload (direct_cpp.py's
    # DirectPublisher.__init__), not a Python method.
    assert not inspect.ismethod(publisher.publish)
    assert publisher.publish.__release_gil__ is True
    assert publisher._native.publish.__release_gil__ is True
    print("DIRECT_CPP_PUBLISH_RELEASE_GIL_MARKER_OK", flush=True)

    publisher.publish(String(data="release-gil-smoke"))
    spin_until(node, lambda: len(received) == 1)
    assert str(received[0].data) == "release-gil-smoke"
    print("DIRECT_CPP_PUBLISH_RELEASE_GIL_ROUNDTRIP_OK", flush=True)

    node.destroy_node()

    # DirectLifecyclePublisher.__init__ forwards straight to
    # DirectPublisher.__init__ (direct_lifecycle.py), so it must carry the
    # same marker -- checked at construction only; the gating/round-trip
    # behavior itself is proven by
    # _direct_cpp_lifecycle_publisher_helper.py.
    lifecycle_node = LifecycleNode("direct_cpp_publish_release_gil_lc_%d" % pid)
    lifecycle_topic = "/direct_cpp/publish_release_gil_lifecycle/p%d" % pid
    lifecycle_publisher = lifecycle_node.create_lifecycle_publisher(
        String, lifecycle_topic, 10)
    assert type(lifecycle_publisher) is DirectLifecyclePublisher
    assert not inspect.ismethod(lifecycle_publisher.publish)
    assert lifecycle_publisher.publish.__release_gil__ is True
    assert lifecycle_publisher._native.publish.__release_gil__ is True
    print("DIRECT_CPP_LIFECYCLE_PUBLISH_RELEASE_GIL_MARKER_OK", flush=True)

    lifecycle_node.destroy_node()
    rclpy.shutdown()
    print("DIRECT_CPP_PUBLISH_RELEASE_GIL_TEARDOWN_OK", flush=True)


if __name__ == "__main__":
    main()
