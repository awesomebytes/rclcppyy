#!/usr/bin/env python3
"""Committed white-box regression guard for Slice 2.5c (PLAN-mte-unlock.md):
``destroy_subscription`` must route through the suite's C++-owned
``ManagedSubscription`` (Slice 2.5a) rather than nulling the callable
directly -- the eager-severing pattern this whole effort fixed. If a
future edit to ``create_subscription``/``destroy_subscription`` ever drops
the ``managed=`` wiring or reintroduces a direct null, this fails loud
instead of silently reintroducing the UAF class.
"""
import os
import sys

import rclcppyy

rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import UInt64  # noqa: E402


def main():
    rclpy.init(args=[])
    node = Node("destroy_no_sever_%d" % os.getpid())
    sub = node.create_subscription(
        UInt64, "/direct_cpp/destroy_no_sever/topic", lambda _m: None, 10)

    native = getattr(sub, "_native", None)
    assert native is not None, "DirectSubscription._native missing"
    managed = getattr(native, "managed", None)
    assert managed is not None, (
        "the underlying suite entity must be constructed with a C++-owned "
        "ManagedSubscription (Slice 2.5a) -- got None, meaning close() "
        "would fall back to nulling the callable directly"
    )
    assert not managed.closed()
    assert managed.entity()

    result = node.destroy_subscription(sub)
    assert result is True
    assert sub.closed
    assert managed.closed()
    assert not managed.entity()
    # Idempotent: destroying twice is a documented no-op, not a double-free.
    assert node.destroy_subscription(sub) is False

    node.destroy_node()
    rclpy.shutdown()
    assert not rclpy.ok()
    print("DESTROY_NO_SEVER_REGRESSION_OK", flush=True)


if __name__ == "__main__":
    main()
