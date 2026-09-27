#!/usr/bin/env python3
"""Prove direct action aliasing rejects a stale supported import."""

from tf2_msgs.action import LookupTransform  # noqa: F401

import rclcppyy


try:
    rclcppyy.enable_cpp_acceleration(profile="direct_cpp")
except RuntimeError as exc:
    assert "tf2_msgs.action._lookup_transform" in str(exc)
else:
    raise AssertionError("direct_cpp accepted a stale action implementation import")

print("DIRECT_CPP_ACTION_STALE_IMPORT_OK")
