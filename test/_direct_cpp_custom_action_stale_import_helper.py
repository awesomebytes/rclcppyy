#!/usr/bin/env python3
"""A registered custom action imported before activation is rejected."""

from rclcppyy_test_interfaces.action import Accumulate  # noqa: F401

import rclcppyy


try:
    rclcppyy.enable_cpp_acceleration(
        profile="direct_cpp",
        interfaces=("rclcppyy_test_interfaces/action/Accumulate",),
    )
except RuntimeError as exc:
    assert "rclcppyy_test_interfaces.action._accumulate" in str(exc)
else:
    raise AssertionError("direct_cpp accepted a stale custom action import")
print("DIRECT_CPP_CUSTOM_ACTION_STALE_IMPORT_OK")
