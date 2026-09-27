#!/usr/bin/env python3
"""Reject AsyncParameterClient imported before direct C++ activation."""

from rclpy.parameter_client import AsyncParameterClient  # noqa: F401

import rclcppyy


try:
    rclcppyy.enable_cpp_acceleration(
        profile="direct_cpp",
        interfaces=(
            "rcl_interfaces/msg/ParameterEvent",
            "rcl_interfaces/srv/GetParameters",
        ),
    )
except RuntimeError as exc:
    assert "direct_cpp must be enabled before importing" in str(exc)
else:
    raise AssertionError("direct_cpp accepted a stale AsyncParameterClient import")
print("DIRECT_CPP_PARAMETER_CLIENT_STALE_IMPORT_REJECTED_OK")
