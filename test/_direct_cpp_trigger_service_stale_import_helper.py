#!/usr/bin/env python3
"""Reject a registered Trigger service imported before activation."""

from std_srvs.srv import Trigger  # noqa: F401

import rclcppyy


try:
    rclcppyy.enable_cpp_acceleration(
        profile="direct_cpp", interfaces=("std_srvs/srv/Trigger",))
except RuntimeError as exc:
    assert "std_srvs.srv._trigger" in str(exc)
else:
    raise AssertionError("direct_cpp accepted a stale Trigger import")
print("DIRECT_CPP_TRIGGER_STALE_IMPORT_OK")
