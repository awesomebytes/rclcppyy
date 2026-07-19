#!/usr/bin/env python3
"""Prove that direct service bindings reject stale generated imports."""

from std_srvs.srv import SetBool  # noqa: F401

import rclcppyy


try:
    rclcppyy.enable_cpp_acceleration(profile="direct_cpp")
except RuntimeError as exc:
    assert "std_srvs.srv._set_bool" in str(exc)
else:
    raise AssertionError("direct_cpp accepted a stale SetBool import")
print("DIRECT_CPP_SERVICE_STALE_IMPORT_OK")
