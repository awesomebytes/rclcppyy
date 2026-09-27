#!/usr/bin/env python3
"""An uninstalled direct interface must fail before product activation."""

import rclcppyy


try:
    rclcppyy.enable_cpp_acceleration(
        profile="direct_cpp", interfaces=("missing_pkg/msg/Missing",))
except TypeError as exc:
    assert "not installed" in str(exc)
else:
    raise AssertionError("an uninstalled interface activated direct_cpp")

assert rclcppyy._ACTIVE_PROFILE is None
assert rclcppyy._ACTIVE_INTERFACES == ()
print("DIRECT_CPP_MISSING_INTERFACE_FAIL_CLOSED_OK", flush=True)
