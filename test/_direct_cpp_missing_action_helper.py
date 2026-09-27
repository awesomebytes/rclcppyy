#!/usr/bin/env python3
"""An unavailable registered action fails before any alias mutation."""

import rclcppyy


try:
    rclcppyy.enable_cpp_acceleration(
        profile="direct_cpp", interfaces=("missing_pkg/action/Missing",))
except TypeError as exc:
    assert "missing_pkg/action/Missing" in str(exc)
    assert "not installed" in str(exc)
else:
    raise AssertionError("direct_cpp accepted an unavailable action interface")

import cppyy  # noqa: E402
from tf2_msgs.action import LookupTransform  # noqa: E402


assert LookupTransform.Goal is not cppyy.gbl.tf2_msgs.action.LookupTransform.Goal
assert hasattr(LookupTransform.Goal, "get_fields_and_field_types")
print("DIRECT_CPP_MISSING_ACTION_TRANSACTIONAL_OK")
