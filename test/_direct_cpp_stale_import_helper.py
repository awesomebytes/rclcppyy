#!/usr/bin/env python3
"""A supported message imported before activation must fail closed."""

from std_msgs.msg import String as original_string

import rclcppyy


try:
    rclcppyy.enable_cpp_acceleration(profile="direct_cpp")
except RuntimeError as exception:
    assert "before importing generated messages" in str(exception)
else:
    raise AssertionError("direct_cpp accepted a stale generated message class")

from std_msgs.msg import String as late_string  # noqa: E402


assert late_string is original_string
print("DIRECT_CPP_STALE_IMPORT_OK")
