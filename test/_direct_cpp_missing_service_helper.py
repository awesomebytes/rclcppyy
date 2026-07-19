#!/usr/bin/env python3
"""An unavailable registered service fails before any alias mutation."""

import rclcppyy


try:
    rclcppyy.enable_cpp_acceleration(
        profile="direct_cpp", interfaces=("missing_pkg/srv/Missing",))
except TypeError as exc:
    assert "missing_pkg/srv/Missing" in str(exc)
    assert "not installed" in str(exc)
else:
    raise AssertionError("direct_cpp accepted an unavailable service interface")

import cppyy  # noqa: E402
from std_srvs.srv import SetBool, SetBool_Request, SetBool_Response  # noqa: E402


assert SetBool.Request is SetBool_Request
assert SetBool.Response is SetBool_Response
assert SetBool.Request is not cppyy.gbl.std_srvs.srv.SetBool.Request
assert SetBool.Response is not cppyy.gbl.std_srvs.srv.SetBool.Response
print("DIRECT_CPP_MISSING_SERVICE_TRANSACTIONAL_OK")
