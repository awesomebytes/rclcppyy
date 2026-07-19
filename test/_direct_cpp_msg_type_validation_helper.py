#!/usr/bin/env python3
"""``check_is_valid_msg_type`` parity for registered direct message aliases.

3.A (direct_messages._mirror_type_support_metadata) deliberately withholds the
four conversion capsules stock ``check_is_valid_msg_type`` asserts, so
unpatched it would reject a perfectly valid rebound cppyy message with a
misleading "this might be a service or action" RuntimeError. The direct_cpp
patch accepts a registered direct message alias and still delegates
everything else -- including a genuinely non-message type -- to stock, which
rejects it precisely.
"""

import inspect

import cppyy
import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy.type_support as type_support  # noqa: E402
import std_srvs.srv  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402


assert Time is cppyy.gbl.builtin_interfaces.msg.Time

assert str(inspect.signature(type_support.check_is_valid_msg_type)) == "(msg_type)"
print("DIRECT_CPP_MSG_TYPE_SIGNATURE_OK", flush=True)

# A registered direct message alias validates cleanly.
type_support.check_is_valid_msg_type(Time)
print("DIRECT_CPP_MSG_TYPE_ACCEPTED_OK", flush=True)

# A genuinely non-message type (a stock service class, never registered as a
# message alias) still rejects with stock's precise RuntimeError -- fail
# closed, no silent acceptance of an unregistered/foreign type.
try:
    type_support.check_is_valid_msg_type(std_srvs.srv.SetBool)
except RuntimeError as exc:
    assert "this might be a service or action" in str(exc)
else:
    raise AssertionError("a service class passed the message-type validator")
print("DIRECT_CPP_MSG_TYPE_REJECTED_OK", flush=True)
