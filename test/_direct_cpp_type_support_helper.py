#!/usr/bin/env python3
"""Direct-rebound cppyy messages must not break stock type-support validation.

Reproduces the reported crash exactly: ``rclpy.type_support`` walks a *stock*
service's generated type-support import (``std_srvs.srv.SetBool``, and every
lifecycle service), which transitively reaches a *rebound* cppyy message
(``builtin_interfaces.msg.Time``, pulled in via the default
``tf2_msgs/action/LookupTransform`` -> ``action_msgs/msg/GoalInfo.stamp``
dependency closure) and used to raise ``AttributeError`` deep inside stock
generated code we cannot edit.
"""

import importlib

import cppyy
import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import lifecycle_msgs.srv  # noqa: E402
import rclpy.type_support as type_support  # noqa: E402
import service_msgs.msg  # noqa: E402
import std_srvs.srv  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402
from std_msgs.msg import String  # noqa: E402


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("a Python-message conversion path ran")


bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
bringup.convert_python_msg_to_cpp = forbidden_boundary


assert Time is cppyy.gbl.builtin_interfaces.msg.Time
assert String is cppyy.gbl.std_msgs.msg.String
print("DIRECT_CPP_TYPE_SUPPORT_ALIAS_RETAINED_OK", flush=True)

# The reported crash: check_is_valid_srv_type on a *stock* service class
# whose type-support import transitively walks the auto-generated
# <Service>_Event message's ServiceEventInfo.stamp field, which is the
# rebound cppyy Time.
type_support.check_is_valid_srv_type(std_srvs.srv.SetBool)
type_support.check_is_valid_srv_type(lifecycle_msgs.srv.ChangeState)
type_support.check_is_valid_srv_type(lifecycle_msgs.srv.GetState)
type_support.check_is_valid_srv_type(lifecycle_msgs.srv.GetAvailableStates)
type_support.check_is_valid_srv_type(lifecycle_msgs.srv.GetAvailableTransitions)
print("DIRECT_CPP_TYPE_SUPPORT_SRV_OK", flush=True)

# Transitive walk: a stock message (ServiceEventInfo, never rebound) whose
# own field (stamp) reaches the rebound Time.
type_support.check_for_type_support(service_msgs.msg.ServiceEventInfo)
type_support.check_is_valid_msg_type(service_msgs.msg.ServiceEventInfo)
print("DIRECT_CPP_TYPE_SUPPORT_TRANSITIVE_OK", flush=True)

# Direct manifestation: validating the rebound type itself.
type_support.check_for_type_support(Time)
print("DIRECT_CPP_TYPE_SUPPORT_DIRECT_OK", flush=True)

# No-conversion-bridge tripwire: only the read-only _TYPE_SUPPORT identity
# capsule is exposed on the cppyy metaclass, never the four conversion
# capsules -- exposing those would let a cppyy object round-trip through
# stock C conversion, a silent bridge on a path that claims to be
# accelerated.
time_metaclass = Time.__class__
assert time_metaclass._TYPE_SUPPORT is not None
for forbidden in (
    "_CREATE_ROS_MESSAGE", "_CONVERT_FROM_PY", "_CONVERT_TO_PY", "_DESTROY_ROS_MESSAGE",
):
    assert not hasattr(time_metaclass, forbidden), forbidden
print("DIRECT_CPP_TYPE_SUPPORT_NO_BRIDGE_OK", flush=True)

# Two different rebound messages get distinct metaclasses with independent
# _TYPE_SUPPORT state (cppyy mints one metaclass per instantiated class).
string_metaclass = String.__class__
assert string_metaclass is not time_metaclass
type_support.check_for_type_support(String)
assert string_metaclass._TYPE_SUPPORT is not None
assert string_metaclass._TYPE_SUPPORT is not time_metaclass._TYPE_SUPPORT
print("DIRECT_CPP_TYPE_SUPPORT_DISTINCT_METACLASS_OK", flush=True)
