#!/usr/bin/env python3
"""Fresh-process proof for direct dynamic undeclare and mutation limits."""

import importlib
import os

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from rcl_interfaces.msg import (  # noqa: E402
    ParameterDescriptor,
    ParameterValue,
    SetParametersResult,
)
from rclcppyy.policy import BackendUnavailableError  # noqa: E402
from rclpy.exceptions import (  # noqa: E402
    ParameterImmutableException,
    ParameterNotDeclaredException,
)
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402


assert ParameterDescriptor is cppyy.gbl.rcl_interfaces.msg.ParameterDescriptor
assert ParameterValue is cppyy.gbl.rcl_interfaces.msg.ParameterValue
assert SetParametersResult is cppyy.gbl.rcl_interfaces.msg.SetParametersResult

boundary_calls = {
    "conversion": 0,
    "serialization": 0,
    "cdr": 0,
}


def poison(kind):
    def forbidden(*_args, **_kwargs):
        boundary_calls[kind] += 1
        raise AssertionError("direct parameter mutation crossed the %s boundary" % kind)

    return forbidden


conversion = poison("conversion")
serialization = poison("serialization")
cdr = poison("cdr")
kit = importlib.import_module("rclcpp_kit")
kit_bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
product_bringup = importlib.import_module("rclcppyy.bringup_rclcpp")
product_node = importlib.import_module("rclcppyy.node")
kit_serialization = importlib.import_module("rclcpp_kit.serialization")
product_serialization = importlib.import_module("rclcppyy.serialization")
rclpy_serialization = importlib.import_module("rclpy.serialization")

kit.convert_python_msg_to_cpp = conversion
kit_bringup.convert_python_msg_to_cpp = conversion
product_bringup.convert_python_msg_to_cpp = conversion
product_node.convert_python_msg_to_cpp = conversion
for module in (kit_serialization, product_serialization, rclpy_serialization):
    module.serialize_message = serialization
    module.deserialize_message = serialization
for module in (kit_serialization, product_serialization):
    module.serialized_message_from_bytes = cdr
    module.serialized_message_to_bytes = cdr


rclpy.init(args=[])
node = Node("direct_parameter_undeclare_%d" % os.getpid())

dynamic_descriptor = ParameterDescriptor(
    description="removable exact C++ value",
    dynamic_typing=True,
)
declared = node.declare_parameter("removable", 7, dynamic_descriptor)
retained_dynamic = node.get_parameter("removable")
assert retained_dynamic is declared
assert type(node.describe_parameter("removable")) is ParameterDescriptor

callback_events = []


def pre_callback(parameters):
    callback_events.append(("pre", tuple(parameters)))
    return parameters


def on_callback(parameters):
    callback_events.append(("on", tuple(parameters)))
    return SetParametersResult(successful=True)


def post_callback(parameters):
    callback_events.append(("post", tuple(parameters)))


node.add_pre_set_parameters_callback(pre_callback)
node.add_on_set_parameters_callback(on_callback)
node.add_post_set_parameters_callback(post_callback)
before_undeclare = node.direct_cpp_parameter_cache_stats()
node.undeclare_parameter("removable")
after_undeclare = node.direct_cpp_parameter_cache_stats()
assert callback_events == []
assert not node.has_parameter("removable")
assert "removable" not in {str(name) for name in node.list_parameters([], 0).names}
assert after_undeclare["invalidations"] == before_undeclare["invalidations"] + 1
assert after_undeclare["pending_invalidations"] == 0
assert retained_dynamic.value == 7
assert type(retained_dynamic.get_parameter_value()) is ParameterValue

node.remove_pre_set_parameters_callback(pre_callback)
node.remove_on_set_parameters_callback(on_callback)
node.remove_post_set_parameters_callback(post_callback)
replacement = node.declare_parameter("removable", 9, dynamic_descriptor)
assert replacement.value == 9
assert replacement is not retained_dynamic
assert retained_dynamic.value == 7
print("DIRECT_PARAMETER_DYNAMIC_UNDECLARE_OK")

read_only_descriptor = ParameterDescriptor(read_only=True, dynamic_typing=True)
read_only = node.declare_parameter("read_only", 11, read_only_descriptor)
try:
    node.undeclare_parameter("read_only")
except ParameterImmutableException:
    pass
else:
    raise AssertionError("read-only parameter was undeclared")
assert node.get_parameter("read_only") is read_only

static = node.declare_parameter("static", 13)
before_static = node.direct_cpp_parameter_cache_stats()
try:
    node.undeclare_parameter("static")
except BackendUnavailableError as exception:
    assert "public rclcpp rejects this operation" in str(exception)
    assert "rclpy permits it" in str(exception)
else:
    raise AssertionError("Jazzy rclcpp unexpectedly undeclared a static parameter")
assert node.get_parameter("static") is static
assert node.direct_cpp_parameter_cache_stats() == before_static

try:
    node.undeclare_parameter("missing")
except ParameterNotDeclaredException:
    pass
else:
    raise AssertionError("missing parameter undeclare did not preserve rclpy error")
print("DIRECT_PARAMETER_UNDECLARE_ERRORS_OK")

original_descriptor = node.describe_parameter("static")
new_descriptor = ParameterDescriptor(description="unsupported replacement")
alternative = Parameter("static", value=17).get_parameter_value()
assert type(alternative) is ParameterValue
for alternative_value in (None, alternative):
    try:
        node.set_descriptor("static", new_descriptor, alternative_value)
    except BackendUnavailableError as exception:
        assert "public rclcpp has no descriptor mutation operation" in str(exception)
        assert "atomicity, callbacks, and parameter events" in str(exception)
    else:
        raise AssertionError("unsupported descriptor mutation succeeded")
try:
    node.set_descriptor("static", object())
except TypeError as exception:
    assert "actual direct_cpp C++ ParameterDescriptor" in str(exception)
else:
    raise AssertionError("non-C++ descriptor reached the unsupported operation")
try:
    node.set_descriptor("static", new_descriptor, object())
except TypeError as exception:
    assert "actual direct_cpp C++ ParameterValue" in str(exception)
else:
    raise AssertionError("non-C++ alternative value reached the unsupported operation")
current_descriptor = node.describe_parameter("static")
assert str(current_descriptor.description) == str(original_descriptor.description)
assert node.get_parameter("static") is static
print("DIRECT_PARAMETER_DESCRIPTOR_MUTATION_LIMIT_OK")

node.destroy_node()
rclpy.shutdown()
assert retained_dynamic.value == 7
assert static.value == 13
assert boundary_calls == {"conversion": 0, "serialization": 0, "cdr": 0}
print("DIRECT_PARAMETER_UNDECLARE_RETAINED_EXACT_CPP_OK")
