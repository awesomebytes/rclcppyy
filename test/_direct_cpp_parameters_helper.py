#!/usr/bin/env python3
"""Fresh-process proof for native direct_cpp parameter ownership."""

import importlib
import os
import warnings

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from rcl_interfaces.msg import (  # noqa: E402
    FloatingPointRange,
    IntegerRange,
    ListParametersResult,
    Parameter as ParameterMsg,
    ParameterDescriptor,
    ParameterValue,
    SetParametersResult,
)
from rclcpp_kit import native_parameters  # noqa: E402
from rclcpp_kit.native_parameters import NativeParameter  # noqa: E402
from rclcppyy.policy import BackendUnavailableError  # noqa: E402
from rclpy.exceptions import (  # noqa: E402
    ParameterAlreadyDeclaredException,
    ParameterNotDeclaredException,
    ParameterUninitializedException,
)
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402


assert ParameterMsg is cppyy.gbl.rcl_interfaces.msg.Parameter
assert ParameterValue is cppyy.gbl.rcl_interfaces.msg.ParameterValue
assert ParameterDescriptor is cppyy.gbl.rcl_interfaces.msg.ParameterDescriptor
assert IntegerRange is cppyy.gbl.rcl_interfaces.msg.IntegerRange
assert FloatingPointRange is cppyy.gbl.rcl_interfaces.msg.FloatingPointRange
assert ListParametersResult is cppyy.gbl.rcl_interfaces.msg.ListParametersResult
assert SetParametersResult is cppyy.gbl.rcl_interfaces.msg.SetParametersResult


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("an application-message conversion or serialization path ran")


bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
kit = importlib.import_module("rclcpp_kit")
serialization = importlib.import_module("rclcpp_kit.serialization")
rclpy_serialization = importlib.import_module("rclpy.serialization")
bringup.convert_python_msg_to_cpp = forbidden_boundary
kit.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary
serialization.serialized_message_from_bytes = forbidden_boundary
serialization.serialized_message_to_bytes = forbidden_boundary
rclpy_serialization.serialize_message = forbidden_boundary
rclpy_serialization.deserialize_message = forbidden_boundary


CASES = (
    (Parameter.Type.NOT_SET, None),
    (Parameter.Type.BOOL, True),
    (Parameter.Type.INTEGER, 7),
    (Parameter.Type.DOUBLE, 2.5),
    (Parameter.Type.STRING, "native"),
    (Parameter.Type.BYTE_ARRAY, [b"\x00", b"\xff"]),
    (Parameter.Type.BOOL_ARRAY, [True, False]),
    (Parameter.Type.INTEGER_ARRAY, [-2, 5]),
    (Parameter.Type.DOUBLE_ARRAY, [-1.5, 4.25]),
    (Parameter.Type.STRING_ARRAY, ["a", "b"]),
)


assert rclpy.Parameter is Parameter
assert Parameter.__module__ == "rclpy.parameter"
for index, (type_, value) in enumerate(CASES):
    parameter = Parameter("case_%d" % index, type_, value)
    assert not hasattr(parameter, "__dict__")
    assert isinstance(parameter._rclcppyy_native_parameter, NativeParameter)
    assert isinstance(parameter._rclcppyy_native_parameter.native, cppyy.gbl.rclcpp.Parameter)
    assert parameter._rclcppyy_native_parameter.type_code == int(
        parameter._rclcppyy_native_parameter.native.get_type())
    assert parameter.name == "case_%d" % index
    assert parameter.type_ is type_
    assert parameter.value == value
    assert isinstance(
        parameter.get_parameter_value(),
        cppyy.gbl.rcl_interfaces.msg.ParameterValue,
    )
    message = parameter.to_parameter_msg()
    assert isinstance(message, cppyy.gbl.rcl_interfaces.msg.Parameter)
    restored = Parameter.from_parameter_msg(message)
    assert restored.name == parameter.name
    assert restored.type_ is type_
    assert restored.value == value

for type_ in (
    Parameter.Type.BYTE_ARRAY,
    Parameter.Type.BOOL_ARRAY,
    Parameter.Type.INTEGER_ARRAY,
    Parameter.Type.DOUBLE_ARRAY,
    Parameter.Type.STRING_ARRAY,
):
    parameter = Parameter("empty_%s" % type_.name.lower(), type_, [])
    assert parameter._rclcppyy_native_parameter.type_code == int(
        parameter._rclcppyy_native_parameter.native.get_type())
    assert parameter.type_ is type_
    assert parameter.value == []

snapshot_source = Parameter("snapshot", Parameter.Type.INTEGER_ARRAY, [1, 2])
snapshot = snapshot_source.value
snapshot.append(3)
assert snapshot_source.value == [1, 2]

cpp_message = ParameterMsg(
    name="cpp_control_message",
    value=ParameterValue(type=Parameter.Type.INTEGER.value, integer_value=41),
)
restored = Parameter.from_parameter_msg(cpp_message)
assert restored.name == "cpp_control_message"
assert restored.value == 41
assert isinstance(restored._rclcppyy_native_parameter.native, cppyy.gbl.rclcpp.Parameter)
try:
    Parameter.from_parameter_msg(object())
except TypeError as error:
    assert "actual direct_cpp C++ Parameter" in str(error)
else:
    raise AssertionError("direct_cpp accepted a non-C++ Parameter message")
print("DIRECT_CPP_PARAMETERS_OWNING_FACADE_OK")


rclpy.init(args=["--ros-args", "-r", "__node:=ignored_by_explicit_name"])
node = Node("direct_parameters_%d" % os.getpid())
direct_module = importlib.import_module("rclcppyy.direct_cpp")
runtime = direct_module._runtime()

with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    dynamic = node.declare_parameter("dynamic")
assert dynamic.type_ is Parameter.Type.NOT_SET
assert dynamic.value is None
typed = node.declare_parameter("typed", Parameter.Type.INTEGER)
assert typed.type_ is Parameter.Type.NOT_SET
try:
    node.get_parameter("typed")
except ParameterUninitializedException:
    pass
else:
    raise AssertionError("uninitialized static parameter was returned")

declared = []
for index, (type_, value) in enumerate(CASES[1:], start=1):
    name = "declared_%d" % index
    declared.append(node.declare_parameter(name, value))
    assert node.has_parameter(name)
    assert node.get_parameter(name).value == value

batch = node.declare_parameters(
    "group",
    [
        ("enabled", True),
        ("count", 3, ParameterDescriptor(description="native descriptor")),
    ],
)
assert [parameter.name for parameter in batch] == ["group.enabled", "group.count"]
assert node.get_parameter_types(["group.enabled", "group.count"]) == [1, 2]
assert node.get_parameter_type("group.count") == Parameter.Type.INTEGER.value
assert str(node.describe_parameter("group.count").description) == "native descriptor"
descriptors = node.describe_parameters(["group.enabled", "group.count"])
assert all(isinstance(value, cppyy.gbl.rcl_interfaces.msg.ParameterDescriptor)
           for value in descriptors)
listed = node.list_parameters(["group"], 10)
assert {str(name) for name in listed.names} == {"group.enabled", "group.count"}
try:
    node.declare_parameters("", [("group.count", 9), ("never_created", 1)])
except ParameterAlreadyDeclaredException:
    pass
else:
    raise AssertionError("duplicate declaration succeeded")
assert not node.has_parameter("never_created")
try:
    node.declare_parameter("python_descriptor", 1, object())
except TypeError as error:
    assert "actual direct_cpp C++ ParameterDescriptor" in str(error)
else:
    raise AssertionError("direct_cpp accepted a non-C++ parameter descriptor")
assert not node.has_parameter("python_descriptor")
try:
    node.get_parameter("missing")
except ParameterNotDeclaredException:
    pass
else:
    raise AssertionError("undeclared parameter access succeeded")
print("DIRECT_CPP_PARAMETERS_NATIVE_NODE_API_OK")


node._set_direct_parameter_cache_hit_tracking(True)
cache_stats_before_gets = node.direct_cpp_parameter_cache_stats()
native_parameters.reset_checked_parameter_stats()
optimized_get = node.get_parameter("group.count")
assert optimized_get.value == 3
assert node.get_parameter("group.count") is optimized_get
try:
    node.get_parameter("typed")
except ParameterUninitializedException:
    pass
else:
    raise AssertionError("checked get returned a static uninitialized parameter")
assert node.get_parameter("dynamic").value is None
try:
    node.get_parameter("checked_missing")
except ParameterNotDeclaredException:
    pass
else:
    raise AssertionError("checked get returned an undeclared parameter")
permissive = Node(
    "direct_parameters_permissive_%d" % os.getpid(),
    allow_undeclared_parameters=True,
)
permissive_missing = permissive.get_parameter("missing")
assert permissive_missing.type_ is Parameter.Type.NOT_SET
assert permissive_missing.value is None
assert not permissive.has_parameter("missing")
optimized_native = optimized_get._rclcppyy_native_parameter
assert type(optimized_native.native) is cppyy.gbl.rclcpp.Parameter
checked_get_stats = native_parameters.checked_parameter_stats()
assert checked_get_stats.to_dict() == {
    "calls": 3,
    "node_value_copies": 1,
    "result_copies": 0,
}
cache_stats_after_gets = node.direct_cpp_parameter_cache_stats()
assert cache_stats_after_gets["hit_tracking_enabled"] is True
assert cache_stats_after_gets["hits"] == 3
assert cache_stats_after_gets["misses"] == (
    cache_stats_before_gets["misses"] + 2)
assert cache_stats_after_gets["size"] <= cache_stats_after_gets["capacity"]
print("DIRECT_CPP_PARAMETERS_CHECKED_GET_OK")


retained_callback_parameter = []
events = []
post_cache_values = []


def pre_callback(parameters):
    events.append(("pre", [parameter.value for parameter in parameters]))
    return [Parameter("group.count", value=parameters[0].value + 1)]


def on_callback(parameters):
    retained_callback_parameter[:] = parameters
    events.append(("on", [parameter.value for parameter in parameters]))
    return SetParametersResult(successful=True)


def post_callback(parameters):
    post_cache_values.extend(
        node.get_parameter(parameter.name).value for parameter in parameters)
    events.append(("post", [parameter.value for parameter in parameters]))


node.add_pre_set_parameters_callback(pre_callback)
node.add_on_set_parameters_callback(on_callback)
node.add_post_set_parameters_callback(post_callback)
result = node.set_parameters([Parameter("group.count", value=10)])
assert bool(result[0].successful)
assert node.get_parameter("group.count").value == 11
assert events == [("pre", [10]), ("on", [11]), ("post", [11])]
assert post_cache_values == [11]


def reject_callback(_parameters):
    return SetParametersResult(successful=False, reason="rejected")


node.add_on_set_parameters_callback(reject_callback)
result = node.set_parameters_atomically([Parameter("group.count", value=20)])
assert not bool(result.successful)
assert str(result.reason) == "rejected"
assert node.get_parameter("group.count").value == 11
node.remove_on_set_parameters_callback(reject_callback)


def non_cpp_result(_parameters):
    return object()


node.add_on_set_parameters_callback(non_cpp_result)
try:
    node.set_parameters([Parameter("group.count", value=29)])
except TypeError as exception:
    assert "actual direct_cpp C++ SetParametersResult" in str(exception)
else:
    raise AssertionError("direct_cpp accepted a non-C++ parameter callback result")
assert node.get_parameter("group.count").value == 11
node.remove_on_set_parameters_callback(non_cpp_result)


def on_exception(_parameters):
    raise RuntimeError("on callback failed")


node.add_on_set_parameters_callback(on_exception)
try:
    node.set_parameters([Parameter("group.count", value=30)])
except RuntimeError as exception:
    assert str(exception) == "on callback failed"
else:
    raise AssertionError("on callback exception did not reach the local caller")
assert node.get_parameter("group.count").value == 11
node.remove_on_set_parameters_callback(on_exception)


def post_exception(_parameters):
    raise RuntimeError("post callback failed")


node.add_post_set_parameters_callback(post_exception)
try:
    node.set_parameters([Parameter("group.count", value=40)])
except RuntimeError as exception:
    assert str(exception) == "post callback failed"
else:
    raise AssertionError("post callback exception did not reach the local caller")
assert node.get_parameter("group.count").value == 41
node.remove_post_set_parameters_callback(post_exception)

node.remove_pre_set_parameters_callback(pre_callback)
node.remove_on_set_parameters_callback(on_callback)
node.remove_post_set_parameters_callback(post_callback)
assert node._direct_cpp_parameter_callback_bridges["pre"] is None
assert node._direct_cpp_parameter_callback_bridges["on"] is None
assert node._direct_cpp_parameter_callback_bridges["post"] is not None
assert node._post_set_parameters_callbacks == []
print("DIRECT_CPP_PARAMETERS_CALLBACKS_OK")


before_names = {str(name) for name in node.list_parameters([], 0).names}
for operation in (
    lambda: node.undeclare_parameter("group.count"),
    lambda: node.set_descriptor("group.count", ParameterDescriptor()),
    lambda: node.set_parameters([Parameter("group.count")]),
):
    try:
        operation()
    except BackendUnavailableError:
        pass
    else:
        raise AssertionError("unsupported parameter mutation succeeded")
    assert {str(name) for name in node.list_parameters([], 0).names} == before_names
assert node.get_parameter("group.count").value == 41
print("DIRECT_CPP_PARAMETERS_FAIL_CLOSED_OK")


retained_get_parameter = node.get_parameter("group.count")
retained_optimized_get = optimized_get
assert retained_callback_parameter
rclpy.shutdown()
assert retained_get_parameter.value == 41
assert retained_optimized_get.value == 3
assert retained_callback_parameter[0].value == 41
assert node._direct_cpp_node is None
assert permissive._direct_cpp_node is None

rclpy.init()
second = Node("direct_parameters_reinit_%d" % os.getpid())
second.declare_parameter("again", "native")
assert second.get_parameter("again").value == "native"
second.destroy_node()
rclpy.shutdown()
print("DIRECT_CPP_PARAMETERS_RETAINED_REINIT_OK")
