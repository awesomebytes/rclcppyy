#!/usr/bin/env python3
"""Live native NodeOptions proof for the direct_cpp profile."""

import importlib
import os
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from rclcppyy.policy import BackendUnavailableError  # noqa: E402
from rclpy.context import Context  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("conversion or serialization entered NodeOptions proof")


kit = importlib.import_module("rclcpp_kit")
bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
serialization = importlib.import_module("rclcpp_kit.serialization")
rclpy_serialization = importlib.import_module("rclpy.serialization")
kit.convert_python_msg_to_cpp = forbidden_boundary
bringup.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary
rclpy_serialization.serialize_message = forbidden_boundary
rclpy_serialization.deserialize_message = forbidden_boundary


def service_names(node):
    deadline = time.monotonic() + 3.0
    names = set()
    while time.monotonic() < deadline:
        names = {name for name, _types in node.get_service_names_and_types()}
        if names:
            break
        time.sleep(0.01)
    return names


rclpy.init(args=[
    "--ros-args",
    "-r", "__node:=global_options_name",
    "-p", "global_override:=11",
])
direct_module = importlib.import_module("rclcppyy.direct_cpp")
runtime = direct_module._runtime()


def assert_rejected_without_node(operation):
    before_facades = tuple(runtime.nodes)
    before_native = tuple(runtime.session.nodes)
    try:
        operation()
    except (BackendUnavailableError, TypeError):
        pass
    else:
        raise AssertionError("invalid direct NodeOptions input succeeded")
    assert tuple(runtime.nodes) == before_facades
    assert tuple(runtime.session.nodes) == before_native


foreign_context = Context()
assert_rejected_without_node(
    lambda: Node("foreign_context", context=foreign_context))
assert_rejected_without_node(
    lambda: Node("invalid_cli", cli_args=("--ros-args",)))
assert_rejected_without_node(
    lambda: Node("invalid_cli_value", cli_args=[object()]))
assert_rejected_without_node(
    lambda: Node("invalid_override_list", parameter_overrides=()))
assert_rejected_without_node(
    lambda: Node("invalid_override", parameter_overrides=[object()]))
for option in (
    "use_global_arguments",
    "enable_rosout",
    "start_parameter_services",
    "allow_undeclared_parameters",
    "automatically_declare_parameters_from_overrides",
    "enable_logger_service",
):
    assert_rejected_without_node(
        lambda selected=option: Node("invalid_bool", **{selected: 1}))
print("DIRECT_CPP_NODE_OPTIONS_FAIL_BEFORE_MUTATION_OK")


local = Node(
    "local_requested_%d" % os.getpid(),
    context=runtime.context,
    cli_args=[
        "--ros-args",
        "-r", "__node:=local_options_name",
        "-p", "local_cli_override:=22",
    ],
    use_global_arguments=False,
    enable_rosout=False,
    start_parameter_services=False,
    parameter_overrides=[Parameter("explicit_override", value=17)],
    allow_undeclared_parameters=True,
    automatically_declare_parameters_from_overrides=True,
    enable_logger_service=True,
)
assert local.context is runtime.context
assert local.get_name() == "local_options_name"
assert local.get_parameter("local_cli_override").value == 22
explicit = local.get_parameter("explicit_override")
assert explicit.value == 17
assert isinstance(
    explicit._rclcppyy_native_parameter.native,
    cppyy.gbl.rclcpp.Parameter,
)
missing = local.get_parameter("missing")
assert missing.type_ is Parameter.Type.NOT_SET and missing.value is None
assert local.get_parameter_type("missing") == Parameter.Type.NOT_SET.value
assert local.get_parameter_types(["missing", "explicit_override"]) == [0, 2]
assert isinstance(
    local.describe_parameter("missing"),
    cppyy.gbl.rcl_interfaces.msg.ParameterDescriptor,
)
implicit_result = local.set_parameters_atomically([
    Parameter("implicit_parameter", value=31),
])
assert isinstance(
    implicit_result,
    cppyy.gbl.rcl_interfaces.msg.SetParametersResult,
)
assert implicit_result.successful
assert local.get_parameter("implicit_parameter").value == 31

local_services = service_names(local)
local_prefix = "/local_options_name/"
assert local_prefix + "get_parameters" not in local_services
assert local_prefix + "list_parameters" not in local_services
assert local_prefix + "get_logger_levels" in local_services
assert local_prefix + "set_logger_levels" in local_services
assert local.count_publishers("/rosout") == 0
print("DIRECT_CPP_NODE_OPTIONS_LOCAL_NATIVE_OK")

local.destroy_node()


deferred = Node(
    "deferred_override",
    use_global_arguments=False,
    start_parameter_services=False,
    parameter_overrides=[Parameter("deferred", value=41)],
    automatically_declare_parameters_from_overrides=False,
)
assert not deferred.has_parameter("deferred")
effective = deferred.declare_parameter("deferred", 1)
assert effective.value == 41
assert isinstance(
    effective._rclcppyy_native_parameter.native,
    cppyy.gbl.rclcpp.Parameter,
)
deferred.destroy_node()
print("DIRECT_CPP_NODE_OPTIONS_DEFERRED_OVERRIDE_OK")


global_node = Node(
    "global_requested",
    automatically_declare_parameters_from_overrides=True,
)
assert global_node.get_name() == "global_options_name"
global_parameter = global_node.get_parameter("global_override")
assert global_parameter.value == 11
assert isinstance(
    global_parameter._rclcppyy_native_parameter.native,
    cppyy.gbl.rclcpp.Parameter,
)
global_services = service_names(global_node)
global_prefix = "/global_options_name/"
assert global_prefix + "get_parameters" in global_services
assert global_prefix + "list_parameters" in global_services
assert global_prefix + "get_logger_levels" not in global_services
assert global_node.count_publishers("/rosout") >= 1
global_node.destroy_node()
print("DIRECT_CPP_NODE_OPTIONS_GLOBAL_NATIVE_OK")


records = [
    record for record in rclcppyy.status()["nodes"]
    if record["reason"] == "direct_cpp owns one NativeSession rclcpp node"
]
assert len(records) == 3
assert records[0]["metadata"]["context"] == "direct_runtime"
assert records[0]["metadata"]["cli_arguments"] == 5
assert records[0]["metadata"]["parameter_overrides"] == 1
assert records[0]["metadata"]["allow_undeclared_parameters"] is True
assert records[0]["metadata"]["enable_logger_service"] is True
assert records[1]["metadata"]["automatically_declare_parameters_from_overrides"] is False
assert records[2]["metadata"]["use_global_arguments"] is True

rclpy.shutdown()
assert runtime.session is None
assert runtime.nodes == []
print("DIRECT_CPP_NODE_OPTIONS_NO_CONVERSION_TEARDOWN_OK")
