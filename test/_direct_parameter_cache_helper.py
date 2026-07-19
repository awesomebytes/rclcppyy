#!/usr/bin/env python3
"""Focused exact-C++ parameter-cache capacity and failure proof."""

import importlib
import os

import rclcppyy


CACHE_ENV = "RCLCPPYY_DIRECT_PARAMETER_CACHE_CAPACITY"
os.environ[CACHE_ENV] = "2"
rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult  # noqa: E402
from rclcpp_kit import native_parameters  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("parameter cache used conversion, serialization, or CDR")


kit = importlib.import_module("rclcpp_kit")
bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
serialization = importlib.import_module("rclcpp_kit.serialization")
rclpy_serialization = importlib.import_module("rclpy.serialization")
kit.convert_python_msg_to_cpp = forbidden_boundary
bringup.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary
serialization.serialized_message_from_bytes = forbidden_boundary
serialization.serialized_message_to_bytes = forbidden_boundary
rclpy_serialization.serialize_message = forbidden_boundary
rclpy_serialization.deserialize_message = forbidden_boundary


rclpy.init(args=[])
runtime = importlib.import_module("rclcppyy.direct_cpp")._runtime()

capacity_node = Node("direct_parameter_cache_capacity_%d" % os.getpid())
capacity_node.declare_parameter("a", 1)
dynamic_descriptor = ParameterDescriptor(dynamic_typing=True)
capacity_node.declare_parameter("b", 2, dynamic_descriptor)
capacity_node.declare_parameter("c", 3)
capacity_node._set_direct_parameter_cache_hit_tracking(True)

native_parameters.reset_checked_parameter_stats()
retained_a_one = capacity_node.get_parameter("a")
assert capacity_node.get_parameter("a") is retained_a_one
first_c = capacity_node.get_parameter("c")
second_c = capacity_node.get_parameter("c")
assert first_c.value == 3 and second_c.value == 3
assert first_c is not second_c
assert native_parameters.checked_parameter_stats().to_dict() == {
    "calls": 2,
    "node_value_copies": 2,
    "result_copies": 0,
}
capacity_stats = capacity_node.direct_cpp_parameter_cache_stats()
assert capacity_stats["enabled"] is True
assert capacity_stats["capacity"] == 2
assert capacity_stats["size"] == 2
assert capacity_stats["max_size"] == 2
assert capacity_stats["hits"] == 2
assert capacity_stats["misses"] == 2
assert capacity_stats["capacity_skips"] >= 3
assert type(
    retained_a_one._rclcppyy_native_parameter.native
) is cppyy.gbl.rclcpp.Parameter
print("DIRECT_PARAMETER_CACHE_CAPACITY_OK")

accepted = capacity_node.set_parameters_atomically([
    Parameter("a", value=10)])
assert accepted.successful
retained_a_ten = capacity_node.get_parameter("a")
assert retained_a_ten.value == 10
assert retained_a_one.value == 1
assert retained_a_ten is not retained_a_one

retained_b_two = capacity_node.get_parameter("b")
assert retained_b_two.type_ is Parameter.Type.INTEGER
assert retained_b_two._rclcppyy_native_parameter.type_code == int(
    retained_b_two._rclcppyy_native_parameter.native.get_type())
type_replacement = capacity_node.set_parameters_atomically([
    Parameter("b", value="two")])
assert type_replacement.successful
retained_b_string = capacity_node.get_parameter("b")
assert retained_b_string is not retained_b_two
assert retained_b_string.type_ is Parameter.Type.STRING
assert retained_b_string.value == "two"
assert retained_b_string._rclcppyy_native_parameter.type_code == int(
    retained_b_string._rclcppyy_native_parameter.native.get_type())
assert retained_b_two.type_ is Parameter.Type.INTEGER
assert retained_b_two.value == 2
assert type(retained_b_two._rclcppyy_native_parameter.native) is (
    cppyy.gbl.rclcpp.Parameter)
assert type(retained_b_string._rclcppyy_native_parameter.native) is (
    cppyy.gbl.rclcpp.Parameter)
print("DIRECT_PARAMETER_CACHE_TYPE_REPLACEMENT_OK")


def reject(_parameters):
    return SetParametersResult(successful=False, reason="cache rejection")


capacity_node.add_on_set_parameters_callback(reject)
updates_before_reject = capacity_node.direct_cpp_parameter_cache_stats()["updates"]
rejected = capacity_node.set_parameters_atomically([
    Parameter("a", value=20)])
assert not rejected.successful
assert capacity_node.get_parameter("a") is retained_a_ten
assert capacity_node.direct_cpp_parameter_cache_stats()["updates"] == (
    updates_before_reject)
capacity_node.remove_on_set_parameters_callback(reject)


def increment(parameters):
    return [Parameter("a", value=parameters[0].value + 1)]


capacity_node.add_pre_set_parameters_callback(increment)
mutated = capacity_node.set_parameters_atomically([
    Parameter("a", value=30)])
assert mutated.successful
assert capacity_node.get_parameter("a").value == 31
capacity_node.remove_pre_set_parameters_callback(increment)
print("DIRECT_PARAMETER_CACHE_MUTATION_SNAPSHOTS_OK")

override_node = Node(
    "direct_parameter_cache_override_%d" % os.getpid(),
    parameter_overrides=[Parameter("override", value=5)],
    automatically_declare_parameters_from_overrides=True,
)
override_node._set_direct_parameter_cache_hit_tracking(True)
override_first = override_node.get_parameter("override")
assert override_first.value == 5
assert override_node.get_parameter("override") is override_first
override_stats = override_node.direct_cpp_parameter_cache_stats()
assert override_stats["misses"] == 1
assert override_stats["hits"] == 1
assert override_stats["size"] == 1
print("DIRECT_PARAMETER_CACHE_OVERRIDE_OK")

failure_node = Node("direct_parameter_cache_failure_%d" % os.getpid())
failure_node.declare_parameter("value", 1)
user_post_values = []
failure_node.add_post_set_parameters_callback(
    lambda values: user_post_values.extend(value.value for value in values))


def fail_cache_update(_parameter_list, _invalidated_names):
    raise MemoryError("simulated cache update failure")


failure_node._update_direct_parameter_cache = fail_cache_update
failure_result = failure_node.set_parameters_atomically([
    Parameter("value", value=2)])
assert failure_result.successful
assert user_post_values == [2]
failure_stats = failure_node.direct_cpp_parameter_cache_stats()
assert failure_stats["enabled"] is False
assert failure_stats["disabled_reason"] == "post_update_failure"
assert failure_stats["size"] == 0
failure_first = failure_node.get_parameter("value")
failure_second = failure_node.get_parameter("value")
assert failure_first.value == 2 and failure_second.value == 2
assert failure_first is not failure_second
print("DIRECT_PARAMETER_CACHE_FAILURE_ISOLATED_OK")

os.environ[CACHE_ENV] = "0"
disabled_node = Node("direct_parameter_cache_disabled_%d" % os.getpid())
disabled_stats = disabled_node.direct_cpp_parameter_cache_stats()
assert disabled_stats["enabled"] is False
assert disabled_stats["disabled_reason"] == "capacity_zero"
assert disabled_stats["capacity"] == 0
assert disabled_stats["size"] == 0
assert disabled_node._direct_cpp_parameter_callback_bridges["post"] is None
disabled_node.declare_parameter("value", 4)
assert disabled_node.get_parameter("value").value == 4
assert disabled_node.direct_cpp_parameter_cache_stats()["size"] == 0
print("DIRECT_PARAMETER_CACHE_DISABLED_OK")

nodes_before_invalid = len(runtime.nodes)
os.environ[CACHE_ENV] = "invalid"
try:
    Node("direct_parameter_cache_invalid_%d" % os.getpid())
except ValueError as exception:
    assert CACHE_ENV in str(exception)
else:
    raise AssertionError("invalid parameter cache capacity was accepted")
assert len(runtime.nodes) == nodes_before_invalid
print("DIRECT_PARAMETER_CACHE_INVALID_CONFIG_OK")

os.environ[CACHE_ENV] = "2"
for node in (disabled_node, failure_node, override_node, capacity_node):
    node.destroy_node()
assert retained_a_one.value == 1
assert retained_a_ten.value == 10
assert retained_b_two.type_ is Parameter.Type.INTEGER
assert retained_b_two.value == 2
assert retained_b_string.type_ is Parameter.Type.STRING
assert retained_b_string.value == "two"
assert capacity_node.direct_cpp_parameter_cache_stats()["disabled_reason"] == (
    "destroyed")
rclpy.shutdown()

rclpy.init(args=[])
restart_node = Node("direct_parameter_cache_restart_%d" % os.getpid())
assert restart_node.direct_cpp_parameter_cache_stats()["enabled"] is True
restart_node.declare_parameter("value", 8)
restart_value = restart_node.get_parameter("value")
restart_node.destroy_node()
rclpy.shutdown()
assert restart_value.value == 8
print("DIRECT_PARAMETER_CACHE_RESTART_OK")
