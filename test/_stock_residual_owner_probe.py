#!/usr/bin/env python3
"""Fresh-process provenance probe for the residual owner-level rows."""

from __future__ import annotations

import argparse
import importlib
import inspect
import json


PROBE_PREFIX = "RCLCPPYY_STOCK_RESIDUAL_OWNER_PROBE "

DIRECT_NODE_TARGETS = (
    ("rclpy.action.graph", "Node"),
    ("rclpy.lifecycle", "LifecycleNode"),
    ("rclpy.lifecycle", "Node"),
    ("rclpy.lifecycle.node", "LifecycleNode"),
    ("rclpy.lifecycle.node", "Node"),
    ("rclpy.node", "Node"),
    ("rclpy.parameter_client", "Node"),
    ("rclpy.parameter_event_handler", "Node"),
    ("rclpy.wait_for_message", "Node"),
)


def _type_name(value_type):
    return "%s.%s" % (value_type.__module__, value_type.__qualname__)


def _defining_owner(value_type, member):
    return next(owner for owner in value_type.__mro__ if member in owner.__dict__)


parser = argparse.ArgumentParser()
parser.add_argument("--backend", choices=("stock", "direct"), required=True)
args = parser.parse_args()

if args.backend == "direct":
    import rclcppyy

    rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy.action.client as client_module  # noqa: E402
import rclpy.action.server as server_module  # noqa: E402
import rclpy.node as node_module  # noqa: E402
import rclpy.type_support as type_support_module  # noqa: E402


if args.backend == "direct":
    from rclcppyy import direct_cpp

    def _stock_object(module_name, member_name):
        matches = [
            original
            for module, name, original, _replacement in direct_cpp._PATCHES
            if module.__name__ == module_name and name == member_name
        ]
        assert len(matches) == 1, (module_name, member_name, matches)
        return matches[0]

    # The transaction retains the pristine stock objects in its restoration
    # records. Reading them here avoids violating the early-activation rule.
    stock_node = _stock_object("rclpy.node", "Node")
    stock_client_goal_handle = _stock_object(
        "rclpy.action.client", "ClientGoalHandle")
    stock_server_goal_handle = _stock_object(
        "rclpy.action.server", "ServerGoalHandle")
    stock_check_is_valid_msg_type = _stock_object(
        "rclpy.type_support", "check_is_valid_msg_type")
else:
    stock_node = node_module.Node
    stock_client_goal_handle = client_module.ClientGoalHandle
    stock_server_goal_handle = server_module.ServerGoalHandle
    stock_check_is_valid_msg_type = type_support_module.check_is_valid_msg_type

modules = {
    name: importlib.import_module(name)
    for name, _class_name in DIRECT_NODE_TARGETS
}

stock_tolerance = stock_node.__dict__["PARAM_REL_TOL"]
node_members = {}
for module_name, class_name in DIRECT_NODE_TARGETS:
    public_class = getattr(modules[module_name], class_name)
    owner = _defining_owner(public_class, "PARAM_REL_TOL")
    value = getattr(public_class, "PARAM_REL_TOL")
    path = "%s.%s.PARAM_REL_TOL" % (module_name, class_name)
    node_members[path] = {
        "class": _type_name(public_class),
        "owner": _type_name(owner),
        "same_object_as_stock": value is stock_tolerance,
        "value": value,
    }

goal_handle_members = {}
for path, public_class, stock_class in (
    (
        "rclpy.action.client.ClientGoalHandle.__hash__",
        client_module.ClientGoalHandle,
        stock_client_goal_handle,
    ),
    (
        "rclpy.action.server.ServerGoalHandle.__hash__",
        server_module.ServerGoalHandle,
        stock_server_goal_handle,
    ),
):
    owner = _defining_owner(public_class, "__hash__")
    member = owner.__dict__["__hash__"]
    stock_member = stock_class.__dict__["__hash__"]
    goal_handle_members[path] = {
        "class": _type_name(public_class),
        "owner": _type_name(owner),
        "is_none": member is None,
        "same_object_as_stock": member is stock_member,
    }

from builtin_interfaces.msg import Time  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402


validator = type_support_module.check_is_valid_msg_type
valid_result = validator(Time)
try:
    validator(SetBool)
except Exception as exc:  # noqa: BLE001 - exception shape is probe output
    invalid_result = {
        "type": _type_name(type(exc)),
        "message": str(exc),
    }
else:
    raise AssertionError("a service class passed the message-type validator")

validator_payload = {
    "path": "rclpy.type_support.check_is_valid_msg_type",
    "function": _type_name(type(validator)),
    "module": validator.__module__,
    "qualname": validator.__qualname__,
    "same_object_as_stock": validator is stock_check_is_valid_msg_type,
    "signature": str(inspect.signature(validator)),
    "valid_message_has_cpp_name": hasattr(Time, "__cpp_name__"),
    "valid_result_is_none": valid_result is None,
    "invalid": invalid_result,
}

payload = {
    "backend": args.backend,
    "node_members": node_members,
    "goal_handle_members": goal_handle_members,
    "validator": validator_payload,
}
print(PROBE_PREFIX + json.dumps(payload, sort_keys=True), flush=True)
