#!/usr/bin/env python3
"""Fresh-process stock/direct differential for the graph utility surface.

Batch (b) (PLAN-prove-authority §2c, uncommitted): proves that under the
direct profile, the graph-area symbols (topic/service/namespace/node-name
validation, ``TopicEndpointInfo``/``TopicEndpointTypeEnum``/``TypeHash``, and
the stock QoS-enum re-exports) keep their stock identity, member values, and
signatures unchanged -- not a re-implementation with matching names.
"""

from __future__ import annotations

import argparse
import json


PROBE_PREFIX = "RCLCPPYY_STOCK_GRAPH_IDENTITY_PROBE "

# (module, name) -> "class" | "function" | "constant"
SYMBOLS = (
    ("expand_topic_name", "expand_topic_name", "function"),
    ("topic_endpoint_info", "QoSHistoryPolicy", "class"),
    ("topic_endpoint_info", "QoSPresetProfiles", "class"),
    ("topic_endpoint_info", "QoSProfile", "class"),
    ("topic_endpoint_info", "TopicEndpointInfo", "class"),
    ("topic_endpoint_info", "TopicEndpointTypeEnum", "class"),
    ("topic_endpoint_info", "TypeHash", "class"),
    ("topic_or_service_is_hidden", "HIDDEN_TOPIC_PREFIX", "constant"),
    ("topic_or_service_is_hidden", "topic_or_service_is_hidden", "function"),
    ("validate_full_topic_name", "InvalidServiceNameException", "class"),
    ("validate_full_topic_name", "InvalidTopicNameException", "class"),
    ("validate_full_topic_name", "validate_full_topic_name", "function"),
    ("validate_namespace", "InvalidNamespaceException", "class"),
    ("validate_namespace", "validate_namespace", "function"),
    ("validate_node_name", "InvalidNodeNameException", "class"),
    ("validate_node_name", "validate_node_name", "function"),
    ("validate_topic_name", "InvalidServiceNameException", "class"),
    ("validate_topic_name", "InvalidTopicNameException", "class"),
    ("validate_topic_name", "TOPIC_SEPARATOR_STRING", "constant"),
    ("validate_topic_name", "validate_topic_name", "function"),
)

# Classes this area owns outright (not the QoS re-exports); their public
# members are enumerated too.
OWNED_CLASS_MEMBERS = {
    "TopicEndpointInfo": (
        "__eq__", "__hash__", "__init__", "__str__", "endpoint_gid",
        "endpoint_type", "node_name", "node_namespace", "qos_profile",
        "topic_type", "topic_type_hash",
    ),
    "TopicEndpointTypeEnum": (
        "INVALID", "PUBLISHER", "SUBSCRIPTION",
        "as_integer_ratio", "bit_count", "bit_length", "conjugate",
        "denominator", "from_bytes", "imag", "is_integer", "numerator",
        "real", "to_bytes",
    ),
    "TypeHash": ("__eq__", "__hash__", "__init__", "__str__", "value", "version"),
    "InvalidServiceNameException": ("__init__", "add_note", "args", "with_traceback"),
    "InvalidTopicNameException": ("__init__", "add_note", "args", "with_traceback"),
    "InvalidNamespaceException": ("__init__", "add_note", "args", "with_traceback"),
    "InvalidNodeNameException": ("__init__", "add_note", "args", "with_traceback"),
}


def _type_descriptor(value_type):
    return {
        "module": value_type.__module__,
        "qualname": value_type.__qualname__,
        "mro": [klass.__name__ for klass in value_type.__mro__],
        "has_cpp_name": hasattr(value_type, "__cpp_name__"),
    }


def _member_descriptor(class_type, name):
    import enum

    try:
        raw = getattr(class_type, name)
    except AttributeError:
        return {"present": False}
    if isinstance(raw, enum.Enum):
        return {"present": True, "int_value": int(raw)}
    if isinstance(raw, (property, staticmethod, classmethod)) or callable(raw):
        return {"present": True, "callable_or_descriptor": True}
    return {"present": True, "value": repr(raw)}


parser = argparse.ArgumentParser()
parser.add_argument("--backend", choices=("stock", "direct"), required=True)
args = parser.parse_args()

if args.backend == "direct":
    import rclcppyy

    rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import importlib  # noqa: E402


modules = {
    name: importlib.import_module("rclpy.%s" % name)
    for name in {module for module, _name, _kind in SYMBOLS}
}

symbols = {}
for module_name, name, kind in SYMBOLS:
    module = modules[module_name]
    value = getattr(module, name)
    key = "%s.%s" % (module_name, name)
    if kind == "class":
        descriptor = _type_descriptor(value)
        if name in OWNED_CLASS_MEMBERS:
            descriptor["members"] = {
                member: _member_descriptor(value, member)
                for member in OWNED_CLASS_MEMBERS[name]
            }
        symbols[key] = descriptor
    elif kind == "function":
        symbols[key] = {"module": value.__module__, "qualname": value.__qualname__}
    else:
        symbols[key] = {"value": repr(value)}

# Cross-module identity: validate_full_topic_name and validate_topic_name
# both define InvalidServiceNameException/InvalidTopicNameException -- prove
# whether they are literally the same class or two distinct definitions.
cross_module_identity = {
    "InvalidServiceNameException": (
        modules["validate_full_topic_name"].InvalidServiceNameException
        is modules["validate_topic_name"].InvalidServiceNameException
    ),
    "InvalidTopicNameException": (
        modules["validate_full_topic_name"].InvalidTopicNameException
        is modules["validate_topic_name"].InvalidTopicNameException
    ),
}

# The re-exported QoS enums must be the exact objects rclpy.qos defines.
import rclpy.qos as qos_module  # noqa: E402

qos_reexport_identity = {
    name: getattr(modules["topic_endpoint_info"], name) is getattr(qos_module, name)
    for name in ("QoSHistoryPolicy", "QoSPresetProfiles", "QoSProfile")
}

payload = {
    "backend": args.backend,
    "symbols": symbols,
    "cross_module_identity": cross_module_identity,
    "qos_reexport_identity": qos_reexport_identity,
}
print(PROBE_PREFIX + json.dumps(payload, sort_keys=True), flush=True)
