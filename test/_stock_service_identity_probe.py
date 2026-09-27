#!/usr/bin/env python3
"""Fresh-process stock/direct differential for the service utility surface.

Batch (b) (PLAN-prove-authority §2c, uncommitted): proves that under the
direct profile, the service-native ``Client``/``Service``/
``ServiceIntrospectionState`` and the re-exported stock ``QoSProfile`` keep
their stock identity, member presence, and (for the introspection-state
enum) value semantics unchanged. ``Clock`` (Lane 1), ``CallbackGroup``/
``Future`` (Lane 2), and ``Context`` re-exports are deliberately excluded
(Finding E) -- recorded as facts here, never annotated by this lane.
"""

from __future__ import annotations

import argparse
import json


PROBE_PREFIX = "RCLCPPYY_STOCK_SERVICE_IDENTITY_PROBE "

OWNED_CLASSES = {
    "client": {
        "Client": (
            "__init__", "call", "call_async", "configure_introspection",
            "destroy", "get_pending_request", "handle",
            "logger_name", "remove_pending_request", "service_is_ready",
            "service_name", "wait_for_service",
        ),
        "ServiceIntrospectionState": (
            "CONTENTS", "METADATA", "OFF", "__eq__", "__hash__", "__init__",
            "__ne__", "__repr__", "__str__", "name", "value",
        ),
    },
    "service": {
        "Service": (
            "__init__", "configure_introspection", "destroy", "handle",
            "logger_name", "send_response", "service_name",
        ),
        "ServiceIntrospectionState": (
            "CONTENTS", "METADATA", "OFF", "__eq__", "__hash__", "__init__",
            "__ne__", "__repr__", "__str__", "name", "value",
        ),
    },
    "service_introspection": {
        "ServiceIntrospectionState": (
            "CONTENTS", "METADATA", "OFF", "__eq__", "__hash__", "__init__",
            "__ne__", "__repr__", "__str__", "name", "value",
        ),
    },
}

# Excluded from this lane's ownership (Finding E) -- probed only to record
# the fact that they resolve to another area's/lane's exact stock object,
# never annotated here.
EXCLUDED_REEXPORTS = {
    "client": ("CallbackGroup", "Clock", "Context", "Future"),
    "service": ("CallbackGroup", "Clock"),
}


def _type_descriptor(value_type):
    return {
        "module": value_type.__module__,
        "qualname": value_type.__qualname__,
        "mro": [klass.__name__ for klass in value_type.__mro__],
        "has_cpp_name": hasattr(value_type, "__cpp_name__"),
    }


def _member_descriptor(class_type, name):
    try:
        raw = getattr(class_type, name)
    except AttributeError:
        return {"present": False}
    if type(raw).__name__ == class_type.__name__:
        # An enum-shaped member (Python enum.Enum or a pybind11 enum, e.g.
        # ServiceIntrospectionState.OFF): int-convertible either way.
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
    for name in ("client", "service", "service_introspection", "qos")
}

classes = {}
for module_name, class_members in OWNED_CLASSES.items():
    module = modules[module_name]
    for class_name, member_names in class_members.items():
        class_type = getattr(module, class_name)
        key = "%s.%s" % (module_name, class_name)
        descriptor = _type_descriptor(class_type)
        descriptor["members"] = {
            name: _member_descriptor(class_type, name) for name in member_names
        }
        classes[key] = descriptor

# The stock QoSProfile re-export, at both module aliases.
qos_reexport_identity = {
    module_name: getattr(modules[module_name], "QoSProfile") is modules["qos"].QoSProfile
    for module_name in ("client", "service")
}

# Cross-module identity for the 3-alias ServiceIntrospectionState.
introspection_state_identity = {
    "client_is_service": modules["client"].ServiceIntrospectionState is modules["service"].ServiceIntrospectionState,
    "service_is_service_introspection": (
        modules["service"].ServiceIntrospectionState
        is modules["service_introspection"].ServiceIntrospectionState
    ),
}

excluded_owner = {
    "%s.%s" % (module_name, name): getattr(modules[module_name], name).__module__
    for module_name, names in EXCLUDED_REEXPORTS.items()
    for name in names
}

payload = {
    "backend": args.backend,
    "classes": classes,
    "qos_reexport_identity": qos_reexport_identity,
    "introspection_state_identity": introspection_state_identity,
    "excluded_owner": excluded_owner,
}
print(PROBE_PREFIX + json.dumps(payload, sort_keys=True), flush=True)
