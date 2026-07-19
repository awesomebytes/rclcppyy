#!/usr/bin/env python3
"""Fresh-process stock/direct differential for the context/signals surface.

Batch (b) (PLAN-prove-authority §2c, uncommitted): proves that under the
direct profile, ``rclpy.context``/``rclpy.destroyable``/``rclpy.signals`` keep
their stock ``Context``/``ContextHandle``/``DestroyableType`` and
signal-handler surface unchanged -- identical type identity, member
presence, and (for ``SignalHandlerOptions``) enum value semantics.
"""

from __future__ import annotations

import argparse
import enum
import json


PROBE_PREFIX = "RCLCPPYY_STOCK_CONTEXT_IDENTITY_PROBE "

OWNED_CLASSES = {
    "context": {
        "Context": (
            "__enter__", "__exit__", "__init__", "destroy", "get_domain_id",
            "handle", "init", "ok", "on_shutdown", "shutdown", "try_shutdown",
        ),
        "ContextHandle": (
            "__enter__", "__exit__", "__init__", "destroy_when_not_in_use",
            "get_domain_id", "ok", "shutdown",
        ),
        "DestroyableType": ("__enter__", "__exit__", "__init__", "destroy_when_not_in_use"),
    },
    "destroyable": {
        "DestroyableType": ("__enter__", "__exit__", "__init__", "destroy_when_not_in_use"),
    },
    "signals": {
        "GuardCondition": ("__init__", "destroy", "handle", "trigger"),
        "InvalidHandle": ("add_note", "args", "with_traceback"),
        "SignalHandlerGuardCondition": ("__init__", "destroy", "handle", "trigger"),
        "SignalHandlerOptions": (
            "ALL", "NO", "SIGINT", "SIGTERM",
            "__eq__", "__hash__", "__init__", "__ne__", "__repr__", "__str__",
            "name", "value",
        ),
    },
}
SIGNAL_FUNCTIONS = (
    "get_current_signal_handlers_options",
    "install_signal_handlers",
    "uninstall_signal_handlers",
)


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
    if isinstance(raw, enum.Enum):
        return {"present": True, "int_value": int(raw)}
    if type(raw).__name__ == class_type.__name__:
        # A pybind11 enum member (e.g. SignalHandlerOptions.NO): not a Python
        # enum.Enum subclass, but still int-convertible like one.
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


modules = {name: importlib.import_module("rclpy.%s" % name) for name in OWNED_CLASSES}

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

functions = {
    "signals.%s" % name: {
        "module": getattr(modules["signals"], name).__module__,
        "qualname": getattr(modules["signals"], name).__qualname__,
    }
    for name in SIGNAL_FUNCTIONS
}

cross_module_identity = {
    "DestroyableType": (
        modules["context"].DestroyableType is modules["destroyable"].DestroyableType
    ),
}
guard_condition_relationship = {
    "signal_handler_is_guard_condition_subclass": issubclass(
        modules["signals"].SignalHandlerGuardCondition, modules["signals"].GuardCondition),
    "signal_handler_is_guard_condition_itself": (
        modules["signals"].SignalHandlerGuardCondition is modules["signals"].GuardCondition
    ),
}

payload = {
    "backend": args.backend,
    "classes": classes,
    "functions": functions,
    "cross_module_identity": cross_module_identity,
    "guard_condition_relationship": guard_condition_relationship,
}
print(PROBE_PREFIX + json.dumps(payload, sort_keys=True), flush=True)
