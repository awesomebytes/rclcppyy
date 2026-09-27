#!/usr/bin/env python3
"""Fresh-process stock/direct differential for the time-enum surface.

Batch (b) (PLAN-prove-authority §2c/§6, uncommitted): proves that under the
direct profile, ``ClockType`` (at all 4 of its public module aliases) and
``ClockChange`` keep their stock ``IntEnum`` identity, member values, and
int-method semantics unchanged. This is the pure-enum-identity row split
Lane 1 (clock-rate) leaves to this lane -- Lane 1 owns the *behavioral*
clock/rate/timer rows; everything else in the time area (Clock, Duration,
Time, TimeSource, the signal/context surface, ...) was already annotated in
Wave 1 and is untouched here. ``time_source.Parameter``/``QoSProfile``/
``ReliabilityPolicy`` are foreign re-exports (parameter/qos lanes) and are
recorded as facts, never claimed (Finding E).
"""

from __future__ import annotations

import argparse
import json


PROBE_PREFIX = "RCLCPPYY_STOCK_TIME_ENUM_IDENTITY_PROBE "

CLOCK_TYPE_MODULES = ("clock", "clock_type", "time", "time_source")
CLOCK_TYPE_MEMBERS = (
    "ROS_TIME", "STEADY_TIME", "SYSTEM_TIME", "UNINITIALIZED",
    "as_integer_ratio", "bit_count", "bit_length", "conjugate", "denominator",
    "from_bytes", "imag", "is_integer", "numerator", "real", "to_bytes",
)
CLOCK_CHANGE_MODULE = "clock"
CLOCK_CHANGE_MEMBERS = (
    "ROS_TIME_ACTIVATED", "ROS_TIME_DEACTIVATED", "ROS_TIME_NO_CHANGE",
    "SYSTEM_TIME_NO_CHANGE",
    "as_integer_ratio", "bit_count", "bit_length", "conjugate", "denominator",
    "from_bytes", "imag", "is_integer", "numerator", "real", "to_bytes",
)
# Foreign re-exports in rclpy.time_source -- recorded, never claimed here.
EXCLUDED_REEXPORTS = ("Parameter", "QoSProfile", "ReliabilityPolicy")


def _type_descriptor(value_type):
    return {
        "module": value_type.__module__,
        "qualname": value_type.__qualname__,
        "mro": [klass.__name__ for klass in value_type.__mro__],
        "has_cpp_name": hasattr(value_type, "__cpp_name__"),
    }


def _member_descriptor(enum_type, name):
    raw = getattr(enum_type, name)
    if type(raw) is enum_type:
        return {"kind": "enum_member", "int_value": int(raw), "name": raw.name}
    return {"kind": "int_method_or_property", "callable_or_descriptor": True}


parser = argparse.ArgumentParser()
parser.add_argument("--backend", choices=("stock", "direct"), required=True)
args = parser.parse_args()

if args.backend == "direct":
    import rclcppyy

    rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import importlib  # noqa: E402


modules = {
    name: importlib.import_module("rclpy.%s" % name)
    for name in set(CLOCK_TYPE_MODULES) | {CLOCK_CHANGE_MODULE}
}

classes = {}
for module_name in CLOCK_TYPE_MODULES:
    clock_type = modules[module_name].ClockType
    key = "%s.ClockType" % module_name
    descriptor = _type_descriptor(clock_type)
    descriptor["members"] = {
        name: _member_descriptor(clock_type, name) for name in CLOCK_TYPE_MEMBERS
    }
    classes[key] = descriptor

clock_change = modules[CLOCK_CHANGE_MODULE].ClockChange
classes["%s.ClockChange" % CLOCK_CHANGE_MODULE] = {
    **_type_descriptor(clock_change),
    "members": {
        name: _member_descriptor(clock_change, name) for name in CLOCK_CHANGE_MEMBERS
    },
}

# Cross-module identity: all 4 ClockType aliases and every module's re-import
# of ClockChange (only one home today) should be the exact same class object.
clock_type_identity = {
    module_name: modules[module_name].ClockType is modules["clock"].ClockType
    for module_name in CLOCK_TYPE_MODULES
}

excluded_owner = {
    name: getattr(modules["time_source"], name).__module__
    for name in EXCLUDED_REEXPORTS
}

payload = {
    "backend": args.backend,
    "classes": classes,
    "clock_type_identity": clock_type_identity,
    "excluded_owner": excluded_owner,
}
print(PROBE_PREFIX + json.dumps(payload, sort_keys=True), flush=True)
