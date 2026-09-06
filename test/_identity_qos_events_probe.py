#!/usr/bin/env python3
"""Fresh-process stock/direct differential for the QoS event type surface.

Cluster 2 (identity-proof plan §1): proves that QoS-owned public types in
``rclpy.event_handler`` (and its ``rclpy.qos_event`` deprecation alias) stay
exact stock objects under the direct profile, while the foreign
``CallbackGroup`` re-export follows its direct facade. The cross-module alias
identities still hold. Foreign re-exports remain excluded from this lane's
annotation authority because they belong to the executor lane.
"""

from __future__ import annotations

import argparse
import inspect
import json


PROBE_PREFIX = "RCLCPPYY_IDENTITY_QOS_EVENTS_PROBE "


def _public_classes(module):
    result = {}
    for name in sorted(vars(module)):
        if name.startswith("_"):
            continue
        value = getattr(module, name)
        if not inspect.isclass(value):
            continue
        origin = getattr(value, "__module__", "") or ""
        if (
            name == "CallbackGroup"
            or origin == module.__name__
            or origin.startswith("rclpy.")
        ):
            result[name] = value
    return result


def _public_names(module):
    names = set()
    for name in vars(module):
        if name.startswith("_"):
            continue
        value = getattr(module, name)
        if inspect.ismodule(value):
            continue
        names.add(name)
    return names


def _member_names(class_type):
    try:
        names = dir(class_type)
    except Exception:
        return []
    return sorted(name for name in set(names) if not name.startswith("_"))


parser = argparse.ArgumentParser()
parser.add_argument("--backend", choices=("stock", "direct"), required=True)
args = parser.parse_args()

if args.backend == "direct":
    import rclcppyy

    rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy.event_handler as event_handler  # noqa: E402
import rclpy.qos_event as qos_event  # noqa: E402


types = {}
for name, class_type in _public_classes(event_handler).items():
    types[name] = {
        "module": class_type.__module__,
        "qualname": class_type.__qualname__,
        "kind": "class",
        "member_names": _member_names(class_type),
        "has_cpp_name": hasattr(class_type, "__cpp_name__"),
    }

shared_names = sorted(_public_names(event_handler) & _public_names(qos_event))
module_aliases = {
    name: getattr(qos_event, name) is getattr(event_handler, name)
    for name in shared_names
}

payload = {
    "backend": args.backend,
    "types": types,
    "shared_names": shared_names,
    "module_aliases": module_aliases,
}
print(PROBE_PREFIX + json.dumps(payload, sort_keys=True), flush=True)
