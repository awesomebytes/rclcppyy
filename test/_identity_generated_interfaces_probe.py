#!/usr/bin/env python3
"""Fresh-process proof for the generated-interface representation split.

Finding C (identity-proof plan §0/§2 file 3): the 19 message interfaces
rebind to their generated C++ representation under the direct profile, while
the one service and one action facade stay the stock Python classes. The
direct backend self-discovers its installed interface set from
``rclcppyy.direct_cpp``'s own binding registries (never hand-typed); the
stock backend has no such registry, so it takes the same set as an explicit
argument for the cross-check.
"""

from __future__ import annotations

import argparse
import importlib
import json


PROBE_PREFIX = "RCLCPPYY_IDENTITY_GENERATED_INTERFACES_PROBE "

_INTERFACE_SUBPACKAGE = {"message": "msg", "service": "srv", "action": "action"}


def _resolve(interface, kind):
    package, _, type_name = interface.split("/")
    module_name = f"{package}.{_INTERFACE_SUBPACKAGE[kind]}"
    module = importlib.import_module(module_name)
    type_object = getattr(module, type_name)
    return {
        "interface": interface,
        "kind": kind,
        "module": type_object.__module__,
        "qualname": type_object.__qualname__,
        "has_cpp_name": hasattr(type_object, "__cpp_name__"),
        "resolved_repr": f"{type_object.__module__}.{type_object.__qualname__}",
    }


def _discover_direct_interfaces():
    direct_cpp = importlib.import_module("rclcppyy.direct_cpp")
    groups = (
        ("message", direct_cpp._MESSAGE_INSTALLATION),
        ("service", direct_cpp._SERVICE_INSTALLATION),
        ("action", direct_cpp._ACTION_INSTALLATION),
    )
    result = []
    for kind, installation in groups:
        for binding in getattr(installation, "bindings", ()):
            result.append({"interface": binding.interface, "kind": kind})
    return sorted(result, key=lambda row: (row["kind"], row["interface"]))


parser = argparse.ArgumentParser()
parser.add_argument("--backend", choices=("stock", "direct"), required=True)
parser.add_argument(
    "--interfaces", default=None,
    help="JSON list of {interface, kind}; required for --backend stock, "
         "ignored for --backend direct (self-discovered).",
)
args = parser.parse_args()

if args.backend == "direct":
    import rclcppyy

    rclcppyy.enable_cpp_acceleration(profile="direct_cpp")
    interfaces = _discover_direct_interfaces()
else:
    assert args.interfaces is not None, "stock backend requires --interfaces"
    interfaces = json.loads(args.interfaces)

resolved = [_resolve(row["interface"], row["kind"]) for row in interfaces]
payload = {
    "backend": args.backend,
    "interfaces": sorted(resolved, key=lambda row: (row["kind"], row["interface"])),
}
print(PROBE_PREFIX + json.dumps(payload, sort_keys=True), flush=True)
