#!/usr/bin/env python3
"""Fresh-process provenance probe for the stock ``Parameter.Type`` owner.

The direct profile replaces every public ``Parameter`` alias with one native
facade, but that facade deliberately retains the stock nested
``Parameter.Type`` enum.  Source ownership, values, and alias identity
distinguish owner retention from matching names and signatures.
"""

from __future__ import annotations

import argparse
import importlib
import inspect
import json


PROBE_PREFIX = "RCLCPPYY_STOCK_PARAMETER_TYPE_IDENTITY_PROBE "
PARAMETER_MODULES = (
    "rclpy",
    "rclpy.node",
    "rclpy.parameter",
    "rclpy.parameter_client",
    "rclpy.parameter_event_handler",
    "rclpy.parameter_service",
    "rclpy.qos_overriding_options",
    "rclpy.time_source",
    "rclpy.type_description_service",
)
TYPE_VALUES = (
    "NOT_SET",
    "BOOL",
    "INTEGER",
    "DOUBLE",
    "STRING",
    "BYTE_ARRAY",
    "BOOL_ARRAY",
    "INTEGER_ARRAY",
    "DOUBLE_ARRAY",
    "STRING_ARRAY",
)


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("stock", "direct"), required=True)
    return parser.parse_args()


def _type_descriptor(parameter_type):
    return {
        "module": parameter_type.__module__,
        "qualname": parameter_type.__qualname__,
        "mro": [klass.__name__ for klass in parameter_type.__mro__],
        "has_cpp_name": hasattr(parameter_type, "__cpp_name__"),
        "source_file": inspect.getsourcefile(parameter_type),
        "values": {
            name: int(getattr(parameter_type, name).value)
            for name in TYPE_VALUES
        },
    }


def main():
    args = _arguments()
    captured_stock_parameter = None
    if args.backend == "direct":
        import rclcppyy

        rclcppyy.enable_cpp_acceleration(profile="direct_cpp")
        from rclcppyy.direct_cpp import _PATCHES

        captured_stock_parameter = next(
            original
            for module, name, original, _replacement in _PATCHES
            if module.__name__ == "rclpy.parameter" and name == "Parameter"
        )

    modules = {
        name: importlib.import_module(name)
        for name in PARAMETER_MODULES
    }
    canonical_parameter = modules["rclpy.parameter"].Parameter
    if captured_stock_parameter is None:
        captured_stock_parameter = canonical_parameter
    parameter_alias_identity = {
        name: modules[name].Parameter is canonical_parameter
        for name in PARAMETER_MODULES
    }
    type_alias_identity = {
        name: modules[name].Parameter.Type is canonical_parameter.Type
        for name in PARAMETER_MODULES
    }

    payload = {
        "backend": args.backend,
        "parameter_alias_identity": parameter_alias_identity,
        "parameter_is_captured_stock": (
            canonical_parameter is captured_stock_parameter),
        "parameter_init_source_file": inspect.getsourcefile(
            canonical_parameter.__init__),
        "type_alias_identity": type_alias_identity,
        "type_descriptor": _type_descriptor(canonical_parameter.Type),
        "type_is_captured_stock": (
            canonical_parameter.Type is captured_stock_parameter.Type),
    }
    print(PROBE_PREFIX + json.dumps(payload, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
