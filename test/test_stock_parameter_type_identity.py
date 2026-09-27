"""Owner-identity proof for the stock ``Parameter.Type`` nested enum.

These tests are the evidence boundary for the 18-row Parameter mixed cluster:
the nine ``Parameter.Type`` aliases retain the exact pre-activation stock
object, while the nine enclosing ``Parameter`` aliases are replaced and stay
unassessed because their live ledger attribution is ``stock``.
"""

from __future__ import annotations

import json

from _run_helper import format_output, run_helper


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
ANNOTATED_PATHS = frozenset(
    "%s.Parameter.Type" % module_name
    for module_name in PARAMETER_MODULES
)
DECLINED_PATHS = frozenset(
    "%s.Parameter" % module_name
    for module_name in PARAMETER_MODULES
)


def _probe(backend):
    process = run_helper(
        "_stock_parameter_type_identity_probe.py", "--backend", backend)
    assert process.returncode == 0, format_output(process)
    lines = [
        line for line in process.stdout.splitlines()
        if line.startswith(PROBE_PREFIX)
    ]
    assert len(lines) == 1, format_output(process)
    return json.loads(lines[0][len(PROBE_PREFIX):])


def test_parameter_mixed_cluster_has_exact_annotation_and_decline_sets():
    assert ANNOTATED_PATHS.isdisjoint(DECLINED_PATHS)
    assert len(ANNOTATED_PATHS) == 9
    assert len(DECLINED_PATHS) == 9


def test_parameter_type_is_the_exact_pristine_stock_object_under_direct():
    direct = _probe("direct")

    assert direct["type_is_captured_stock"] is True
    assert direct["type_alias_identity"] == dict.fromkeys(PARAMETER_MODULES, True)
    assert direct["type_descriptor"]["source_file"].endswith(
        "/rclpy/parameter.py")


def test_parameter_type_metadata_and_values_match_in_fresh_processes():
    stock = _probe("stock")
    direct = _probe("direct")

    assert direct["type_descriptor"] == stock["type_descriptor"]
    assert direct["type_descriptor"] == {
        "has_cpp_name": False,
        "module": "rclpy.parameter",
        "mro": ["Type", "Enum", "object"],
        "qualname": "Parameter.Type",
        "source_file": stock["type_descriptor"]["source_file"],
        "values": {
            "BOOL": 1,
            "BOOL_ARRAY": 6,
            "BYTE_ARRAY": 5,
            "DOUBLE": 3,
            "DOUBLE_ARRAY": 8,
            "INTEGER": 2,
            "INTEGER_ARRAY": 7,
            "NOT_SET": 0,
            "STRING": 4,
            "STRING_ARRAY": 9,
        },
    }


def test_parameter_facade_is_replaced_consistently_but_not_stock():
    stock = _probe("stock")
    direct = _probe("direct")

    assert direct["parameter_alias_identity"] == dict.fromkeys(
        PARAMETER_MODULES, True)
    assert stock["parameter_is_captured_stock"] is True
    assert direct["parameter_is_captured_stock"] is False
    assert stock["parameter_init_source_file"].endswith("/rclpy/parameter.py")
    assert direct["parameter_init_source_file"].endswith(
        "/rclcppyy/direct_parameters.py")
