"""Stock graph-utility-surface identity + value-semantics proofs.

Evidence for the prove-authority lane's ``graph.utility_surface`` batch-(b)
annotations (PLAN-prove-authority §2c, uncommitted): the direct profile keeps
topic/service/namespace/node-name validation, ``TopicEndpointInfo``/
``TopicEndpointTypeEnum``/``TypeHash``, and the re-exported stock QoS enums
as the exact stock objects, not a re-implementation with matching names.
"""

from __future__ import annotations

import json

from _run_helper import format_output, run_helper


PROBE_PREFIX = "RCLCPPYY_STOCK_GRAPH_IDENTITY_PROBE "


def _probe(backend):
    process = run_helper("_stock_graph_identity_probe.py", "--backend", backend)
    assert process.returncode == 0, format_output(process)
    lines = [
        line for line in process.stdout.splitlines()
        if line.startswith(PROBE_PREFIX)
    ]
    assert len(lines) == 1, format_output(process)
    return json.loads(lines[0][len(PROBE_PREFIX):])


def test_graph_symbols_are_identical_under_both_backends():
    stock = _probe("stock")
    direct = _probe("direct")

    stock_payload = {key: value for key, value in stock.items() if key != "backend"}
    direct_payload = {key: value for key, value in direct.items() if key != "backend"}
    assert stock_payload == direct_payload


def test_graph_owned_classes_and_functions_are_stock_not_cppyy_under_direct():
    direct = _probe("direct")

    for key, descriptor in direct["symbols"].items():
        if "module" not in descriptor:
            continue  # constants carry no module/type descriptor
        assert descriptor["module"].startswith("rclpy."), key
        if "has_cpp_name" in descriptor:
            assert descriptor["has_cpp_name"] is False, key


def test_topic_endpoint_info_and_type_hash_and_enum_members_are_all_present():
    direct = _probe("direct")

    info = direct["symbols"]["topic_endpoint_info.TopicEndpointInfo"]["members"]
    assert all(member["present"] for member in info.values())

    enum = direct["symbols"]["topic_endpoint_info.TopicEndpointTypeEnum"]["members"]
    assert all(member["present"] for member in enum.values())
    assert enum["INVALID"]["int_value"] == 0
    assert enum["PUBLISHER"]["int_value"] == 1
    assert enum["SUBSCRIPTION"]["int_value"] == 2

    type_hash = direct["symbols"]["topic_endpoint_info.TypeHash"]["members"]
    assert all(member["present"] for member in type_hash.values())


def test_duplicate_named_exceptions_across_modules_share_identity():
    direct = _probe("direct")
    assert direct["cross_module_identity"] == {
        "InvalidServiceNameException": True,
        "InvalidTopicNameException": True,
    }


def test_qos_enum_reexports_are_the_exact_rclpy_qos_objects():
    direct = _probe("direct")
    assert direct["qos_reexport_identity"] == {
        "QoSHistoryPolicy": True,
        "QoSPresetProfiles": True,
        "QoSProfile": True,
    }
