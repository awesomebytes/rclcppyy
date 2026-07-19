"""Stock QoS policy identity and value-semantics proofs (cluster 1).

Evidence for the identity-proof lane's ``qos.stock_surface`` annotations: the
direct profile keeps ``rclpy.qos`` / ``rclpy.qos_overriding_options`` as the
exact stock objects, not a re-implementation with matching names/signatures.
"""

from __future__ import annotations

import json

from _run_helper import format_output, run_helper


PROBE_PREFIX = "RCLCPPYY_IDENTITY_QOS_PROBE "


def _probe(backend):
    process = run_helper("_identity_qos_probe.py", "--backend", backend)
    assert process.returncode == 0, format_output(process)
    lines = [
        line for line in process.stdout.splitlines()
        if line.startswith(PROBE_PREFIX)
    ]
    assert len(lines) == 1, format_output(process)
    return json.loads(lines[0][len(PROBE_PREFIX):])


def test_qos_policy_enums_are_identical_stock_objects_under_direct_profile():
    stock = _probe("stock")
    direct = _probe("direct")

    stock_payload = {key: value for key, value in stock.items() if key != "backend"}
    direct_payload = {key: value for key, value in direct.items() if key != "backend"}
    assert stock_payload == direct_payload

    values = {
        name: {member["name"]: member["value"] for member in block["members"]}
        for name, block in direct["enums"].items()
    }
    assert values["ReliabilityPolicy"]["SYSTEM_DEFAULT"] == 0
    assert values["ReliabilityPolicy"]["RELIABLE"] == 1
    assert values["ReliabilityPolicy"]["BEST_EFFORT"] == 2
    assert values["ReliabilityPolicy"]["UNKNOWN"] == 3
    assert values["ReliabilityPolicy"]["BEST_AVAILABLE"] == 4
    assert values["HistoryPolicy"] == {
        "SYSTEM_DEFAULT": 0, "KEEP_LAST": 1, "KEEP_ALL": 2, "UNKNOWN": 3,
    }
    assert values["DurabilityPolicy"] == {
        "SYSTEM_DEFAULT": 0, "TRANSIENT_LOCAL": 1, "VOLATILE": 2,
        "UNKNOWN": 3, "BEST_AVAILABLE": 4,
    }
    assert values["LivelinessPolicy"] == {
        "SYSTEM_DEFAULT": 0, "AUTOMATIC": 1, "MANUAL_BY_TOPIC": 3,
        "UNKNOWN": 4, "BEST_AVAILABLE": 5,
    }


def test_qos_enum_and_preset_types_are_stock_not_cppyy_under_direct_profile():
    direct = _probe("direct")

    for name in (
        "HistoryPolicy", "ReliabilityPolicy", "DurabilityPolicy",
        "LivelinessPolicy", "QoSPolicyKind", "QoSPolicyEnum",
    ):
        descriptor = direct["enums"][name]["type"]
        assert descriptor["has_cpp_name"] is False
        assert descriptor["module"] == "rclpy.qos"

    # QoSCompatibility is a pybind11 enum defined in the extension module, not
    # rclpy.qos itself, but it must still be the stock pybind11 type, not a
    # cppyy-backed replacement.
    compatibility = direct["enums"]["QoSCompatibility"]["type"]
    assert compatibility["has_cpp_name"] is False
    assert compatibility["module"] == "rclpy._rclpy_pybind11"

    preset = direct["preset"]
    assert preset["type"]["has_cpp_name"] is False
    assert preset["type"]["module"] == "rclpy.qos"
    assert preset["profile_type"]["has_cpp_name"] is False
    assert preset["profile_type"]["module"] == "rclpy.qos"
    for member in preset["members"].values():
        assert member["profile_type"]["has_cpp_name"] is False
        assert member["profile_type"]["module"] == "rclpy.qos"
        # Every member resolves to one of the 9 module-level qos_profile_*
        # singletons -- not necessarily its own same-named one. QoSProfile
        # defines value equality and qos_profile_default/services_default
        # (and parameters/parameter_events) happen to compare equal, so
        # stock Python enum aliasing makes SERVICES_DEFAULT literally *be*
        # the DEFAULT member, and PARAMETER_EVENTS literally *be* PARAMETERS.
        assert member["resolved_singleton"] is not None
    assert preset["members"]["SERVICES_DEFAULT"]["canonical_member_name"] == "DEFAULT"
    assert preset["members"]["PARAMETER_EVENTS"]["canonical_member_name"] == "PARAMETERS"
    for descriptor in preset["singleton_types"].values():
        assert descriptor["has_cpp_name"] is False
        assert descriptor["module"] == "rclpy.qos"

    for descriptor in direct["exceptions"].values():
        assert descriptor["has_cpp_name"] is False


def test_qos_module_aliases_hold_under_both_backends():
    for backend in ("stock", "direct"):
        payload = _probe(backend)
        assert all(payload["intra_module_aliases"].values()), backend
        assert all(payload["cross_module_aliases"].values()), backend
