"""Stock service-utility-surface identity + value-semantics proofs.

Evidence for the prove-authority lane's ``service.extended_stock_surface``
batch-(b) annotations (PLAN-prove-authority §2c, uncommitted): the direct
profile keeps ``Client``/``Service``/``ServiceIntrospectionState`` and the
re-exported stock ``QoSProfile`` as the exact stock objects. ``Clock``
(Lane 1), ``CallbackGroup``/``Future`` (Lane 2), and ``Context``
re-exports are recorded as facts here (excluded_owner) but never claimed --
Finding E: a type owned by another area/lane is not double-annotated.
"""

from __future__ import annotations

import json

from _run_helper import format_output, run_helper


PROBE_PREFIX = "RCLCPPYY_STOCK_SERVICE_IDENTITY_PROBE "


def _probe(backend):
    process = run_helper("_stock_service_identity_probe.py", "--backend", backend)
    assert process.returncode == 0, format_output(process)
    lines = [
        line for line in process.stdout.splitlines()
        if line.startswith(PROBE_PREFIX)
    ]
    assert len(lines) == 1, format_output(process)
    return json.loads(lines[0][len(PROBE_PREFIX):])


def test_service_symbols_are_identical_under_both_backends():
    stock = _probe("stock")
    direct = _probe("direct")

    stock_payload = {key: value for key, value in stock.items() if key != "backend"}
    direct_payload = {key: value for key, value in direct.items() if key != "backend"}
    assert stock_payload == direct_payload


def test_service_owned_classes_are_stock_not_cppyy_under_direct():
    direct = _probe("direct")

    for descriptor in direct["classes"].values():
        assert descriptor["module"].startswith("rclpy."), descriptor
        assert descriptor["has_cpp_name"] is False
        assert all(member["present"] for member in descriptor["members"].values())


def test_service_introspection_state_values_and_cross_module_identity():
    direct = _probe("direct")
    members = direct["classes"]["client.ServiceIntrospectionState"]["members"]
    assert members["OFF"]["int_value"] == 0
    assert members["METADATA"]["int_value"] == 1
    assert members["CONTENTS"]["int_value"] == 2
    assert direct["introspection_state_identity"] == {
        "client_is_service": True,
        "service_is_service_introspection": True,
    }


def test_qos_profile_reexport_is_the_exact_rclpy_qos_object():
    direct = _probe("direct")
    assert direct["qos_reexport_identity"] == {"client": True, "service": True}


def test_lane_owned_reexports_are_recorded_but_not_claimed():
    direct = _probe("direct")
    # Clock/CallbackGroup/Future/Context resolve to their owning modules --
    # this lane records that fact and defers annotation to their own areas.
    assert direct["excluded_owner"]["client.Clock"] == "rclpy.clock"
    assert direct["excluded_owner"]["service.Clock"] == "rclpy.clock"
    assert direct["excluded_owner"]["client.Context"] == "rclpy.context"
    for key in ("client.CallbackGroup", "service.CallbackGroup", "client.Future"):
        assert direct["excluded_owner"][key].startswith("rclpy.")
