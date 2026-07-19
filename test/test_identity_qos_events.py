"""Stock QoS event type and module-alias identity proofs (cluster 2).

Evidence for the identity-proof lane's ``qos.stock_surface`` annotations over
``rclpy.event_handler`` / ``rclpy.qos_event``: every public type is the exact
stock type object under the direct profile, and the deprecation-alias module
(`qos_event`) shares identity with `event_handler` for every public name.
"""

from __future__ import annotations

import json

from _run_helper import format_output, run_helper


PROBE_PREFIX = "RCLCPPYY_IDENTITY_QOS_EVENTS_PROBE "

# The three names event_handler re-exports from other lanes' modules. Proving
# their identity here is harmless, but they carry no qos.stock_surface
# annotation authority for this lane (allocation plan §1 owns them).
FOREIGN_REEXPORTS = ("CallbackGroup", "Waitable", "NumberOfEntities")


def _probe(backend):
    process = run_helper("_identity_qos_events_probe.py", "--backend", backend)
    assert process.returncode == 0, format_output(process)
    lines = [
        line for line in process.stdout.splitlines()
        if line.startswith(PROBE_PREFIX)
    ]
    assert len(lines) == 1, format_output(process)
    return json.loads(lines[0][len(PROBE_PREFIX):])


def test_qos_event_types_are_identical_stock_objects_under_direct_profile():
    stock = _probe("stock")
    direct = _probe("direct")

    stock_payload = {key: value for key, value in stock.items() if key != "backend"}
    direct_payload = {key: value for key, value in direct.items() if key != "backend"}
    assert stock_payload == direct_payload
    assert len(direct["types"]) == 20


def test_qos_event_types_are_stock_not_cppyy_under_direct_profile():
    direct = _probe("direct")

    for name, descriptor in direct["types"].items():
        assert descriptor["has_cpp_name"] is False, name
        assert descriptor["module"].startswith("rclpy."), name

    for name in FOREIGN_REEXPORTS:
        assert name in direct["types"]


def test_qos_event_module_aliases_hold_under_both_backends():
    for backend in ("stock", "direct"):
        payload = _probe(backend)
        assert payload["shared_names"], backend
        assert all(payload["module_aliases"].values()), backend
