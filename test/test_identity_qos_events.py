"""QoS event type and module-alias identity proofs (cluster 2).

Evidence for the identity-proof lane's ``qos.stock_surface`` annotations over
``rclpy.event_handler`` / ``rclpy.qos_event``: QoS-owned public types remain
exact stock objects under the direct profile, the foreign ``CallbackGroup``
alias is the direct facade, and the deprecation-alias module (`qos_event`)
shares identity with `event_handler` for every public name.
"""

from __future__ import annotations

import json

from _run_helper import format_output, run_helper


PROBE_PREFIX = "RCLCPPYY_IDENTITY_QOS_EVENTS_PROBE "

# These names event_handler re-exports from other lanes' modules carry no
# qos.stock_surface annotation authority for this lane (allocation plan §1).
DIRECT_REEXPORT = "CallbackGroup"
STOCK_REEXPORTS = ("Waitable", "NumberOfEntities")


def _probe(backend):
    process = run_helper("_identity_qos_events_probe.py", "--backend", backend)
    assert process.returncode == 0, format_output(process)
    lines = [
        line for line in process.stdout.splitlines()
        if line.startswith(PROBE_PREFIX)
    ]
    assert len(lines) == 1, format_output(process)
    return json.loads(lines[0][len(PROBE_PREFIX):])


def test_qos_event_stock_types_are_identical_under_direct_profile():
    stock = _probe("stock")
    direct = _probe("direct")

    assert len(direct["types"]) == 20
    stock_payload = {key: value for key, value in stock.items() if key != "backend"}
    direct_payload = {key: value for key, value in direct.items() if key != "backend"}
    stock_payload["types"].pop(DIRECT_REEXPORT)
    direct_payload["types"].pop(DIRECT_REEXPORT)
    assert stock_payload == direct_payload


def test_qos_event_types_are_stock_not_cppyy_under_direct_profile():
    direct = _probe("direct")

    callback_group = direct["types"][DIRECT_REEXPORT]
    assert callback_group["module"] == "rclcppyy.direct_callback_groups"
    assert callback_group["qualname"] == "DirectCallbackGroup"

    for name, descriptor in direct["types"].items():
        if name == DIRECT_REEXPORT:
            continue
        assert descriptor["has_cpp_name"] is False, name
        assert descriptor["module"].startswith("rclpy."), name

    for name in STOCK_REEXPORTS:
        assert name in direct["types"]


def test_qos_event_module_aliases_hold_under_both_backends():
    for backend in ("stock", "direct"):
        payload = _probe(backend)
        assert payload["shared_names"], backend
        assert all(payload["module_aliases"].values()), backend
