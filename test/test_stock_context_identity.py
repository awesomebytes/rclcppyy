"""Stock context/signals-surface identity + value-semantics proofs.

Evidence for the prove-authority lane's ``context.stock_initialization``
batch-(b) annotations (PLAN-prove-authority §2c, uncommitted): the direct
profile keeps ``Context``/``ContextHandle``/``DestroyableType`` and the
signal-handler surface as the exact stock objects. ``GuardCondition``'s
relationship to ``SignalHandlerGuardCondition`` and the executor lane's
ownership are recorded as facts here, not annotated -- Lane 2 coordination
decides which of these two classes' rows this lane may claim (PLAN §3, §6).
"""

from __future__ import annotations

import json

from _run_helper import format_output, run_helper


PROBE_PREFIX = "RCLCPPYY_STOCK_CONTEXT_IDENTITY_PROBE "


def _probe(backend):
    process = run_helper("_stock_context_identity_probe.py", "--backend", backend)
    assert process.returncode == 0, format_output(process)
    lines = [
        line for line in process.stdout.splitlines()
        if line.startswith(PROBE_PREFIX)
    ]
    assert len(lines) == 1, format_output(process)
    return json.loads(lines[0][len(PROBE_PREFIX):])


def test_context_symbols_are_identical_under_both_backends():
    stock = _probe("stock")
    direct = _probe("direct")

    stock_payload = {key: value for key, value in stock.items() if key != "backend"}
    direct_payload = {key: value for key, value in direct.items() if key != "backend"}
    assert stock_payload == direct_payload


def test_context_classes_and_functions_are_stock_not_cppyy_under_direct():
    direct = _probe("direct")

    for descriptor in direct["classes"].values():
        assert descriptor["module"].startswith("rclpy."), descriptor
        assert descriptor["has_cpp_name"] is False
        assert all(member["present"] for member in descriptor["members"].values())

    for descriptor in direct["functions"].values():
        assert descriptor["module"].startswith("rclpy.")


def test_signal_handler_options_are_the_expected_flag_values():
    direct = _probe("direct")
    members = direct["classes"]["signals.SignalHandlerOptions"]["members"]
    assert members["NO"]["int_value"] == 0
    assert members["SIGINT"]["int_value"] == 1
    assert members["SIGTERM"]["int_value"] == 2
    assert members["ALL"]["int_value"] == 3


def test_destroyable_type_is_the_same_class_across_its_two_module_aliases():
    direct = _probe("direct")
    assert direct["cross_module_identity"] == {"DestroyableType": True}


def test_guard_condition_relationship_is_recorded_for_lane_2_coordination():
    direct = _probe("direct")
    relationship = direct["guard_condition_relationship"]
    assert isinstance(relationship["signal_handler_is_guard_condition_subclass"], bool)
    assert isinstance(relationship["signal_handler_is_guard_condition_itself"], bool)
