"""Stock time-enum identity + value-semantics proofs.

Evidence for the prove-authority lane's ``time.stock_surface`` batch-(b)
annotations (PLAN-prove-authority §2c/§6, uncommitted): the direct profile
keeps ``ClockType`` (at all 4 public module aliases) and ``ClockChange`` as
the exact stock IntEnum objects, not a re-implementation with matching
names. This is the row boundary Lane 1 (clock-rate) leaves to this lane --
pure enum identity, not the behavioral clock/rate/timer rows. Everything
else in the time area was already annotated in Wave 1 and stays untouched.
"""

from __future__ import annotations

import json

from _run_helper import format_output, run_helper


PROBE_PREFIX = "RCLCPPYY_STOCK_TIME_ENUM_IDENTITY_PROBE "

CLOCK_TYPE_MODULES = ("clock", "clock_type", "time", "time_source")


def _probe(backend):
    process = run_helper("_stock_time_enum_identity_probe.py", "--backend", backend)
    assert process.returncode == 0, format_output(process)
    lines = [
        line for line in process.stdout.splitlines()
        if line.startswith(PROBE_PREFIX)
    ]
    assert len(lines) == 1, format_output(process)
    return json.loads(lines[0][len(PROBE_PREFIX):])


def test_time_enum_symbols_are_identical_under_both_backends():
    stock = _probe("stock")
    direct = _probe("direct")

    stock_payload = {key: value for key, value in stock.items() if key != "backend"}
    direct_payload = {key: value for key, value in direct.items() if key != "backend"}
    assert stock_payload == direct_payload


def test_clock_type_and_clock_change_are_stock_not_cppyy_under_direct():
    direct = _probe("direct")

    for descriptor in direct["classes"].values():
        assert descriptor["module"].startswith("rclpy."), descriptor
        assert descriptor["has_cpp_name"] is False
        assert "IntEnum" in descriptor["mro"] or "IntFlag" in descriptor["mro"]


def test_clock_type_enum_values_match_across_all_four_module_aliases():
    direct = _probe("direct")
    expected = {"UNINITIALIZED": 0, "ROS_TIME": 1, "SYSTEM_TIME": 2, "STEADY_TIME": 3}
    for module_name in CLOCK_TYPE_MODULES:
        members = direct["classes"]["%s.ClockType" % module_name]["members"]
        for name, value in expected.items():
            assert members[name]["int_value"] == value, (module_name, name)


def test_clock_change_enum_values_are_correct():
    direct = _probe("direct")
    members = direct["classes"]["clock.ClockChange"]["members"]
    assert members["ROS_TIME_NO_CHANGE"]["int_value"] == 1
    assert members["ROS_TIME_ACTIVATED"]["int_value"] == 2
    assert members["ROS_TIME_DEACTIVATED"]["int_value"] == 3
    assert members["SYSTEM_TIME_NO_CHANGE"]["int_value"] == 4


def test_clock_type_is_the_same_class_object_at_every_module_alias():
    direct = _probe("direct")
    assert direct["clock_type_identity"] == dict.fromkeys(CLOCK_TYPE_MODULES, True)


def test_time_source_foreign_reexports_are_recorded_but_not_claimed():
    direct = _probe("direct")
    # Parameter/QoSProfile/ReliabilityPolicy in rclpy.time_source belong to
    # the parameter/qos areas -- recorded here, never annotated by this lane.
    assert direct["excluded_owner"]["Parameter"] == "rclpy.parameter"
    assert direct["excluded_owner"]["QoSProfile"] == "rclpy.qos"
    assert direct["excluded_owner"]["ReliabilityPolicy"] == "rclpy.qos"
