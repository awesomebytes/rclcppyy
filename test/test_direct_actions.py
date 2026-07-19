"""Unit coverage for explicit direct-C++ action registration."""

import sys
import types

import pytest

from rclcppyy import direct_actions


def test_normalize_action_interfaces_is_canonical_and_deduplicated():
    assert direct_actions.normalize_interfaces(
        "tf2_msgs/action/LookupTransform") == (
            "tf2_msgs/action/LookupTransform",
        )
    assert direct_actions.normalize_interfaces((
        "tf2_msgs/action/LookupTransform",
        "tf2_msgs/action/LookupTransform",
    )) == ("tf2_msgs/action/LookupTransform",)


@pytest.mark.parametrize(
    "interfaces",
    (
        ("tf2_msgs/msg/LookupTransform",),
        ("tf2_msgs/action/lookupTransform",),
        ("tf2_msgs/srv/LookupTransform",),
        ("tf2_msgs/LookupTransform",),
    ),
)
def test_normalize_action_interfaces_rejects_noncanonical_names(interfaces):
    with pytest.raises(ValueError, match="package/action/Action"):
        direct_actions.normalize_interfaces(interfaces)


def test_registered_registry_accepts_all_direct_interface_kinds():
    assert direct_actions.normalize_registered_interfaces((
        "tf2_msgs/action/LookupTransform",
        "std_srvs/srv/Trigger",
        "std_msgs/msg/Header",
        "tf2_msgs/action/LookupTransform",
    )) == (
        "std_msgs/msg/Header",
        "std_srvs/srv/Trigger",
        "tf2_msgs/action/LookupTransform",
    )
    with pytest.raises(ValueError, match="package/action/Action"):
        direct_actions.normalize_registered_interfaces(("unknown/thing/Type",))


def test_action_dependency_scan_can_exclude_synthetic_payloads():
    class Envelope:
        @staticmethod
        def get_fields_and_field_types():
            return {
                "goal_id": "unique_identifier_msgs/UUID",
                "goal": "example_actions/Accumulate_Goal",
                "header": "std_msgs/msg/Header",
            }

    assert direct_actions._dependencies(
        Envelope, ("example_actions/msg/Accumulate_Goal",)) == (
            "std_msgs/msg/Header",
            "unique_identifier_msgs/msg/UUID",
        )


def test_stale_generated_action_module_is_rejected(monkeypatch):
    name = "example_actions.action._accumulate"
    monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    with pytest.raises(RuntimeError, match=name):
        direct_actions.assert_early_imports()


def test_installation_preserves_default_lookup_transform_accessor():
    class Action:
        pass

    class CppTypes:
        cpp_name = "unused"

    bindings = tuple(
        direct_actions.DirectActionBinding(
            interface=interface,
            action_type=Action,
            cpp_types=CppTypes(),
            original_goal_type=Action,
            original_feedback_type=Action,
            original_result_type=Action,
            header="unused.hpp",
        )
        for interface in (
            "example_actions/action/Accumulate",
            "tf2_msgs/action/LookupTransform",
        )
    )
    installation = direct_actions.DirectActionInstallation((), bindings)
    assert installation.bindings == bindings
    assert installation.binding.interface == "tf2_msgs/action/LookupTransform"
