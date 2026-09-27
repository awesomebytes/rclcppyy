#!/usr/bin/env python3
"""Fresh-process stock/direct differential for the QoS policy value surface.

Cluster 1 (allocation plan §1 / identity-proof plan §1): proves that under the
direct profile, ``rclpy.qos`` and ``rclpy.qos_overriding_options`` keep their
stock policy enums, presets, and exceptions unchanged -- identical type
identity, member values, and value semantics, not merely matching names and
signatures.
"""

from __future__ import annotations

import argparse
import json


PROBE_PREFIX = "RCLCPPYY_IDENTITY_QOS_PROBE "

# The policy/kind enums native to rclpy.qos. QoSPolicyEnum is the shared
# IntEnum base (no members of its own); QoSCompatibility is a pybind11 enum,
# not a Python enum.Enum subclass, so member retrieval and supported
# operations differ slightly -- handled generically below.
ENUM_NAMES = (
    "HistoryPolicy",
    "ReliabilityPolicy",
    "DurabilityPolicy",
    "LivelinessPolicy",
    "QoSPolicyKind",
    "QoSPolicyEnum",
    "QoSCompatibility",
)

# Q-prefixed intra-module aliases for the four data-policy enums.
INTRA_MODULE_ALIASES = {
    "HistoryPolicy": "QoSHistoryPolicy",
    "ReliabilityPolicy": "QoSReliabilityPolicy",
    "DurabilityPolicy": "QoSDurabilityPolicy",
    "LivelinessPolicy": "QoSLivelinessPolicy",
}

# Names cross-aliased from rclpy.qos into rclpy.qos_overriding_options.
CROSS_MODULE_NAMES = (
    "QoSHistoryPolicy",
    "QoSReliabilityPolicy",
    "QoSDurabilityPolicy",
    "QoSLivelinessPolicy",
    "QoSPolicyKind",
)

# The 9 module-level profile singletons, one per QoSPresetProfiles member name.
# Note: QoSProfile defines value equality, and qos_profile_default happens to
# equal qos_profile_services_default (likewise parameters/parameter_events),
# so stock Python enum aliasing makes SERVICES_DEFAULT literally *be* the
# DEFAULT member (and PARAMETER_EVENTS literally *be* PARAMETERS) -- not a
# same-named 1:1 mapping. The probe discovers the actual resolved singleton
# rather than assume the same-named one, so the identity claim stays honest.
SINGLETON_NAMES = (
    "qos_profile_unknown",
    "qos_profile_default",
    "qos_profile_system_default",
    "qos_profile_sensor_data",
    "qos_profile_parameters",
    "qos_profile_action_status_default",
    "qos_profile_best_available",
    "qos_profile_parameter_events",
    "qos_profile_services_default",
)

EXCEPTIONS = {
    "qos": ("InvalidQoSProfileException",),
    "qos_overriding_options": ("InvalidQosOverridesError",),
}


parser = argparse.ArgumentParser()
parser.add_argument("--backend", choices=("stock", "direct"), required=True)
args = parser.parse_args()

if args.backend == "direct":
    import rclcppyy

    rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy.qos as qos  # noqa: E402
import rclpy.qos_overriding_options as qos_overriding_options  # noqa: E402


def _type_descriptor(value_type):
    return {
        "module": value_type.__module__,
        "qualname": value_type.__qualname__,
        "mro": [klass.__name__ for klass in value_type.__mro__],
        "has_cpp_name": hasattr(value_type, "__cpp_name__"),
    }


def _members(enum_type):
    try:
        return list(enum_type)
    except TypeError:
        return list(enum_type.__members__.values())


def _ordering(left, right):
    try:
        return bool(left < right)
    except TypeError as exception:
        return ["unsupported", type(exception).__name__]


def _enum_block(enum_type):
    members = _members(enum_type)
    return {
        "type": _type_descriptor(enum_type),
        "members": [
            {
                "name": member.name,
                "value": int(member),
                "equals_int": member == int(member),
                "hash_matches_int": hash(member) == hash(int(member)),
                "repr": repr(member),
                "str": str(member),
            }
            for member in members
        ],
        "ordering": [
            [left.name, right.name, _ordering(left, right)]
            for left, right in zip(members, members[1:])
        ],
    }


def _resolved_singleton(value):
    for singleton_name in SINGLETON_NAMES:
        if value is getattr(qos, singleton_name):
            return singleton_name
    return None


def _preset_block():
    profile_type = qos.QoSProfile
    members = {}
    for name, member in qos.QoSPresetProfiles.__members__.items():
        members[name] = {
            "profile_type": _type_descriptor(type(member.value)),
            "canonical_member_name": member.name,
            "resolved_singleton": _resolved_singleton(member.value),
        }
    return {
        "type": _type_descriptor(qos.QoSPresetProfiles),
        "profile_type": _type_descriptor(profile_type),
        "members": members,
        "singleton_types": {
            name: _type_descriptor(type(getattr(qos, name)))
            for name in SINGLETON_NAMES
        },
    }


enums = {name: _enum_block(getattr(qos, name)) for name in ENUM_NAMES}

intra_module_aliases = {
    name: getattr(qos, name) is getattr(qos, alias)
    for name, alias in INTRA_MODULE_ALIASES.items()
}
cross_module_aliases = {
    name: getattr(qos_overriding_options, name) is getattr(qos, name)
    for name in CROSS_MODULE_NAMES
}
exceptions = {
    f"{module_name}.{name}": _type_descriptor(
        getattr(qos if module_name == "qos" else qos_overriding_options, name)
    )
    for module_name, names in EXCEPTIONS.items()
    for name in names
}

payload = {
    "backend": args.backend,
    "enums": enums,
    "preset": _preset_block(),
    "intra_module_aliases": intra_module_aliases,
    "cross_module_aliases": cross_module_aliases,
    "exceptions": exceptions,
}
print(PROBE_PREFIX + json.dumps(payload, sort_keys=True), flush=True)
