#!/usr/bin/env python3
"""Activation-level proof for the direct-facade signature mirror.

A mirrored facade must present stock's exact Signature after activation. A
facade the gate must leave alone -- a payload-tainted stock counterpart, or a
genuine parameter-set divergence -- must keep rendering its own, still-
divergent signature. This pins the partition against future drift in the
fast lane, ahead of a full ledger regen.
"""

import inspect

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

from rclcppyy.direct_cpp import _PATCHES  # noqa: E402

_BY_NAME = {
    name: (original, replacement) for _module, name, original, replacement in _PATCHES
}


def _member_signature(cls, name):
    return inspect.signature(inspect.getattr_static(cls, name))


def assert_mirrored(class_name, member_name):
    stock, direct = _BY_NAME[class_name]
    stock_signature = _member_signature(stock, member_name)
    direct_signature = _member_signature(direct, member_name)
    assert direct_signature == stock_signature, (
        "%s.%s: direct=%s stock=%s"
        % (class_name, member_name, direct_signature, stock_signature)
    )


def assert_still_divergent(class_name, member_name):
    stock, direct = _BY_NAME[class_name]
    stock_signature = _member_signature(stock, member_name)
    direct_signature = _member_signature(direct, member_name)
    assert direct_signature != stock_signature, (
        "%s.%s: expected to stay divergent, both render %s"
        % (class_name, member_name, direct_signature)
    )


# Representative clean-mirrorable facades (signature-normalization plan's
# population analysis: annotation-absence and stringified-annotation rows).
assert_mirrored("Executor", "create_task")
assert_mirrored("CallbackGroup", "can_execute")
assert_mirrored("Node", "get_name")

# Representative facades the gate must leave alone: a payload-tainted stock
# counterpart (declare_parameter forwards a C++ descriptor payload), and a
# genuine parameter-set divergence (Executor.__init__ gains num_threads).
assert_still_divergent("Node", "declare_parameter")
assert_still_divergent("Executor", "__init__")

print("DIRECT_CPP_SIGNATURE_MIRROR_OK")
