"""Focused unit tests for the hand-built payload-tainted parameter
signatures (wave 7 slice 3). Real classes, no activation, no ROS node --
importing ``rcl_interfaces.msg``/``rclpy.node``/``rclpy.parameter`` needs no
running graph, and this file never calls ``enable_cpp_acceleration``, so the
stock classes stay pristine for the whole process.

Comparing straight against ``inspect.signature(getattr(rclpy.node.Node,
name))`` (rather than a transcribed literal string) makes this test
self-correcting against whatever the environment's stock rclpy actually
declares, matching this repo's "re-derive at implementation time, don't
trust a stale transcription" convention.
"""

import inspect

import pytest

from rcl_interfaces.msg import (
    ListParametersResult,
    ParameterDescriptor,
    ParameterValue,
    SetParametersResult,
)
from rclpy.node import Node
from rclpy.parameter import Parameter

from rclcppyy import _payload_signature


PRISTINE_CLASSES = {
    "parameter_descriptor": ParameterDescriptor,
    "parameter_value": ParameterValue,
    "set_parameters_result": SetParametersResult,
    "list_parameters_result": ListParametersResult,
    "parameter_class": Parameter,
}


def test_method_names_cover_exactly_the_documented_eleven():
    assert _payload_signature.METHOD_NAMES == (
        "declare_parameter",
        "declare_parameters",
        "describe_parameter",
        "describe_parameters",
        "list_parameters",
        "add_on_set_parameters_callback",
        "remove_on_set_parameters_callback",
        "set_parameters",
        "set_parameters_atomically",
        "set_descriptor",
        "get_parameters_by_prefix",
    )


def test_build_signatures_returns_exactly_the_documented_eleven():
    signatures = _payload_signature.build_signatures(**PRISTINE_CLASSES)
    assert set(signatures) == set(_payload_signature.METHOD_NAMES)
    assert len(signatures) == 11


@pytest.mark.parametrize("name", _payload_signature.METHOD_NAMES)
def test_each_built_signature_matches_stock_byte_for_byte(name):
    signatures = _payload_signature.build_signatures(**PRISTINE_CLASSES)
    stock_signature = inspect.signature(getattr(Node, name))
    assert signatures[name] == stock_signature
    assert str(signatures[name]) == str(stock_signature)


def test_get_parameters_by_prefix_return_hint_is_plain_typing_generics():
    """Unlike the other 10 methods, stock's declared return hint here is
    plain ``typing`` generics (``Sequence[bool]`` etc., no message type) --
    it is included in this table only because the method is missing from
    ``DirectNode`` entirely prior to this slice, not because its hint is
    payload-tainted."""
    stock_signature = inspect.signature(Node.get_parameters_by_prefix)
    assert "cppyy" not in str(stock_signature)


def test_install_signatures_assigns_signature_attribute_on_target_class():
    class TargetNode:
        def declare_parameter(self, name, value=None, descriptor=None,
                               ignore_override=False):
            pass

        def declare_parameters(self, namespace, parameters,
                                ignore_override=False):
            pass

        def describe_parameter(self, name):
            pass

        def describe_parameters(self, names):
            pass

        def list_parameters(self, prefixes, depth):
            pass

        def add_on_set_parameters_callback(self, callback):
            pass

        def remove_on_set_parameters_callback(self, callback):
            pass

        def set_parameters(self, parameter_list):
            pass

        def set_parameters_atomically(self, parameter_list):
            pass

        def set_descriptor(self, name, descriptor, alternative_value=None):
            pass

        def get_parameters_by_prefix(self, prefix):
            pass

    assert "__signature__" not in vars(TargetNode.declare_parameter)
    returned = _payload_signature.install_signatures(
        TargetNode, **PRISTINE_CLASSES)

    for name in _payload_signature.METHOD_NAMES:
        member = vars(TargetNode)[name]
        assert member.__signature__ is returned[name]
        assert inspect.signature(member) == returned[name]


def test_install_signatures_is_idempotent():
    class TargetNode:
        def declare_parameter(self, name, value=None, descriptor=None,
                               ignore_override=False):
            pass

        def declare_parameters(self, namespace, parameters,
                                ignore_override=False):
            pass

        def describe_parameter(self, name):
            pass

        def describe_parameters(self, names):
            pass

        def list_parameters(self, prefixes, depth):
            pass

        def add_on_set_parameters_callback(self, callback):
            pass

        def remove_on_set_parameters_callback(self, callback):
            pass

        def set_parameters(self, parameter_list):
            pass

        def set_parameters_atomically(self, parameter_list):
            pass

        def set_descriptor(self, name, descriptor, alternative_value=None):
            pass

        def get_parameters_by_prefix(self, prefix):
            pass

    first = _payload_signature.install_signatures(
        TargetNode, **PRISTINE_CLASSES)
    before = TargetNode.declare_parameter.__signature__
    second = _payload_signature.install_signatures(
        TargetNode, **PRISTINE_CLASSES)

    assert first["declare_parameter"] == second["declare_parameter"]
    assert TargetNode.declare_parameter.__signature__ == before
