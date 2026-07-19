"""Native parameter facade support for the ``direct_cpp`` profile.

Parameter payloads remain owning ``rclcpp::Parameter`` values.  Python values
are materialized only by the public ``Parameter.value`` property.
"""

from __future__ import annotations

import cppyy

from rclcpp_kit import native_parameters


_FACADE = None
CONTROL_MESSAGE_INTERFACES = (
    "rcl_interfaces/msg/ListParametersResult",
    "rcl_interfaces/msg/Parameter",
    "rcl_interfaces/msg/ParameterDescriptor",
    "rcl_interfaces/msg/SetParametersResult",
)


def prepare(original_parameter_class):
    """Create the facade while retaining the stock nested ``Type`` enum."""
    global _FACADE
    if _FACADE is not None:
        return _FACADE

    parameter_type = original_parameter_class.Type

    class DirectParameter:
        __slots__ = ("_rclcppyy_native_parameter",)

        Type = parameter_type

        def __init__(self, name, type_=None, value=None):
            if type_ is None:
                type_ = self.Type.from_parameter_value(value)
            if not isinstance(type_, self.Type):
                raise TypeError(
                    "type must be an instance of '{}'".format(repr(self.Type)))
            if not type_.check(value):
                raise ValueError(
                    "Type '{}' and value '{}' do not agree".format(type_, value))
            self._rclcppyy_native_parameter = native_parameters.make_parameter(
                name, int(type_.value), value)

        @classmethod
        def _from_native(cls, value):
            if not isinstance(value, native_parameters.NativeParameter):
                raise TypeError("expected an owning NativeParameter")
            result = cls.__new__(cls)
            result._rclcppyy_native_parameter = value
            return result

        @classmethod
        def from_parameter_msg(cls, param_msg):
            cpp_message_type = cppyy.gbl.rcl_interfaces.msg.Parameter
            if not isinstance(param_msg, cpp_message_type):
                raise TypeError(
                    "param_msg must be an actual direct_cpp C++ Parameter message")
            native = native_parameters.NativeParameter(
                cppyy.gbl.rclcpp.Parameter.from_parameter_msg(param_msg))
            return cls._from_native(native)

        @property
        def name(self):
            return self._rclcppyy_native_parameter.name

        @property
        def type_(self):
            return self.Type(self._rclcppyy_native_parameter.type_code)

        @property
        def value(self):
            return self._rclcppyy_native_parameter.value_snapshot()

        def get_parameter_value(self):
            return self._rclcppyy_native_parameter.native.get_value_message()

        def to_parameter_msg(self):
            return self._rclcppyy_native_parameter.native.to_parameter_msg()

    DirectParameter.__name__ = "Parameter"
    DirectParameter.__qualname__ = "Parameter"
    DirectParameter.__module__ = "rclpy.parameter"
    _FACADE = DirectParameter
    return DirectParameter


def parameter_class():
    if _FACADE is None:
        raise RuntimeError("direct_cpp parameter facade is not prepared")
    return _FACADE


def native_parameter(value):
    if not isinstance(value, parameter_class()):
        raise TypeError("parameter list must contain only rclpy.parameter.Parameter")
    return value._rclcppyy_native_parameter


def wrap_native(value):
    return parameter_class()._from_native(value)


def descriptor_to_cpp(descriptor, *, name="", type_code=0):
    """Copy an exact C++ descriptor before applying declaration metadata."""
    descriptor_type = cppyy.gbl.rcl_interfaces.msg.ParameterDescriptor
    if descriptor is None:
        result = descriptor_type()
    elif isinstance(descriptor, descriptor_type):
        result = descriptor_type(descriptor)
    else:
        raise TypeError(
            "descriptor must be an actual direct_cpp C++ ParameterDescriptor")
    result.name = str(name)
    result.type = int(type_code)
    return result


def result_to_cpp(result):
    result_type = cppyy.gbl.rcl_interfaces.msg.SetParametersResult
    if not isinstance(result, result_type):
        raise TypeError(
            "parameter callback must return an actual direct_cpp C++ "
            "SetParametersResult")
    return result_type(result)


__all__ = [
    "CONTROL_MESSAGE_INTERFACES",
    "descriptor_to_cpp",
    "native_parameter",
    "parameter_class",
    "prepare",
    "result_to_cpp",
    "wrap_native",
]
