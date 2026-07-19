"""Native parameter facade support for the ``direct_cpp`` profile.

Parameter payloads remain owning ``rclcpp::Parameter`` values.  Python values
are materialized only by the public ``Parameter.value`` property.
"""

from __future__ import annotations

from typing import Any

import cppyy

from rclcpp_kit import native_parameters


_FACADE = None


def _uint8(value: Any) -> int:
    if isinstance(value, str):
        return ord(value)
    if isinstance(value, bytes):
        return value[0]
    return int(value)


def _value_from_parameter_message(value: Any, type_code: int) -> Any:
    if type_code == native_parameters.PARAMETER_NOT_SET:
        return None
    if type_code == native_parameters.PARAMETER_BOOL:
        return bool(value.bool_value)
    if type_code == native_parameters.PARAMETER_INTEGER:
        return int(value.integer_value)
    if type_code == native_parameters.PARAMETER_DOUBLE:
        return float(value.double_value)
    if type_code == native_parameters.PARAMETER_STRING:
        return str(value.string_value)
    if type_code == native_parameters.PARAMETER_BYTE_ARRAY:
        return [bytes((_uint8(item),)) for item in value.byte_array_value]
    if type_code == native_parameters.PARAMETER_BOOL_ARRAY:
        return [bool(item) for item in value.bool_array_value]
    if type_code == native_parameters.PARAMETER_INTEGER_ARRAY:
        return [int(item) for item in value.integer_array_value]
    if type_code == native_parameters.PARAMETER_DOUBLE_ARRAY:
        return [float(item) for item in value.double_array_value]
    if type_code == native_parameters.PARAMETER_STRING_ARRAY:
        return [str(item) for item in value.string_array_value]
    raise ValueError("unknown parameter type code %d" % type_code)


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
            if isinstance(param_msg, cpp_message_type):
                native = native_parameters.NativeParameter(
                    cppyy.gbl.rclcpp.Parameter.from_parameter_msg(param_msg))
                return cls._from_native(native)
            type_code = _uint8(param_msg.value.type)
            return cls(
                str(param_msg.name),
                cls.Type(type_code),
                _value_from_parameter_message(param_msg.value, type_code),
            )

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
    """Adapt a parameter control descriptor without touching app messages."""
    descriptor_type = cppyy.gbl.rcl_interfaces.msg.ParameterDescriptor
    if descriptor is None:
        result = descriptor_type()
    elif isinstance(descriptor, descriptor_type):
        result = descriptor_type(descriptor)
    else:
        required = (
            "description",
            "additional_constraints",
            "read_only",
            "dynamic_typing",
            "integer_range",
            "floating_point_range",
        )
        if not all(hasattr(descriptor, field) for field in required):
            raise TypeError("descriptor must be a ParameterDescriptor")
        result = descriptor_type()
        result.description = str(descriptor.description)
        result.additional_constraints = str(descriptor.additional_constraints)
        result.read_only = bool(descriptor.read_only)
        result.dynamic_typing = bool(descriptor.dynamic_typing)
        for source in descriptor.integer_range:
            target = cppyy.gbl.rcl_interfaces.msg.IntegerRange()
            target.from_value = int(source.from_value)
            target.to_value = int(source.to_value)
            target.step = int(source.step)
            result.integer_range.push_back(target)
        for source in descriptor.floating_point_range:
            target = cppyy.gbl.rcl_interfaces.msg.FloatingPointRange()
            target.from_value = float(source.from_value)
            target.to_value = float(source.to_value)
            target.step = float(source.step)
            result.floating_point_range.push_back(target)
    result.name = str(name)
    result.type = int(type_code)
    return result


def result_to_cpp(result):
    result_type = cppyy.gbl.rcl_interfaces.msg.SetParametersResult
    if isinstance(result, result_type):
        return result_type(result)
    if not hasattr(result, "successful") or not hasattr(result, "reason"):
        raise TypeError("parameter callback must return SetParametersResult")
    return native_parameters.make_set_parameters_result(
        bool(result.successful), str(result.reason))


__all__ = [
    "descriptor_to_cpp",
    "native_parameter",
    "parameter_class",
    "prepare",
    "result_to_cpp",
    "wrap_native",
]
