"""Transactional ``std_srvs/SetBool`` bindings for ``direct_cpp``."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import sys
from typing import Any


_IMPLEMENTATION_MODULE = "std_srvs.srv._set_bool"
_PYTHONIZED: dict[Any, Any] = {}


@dataclass(frozen=True)
class DirectServiceBinding:
    service_type: type
    original_request_type: type
    original_response_type: type
    cpp_request_type: Any
    cpp_response_type: Any
    cpp_type_name: str


class DirectServiceInstallation:
    def __init__(self, replacements, binding, pythonizations=()):
        self._replacements = tuple(replacements)
        self._pythonizations = tuple(pythonizations)
        self.binding = binding
        self._restored = False

    def restore(self):
        if self._restored:
            return
        for owner, name, original, replacement in reversed(self._replacements):
            if getattr(owner, name, None) is replacement:
                setattr(owner, name, original)
        for cpp_type, original_init, direct_init in reversed(self._pythonizations):
            if cpp_type.__init__ is direct_init:
                cpp_type.__init__ = original_init
                _PYTHONIZED.pop(cpp_type, None)
        self._restored = True


def assert_early_imports() -> None:
    if _IMPLEMENTATION_MODULE in sys.modules:
        raise RuntimeError(
            "direct_cpp must be enabled before importing supported services: %s" %
            _IMPLEMENTATION_MODULE
        )


def _pythonize_constructor(cpp_type: Any, fields: tuple[str, ...]):
    if cpp_type in _PYTHONIZED:
        return None
    original_init = cpp_type.__init__
    allowed = frozenset(fields)

    def direct_init(self, *args, **kwargs):
        if args and kwargs:
            raise TypeError(
                "C++ service message constructors cannot mix positional and keyword values")
        if args:
            original_init(self, *args)
            return
        original_init(self)
        unknown = sorted(set(kwargs) - allowed)
        if unknown:
            raise TypeError(
                "unknown service message constructor fields: %s" % ", ".join(unknown))
        for name, value in kwargs.items():
            setattr(self, name, value)

    cpp_type.__init__ = direct_init
    _PYTHONIZED[cpp_type] = original_init
    return cpp_type, original_init, direct_init


def install() -> DirectServiceInstallation:
    """Install actual C++ SetBool request/response classes atomically."""
    import cppyy
    from rclcpp_kit.bringup_rclcpp import bringup_rclcpp
    from rclcpp_kit.native_service import _service_spec

    bringup_rclcpp()
    generated_module = importlib.import_module(_IMPLEMENTATION_MODULE)
    public_module = importlib.import_module("std_srvs.srv")
    service_type = generated_module.SetBool
    if public_module.SetBool is not service_type:
        raise RuntimeError("std_srvs.srv.SetBool changed before direct alias installation")
    original_request = generated_module.SetBool_Request
    original_response = generated_module.SetBool_Response
    if service_type.Request is not original_request or service_type.Response is not original_response:
        raise RuntimeError("SetBool request/response aliases changed before direct installation")
    if (
        public_module.SetBool_Request is not original_request
        or public_module.SetBool_Response is not original_response
    ):
        raise RuntimeError("public SetBool aliases changed before direct installation")

    cpp_type_name, _, _ = _service_spec(service_type)
    cpp_service = cppyy.gbl.std_srvs.srv.SetBool
    cpp_request = cpp_service.Request
    cpp_response = cpp_service.Response
    binding = DirectServiceBinding(
        service_type=service_type,
        original_request_type=original_request,
        original_response_type=original_response,
        cpp_request_type=cpp_request,
        cpp_response_type=cpp_response,
        cpp_type_name=cpp_type_name,
    )
    targets = (
        (generated_module, "SetBool_Request", original_request, cpp_request),
        (generated_module, "SetBool_Response", original_response, cpp_response),
        (public_module, "SetBool_Request", original_request, cpp_request),
        (public_module, "SetBool_Response", original_response, cpp_response),
        (service_type, "Request", original_request, cpp_request),
        (service_type, "Response", original_response, cpp_response),
    )
    replacements = []
    pythonizations = []
    try:
        for cpp_type, fields in (
            (cpp_request, ("data",)),
            (cpp_response, ("success", "message")),
        ):
            pythonization = _pythonize_constructor(cpp_type, fields)
            if pythonization is not None:
                pythonizations.append(pythonization)
        for owner, name, original, replacement in targets:
            if getattr(owner, name) is not original:
                raise RuntimeError("SetBool aliases changed during direct installation")
            setattr(owner, name, replacement)
            replacements.append((owner, name, original, replacement))
    except Exception:
        DirectServiceInstallation(replacements, binding, pythonizations).restore()
        raise
    return DirectServiceInstallation(replacements, binding, pythonizations)


def resolve_supported_type(service_type: Any, installation: DirectServiceInstallation):
    binding = installation.binding
    if service_type is not binding.service_type:
        raise TypeError("direct_cpp currently supports only std_srvs.srv.SetBool")
    if (
        service_type.Request is not binding.cpp_request_type
        or service_type.Response is not binding.cpp_response_type
    ):
        raise TypeError("direct_cpp SetBool request/response aliases are not active")
    return binding


__all__ = [
    "DirectServiceBinding",
    "DirectServiceInstallation",
    "assert_early_imports",
    "install",
    "resolve_supported_type",
]
