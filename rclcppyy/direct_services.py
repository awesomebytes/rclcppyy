"""Transactional generated-C++ service bindings for ``direct_cpp``."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import re
import sys
from typing import Any


DEFAULT_INTERFACES = ("std_srvs/srv/SetBool",)
_INTERFACE_NAME = re.compile(
    r"^([A-Za-z][A-Za-z0-9_]*)/srv/([A-Z][A-Za-z0-9_]*)$")
_FIELD_INTERFACE = re.compile(
    r"(?<![A-Za-z0-9_])([A-Za-z][A-Za-z0-9_]*)/(?:msg/)?"
    r"([A-Z][A-Za-z0-9_]*)")
_IMPLEMENTATION_MODULE = re.compile(
    r"^[A-Za-z][A-Za-z0-9_]*\.srv\._[A-Za-z0-9_]+$")
_PYTHONIZED: dict[Any, Any] = {}


@dataclass(frozen=True)
class DirectServiceBinding:
    interface: str
    service_type: type
    original_request_type: type
    original_response_type: type
    cpp_request_type: Any
    cpp_response_type: Any
    cpp_type_name: str
    header: str
    request_fields: tuple[str, ...]
    response_fields: tuple[str, ...]


@dataclass(frozen=True)
class DirectServicePlan:
    bindings: tuple[DirectServiceBinding, ...]
    targets: tuple[tuple[Any, str, Any, Any], ...]
    message_dependencies: tuple[str, ...]


class DirectServiceInstallation:
    def __init__(self, replacements, bindings, pythonizations=()):
        self._replacements = tuple(replacements)
        self._pythonizations = tuple(pythonizations)
        self.bindings = tuple(bindings)
        self.binding = next(
            binding for binding in self.bindings
            if binding.interface == DEFAULT_INTERFACES[0]
        )
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


def normalize_interfaces(interfaces=()) -> tuple[str, ...]:
    values = (interfaces,) if isinstance(interfaces, str) else interfaces
    try:
        selected = tuple(values)
    except TypeError as exc:
        raise TypeError("interfaces must be an iterable of canonical names") from exc
    if any(not isinstance(value, str) or not value for value in selected):
        raise TypeError("interfaces must contain only non-empty strings")
    invalid = sorted(
        value for value in selected if _INTERFACE_NAME.fullmatch(value) is None)
    if invalid:
        raise ValueError(
            "direct_cpp service interfaces must use package/srv/Service names: %s" %
            ", ".join(invalid)
        )
    return tuple(sorted(set(selected)))


def normalize_registered_interfaces(interfaces=()) -> tuple[str, ...]:
    """Validate the public direct interface registry across supported kinds."""
    values = (interfaces,) if isinstance(interfaces, str) else interfaces
    try:
        selected = tuple(values)
    except TypeError as exc:
        raise TypeError("interfaces must be an iterable of canonical names") from exc
    if any(not isinstance(value, str) or not value for value in selected):
        raise TypeError("interfaces must contain only non-empty strings")

    messages = tuple(value for value in selected if "/msg/" in value)
    services = tuple(value for value in selected if "/srv/" in value)
    unknown = sorted(set(selected) - set(messages) - set(services))
    if unknown:
        raise ValueError(
            "direct_cpp interfaces must use package/msg/Message or "
            "package/srv/Service names: %s" % ", ".join(unknown)
        )
    from rclcppyy.direct_messages import normalize_interfaces as normalize_messages

    normalized = normalize_messages(messages) + normalize_interfaces(services)
    return tuple(sorted(set(normalized)))


def assert_early_imports() -> None:
    stale = sorted(
        name for name in sys.modules if _IMPLEMENTATION_MODULE.fullmatch(name))
    if stale:
        raise RuntimeError(
            "direct_cpp must be enabled before importing generated services: %s" %
            ", ".join(stale)
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


def _dependencies(message_type: type) -> tuple[str, ...]:
    dependencies = set()
    for field_type in message_type.get_fields_and_field_types().values():
        for package, name in _FIELD_INTERFACE.findall(field_type):
            dependencies.add("%s/msg/%s" % (package, name))
    return tuple(sorted(dependencies))


def _resolve_cpp_name(cpp_type_name: str) -> Any:
    import cppyy

    result = cppyy.gbl
    for component in cpp_type_name.split("::"):
        result = getattr(result, component)
    return result


def prepare(interfaces=()) -> DirectServicePlan:
    """Resolve and validate all services without changing any Python aliases."""
    from rclcpp_kit.bringup_rclcpp import bringup_rclcpp
    from rclcpp_kit.native_service import _service_spec
    from rosidl_pycommon import convert_camel_case_to_lower_case_underscore
    from rosidl_runtime_py.utilities import get_service

    normalized = normalize_interfaces(interfaces)
    bringup_rclcpp()
    bindings = []
    targets = []
    dependencies = set()
    seen = set()
    for interface in DEFAULT_INTERFACES + normalized:
        if interface in seen:
            continue
        seen.add(interface)
        match = _INTERFACE_NAME.fullmatch(interface)
        package, name = match.groups()
        try:
            service_type = get_service(interface)
        except (ImportError, AttributeError, ValueError) as exc:
            raise TypeError(
                "direct_cpp service interface is not installed: %s" % interface
            ) from exc
        expected_module = "%s.srv._%s" % (
            package, convert_camel_case_to_lower_case_underscore(name))
        if service_type.__module__ != expected_module or service_type.__name__ != name:
            raise TypeError(
                "direct_cpp service did not resolve to exact canonical interface %s" %
                interface)
        generated_module = importlib.import_module(expected_module)
        public_module = importlib.import_module("%s.srv" % package)
        if getattr(generated_module, name) is not service_type:
            raise RuntimeError("%s changed before direct alias installation" % interface)
        if getattr(public_module, name) is not service_type:
            raise RuntimeError("%s public alias changed before direct installation" % interface)

        request_name = "%s_Request" % name
        response_name = "%s_Response" % name
        original_request = getattr(generated_module, request_name)
        original_response = getattr(generated_module, response_name)
        if (
            service_type.Request is not original_request
            or service_type.Response is not original_response
            or getattr(public_module, request_name) is not original_request
            or getattr(public_module, response_name) is not original_response
        ):
            raise RuntimeError(
                "%s request/response aliases changed before direct installation" %
                interface)

        cpp_type_name, header, resolved_package = _service_spec(service_type)
        if (
            cpp_type_name != "%s::srv::%s" % (package, name)
            or resolved_package != package
        ):
            raise TypeError(
                "direct_cpp service C++ type does not match canonical interface %s" %
                interface)
        cpp_service = _resolve_cpp_name(cpp_type_name)
        binding = DirectServiceBinding(
            interface=interface,
            service_type=service_type,
            original_request_type=original_request,
            original_response_type=original_response,
            cpp_request_type=cpp_service.Request,
            cpp_response_type=cpp_service.Response,
            cpp_type_name=cpp_type_name,
            header=header,
            request_fields=tuple(original_request.get_fields_and_field_types()),
            response_fields=tuple(original_response.get_fields_and_field_types()),
        )
        bindings.append(binding)
        targets.extend((
            (generated_module, request_name, original_request, binding.cpp_request_type),
            (generated_module, response_name, original_response, binding.cpp_response_type),
            (public_module, request_name, original_request, binding.cpp_request_type),
            (public_module, response_name, original_response, binding.cpp_response_type),
            (service_type, "Request", original_request, binding.cpp_request_type),
            (service_type, "Response", original_response, binding.cpp_response_type),
        ))
        dependencies.update(_dependencies(original_request))
        dependencies.update(_dependencies(original_response))
    return DirectServicePlan(
        bindings=tuple(bindings),
        targets=tuple(targets),
        message_dependencies=tuple(sorted(dependencies)),
    )


def install(interfaces=(), *, plan=None) -> DirectServiceInstallation:
    """Atomically install every prepared generated C++ service alias."""
    if plan is not None and normalize_interfaces(interfaces):
        raise ValueError("install accepts either interfaces or a prepared plan")
    selected_plan = prepare(interfaces) if plan is None else plan
    if not isinstance(selected_plan, DirectServicePlan):
        raise TypeError("plan must be a DirectServicePlan")

    replacements = []
    pythonizations = []
    try:
        for binding in selected_plan.bindings:
            for cpp_type, fields in (
                (binding.cpp_request_type, binding.request_fields),
                (binding.cpp_response_type, binding.response_fields),
            ):
                pythonization = _pythonize_constructor(cpp_type, fields)
                if pythonization is not None:
                    pythonizations.append(pythonization)
        for owner, name, original, replacement in selected_plan.targets:
            if getattr(owner, name) is not original:
                raise RuntimeError(
                    "%s.%s changed during direct service installation" %
                    (owner.__name__, name))
            setattr(owner, name, replacement)
            replacements.append((owner, name, original, replacement))
    except Exception:
        DirectServiceInstallation(
            replacements, selected_plan.bindings, pythonizations).restore()
        raise
    return DirectServiceInstallation(
        replacements, selected_plan.bindings, pythonizations)


def resolve_supported_type(
    service_type: Any,
    installation: DirectServiceInstallation,
) -> DirectServiceBinding:
    binding = next(
        (candidate for candidate in installation.bindings
         if service_type is candidate.service_type),
        None,
    )
    if binding is None:
        raise TypeError(
            "direct_cpp service type is not registered; active interfaces: %s" %
            ", ".join(item.interface for item in installation.bindings)
        )
    if (
        service_type.Request is not binding.cpp_request_type
        or service_type.Response is not binding.cpp_response_type
    ):
        raise TypeError(
            "direct_cpp C++ aliases are not active for %s" % binding.interface)
    return binding


__all__ = [
    "DEFAULT_INTERFACES",
    "DirectServiceBinding",
    "DirectServiceInstallation",
    "DirectServicePlan",
    "assert_early_imports",
    "install",
    "normalize_interfaces",
    "normalize_registered_interfaces",
    "prepare",
    "resolve_supported_type",
]
