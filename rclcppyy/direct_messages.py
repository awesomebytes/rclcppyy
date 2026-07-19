"""Transactional aliases to actual generated C++ messages for ``direct_cpp``."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import re
import sys
from typing import Any


DEFAULT_INTERFACES = (
    "std_msgs/msg/String",
    "std_msgs/msg/UInt64",
)
_INTERFACE_NAME = re.compile(
    r"^([A-Za-z][A-Za-z0-9_]*)/msg/([A-Z][A-Za-z0-9_]*)$")
_FIELD_INTERFACE = re.compile(
    r"(?<![A-Za-z0-9_])([A-Za-z][A-Za-z0-9_]*)/(?:msg/)?"
    r"([A-Z][A-Za-z0-9_]*)")
_IMPLEMENTATION_MODULE = re.compile(
    r"^[A-Za-z][A-Za-z0-9_]*\.msg\._[A-Za-z0-9_]+$")
_PYTHONIZED: dict[Any, Any] = {}


@dataclass(frozen=True)
class DirectMessageBinding:
    interface: str
    original_type: type
    cpp_type: Any
    cpp_type_name: str
    header: str
    fields: tuple[str, ...]


class DirectMessageInstallation:
    def __init__(self, replacements, bindings, pythonizations=()):
        self._replacements = tuple(replacements)
        self._pythonizations = tuple(pythonizations)
        self.bindings = tuple(bindings)
        self._restored = False

    def restore(self):
        if self._restored:
            return
        for module, name, original, cpp_type in reversed(self._replacements):
            if getattr(module, name, None) is cpp_type:
                setattr(module, name, original)
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
            "direct_cpp message interfaces must use package/msg/Message names: %s" %
            ", ".join(invalid)
        )
    return tuple(sorted(set(selected)))


def assert_early_imports() -> None:
    stale = sorted(
        name for name in sys.modules if _IMPLEMENTATION_MODULE.fullmatch(name))
    if stale:
        raise RuntimeError(
            "direct_cpp must be enabled before importing generated messages: %s" %
            ", ".join(stale)
        )


def _pythonize_constructor(binding: DirectMessageBinding):
    cpp_type = binding.cpp_type
    if cpp_type in _PYTHONIZED:
        return None
    original_init = cpp_type.__init__
    allowed = frozenset(binding.fields)

    def direct_init(self, *args, **kwargs):
        if args and kwargs:
            raise TypeError("C++ message constructors cannot mix positional and keyword values")
        if args:
            original_init(self, *args)
            return
        original_init(self)
        unknown = sorted(set(kwargs) - allowed)
        if unknown:
            raise TypeError("unknown message constructor fields: %s" % ", ".join(unknown))
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


def _prepare_bindings(interfaces: tuple[str, ...]):
    from rclcpp_kit.bringup_rclcpp import bringup_rclcpp
    from rclcpp_kit.direct_message_types import load_message_type
    from rosidl_runtime_py.utilities import get_message

    bringup_rclcpp()
    pending = list(DEFAULT_INTERFACES + interfaces)
    seen = set()
    bindings = []
    targets = []
    while pending:
        interface = pending.pop(0)
        if interface in seen:
            continue
        seen.add(interface)
        match = _INTERFACE_NAME.fullmatch(interface)
        package, name = match.groups()
        try:
            original = get_message(interface)
        except (ImportError, AttributeError, ValueError) as exc:
            raise TypeError(
                "direct_cpp message interface is not installed: %s" % interface
            ) from exc
        generated_module = importlib.import_module(original.__module__)
        public_module = importlib.import_module("%s.msg" % package)
        if getattr(generated_module, name) is not original:
            raise RuntimeError("%s changed before direct alias installation" % interface)
        if getattr(public_module, name) is not original:
            raise RuntimeError("%s public alias changed before direct installation" % interface)
        descriptor = load_message_type(package, name)
        binding = DirectMessageBinding(
            interface=interface,
            original_type=original,
            cpp_type=descriptor.cpp_type,
            cpp_type_name=descriptor.cpp_type_name,
            header=descriptor.header,
            fields=tuple(original.get_fields_and_field_types()),
        )
        bindings.append(binding)
        targets.extend((
            (generated_module, name, original, descriptor.cpp_type),
            (public_module, name, original, descriptor.cpp_type),
        ))
        for dependency in _dependencies(original):
            if dependency not in seen and dependency not in pending:
                pending.append(dependency)
    return tuple(bindings), tuple(targets)


def install(interfaces=()) -> DirectMessageInstallation:
    """Resolve requested C++ types and dependencies, then atomically alias them."""
    normalized = normalize_interfaces(interfaces)
    bindings, targets = _prepare_bindings(normalized)

    replacements = []
    pythonizations = []
    try:
        for binding in bindings:
            pythonization = _pythonize_constructor(binding)
            if pythonization is not None:
                pythonizations.append(pythonization)
        for module, name, original, cpp_type in targets:
            if getattr(module, name) is not original:
                raise RuntimeError(
                    "%s.%s changed during direct alias installation" %
                    (module.__name__, name)
                )
            setattr(module, name, cpp_type)
            replacements.append((module, name, original, cpp_type))
    except Exception:
        DirectMessageInstallation(
            replacements, bindings, pythonizations).restore()
        raise
    return DirectMessageInstallation(replacements, bindings, pythonizations)


__all__ = [
    "DEFAULT_INTERFACES",
    "DirectMessageBinding",
    "DirectMessageInstallation",
    "assert_early_imports",
    "install",
    "normalize_interfaces",
]
