"""Transactional aliases to actual generated C++ messages for ``direct_cpp``."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import sys
from typing import Any


_IMPLEMENTATION_MODULES = (
    "std_msgs.msg._u_int64",
    "std_msgs.msg._string",
)
_PYTHONIZED: dict[Any, Any] = {}


@dataclass(frozen=True)
class DirectMessageBinding:
    original_type: type
    cpp_type: Any
    cpp_type_name: str


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


def assert_early_imports() -> None:
    stale = sorted(name for name in _IMPLEMENTATION_MODULES if name in sys.modules)
    if stale:
        raise RuntimeError(
            "direct_cpp must be enabled before importing supported messages: %s" %
            ", ".join(stale)
        )


def _pythonize_constructor(binding: DirectMessageBinding):
    cpp_type = binding.cpp_type
    if cpp_type in _PYTHONIZED:
        return None
    original_init = cpp_type.__init__

    def direct_init(self, *args, **kwargs):
        if args and kwargs:
            raise TypeError("C++ message constructors cannot mix positional and keyword values")
        if args:
            original_init(self, *args)
            return
        original_init(self)
        unknown = sorted(set(kwargs) - {"data"})
        if unknown:
            raise TypeError("unknown message constructor fields: %s" % ", ".join(unknown))
        if "data" in kwargs:
            self.data = kwargs["data"]

    cpp_type.__init__ = direct_init
    _PYTHONIZED[cpp_type] = original_init
    return cpp_type, original_init, direct_init


def install() -> DirectMessageInstallation:
    """Resolve both headers/classes, then atomically replace their import aliases."""
    from rclcpp_kit.bringup_rclcpp import _resolve_message_type, bringup_rclcpp

    bringup_rclcpp()
    specifications = (
        ("std_msgs.msg._u_int64", "UInt64"),
        ("std_msgs.msg._string", "String"),
    )
    bindings = []
    targets = []
    for module_name, name in specifications:
        generated_module = importlib.import_module(module_name)
        public_module = importlib.import_module("std_msgs.msg")
        original = getattr(generated_module, name)
        if getattr(public_module, name) is not original:
            raise RuntimeError("std_msgs.msg.%s changed before direct alias installation" % name)
        cpp_type_name, cpp_type = _resolve_message_type(original)
        binding = DirectMessageBinding(original, cpp_type, cpp_type_name)
        bindings.append(binding)
        targets.extend((
            (generated_module, name, original, cpp_type),
            (public_module, name, original, cpp_type),
        ))

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
    "DirectMessageBinding",
    "DirectMessageInstallation",
    "assert_early_imports",
    "install",
]
