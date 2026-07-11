"""Deprecated shim — moved to ``cppyy_kit.freeze`` in rclcppyy 0.2.0.

The L0->L1 "freeze" (Cling PCH) tooling was carved into the standalone ``cppyy_kit``
base package. Install ``cppyy-kit`` and ``import cppyy_kit.freeze`` directly; this
re-export keeps ``from rclcppyy.kits import freeze`` working for now.
"""
import importlib as _importlib
import warnings as _warnings

_warnings.warn(
    "rclcppyy.kits.freeze has moved to cppyy_kit.freeze (conda: cppyy-kit) in "
    "rclcppyy 0.2.0; import 'cppyy_kit.freeze' directly.",
    DeprecationWarning, stacklevel=2)

try:
    # import_module returns the submodule regardless of any package-attribute shadow.
    _src = _importlib.import_module("cppyy_kit.freeze")
except ImportError as _exc:  # pragma: no cover - exercised when cppyy_kit isn't installed
    raise ImportError(
        "rclcppyy.kits.freeze re-exports cppyy_kit.freeze, which is not installed in "
        "this environment. Install it (conda: cppyy-kit) and import "
        "'cppyy_kit.freeze' directly."
    ) from _exc

globals().update({_k: _v for _k, _v in vars(_src).items()
                  if not _k.startswith("__")})
