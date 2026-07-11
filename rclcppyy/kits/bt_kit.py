"""Deprecated shim — the bt_kit kit moved to the standalone ``bt_kit`` package.

The cppyy kits were carved out of rclcppyy into their own packages (M1/M3). Install
``ros-jazzy-bt-kit`` and ``import bt_kit`` directly; this re-export keeps
``from rclcppyy.kits import bt_kit`` working for now.
"""
import warnings as _warnings

_warnings.warn(
    "rclcppyy.kits.bt_kit has moved to the standalone 'bt_kit' package "
    "(conda: ros-jazzy-bt-kit) in rclcppyy 0.2.0; import 'bt_kit' directly.",
    DeprecationWarning, stacklevel=2)

try:
    import bt_kit as _src
except ImportError as _exc:  # pragma: no cover - exercised when the kit isn't installed
    raise ImportError(
        "rclcppyy.kits.bt_kit re-exports the standalone 'bt_kit' package, which is "
        "not installed in this environment. Install it (conda: ros-jazzy-bt-kit, e.g. "
        "`pixi add ros-jazzy-bt-kit`) and import 'bt_kit' directly."
    ) from _exc

globals().update({_k: _v for _k, _v in vars(_src).items()
                  if not _k.startswith("__")})
