"""Deprecated shim — the nav2_kit kit moved to the standalone ``nav2_kit`` package.

The cppyy kits were carved out of rclcppyy into their own packages (M1/M3). Install
``ros-jazzy-nav2-kit`` and ``import nav2_kit`` directly; this re-export keeps
``from rclcppyy.kits import nav2_kit`` working for now.
"""
import warnings as _warnings

_warnings.warn(
    "rclcppyy.kits.nav2_kit has moved to the standalone 'nav2_kit' package "
    "(conda: ros-jazzy-nav2-kit) in rclcppyy 0.2.0; import 'nav2_kit' directly.",
    DeprecationWarning, stacklevel=2)

try:
    import nav2_kit as _src
except ImportError as _exc:  # pragma: no cover - exercised when the kit isn't installed
    raise ImportError(
        "rclcppyy.kits.nav2_kit re-exports the standalone 'nav2_kit' package, which is "
        "not installed in this environment. Install it (conda: ros-jazzy-nav2-kit, e.g. "
        "`pixi add ros-jazzy-nav2-kit`) and import 'nav2_kit' directly."
    ) from _exc

globals().update({_k: _v for _k, _v in vars(_src).items()
                  if not _k.startswith("__")})
