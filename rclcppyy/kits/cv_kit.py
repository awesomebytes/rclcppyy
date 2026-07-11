"""Deprecated shim — the cv_kit kit moved to the standalone ``cv_kit`` package.

The cppyy kits were carved out of rclcppyy into their own packages (M1/M3). Install
``ros-jazzy-cv-kit`` and ``import cv_kit`` directly; this re-export keeps
``from rclcppyy.kits import cv_kit`` working for now.
"""
import warnings as _warnings

_warnings.warn(
    "rclcppyy.kits.cv_kit has moved to the standalone 'cv_kit' package "
    "(conda: ros-jazzy-cv-kit) in rclcppyy 0.2.0; import 'cv_kit' directly.",
    DeprecationWarning, stacklevel=2)

try:
    import cv_kit as _src
except ImportError as _exc:  # pragma: no cover - exercised when the kit isn't installed
    raise ImportError(
        "rclcppyy.kits.cv_kit re-exports the standalone 'cv_kit' package, which is "
        "not installed in this environment. Install it (conda: ros-jazzy-cv-kit, e.g. "
        "`pixi add ros-jazzy-cv-kit`) and import 'cv_kit' directly."
    ) from _exc

globals().update({_k: _v for _k, _v in vars(_src).items()
                  if not _k.startswith("__")})
