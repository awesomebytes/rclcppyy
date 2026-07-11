"""Deprecated shim — moved to ``rclcpp_kit.tf`` in rclcppyy 0.2.0.

The tf2 C++ transform stack (``TransformListener`` etc.) was carved into the
standalone ``rclcpp_kit`` package. Import from ``rclcpp_kit.tf`` instead; this
re-export keeps ``from rclcppyy import tf`` / ``from rclcppyy.tf import ...``
working for now.
"""
import importlib as _importlib
import warnings as _warnings

_warnings.warn(
    "rclcppyy.tf has moved to rclcpp_kit.tf (rclcppyy 0.2.0); import from "
    "rclcpp_kit.tf instead — this shim will be removed in a future release.",
    DeprecationWarning, stacklevel=2)

_src = _importlib.import_module("rclcpp_kit.tf")

globals().update({_k: _v for _k, _v in vars(_src).items()
                  if not _k.startswith("__")})
