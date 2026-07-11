"""Deprecated shim — moved to ``rclcpp_kit.serialization`` in rclcppyy 0.2.0.

The C++ CDR serialization helpers were carved into the standalone ``rclcpp_kit``
package. Import from ``rclcpp_kit.serialization`` instead; this re-export keeps
``from rclcppyy.serialization import ...`` working for now.
"""
import importlib as _importlib
import warnings as _warnings

_warnings.warn(
    "rclcppyy.serialization has moved to rclcpp_kit.serialization (rclcppyy 0.2.0); "
    "import from rclcpp_kit.serialization instead — this shim will be removed in a "
    "future release.",
    DeprecationWarning, stacklevel=2)

_src = _importlib.import_module("rclcpp_kit.serialization")

globals().update({_k: _v for _k, _v in vars(_src).items()
                  if not _k.startswith("__")})
