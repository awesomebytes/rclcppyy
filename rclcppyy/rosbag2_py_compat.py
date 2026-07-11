"""Deprecated shim — moved to ``rclcpp_kit.rosbag2_py_compat`` in rclcppyy 0.2.0.

The ``rosbag2_py``-compatible facade backed by ``rosbag2_cpp`` was carved into the
standalone ``rclcpp_kit`` package. Import from ``rclcpp_kit.rosbag2_py_compat``
instead; this re-export keeps ``from rclcppyy.rosbag2_py_compat import ...``
working for now.
"""
import importlib as _importlib
import warnings as _warnings

_warnings.warn(
    "rclcppyy.rosbag2_py_compat has moved to rclcpp_kit.rosbag2_py_compat (rclcppyy "
    "0.2.0); import from rclcpp_kit.rosbag2_py_compat instead — this shim will be "
    "removed in a future release.",
    DeprecationWarning, stacklevel=2)

_src = _importlib.import_module("rclcpp_kit.rosbag2_py_compat")

globals().update({_k: _v for _k, _v in vars(_src).items()
                  if not _k.startswith("__")})
