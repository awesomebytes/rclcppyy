"""Deprecated shim — moved to ``rclcpp_kit.rosbag2_cpp`` in rclcppyy 0.2.0.

The C++ ``rosbag2_cpp`` reader/writer wrappers were carved into the standalone
``rclcpp_kit`` package. Import from ``rclcpp_kit.rosbag2_cpp`` instead; this
re-export keeps ``from rclcppyy import rosbag2_cpp`` working for now.
"""
import importlib as _importlib
import warnings as _warnings

_warnings.warn(
    "rclcppyy.rosbag2_cpp has moved to rclcpp_kit.rosbag2_cpp (rclcppyy 0.2.0); "
    "import from rclcpp_kit.rosbag2_cpp instead — this shim will be removed in a "
    "future release.",
    DeprecationWarning, stacklevel=2)

_src = _importlib.import_module("rclcpp_kit.rosbag2_cpp")

globals().update({_k: _v for _k, _v in vars(_src).items()
                  if not _k.startswith("__")})
