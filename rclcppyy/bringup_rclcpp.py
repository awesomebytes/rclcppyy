"""Deprecated shim — moved to ``rclcpp_kit.bringup_rclcpp`` in rclcppyy 0.2.0.

rclcppyy's rclcpp core (bringup, C++ message resolution/conversion, the shared
``convert_python_msg_to_cpp``, and the rclpy-style ``rclcpp.Node`` adapters) was
carved into the standalone ``rclcpp_kit`` package. This module re-exports it
unchanged so existing ``from rclcppyy.bringup_rclcpp import ...`` imports keep
working; new code should import from ``rclcpp_kit.bringup_rclcpp``.

Unlike the other moved modules, this shim does **not** emit a DeprecationWarning:
rclcppyy's own product code (``node.py`` / ``monkey.py`` / ``monkeypatch_messages``)
imports these names internally, so a warning would fire on every ``import
rclcppyy``. The user-facing re-export modules (``serialization`` / ``rosbag2_cpp`` /
``rosbag2_py_compat`` / ``tf``) do warn.
"""
import importlib as _importlib

# Resolve the real submodule explicitly: ``rclcpp_kit/__init__`` binds the
# *function* ``bringup_rclcpp`` as a package attribute, shadowing the submodule of
# the same name, so ``import rclcpp_kit.bringup_rclcpp`` would bind the function.
# ``import_module`` returns the module object regardless.
_src = _importlib.import_module("rclcpp_kit.bringup_rclcpp")

# Faithful re-export: copy every public AND private name (the bringup functions,
# the message adapters, ``_resolve_message_type`` / ``_is_msg_python`` /
# ``convert_python_msg_to_cpp``, the ``_Bound*`` / ``_NodeMethodDescriptor``
# classes, module-level constants) so imports resolve exactly as before. Dunders
# stay this shim module's own.
globals().update({_k: _v for _k, _v in vars(_src).items()
                  if not _k.startswith("__")})
