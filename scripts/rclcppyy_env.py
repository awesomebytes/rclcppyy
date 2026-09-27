#!/usr/bin/env python3
"""``rclcppyy-env``: print shell-eval-able exports that turn on rclcppyy's
RCLCPPYY_ENABLE_HOOK startup hook for the current shell.

    eval $(rclcppyy-env)
    # Every Python ROS node started in this shell now accelerates the instant it
    # imports rclpy -- see rclcppyy.cli for what "eval" is printing and
    # rclcppyy.hook for the underlying startup-hook mechanism.

This file is only the thin executable entry point (installed to
``bin/rclcppyy-env`` by CMakeLists.txt); the implementation lives in
``rclcppyy.cli.env_main``.
"""
import sys

from rclcppyy.cli import env_main

if __name__ == "__main__":
    sys.exit(env_main())
