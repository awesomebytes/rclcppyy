#!/usr/bin/env python3
"""``rclcppyy-run``: run a command with rclcppyy's RCLCPPYY_ENABLE_HOOK startup
hook enabled.

    rclcppyy-run my_node.py
    # Equivalent to: eval $(rclcppyy-env) && python3 my_node.py

    rclcppyy-run ros2 topic hz /some_topic
    # Works for any command, not just a .py script.

This file is only the thin executable entry point (installed to
``bin/rclcppyy-run`` by CMakeLists.txt); the implementation lives in
``rclcppyy.cli.run_main``.
"""
import sys

from rclcppyy.cli import run_main

if __name__ == "__main__":
    sys.exit(run_main())
