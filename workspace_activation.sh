#!/bin/bash
# Sourced by pixi on environment activation. Overlays the colcon workspace so
# `ros2 run rclcppyy ...` and `import rclcppyy` from the install space work.
# Guarded so the very first `pixi install` / `pixi run` (before any build) does
# not fail on a missing install/setup.bash.
if [ -f "$PIXI_PROJECT_ROOT/install/setup.bash" ]; then
    source "$PIXI_PROJECT_ROOT/install/setup.bash"
fi
