#!/bin/bash
# Sourced by pixi on environment activation. Overlays the colcon workspace so
# `ros2 run rclcppyy ...` and `import rclcppyy` from the install space work.
# Guarded so the very first `pixi install` / `pixi run` (before any build) does
# not fail on a missing install/setup.bash.
if [ -f "$PIXI_PROJECT_ROOT/install/setup.bash" ]; then
    source "$PIXI_PROJECT_ROOT/install/setup.bash"
fi

# --- M3 PYTHONPATH bridge (pre-release) --------------------------------------
# rclcppyy 0.2.0 re-exports rclcpp_kit / cppyy_kit (the carved-out suite) via
# deprecation shims, so those packages must be importable. Until the suite is
# published to prefix.dev they are not conda deps (a file:// channel would not
# resolve on CI / other machines); instead we expose the two packages from a
# sibling cppyy_kit *source* checkout. CPPYY_KIT_SRC overrides the default
# (../cppyy_kit); CI sets it to its checkout path.
#
# We expose ONLY cppyy_kit + rclcpp_kit (via a symlink dir), never the whole
# cppyy_kit repo root -- that would put every domain-kit container dir (bt_kit/,
# pcl_kit/, ...) on the path as an empty namespace package, so the domain-kit
# shims would wrongly resolve. This models production, where rclcppyy's env has
# only these two suite packages. Removed once rclcppyy depends on the published
# conda packages.
CPPYY_KIT_SRC="${CPPYY_KIT_SRC:-$PIXI_PROJECT_ROOT/../cppyy_kit}"
if [ -d "$CPPYY_KIT_SRC/cppyy_kit" ] && [ -d "$CPPYY_KIT_SRC/rclcpp_kit/rclcpp_kit" ]; then
    _suite_bridge="$PIXI_PROJECT_ROOT/.suite_bridge"
    mkdir -p "$_suite_bridge"
    ln -sfn "$CPPYY_KIT_SRC/cppyy_kit" "$_suite_bridge/cppyy_kit"
    ln -sfn "$CPPYY_KIT_SRC/rclcpp_kit/rclcpp_kit" "$_suite_bridge/rclcpp_kit"
    export PYTHONPATH="$_suite_bridge${PYTHONPATH:+:$PYTHONPATH}"
fi
