#!/bin/bash
# Sourced by pixi on environment activation. Overlays the colcon workspace so
# `ros2 run rclcppyy ...` and `import rclcppyy` from the install space work.
# Guarded so the very first `pixi install` / `pixi run` (before any build) does
# not fail on a missing install/setup.bash.
if [ -f "$PIXI_PROJECT_ROOT/install/setup.bash" ]; then
    source "$PIXI_PROJECT_ROOT/install/setup.bash"
fi

# The overlay provides entry points and ROS resources, but development commands
# must exercise the checkout rather than a stale copy from an earlier colcon
# build. Package proofs run in a separate fresh environment.
export PYTHONPATH="$PIXI_PROJECT_ROOT${PYTHONPATH:+:$PYTHONPATH}"

# Development branches of rclcppyy and the supporting suite evolve together. Only
# activate the exact reviewed source revision; scripts/ci/verify_suite_source.py
# also checks cleanliness, recipe versions, and active Python roots before tests.
_suite_src="${RCLCPPYY_SUITE_SRC:-$PIXI_PROJECT_ROOT/../cppyy_kit}"
_rclcpp_kit_src="$_suite_src/rclcpp_kit"
_suite_lock="$PIXI_PROJECT_ROOT/suite-source.lock.json"
_expected_suite_commit="$(sed -n 's/.*"commit": "\([0-9a-f]*\)".*/\1/p' "$_suite_lock")"
_actual_suite_commit="$(git -C "$_suite_src" rev-parse HEAD 2>/dev/null || true)"
if [ -n "$_expected_suite_commit" ] && \
   [ "$_actual_suite_commit" = "$_expected_suite_commit" ] && \
   [ -d "$_suite_src/cppyy_kit" ] && [ -d "$_rclcpp_kit_src/rclcpp_kit" ]; then
    export PYTHONPATH="$_rclcpp_kit_src:$_suite_src:$PYTHONPATH"
else
    echo "rclcppyy: reviewed suite source is unavailable; suite-contract will fail." >&2
fi

unset _suite_src _rclcpp_kit_src _suite_lock _expected_suite_commit _actual_suite_commit
