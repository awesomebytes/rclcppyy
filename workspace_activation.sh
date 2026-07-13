#!/bin/bash
# Sourced by pixi on environment activation. Overlays the colcon workspace so
# `ros2 run rclcppyy ...` and `import rclcppyy` from the install space work.
# Guarded so the very first `pixi install` / `pixi run` (before any build) does
# not fail on a missing install/setup.bash.
if [ -f "$PIXI_PROJECT_ROOT/install/setup.bash" ]; then
    source "$PIXI_PROJECT_ROOT/install/setup.bash"
fi

# --- heavy_hz demo bridge (DEV/demo-only, gated to the heavydemo env) ---------
# The default env pins the PUBLISHED suite (cppyy-kit / rclcpp-kit 0.1.0), which
# predates the zero-config auto-PCH. The `heavydemo` env (scripts/heavy_hz_demo/)
# bridges the newer suite from a sibling cppyy_kit *source* checkout: this block
# puts that checkout's cppyy_kit + rclcpp_kit FIRST on PYTHONPATH so `import
# rclcppyy` runs on the newer auto-PCH -- which is what the cold-vs-warm startup
# story measures. It runs AFTER `source setup.bash` above, so the bridge prepend
# wins over the ament/site-packages entries setup.bash adds. Gated on
# PIXI_ENVIRONMENT_NAME (pixi sets it before activation, unlike a feature's
# activation.env which lands too late for this guard), so it is a no-op in every
# other env and nothing here touches the default env's published-channel
# dependency. CPPYY_KIT_SRC overrides the checkout path. Mirrors the removed M3
# bridge; see RELEASING.md "Channel swap" for that history.
if [ "$PIXI_ENVIRONMENT_NAME" = "heavydemo" ]; then
    CPPYY_KIT_SRC="${CPPYY_KIT_SRC:-$PIXI_PROJECT_ROOT/../cppyy_kit}"
    if [ -d "$CPPYY_KIT_SRC/cppyy_kit" ] && [ -d "$CPPYY_KIT_SRC/rclcpp_kit/rclcpp_kit" ]; then
        _suite_bridge="$PIXI_PROJECT_ROOT/.suite_bridge"
        mkdir -p "$_suite_bridge"
        ln -sfn "$CPPYY_KIT_SRC/cppyy_kit" "$_suite_bridge/cppyy_kit"
        ln -sfn "$CPPYY_KIT_SRC/rclcpp_kit/rclcpp_kit" "$_suite_bridge/rclcpp_kit"
        export PYTHONPATH="$_suite_bridge${PYTHONPATH:+:$PYTHONPATH}"
    else
        echo "heavydemo: CPPYY_KIT_SRC=$CPPYY_KIT_SRC has no cppyy_kit/ + rclcpp_kit/rclcpp_kit/;" \
             "rclcppyy will fall back to the published suite 0.1.0 (no auto-PCH)." >&2
    fi
fi
