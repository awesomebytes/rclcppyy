#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "$0")/../.." && pwd)"
user_base="$(mktemp -d)"
output="$(mktemp)"
cleanup() {
  rm -rf "$user_base"
  rm -f "$output"
}
trap cleanup EXIT

user_site="$(PYTHONUSERBASE="$user_base" python -c 'import site; print(site.getusersitepackages())')"
mkdir -p "$user_site"
RCLCPPYY_HOOK_SITE="$user_site" PYTHONUSERBASE="$user_base" python -c \
  'import os; from rclcppyy import hook; hook.install(os.environ["RCLCPPYY_HOOK_SITE"])'

set +e
(
  cd "$repo_root"
  set +u
  source install/setup.bash
  set -u
  PYTHONUSERBASE="$user_base" RCLCPPYY_ENABLE_HOOK=1 \
    timeout --signal=TERM --kill-after=10s 240s \
    ros2 launch --noninteractive rclcppyy hook_probe.launch.py
) 2>&1 | tee "$output"
launch_status=${PIPESTATUS[0]}
set -e

test "$launch_status" -eq 0
grep -q 'HOOK_LAUNCH_CLI_OK' "$output"
echo "LAUNCH_CLI_CONTRACT_OK"
