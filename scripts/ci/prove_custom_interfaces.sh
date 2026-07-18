#!/usr/bin/env bash

# Build a real generated-interface package plus an ahead-of-time rclcpp peer,
# then run the same Python application through stock and compatible backends.
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/../.." && pwd)"
fixture_root="$repo_root/test/fixtures/custom_interfaces"
work_root="${1:-$repo_root/build/custom-interface}"
evidence_root="${2:-$repo_root/build/test-results}"
mkdir -p "$work_root" "$evidence_root"
work_root="$(cd "$work_root" && pwd)"
evidence_root="$(cd "$evidence_root" && pwd)"

colcon --log-base "$work_root/log" build \
  --base-paths "$fixture_root" \
  --build-base "$work_root/build" \
  --install-base "$work_root/install" \
  --packages-select rclcppyy_test_interfaces rclcppyy_test_peer \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --event-handlers console_direct+

# shellcheck disable=SC1091
set +u
source "$work_root/install/setup.bash"
set -u
peer="$work_root/install/rclcppyy_test_peer/lib/rclcppyy_test_peer/interop_peer"
test -x "$peer"

for mode in stock activated; do
  echo "Running custom-interface AOT proof: $mode"
  timeout --signal=TERM --kill-after=10s 300s \
    python "$fixture_root/run_interop.py" \
      --mode "$mode" \
      --peer "$peer" \
      --evidence "$evidence_root/custom-interface-$mode.json"
done

echo "CUSTOM_INTERFACE_AOT_INTEROP_OK"
