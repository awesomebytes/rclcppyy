#!/usr/bin/env bash

# Build the exact local-channel dependency stack for the rclcppyy artifact.
# The suite packages are built first from their checkout, then rclcppyy resolves
# its strict 0.2.0 dependencies from the same isolated channel.
set -euo pipefail

case "$(uname -m)" in
  x86_64) ;;
  aarch64|arm64)
    echo "ARM64 conda build unavailable: cppyy >=3.5 has no Python 3.12 conda package." >&2
    exit 2
    ;;
  *)
    echo "Unsupported package-build architecture: $(uname -m)" >&2
    exit 2
    ;;
esac

repo_root="$(cd "$(dirname "$0")/../.." && pwd)"
suite_checkout="${1:-$repo_root/../cppyy_kit}"
requested_output="${2:-$repo_root/output}"
case "$suite_checkout" in
  /*) ;;
  *) suite_checkout="$repo_root/$suite_checkout" ;;
esac
case "$requested_output" in
  /*) ;;
  *) requested_output="$repo_root/$requested_output" ;;
esac

suite_checkout="$(cd "$suite_checkout" && pwd)"
mkdir -p "$requested_output"
output_dir="$(cd "$requested_output" && pwd)"
test -x "$suite_checkout/recipe/build_rclcpp.sh"

bash "$suite_checkout/recipe/build_rclcpp.sh" "$output_dir"

cd "$repo_root"
rattler-build build \
  --recipe recipe/recipe.yaml \
  -c "file://$output_dir" \
  -c robostack-jazzy \
  -c conda-forge \
  --output-dir "$output_dir"

artifact="$(find "$output_dir" -name 'ros-jazzy-rclcppyy-0.2.0-*.conda' -print -quit)"
if [ -z "$artifact" ]; then
  echo "Expected ros-jazzy-rclcppyy 0.2.0 artifact in $output_dir" >&2
  exit 1
fi
echo "Built artifact: $artifact"
