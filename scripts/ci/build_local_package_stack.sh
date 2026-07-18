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
suite_checkout="${1:-${CPPYY_KIT_SRC:-$repo_root/../cppyy_kit}}"
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

# Package evidence is meaningful only when both source checkouts are exact clean
# commits. Build from git archives so later editor activity cannot alter either
# local-path source while rattler-build is copying it.
if [ -n "$(git -C "$repo_root" status --porcelain --untracked-files=all)" ]; then
  echo "rclcppyy checkout must be clean before package evidence is built" >&2
  exit 1
fi
RCLCPPYY_SUITE_SRC="$suite_checkout" \
  python "$repo_root/scripts/ci/verify_suite_source.py" --suite "$suite_checkout"

snapshot_root="$(mktemp -d)"
trap 'rm -rf "$snapshot_root"' EXIT
suite_snapshot="$snapshot_root/cppyy_kit"
product_snapshot="$snapshot_root/rclcppyy"
mkdir -p "$suite_snapshot" "$product_snapshot"
git -C "$suite_checkout" archive --format=tar HEAD | tar -xf - -C "$suite_snapshot"
git -C "$repo_root" archive --format=tar HEAD | tar -xf - -C "$product_snapshot"

bash "$suite_snapshot/recipe/build_rclcpp.sh" "$output_dir"

cd "$product_snapshot"
rattler-build build \
  --recipe recipe/recipe.yaml \
  -c "file://$output_dir" \
  -c robostack-jazzy \
  -c conda-forge \
  --output-dir "$output_dir"

artifact="$(find "$output_dir" -name 'ros-jazzy-rclcppyy-0.3.0-*.conda' -print -quit)"
if [ -z "$artifact" ]; then
  echo "Expected ros-jazzy-rclcppyy 0.3.0 artifact in $output_dir" >&2
  exit 1
fi
echo "Built artifact: $artifact"
