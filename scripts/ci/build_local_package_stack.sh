#!/usr/bin/env bash

# Build the exact local-channel dependency stack for the rclcppyy artifact.
# The suite packages are built first from their checkout, then rclcppyy resolves
# its strict 0.2.0 dependencies from the same isolated channel.
set -euo pipefail

case "$(uname -m)" in
  x86_64|aarch64|arm64) ;;
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
# commits. Build the suite from a detached clone and the product from a Git
# archive so later editor activity cannot alter either local-path source while
# rattler-build is copying it.
if [ -n "$(git -C "$repo_root" status --porcelain --untracked-files=all)" ]; then
  echo "rclcppyy checkout must be clean before package evidence is built" >&2
  exit 1
fi
product_commit="$(git -C "$repo_root" rev-parse HEAD)"
suite_commit="$(git -C "$suite_checkout" rev-parse HEAD)"
PYTHONPATH="$suite_checkout/rclcpp_kit:$suite_checkout${PYTHONPATH:+:$PYTHONPATH}" \
  RCLCPPYY_SUITE_SRC="$suite_checkout" \
  python "$repo_root/scripts/ci/verify_suite_source.py" --suite "$suite_checkout"

snapshot_root="$(mktemp -d)"
trap 'rm -rf "$snapshot_root"' EXIT
suite_snapshot="$snapshot_root/cppyy_kit"
product_snapshot="$snapshot_root/rclcppyy"
mkdir -p "$product_snapshot"
git clone --quiet --no-hardlinks --no-checkout \
  "$suite_checkout" "$suite_snapshot"
git -C "$suite_snapshot" checkout --quiet --detach "$suite_commit"
git -C "$repo_root" archive --format=tar "$product_commit" | \
  tar -xf - -C "$product_snapshot"

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

python "$repo_root/scripts/ci/write_local_package_attestation.py" \
  --output-dir "$output_dir" \
  --product-commit "$product_commit" \
  --suite-commit "$suite_commit" \
  --attestation "$output_dir/local-package-attestation.json"
