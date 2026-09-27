#!/usr/bin/env bash

# Build the product against provenance-verified bytes retained from the public
# channel. The support packages must already be present in output_dir.
set -euo pipefail

case "$(uname -m)" in
  x86_64|aarch64|arm64) architecture="$(uname -m)" ;;
  *)
    echo "Unsupported package-build architecture: $(uname -m)" >&2
    exit 2
    ;;
esac

repo_root="$(cd "$(dirname "$0")/../.." && pwd)"
suite_checkout="${1:?suite checkout is required}"
requested_output="${2:?output directory is required}"
published_proof="${3:?published support proof is required}"
case "$suite_checkout" in
  /*) ;;
  *) suite_checkout="$repo_root/$suite_checkout" ;;
esac
case "$requested_output" in
  /*) ;;
  *) requested_output="$repo_root/$requested_output" ;;
esac
case "$published_proof" in
  /*) ;;
  *) published_proof="$repo_root/$published_proof" ;;
esac

suite_checkout="$(cd "$suite_checkout" && pwd)"
mkdir -p "$requested_output"
output_dir="$(cd "$requested_output" && pwd)"
published_proof="$(realpath "$published_proof")"

if [ -n "$(git -C "$repo_root" status --porcelain --untracked-files=all)" ]; then
  echo "rclcppyy checkout must be clean before package evidence is built" >&2
  exit 1
fi
if [ -n "$(git -C "$suite_checkout" status --porcelain --untracked-files=all)" ]; then
  echo "suite checkout must be clean before package evidence is built" >&2
  exit 1
fi
product_commit="$(git -C "$repo_root" rev-parse HEAD)"
suite_commit="$(git -C "$suite_checkout" rev-parse HEAD)"
PYTHONPATH="$suite_checkout/rclcpp_kit:$suite_checkout${PYTHONPATH:+:$PYTHONPATH}" \
  RCLCPPYY_SUITE_SRC="$suite_checkout" \
  python "$repo_root/scripts/ci/verify_suite_source.py" --suite "$suite_checkout"

# Re-check the retained files immediately before the solver receives the local
# channel. This prevents a missing dependency from silently falling through to
# a mutable remote channel.
PYTHONPATH="$repo_root${PYTHONPATH:+:$PYTHONPATH}" python - \
  "$published_proof" "$output_dir" "$repo_root/suite-source.lock.json" \
  "$architecture" <<'PY'
import json
from pathlib import Path
import sys

from scripts.ci.verify_published_support import validate_retained_support

validate_retained_support(
    json.loads(Path(sys.argv[1]).read_text(encoding="utf-8")),
    Path(sys.argv[2]),
    suite_lock=json.loads(Path(sys.argv[3]).read_text(encoding="utf-8")),
    architecture=sys.argv[4],
)
print("RETAINED_PUBLISHED_SUPPORT_READY")
PY

snapshot_root="$(mktemp -d)"
trap 'rm -rf "$snapshot_root"' EXIT
product_snapshot="$snapshot_root/rclcppyy"
mkdir -p "$product_snapshot"
git -C "$repo_root" archive --format=tar "$product_commit" | \
  tar -xf - -C "$product_snapshot"

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
  --published-support-proof "$published_proof" \
  --attestation "$output_dir/local-package-attestation.json"
