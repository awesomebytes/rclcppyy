#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 OUTPUT_DIRECTORY" >&2
  exit 2
fi
if [[ -z "${CONDA_PREFIX:-}" ]]; then
  echo "CONDA_PREFIX is required; run inside the Pixi environment" >&2
  exit 2
fi

source_dir="$(cd "$(dirname "$0")/direct_trigger_aot_peer" && pwd)"
output_dir="$1"
build_dir="$output_dir/build"

cmake \
  -S "$source_dir" \
  -B "$build_dir" \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX"
cmake --build "$build_dir" --target direct_trigger_aot_peer
cp "$build_dir/direct_trigger_aot_peer" "$output_dir/direct_trigger_aot_peer"
