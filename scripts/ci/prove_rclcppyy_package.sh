#!/usr/bin/env bash

# Prove the built conda artifact from a throwaway workspace. Keeping the proof
# outside the repository workspace prevents source imports from hiding missing
# runtime dependencies in the package.
set -euo pipefail

case "$(uname -m)" in
  x86_64) platform="linux-64" ;;
  aarch64|arm64)
    echo "ARM64 conda proof unavailable: cppyy >=3.5 has no Python 3.12 conda package." >&2
    exit 2
    ;;
  *)
    echo "Unsupported package-proof architecture: $(uname -m)" >&2
    exit 2
    ;;
esac

output_dir="${1:-output}"
output_dir="$(realpath "$output_dir")"
workdir="$(mktemp -d)"
trap 'rm -rf "$workdir"' EXIT

cat >"$workdir/pixi.toml" <<EOF
[workspace]
name = "rclcppyy-artifact-proof"
channels = ["file://${output_dir}", "https://repo.prefix.dev/awesomebytes", "robostack-jazzy", "conda-forge"]
platforms = ["${platform}"]
version = "0.0.0"

[dependencies]
ros-jazzy-rclcppyy = "*"
EOF

cat >"$workdir/smoke.py" <<'PY'
import sys
import time

import cppyy
import rclcppyy
from builtin_interfaces.msg import Time

print("rclcppyy:", rclcppyy.__file__)
rclcppyy.enable_cpp_acceleration()
r = rclcppyy.bringup_rclcpp()
r.init([])
pub_node = r.Node("package_proof_pub")
sub_node = r.Node("package_proof_sub")
received = []
publisher = pub_node.create_publisher(Time, "package_proof_topic", 10)
subscription = sub_node.create_subscription(
    Time,
    "package_proof_topic",
    lambda message: received.append(int(message.sec)),
    10,
)
executor = r.executors.SingleThreadedExecutor()
executor.add_node(sub_node.get_node_base_interface())

for value in range(5):
    message = Time()
    message.sec = 100 + value
    publisher.publish(message)
    for _ in range(5):
        executor.spin_some(cppyy.gbl.std.chrono.nanoseconds(20_000_000))
        time.sleep(0.02)

print("received:", received)
r.shutdown()
sys.exit(0 if received == [100, 101, 102, 103, 104] else 1)
PY

(
  cd "$workdir"
  unset PYTHONPATH
  pixi run python smoke.py
)
