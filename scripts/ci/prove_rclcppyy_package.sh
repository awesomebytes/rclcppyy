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

[activation.env]
LD_LIBRARY_PATH = "\$CONDA_PREFIX/lib"
RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp"
ROS_AUTOMATIC_DISCOVERY_RANGE = "LOCALHOST"
ROS_DOMAIN_ID = "62"
CPPYY_KIT_NO_AUTOPCH = "1"

[dependencies]
ros-jazzy-rclcppyy = "*"
ros-jazzy-rmw-cyclonedds-cpp = "*"
EOF

cat >"$workdir/smoke.py" <<'PY'
import importlib
import json
import os
from pathlib import Path
import sys
import time

import rclcppyy
import rclpy
from rcl_interfaces.msg import ParameterEvent
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclcpp_kit import borrowed_publish

def assert_conda_version(package, expected):
    records = list(
        (Path(sys.prefix) / "conda-meta").glob(
            "%s-%s-*.json" % (package, expected)))
    assert len(records) == 1, (package, records)
    record = json.loads(records[0].read_text())
    assert (record["name"], record["version"]) == (package, expected), record


native_module = importlib.import_module("rclcpp_kit.native")
native_pipeline_module = importlib.import_module("rclcpp_kit.native_pipeline")
type_adapter_module = importlib.import_module("rclcpp_kit.type_adapter")
for package_name in (
        "cppyy-kit", "ros-jazzy-rclcpp-kit", "ros-jazzy-rclcppyy"):
    assert_conda_version(package_name, "0.2.0")
print("rclcppyy:", rclcppyy.__file__)
print("borrowed_publish:", borrowed_publish.__file__)
print("native:", native_module.__file__)
print("native_pipeline:", native_pipeline_module.__file__)
print("type_adapter:", type_adapter_module.__file__)
rclcppyy.enable_cpp_acceleration()

context = Context()
context.init(args=[])
node = rclpy.create_node(
    "installed_rclcppyy_%d" % os.getpid(),
    namespace="/rclcppyy_package_proof",
    context=context,
)
executor = SingleThreadedExecutor(context=context)
executor.add_node(node)
received = []
topic = "/rclcppyy_package_proof/parameter_events"
subscription = node.create_subscription(
    ParameterEvent, topic, lambda message: received.append(message.node), 10)
publisher = node.create_publisher(ParameterEvent, topic, 10)

assert type(node) is Node
assert type(publisher) is Publisher
assert node.context is context
assert not rclpy.ok(), "the default context must remain uninitialized"

deadline = time.monotonic() + 10.0
while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
    executor.spin_once(timeout_sec=0.05)
assert publisher.get_subscription_count() >= 1

payload = "/rclcppyy_package_proof/source"
publisher.publish(ParameterEvent(node=payload))
deadline = time.monotonic() + 10.0
while not received and time.monotonic() < deadline:
    executor.spin_once(timeout_sec=0.05)
assert received == [payload], received

identity = (node.get_name(), node.get_namespace())
assert node.get_node_names_and_namespaces().count(identity) == 1
assert [(item.node_name, item.node_namespace)
        for item in node.get_publishers_info_by_topic(topic)] == [identity]

status = rclcppyy.status()
entity_backends = {
    record["metadata"].get("entity_type"): record["backend"]
    for record in status["entities"]
}
assert entity_backends["publisher"] == "cpp", status
assert entity_backends["subscription"] == "python", status

assert node.destroy_publisher(publisher)
assert node.destroy_subscription(subscription)
executor.remove_node(node)
node.destroy_node()
executor.shutdown(timeout_sec=1.0)
context.shutdown()
print("INSTALLED_RCLCPPYY_SAME_HANDLE_SERIALIZED_PUBLISH_OK")
PY

(
  cd "$workdir"
  unset PYTHONPATH
  pixi run python smoke.py
)
