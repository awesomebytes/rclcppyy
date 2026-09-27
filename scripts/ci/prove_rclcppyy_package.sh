#!/usr/bin/env bash

# Prove the built conda artifact from a throwaway workspace. Keeping the proof
# outside the repository workspace prevents source imports from hiding missing
# runtime dependencies in the package.
set -euo pipefail

case "$(uname -m)" in
  x86_64) platform="linux-64" ;;
  aarch64|arm64) platform="linux-aarch64" ;;
  *)
    echo "Unsupported package-proof architecture: $(uname -m)" >&2
    exit 2
    ;;
esac

output_dir="${1:-output}"
output_dir="$(realpath "$output_dir")"
published_support_proof="${2:-}"
if [ -n "$published_support_proof" ]; then
  published_support_proof="$(realpath "$published_support_proof")"
  export RCLCPPYY_PUBLISHED_SUPPORT_PROOF="$published_support_proof"
fi
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
import platform
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
from std_srvs.srv import SetBool

def assert_conda_version(package, expected):
    records = list(
        (Path(sys.prefix) / "conda-meta").glob(
            "%s-%s-*.json" % (package, expected)))
    assert len(records) == 1, (package, records)
    record = json.loads(records[0].read_text())
    assert (record["name"], record["version"]) == (package, expected), record
    return record


native_module = importlib.import_module("rclcpp_kit.native")
native_pipeline_module = importlib.import_module("rclcpp_kit.native_pipeline")
native_service_module = importlib.import_module("rclcpp_kit.native_service")
type_adapter_module = importlib.import_module("rclcpp_kit.type_adapter")
installed_records = {}
for package_name in ("cppyy-kit", "ros-jazzy-rclcpp-kit"):
    installed_records[package_name] = assert_conda_version(package_name, "0.2.0")
assert installed_records["cppyy-kit"]["build_number"] == 2
for package_name, package_version in (
    ("gcc", "14.3.0"),
    ("gxx", "14.3.0"),
    ("libgcc", "15.2.0"),
    ("libstdcxx", "15.2.0"),
):
    installed_records[package_name] = assert_conda_version(
        package_name, package_version)
product_record = assert_conda_version("ros-jazzy-rclcppyy", "0.3.0")
assert product_record["build_number"] == 1, product_record
product_dependencies = {item.split()[0] for item in product_record["depends"]}
expected_runtime_dependencies = {
    "ros-jazzy-action-msgs",
    "ros-jazzy-builtin-interfaces",
    "ros-jazzy-geometry-msgs",
    "ros-jazzy-rcl-interfaces",
    "ros-jazzy-rosidl-pycommon",
    "ros-jazzy-rosidl-runtime-py",
    "ros-jazzy-std-msgs",
    "ros-jazzy-std-srvs",
    "ros-jazzy-tf2-msgs",
}
assert expected_runtime_dependencies <= product_dependencies, (
    expected_runtime_dependencies - product_dependencies, product_record)
cppyy_record = assert_conda_version("cppyy", "3.5.0")
installed_records["cppyy"] = cppyy_record
published_proof_path = os.environ.get("RCLCPPYY_PUBLISHED_SUPPORT_PROOF")
if published_proof_path:
    published_proof = json.loads(Path(published_proof_path).read_text())
    assert published_proof["schema"] == "rclcppyy.published-support-proof/v2"
    for package in published_proof["packages"]:
        record = installed_records[package["name"]]
        published = package["published_artifact"]
        assert record["version"] == package["version"], (record, package)
        assert record["build"] == package["build"], (record, package)
        assert record["subdir"] == package["subdir"], (record, package)
        assert record["sha256"] == published["sha256"], (record, package)
        assert record["url"].startswith("file://"), record
        assert record["url"].endswith("/" + package["filename"]), record
    print("INSTALLED_PUBLISHED_SUPPORT_BYTES_OK")
if sys.platform == "linux" and platform.machine() in ("aarch64", "arm64"):
    assert cppyy_record["subdir"] == "linux-aarch64", cppyy_record
    assert cppyy_record["build"].startswith("py312"), cppyy_record
    assert cppyy_record["url"].startswith("file://"), cppyy_record
    if published_proof_path:
        print("INSTALLED_PUBLISHED_CPPYY_ARM_BRIDGE_OK")
    else:
        print("INSTALLED_LOCAL_CPPYY_ARM_BRIDGE_OK")
print("rclcppyy:", rclcppyy.__file__)
print("borrowed_publish:", borrowed_publish.__file__)
print("native:", native_module.__file__)
print("native_pipeline:", native_pipeline_module.__file__)
print("native_service:", native_service_module.__file__)
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

service_name = "/rclcppyy_package_proof/native_set_bool"
with native_module.native(["rclcppyy-package-native-service"]) as native_ros:
    service_node = native_ros.create_node("installed_native_service")
    service_executor = native_ros.create_executor()
    service_executor.add_node(service_node)
    native_service = native_ros.create_native_service(
        service_node,
        SetBool,
        service_name,
        'response->success = request->data; '
        'response->message = request->data ? "enabled" : "disabled";',
    )
    client = node.create_client(SetBool, service_name)
    deadline = time.monotonic() + 10.0
    while not client.service_is_ready() and time.monotonic() < deadline:
        service_executor.spin_some()
        executor.spin_once(timeout_sec=0.02)
    assert client.service_is_ready()

    future = client.call_async(SetBool.Request(data=True))
    deadline = time.monotonic() + 10.0
    while not future.done() and time.monotonic() < deadline:
        service_executor.spin_some()
        executor.spin_once(timeout_sec=0.02)
    assert future.done()
    response = future.result()
    assert response.success is True
    assert response.message == "enabled"
    service_stats = native_service.stats()
    assert service_stats.requests == 1
    assert service_stats.exceptions == 0
    assert service_stats.python_boundary_crossings == 0
    print("INSTALLED_NATIVE_SERVICE_OK")

assert native_service.closed
assert node.destroy_client(client)

status = rclcppyy.status()
entity_backends = {
    record["metadata"].get("entity_type"): record["backend"]
    for record in status["entities"]
}
assert entity_backends["publisher"] == "python", status
assert entity_backends["subscription"] == "python", status

assert node.destroy_publisher(publisher)
assert node.destroy_subscription(subscription)
executor.remove_node(node)
node.destroy_node()
executor.shutdown(timeout_sec=1.0)
context.shutdown()
print("INSTALLED_RCLCPPYY_COMPATIBLE_STOCK_PUBLISH_OK")
PY

cat >"$workdir/publisher_cpp_smoke.py" <<'PY'
import os
import time

import rclcppyy

rclcppyy.enable_cpp_acceleration(profile="publisher_cpp")

import rclpy  # noqa: E402
from rclpy.context import Context  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from std_msgs.msg import String  # noqa: E402


context = Context()
context.init(args=[])
node = rclpy.create_node(
    "installed_publisher_cpp_%d" % os.getpid(), context=context)
executor = SingleThreadedExecutor(context=context)
executor.add_node(node)
received = []
topic = "/rclcppyy_package_proof/publisher_cpp"
subscription = node.create_subscription(
    String, topic, lambda message: received.append(message.data), 10)
publisher = node.create_publisher(String, topic, 10)

deadline = time.monotonic() + 10.0
while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
    executor.spin_once(timeout_sec=0.05)
assert publisher.get_subscription_count() >= 1
publisher.publish(String(data="installed-publisher-cpp"))
while not received and time.monotonic() < deadline:
    executor.spin_once(timeout_sec=0.05)
assert received == ["installed-publisher-cpp"], received
assert publisher._rclcppyy_last_publish_backend == "cpp"
assert publisher._rclcppyy_publish_tainted is False

status = rclcppyy.status()
assert any(
    record["backend"] == "cpp"
    and record["metadata"].get("entity_type") == "publisher"
    and record["metadata"].get("topic") == topic
    for record in status["entities"]
), status
assert any(
    record["backend"] == "cpp"
    and record["metadata"].get("operation") == "publish"
    and record["metadata"].get("topic") == topic
    for record in status["operations"]
), status

assert node.destroy_publisher(publisher)
assert node.destroy_subscription(subscription)
executor.remove_node(node)
assert executor.shutdown(timeout_sec=1.0)
node.destroy_node()
context.shutdown()
print("INSTALLED_RCLCPPYY_PUBLISHER_CPP_OK")
PY

cat >"$workdir/direct_cpp_smoke.py" <<'PY'
import importlib
import os
import time

import rclcppyy

# Activation patches guarded rclpy imports process-globally, so this proof runs
# in its own process and activates before importing rclpy or generated types.
rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402


def cpp_name(value):
    return str(
        getattr(type(value), "__cpp_name__", "")
        or getattr(value, "__cpp_name__", "")
    )


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("direct_cpp crossed a Python message boundary")


bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
serialization = importlib.import_module("rclcpp_kit.serialization")
bringup.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary

assert String is cppyy.gbl.std_msgs.msg.String
rclpy.init(args=[])
node = Node("installed_direct_cpp_%d" % os.getpid())
executor = SingleThreadedExecutor()
assert executor.add_node(node)
runtime = importlib.import_module("rclcppyy.direct_cpp")._runtime()
assert runtime.nodes == [node]
assert runtime.session.nodes == (node._direct_cpp_node,)

topic = "/rclcppyy_package_proof/direct_cpp"
received = []
publisher = node.create_publisher(String, topic, 10)
subscription = node.create_subscription(
    String, topic, lambda message: received.append(message), 10)
assert "rclcpp::Publisher" in cpp_name(publisher.native_entity)
assert "rclcpp::Subscription" in cpp_name(subscription.native_entity)

deadline = time.monotonic() + 10.0
while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
    executor.spin_once(timeout_sec=0.05)
assert publisher.get_subscription_count() == 1

publisher.publish(String(data="installed-direct-cpp"))
deadline = time.monotonic() + 10.0
while not received and time.monotonic() < deadline:
    executor.spin_once(timeout_sec=0.05)
assert len(received) == 1, received
assert type(received[0]) is String
assert str(received[0].data) == "installed-direct-cpp"
assert node._direct_cpp_node is runtime.session.nodes[0]

assert node.destroy_publisher(publisher)
assert node.destroy_subscription(subscription)
executor.remove_node(node)
assert executor.shutdown(timeout_sec=1.0)
node.destroy_node()
rclpy.shutdown()
print("INSTALLED_RCLCPPYY_DIRECT_CPP_OK")
PY

(
  cd "$workdir"
  unset PYTHONPATH
  pixi run python smoke.py
  pixi run python publisher_cpp_smoke.py
  pixi run python direct_cpp_smoke.py
)
