"""Strict evidence contract for the controlled ROS relay-boundary benchmark."""

from __future__ import annotations

import datetime
import json
import math
from pathlib import Path
import re
import statistics
import sys

from _result_schema import environment_metadata


SCHEMA_ID = "rclcppyy.relay-boundary-benchmark/v1"
SAMPLE_SCHEMA = "rclcppyy.relay-boundary-sample/v1"
PREWARM_SCHEMA = "rclcppyy.relay-boundary-prewarm/v1"
RELAY_SCHEMA = "rclcppyy.relay-boundary-relay-event/v1"
DRIVER_SCHEMA = "rclcppyy.relay-boundary-driver-event/v1"
VARIANTS = {
    "stock-rclpy": {
        "execution_model": "same-python-relay-stock-rclpy",
        "cache_kind": "stock-rclpy",
        "python_crossings": True,
    },
    "compatible-rclcppyy": {
        "execution_model": "same-python-relay-compatible-stock-publish",
        "cache_kind": "compatible-stock-publish-authority",
        "python_crossings": True,
    },
    "publisher-cpp-rclcppyy": {
        "execution_model": "same-python-relay-explicit-publisher-cpp",
        "cache_kind": "publisher-cpp-borrowed-publish-route",
        "python_crossings": True,
    },
    "direct-cpp-rclcppyy": {
        "execution_model": "same-python-relay-direct-rclcpp",
        "cache_kind": "direct-cpp-subscription-trampoline",
        "python_crossings": True,
    },
    "direct-lease-rclcppyy": {
        "execution_model": "same-python-relay-direct-rclcpp-shared-lease",
        "cache_kind": "direct-cpp-subscription-lease",
        "python_crossings": True,
    },
    "native-python-callback": {
        "execution_model": "native-rclcpp-python-transform-callback",
        "cache_kind": "subscription-trampoline",
        "python_crossings": True,
    },
    "native-fused": {
        "execution_model": "native-content-addressed-fused-cpp-relay",
        "cache_kind": "fused-pipeline-shared-library",
        "python_crossings": False,
    },
    "aot-staged": {
        "execution_model": "conventional-release-aot-staged-relay",
        "cache_kind": "aot-binary",
        "python_crossings": False,
    },
}
QOS = {
    "history": "keep_last",
    "depth": 1,
    "reliability": "reliable",
    "durability": "volatile",
}
ENDPOINT_ROLES = (
    "driver_publisher",
    "relay_subscription",
    "relay_publisher",
    "driver_subscription",
)
METRICS = (
    "relay_cpu_ns_per_message",
    "latency_p50_ns",
    "latency_p95_ns",
    "latency_p99_ns",
    "latency_max_ns",
    "throughput_messages_per_second",
)
RSS_GUARD_LIMIT_BYTES = 64 * 1024 * 1024
SHA256_PATTERN = re.compile(r"^[a-f0-9]{64}$")
COMMIT_PATTERN = re.compile(r"^[a-f0-9]{40}$")


def _is_int(value) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _is_nonnegative_int(value) -> bool:
    return _is_int(value) and value >= 0


def _is_positive_int(value) -> bool:
    return _is_int(value) and value > 0


def _is_number(value) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def _is_sha256(value) -> bool:
    return isinstance(value, str) and SHA256_PATTERN.fullmatch(value) is not None


def nearest_rank(values: list[int], percentile: int) -> int:
    if not values:
        raise ValueError("latency observations cannot be empty")
    ordered = sorted(values)
    rank = max(1, math.ceil(percentile / 100.0 * len(ordered)))
    return ordered[rank - 1]


def latency_summary(values: list[int]) -> dict[str, int]:
    return {
        "p50": nearest_rank(values, 50),
        "p95": nearest_rank(values, 95),
        "p99": nearest_rank(values, 99),
        "max": max(values),
    }


def expected_input_checksum(total_messages: int) -> int:
    return total_messages * (total_messages + 1) // 2


def transformed(value: int) -> int:
    return value * 2 + 1


def expected_output_checksum(warmup_messages: int, messages: int) -> int:
    return sum(
        transformed(sequence)
        for sequence in range(warmup_messages + 1, warmup_messages + messages + 1)
    )


def _validate_artifact(artifact: dict, *, hit: bool | None = None) -> None:
    if not isinstance(artifact, dict):
        raise ValueError("cache artifact evidence is required")
    if hit is not None and artifact.get("cached") is not hit:
        raise ValueError("cache artifact hit state is invalid")
    if not isinstance(artifact.get("path"), str) or not artifact["path"]:
        raise ValueError("cache artifact path is required")
    if not _is_sha256(artifact.get("sha256")):
        raise ValueError("cache artifact SHA-256 is invalid")
    if not _is_positive_int(artifact.get("size_bytes")):
        raise ValueError("cache artifact size is invalid")


def validate_prewarm(document: dict, *, expect_hits: bool) -> None:
    if not isinstance(document, dict) or document.get("schema") != PREWARM_SCHEMA:
        raise ValueError("unsupported relay prewarm schema")
    if not _is_positive_int(document.get("pid")):
        raise ValueError("relay prewarm PID is invalid")
    if not isinstance(document.get("loaded_rmw"), str) or not document["loaded_rmw"]:
        raise ValueError("relay prewarm RMW evidence is required")
    source_id = document.get("fused_source_id")
    if not isinstance(source_id, str) or not re.fullmatch(r"[a-f0-9]{16}", source_id):
        raise ValueError("relay prewarm fused source id is invalid")
    lease_source_id = document.get("lease_source_id")
    if not isinstance(lease_source_id, str) or not re.fullmatch(
            r"[a-f0-9]{16}", lease_source_id):
        raise ValueError("relay prewarm lease source id is invalid")
    artifacts = document.get("artifacts")
    if not isinstance(artifacts, dict) or set(artifacts) != {
            "subscription", "subscription_lease", "fused_pipeline"}:
        raise ValueError("relay prewarm artifact matrix is incomplete")
    for artifact in artifacts.values():
        _validate_artifact(artifact, hit=expect_hits)
    diagnostics = document.get("stdout_diagnostics", [])
    if not isinstance(diagnostics, list) or any(
            not isinstance(line, str) for line in diagnostics):
        raise ValueError("relay prewarm stdout diagnostics are invalid")


def validate_cache_manifest(manifest: dict, requested_rmw: str) -> None:
    if not isinstance(manifest, dict):
        raise ValueError("relay cache manifest is required")
    required_true = (
        "isolated_root", "autopch_disabled", "fresh_process_per_phase",
        "compilation_excluded_from_samples", "warm_hits_verified",
    )
    if any(manifest.get(field) is not True for field in required_true):
        raise ValueError("relay cache policy evidence is incomplete")
    if manifest.get("persisted_after_run") is not False:
        raise ValueError("relay benchmark cache must be private and temporary")
    phases = manifest.get("phases")
    if not isinstance(phases, dict) or set(phases) != {"cold", "warm"}:
        raise ValueError("relay cache requires cold and warm phases")
    validate_prewarm(phases["cold"], expect_hits=False)
    validate_prewarm(phases["warm"], expect_hits=True)
    if any(phase.get("loaded_rmw") != requested_rmw for phase in phases.values()):
        raise ValueError("relay cache prewarm used the wrong RMW")
    for name in ("subscription", "subscription_lease", "fused_pipeline"):
        cold = phases["cold"]["artifacts"][name]
        warm = phases["warm"]["artifacts"][name]
        if (cold["path"], cold["sha256"], cold["size_bytes"]) != (
                warm["path"], warm["sha256"], warm["size_bytes"]):
            raise ValueError("cold and warm phases selected different cache artifacts")
    if phases["cold"]["fused_source_id"] != phases["warm"]["fused_source_id"]:
        raise ValueError("fused source identity changed between cache phases")
    if phases["cold"]["lease_source_id"] != phases["warm"]["lease_source_id"]:
        raise ValueError("subscription lease source identity changed between cache phases")


def _validate_source_environment(environment: dict) -> None:
    if not isinstance(environment, dict):
        raise ValueError("relay benchmark environment evidence is required")
    source = environment.get("source")
    if not isinstance(source, dict) or not isinstance(source.get("commit"), str) or not (
            COMMIT_PATTERN.fullmatch(source["commit"])):
        raise ValueError("relay benchmark product source commit is required")
    if not isinstance(source.get("dirty"), bool):
        raise ValueError("relay benchmark product dirty state is required")
    dependencies = environment.get("source_dependencies")
    suite = dependencies.get("rclcpp_kit") if isinstance(dependencies, dict) else None
    if not isinstance(suite, dict) or not isinstance(suite.get("commit"), str) or not (
            COMMIT_PATTERN.fullmatch(suite["commit"])):
        raise ValueError("relay benchmark supporting-suite source commit is required")
    if not isinstance(suite.get("dirty"), bool):
        raise ValueError("relay benchmark supporting-suite dirty state is required")


def _validate_build(build: dict) -> None:
    if not isinstance(build, dict):
        raise ValueError("relay AOT build evidence is required")
    if build.get("build_type") != "Release" or build.get("private_build_directory") is not True:
        raise ValueError("relay AOT reference must use a private Release build")
    if build.get("build_directory_persisted") is not False:
        raise ValueError("relay AOT build directory must be temporary")
    if build.get("executable_format") != "ELF":
        raise ValueError("relay AOT reference must be an ELF executable")
    if not isinstance(build.get("compiler"), str) or not build["compiler"]:
        raise ValueError("relay AOT compiler is required")
    if not isinstance(build.get("compiler_version"), str) or not build["compiler_version"]:
        raise ValueError("relay AOT compiler version is required")
    command = build.get("compile_command")
    if not isinstance(command, str) or "-O3" not in command or "-DNDEBUG" not in command:
        raise ValueError("relay AOT compile command must prove Release optimization")
    for field in (
            "source_sha256", "kernel_sha256", "cmake_sha256",
            "compile_commands_sha256", "executable_sha256"):
        if not _is_sha256(build.get(field)):
            raise ValueError("relay AOT %s is invalid" % field)
    if not _is_positive_int(build.get("build_elapsed_ns")):
        raise ValueError("relay AOT build timing is invalid")


def _validate_ready(ready: dict, sample: dict, requested_rmw: str, build: dict, cache: dict) -> None:
    variant = sample["variant"]
    spec = VARIANTS[variant]
    if not isinstance(ready, dict) or ready.get("schema") != RELAY_SCHEMA or ready.get(
            "event") != "ready":
        raise ValueError("relay ready evidence is invalid")
    if ready.get("variant") != variant or ready.get("run_token") != sample["run_token"]:
        raise ValueError("relay ready identity is invalid")
    if ready.get("pid") != sample["relay_pid"] or ready.get(
            "process_group_id") != sample["relay_pid"]:
        raise ValueError("relay ready process-group evidence is invalid")
    if ready.get("node_name") != sample["topology"]["relay_node"]:
        raise ValueError("relay ready node identity is invalid")
    if ready.get("loaded_rmw") != requested_rmw:
        raise ValueError("relay loaded the wrong RMW")
    if ready.get("execution_model") != spec["execution_model"]:
        raise ValueError("relay execution model is invalid")
    artifact = ready.get("cache")
    if not isinstance(artifact, dict) or artifact.get("kind") != spec["cache_kind"]:
        raise ValueError("relay cache route is invalid")
    if variant == "aot-staged":
        if artifact != {"state": "prebuilt", "kind": "aot-binary"}:
            raise ValueError("AOT relay cache evidence is invalid")
    elif variant == "stock-rclpy":
        if artifact != {"state": "not_applicable", "kind": "stock-rclpy"}:
            raise ValueError("stock relay cache evidence is invalid")
    elif variant == "compatible-rclcppyy":
        if artifact != {
                "state": "not_applicable",
                "kind": "compatible-stock-publish-authority"}:
            raise ValueError("compatible relay stock-publish evidence is invalid")
    elif variant == "publisher-cpp-rclcppyy":
        if artifact != {
                "state": "process-warm",
                "kind": "publisher-cpp-borrowed-publish-route",
                "prepared_before_measurement": True}:
            raise ValueError("publisher_cpp relay warm-route evidence is invalid")
    elif variant in ("direct-cpp-rclcppyy", "direct-lease-rclcppyy"):
        if artifact.get("state") != "prebuilt" or artifact.get("hit") is not True:
            raise ValueError("direct_cpp relay cache state is invalid")
        if not _is_sha256(artifact.get("sha256")):
            raise ValueError("direct_cpp relay cache identity is invalid")
        artifact_name = (
            "subscription_lease"
            if variant == "direct-lease-rclcppyy"
            else "subscription"
        )
        expected = cache["phases"]["warm"]["artifacts"][artifact_name]
        if (artifact.get("path"), artifact.get("sha256"), artifact.get("size_bytes")) != (
                expected["path"], expected["sha256"], expected["size_bytes"]):
            raise ValueError("direct_cpp cache differs from the warm manifest")
        if variant == "direct-lease-rclcppyy" and artifact.get(
                "source_id") != cache["phases"]["warm"]["lease_source_id"]:
            raise ValueError("direct_cpp lease source differs from the warm manifest")
    else:
        if artifact.get("state") != "prebuilt":
            raise ValueError("dynamic native relay cache state is invalid")
        if artifact.get("hit") is not True or not _is_sha256(artifact.get("sha256")):
            raise ValueError("dynamic relay must prove a generated-cache hit")
        manifest_name = "subscription" if variant == "native-python-callback" else "fused_pipeline"
        expected = cache["phases"]["warm"]["artifacts"][manifest_name]
        if (artifact.get("path"), artifact.get("sha256"), artifact.get("size_bytes")) != (
                expected["path"], expected["sha256"], expected["size_bytes"]):
            raise ValueError("relay cache artifact differs from the warm manifest")
        if variant == "native-fused" and artifact.get("source_id") != cache[
                "phases"]["warm"]["fused_source_id"]:
            raise ValueError("fused relay source id differs from the warm manifest")
    entity_types = ready.get("entity_types")
    if variant in (
            "stock-rclpy", "compatible-rclcppyy", "publisher-cpp-rclcppyy",
            "direct-cpp-rclcppyy", "direct-lease-rclcppyy"):
        if variant in ("direct-cpp-rclcppyy", "direct-lease-rclcppyy"):
            if not isinstance(entity_types, dict) or set(entity_types) != {
                    "node", "publisher", "subscription", "executor"} or any(
                    "rclcpp" not in value for value in entity_types.values()):
                raise ValueError("direct_cpp concrete C++ entity identities are invalid")
            proof = ready.get("direct_cpp_proof")
            lease = variant == "direct-lease-rclcppyy"
            expected_proof = {
                "actual_cpp_message_class": True,
                "message_cpp_name": "std_msgs::msg::UInt64_<std::allocator<void>>",
                "single_native_node_authority": True,
                "session_node_count": 1,
                "callback_handoff": (
                    "shared_cpp_message_lease" if lease else "one_native_cpp_copy"),
                "subscription_creation_route": (
                    "rclcpp_unique_ptr_subscription_lease"
                    if lease else "prebuilt_subscription_trampoline"),
                "python_message_conversion_guarded": True,
                "serialization_guarded": True,
            }
            if proof != expected_proof:
                raise ValueError("direct_cpp representation or authority proof is invalid")
        elif entity_types != {
                "node": "rclpy.node.Node",
                "publisher": "rclpy.publisher.Publisher",
                "subscription": "rclpy.subscription.Subscription",
                "executor": "rclpy.executors.SingleThreadedExecutor"}:
            raise ValueError("Python relay concrete entity identities are invalid")
        markers = ready.get("backend_markers")
        if not isinstance(markers, dict) or set(markers) != {"publisher", "subscriber"}:
            raise ValueError("Python relay backend markers are incomplete")
        expected_backends = {
            "publisher": (
                "cpp" if variant in (
                    "publisher-cpp-rclcppyy", "direct-cpp-rclcppyy",
                    "direct-lease-rclcppyy")
                else "python"),
            "subscriber": (
                "cpp" if variant in (
                    "direct-cpp-rclcppyy", "direct-lease-rclcppyy") else "python"),
        }
        for role, marker in markers.items():
            if not isinstance(marker, dict) or marker.get(
                    "schema") != "rclcppyy.benchmark-backend/v1" or marker.get("role") != role:
                raise ValueError("Python relay backend marker schema or role is invalid")
            if marker.get("backend") != expected_backends[role]:
                raise ValueError("Python relay backend marker selected the wrong route")
            expected_evidence = (
                "stock_rclpy_entity" if variant == "stock-rclpy"
                else "rclcppyy_status_entity"
            )
            if marker.get("evidence") != expected_evidence or not isinstance(
                    marker.get("metadata"), dict):
                raise ValueError("Python relay backend marker evidence is invalid")
        if variant in ("direct-cpp-rclcppyy", "direct-lease-rclcppyy"):
            publisher_metadata = markers["publisher"]["metadata"]
            subscriber_metadata = markers["subscriber"]["metadata"]
            if "no_conversion" not in publisher_metadata.get("policies", ()) or (
                    "no_conversion" not in subscriber_metadata.get("policies", ())):
                raise ValueError("direct_cpp no-conversion status evidence is invalid")
            expected_handoff = (
                "shared_cpp_message_lease"
                if variant == "direct-lease-rclcppyy"
                else "one_native_cpp_copy"
            )
            required_policy = (
                "subscription_shared_lease"
                if variant == "direct-lease-rclcppyy"
                else "owning_cpp_callback_copy"
            )
            if subscriber_metadata.get("callback_handoff") != expected_handoff or (
                    required_policy not in subscriber_metadata.get("policies", ())):
                raise ValueError("direct_cpp callback handoff status evidence is invalid")
    elif variant != "aot-staged":
        if not isinstance(entity_types, dict) or any(
                "rclcpp" not in value for value in entity_types.values()):
            raise ValueError("dynamic relay concrete rclcpp type evidence is incomplete")


def _validate_report(report: dict, sample: dict, warmup: int, messages: int) -> None:
    variant = sample["variant"]
    total = warmup + messages
    if not isinstance(report, dict) or report.get("schema") != RELAY_SCHEMA or report.get(
            "event") != "report":
        raise ValueError("relay report evidence is invalid")
    if report.get("variant") != variant or report.get("run_token") != sample["run_token"]:
        raise ValueError("relay report identity is invalid")
    for field in ("received", "processed", "published"):
        if report.get(field) != total:
            raise ValueError("relay %s count is invalid" % field)
    if report.get("dropped") != 0 or report.get("exceptions") != 0:
        raise ValueError("relay reported drops or exceptions")
    if report.get("correct") is not True or report.get("teardown_clean") is not True:
        raise ValueError("relay correctness and teardown proof is incomplete")
    if not _is_nonnegative_int(report.get("cpu_time_ns")) or report.get(
            "cpu_clock") != "CLOCK_PROCESS_CPUTIME_ID":
        raise ValueError("relay process CPU timing evidence is invalid")
    _validate_rss_guard(report.get("rss_guard"), "relay")
    crossings = total if VARIANTS[variant]["python_crossings"] else 0
    if report.get("python_callback_count") != crossings or report.get(
            "python_boundary_crossings") != crossings:
        raise ValueError("relay Python-boundary count is invalid")
    if variant in (
            "stock-rclpy", "compatible-rclcppyy", "publisher-cpp-rclcppyy",
            "direct-cpp-rclcppyy", "direct-lease-rclcppyy",
            "native-python-callback", "aot-staged"):
        if report.get("checksum") != expected_input_checksum(total) or report.get("last") != total:
            raise ValueError("relay input checksum evidence is invalid")
    if variant in ("stock-rclpy", "compatible-rclcppyy"):
        if report.get("publish_operation_marker") is not None or report.get(
                "fallback_publish_operations") != 0 or report.get(
                "last_publish_backend") != "python":
            raise ValueError("stock-authority relay publish-route evidence is invalid")
    if variant == "publisher-cpp-rclcppyy":
        marker = report.get("publish_operation_marker")
        if not isinstance(marker, dict) or marker.get(
                "schema") != "rclcppyy.benchmark-backend/v1" or marker.get(
                "role") != "publisher" or marker.get("backend") != "cpp" or marker.get(
                "evidence") != "rclcppyy_status_operation":
            raise ValueError("publisher_cpp relay completed-publish marker is invalid")
        if report.get("fallback_publish_operations") != 0 or report.get(
                "last_publish_backend") != "cpp" or report.get(
                    "publish_route_tainted") is not False:
            raise ValueError("publisher_cpp relay publish route was tainted or fell back")
        counts = report.get("status_operation_counts")
        if not isinstance(counts, dict) or not _is_positive_int(counts.get("cpp")):
            raise ValueError("publisher_cpp relay operation aggregates are invalid")
        if not _is_nonnegative_int(report.get("status_dropped_operation_records")):
            raise ValueError("publisher_cpp relay dropped-status evidence is invalid")
    if variant == "direct-cpp-rclcppyy":
        if report.get("owning_cpp_callback_copies") != total or report.get(
                "cpp_callback_messages") != total or report.get(
                "non_cpp_callback_messages") != 0:
            raise ValueError("direct_cpp callback C++ copy evidence is invalid")
        if report.get("python_message_conversions") != 0 or report.get(
                "serialization_operations") != 0 or report.get(
                "boundary_guard_calls") != {
                    "python_message_conversion": 0,
                    "serialization": 0,
                }:
            raise ValueError("direct_cpp conversion or serialization evidence is invalid")
        if report.get("publish_operation_marker") is not None or report.get(
                "fallback_publish_operations") != 0 or report.get(
                "last_publish_backend") != "cpp" or report.get(
                "publish_route_tainted") is not False:
            raise ValueError("direct_cpp publish-route evidence is invalid")
    if variant == "direct-lease-rclcppyy":
        if report.get("owning_cpp_callback_copies") != 0 or report.get(
                "subscription_leases") != total or report.get(
                "shared_control_blocks") != total or report.get(
                "shared_owner_acquisitions") != total or report.get(
                "lease_python_boundary_crossings") != total or report.get(
                "lease_exceptions") != 0:
            raise ValueError("direct_cpp shared-lease counter evidence is invalid")
        if report.get("cpp_callback_messages") != total or report.get(
                "non_cpp_callback_messages") != 0:
            raise ValueError("direct_cpp lease C++ message evidence is invalid")
        native_address = report.get("native_last_message_address")
        callback_address = report.get("callback_last_message_address")
        if not _is_positive_int(native_address) or callback_address != native_address:
            raise ValueError("direct_cpp lease native-address evidence is invalid")
        if report.get("python_message_conversions") != 0 or report.get(
                "serialization_operations") != 0 or report.get(
                "boundary_guard_calls") != {
                    "python_message_conversion": 0,
                    "serialization": 0,
                }:
            raise ValueError("direct_cpp conversion or serialization evidence is invalid")
        if report.get("publish_operation_marker") is not None or report.get(
                "fallback_publish_operations") != 0 or report.get(
                "last_publish_backend") != "cpp" or report.get(
                "publish_route_tainted") is not False:
            raise ValueError("direct_cpp lease publish-route evidence is invalid")
    if variant == "native-fused" and (
            report.get("compile_cache_hits") != 1 or report.get("compile_cache_misses") != 0):
        raise ValueError("fused relay compile-cache counters are invalid")


def _validate_armed(armed: dict, sample: dict) -> None:
    if not isinstance(armed, dict) or armed.get("schema") != RELAY_SCHEMA or armed.get(
            "event") != "armed":
        raise ValueError("relay armed evidence is invalid")
    if armed.get("variant") != sample["variant"] or armed.get(
            "run_token") != sample["run_token"]:
        raise ValueError("relay armed identity is invalid")
    if armed.get("pid") != sample["relay_pid"] or armed.get(
            "process_group_id") != sample["relay_pid"]:
        raise ValueError("relay armed process-group evidence is invalid")
    if armed.get("cpu_clock") != "CLOCK_PROCESS_CPUTIME_ID":
        raise ValueError("relay armed CPU clock is invalid")


def _validate_driver(driver: dict, sample: dict, requested_rmw: str, warmup: int, messages: int) -> None:
    if not isinstance(driver, dict) or driver.get("schema") != DRIVER_SCHEMA or driver.get(
            "event") != "measured":
        raise ValueError("AOT driver measured evidence is invalid")
    if driver.get("run_token") != sample["run_token"] or driver.get("pid") != sample["driver_pid"]:
        raise ValueError("AOT driver identity is invalid")
    if driver.get("process_group_id") != sample["driver_pid"]:
        raise ValueError("AOT driver process-group evidence is invalid")
    if driver.get("loaded_rmw") != requested_rmw:
        raise ValueError("AOT driver loaded the wrong RMW")
    if driver.get("execution_model") != "identical-release-aot-closed-loop-driver":
        raise ValueError("AOT driver execution model is invalid")
    if driver.get("messages") != messages:
        raise ValueError("AOT driver message count is invalid")
    if driver.get("checksum") != expected_output_checksum(warmup, messages):
        raise ValueError("AOT driver checksum is invalid")
    if driver.get("last") != transformed(warmup + messages):
        raise ValueError("AOT driver final value is invalid")
    if any(driver.get(field) is not True for field in (
            "topology_verified", "qos_verified")):
        raise ValueError("AOT driver route evidence is incomplete")
    endpoints = driver.get("endpoints")
    if not isinstance(endpoints, list) or len(endpoints) != 4:
        raise ValueError("AOT driver must observe exactly four endpoints")
    expected_nodes = {
        "driver_publisher": sample["topology"]["driver_node"],
        "relay_subscription": sample["topology"]["relay_node"],
        "relay_publisher": sample["topology"]["relay_node"],
        "driver_subscription": sample["topology"]["driver_node"],
    }
    expected_topics = {
        "driver_publisher": sample["topology"]["input_topic"],
        "relay_subscription": sample["topology"]["input_topic"],
        "relay_publisher": sample["topology"]["output_topic"],
        "driver_subscription": sample["topology"]["output_topic"],
    }
    observed_roles = []
    for endpoint in endpoints:
        role = endpoint.get("role") if isinstance(endpoint, dict) else None
        observed_roles.append(role)
        if role not in ENDPOINT_ROLES or endpoint.get("node_name") != expected_nodes.get(role):
            raise ValueError("AOT driver endpoint ownership is invalid")
        observed_owner = endpoint.get("observed_node_name")
        ownership_evidence = endpoint.get("ownership_evidence")
        if observed_owner == expected_nodes[role]:
            if ownership_evidence != "middleware-graph-owner":
                raise ValueError("AOT driver exact owner evidence is invalid")
        elif role in ("relay_subscription", "relay_publisher") and (
                requested_rmw == "rmw_cyclonedds_cpp" and
                observed_owner == "_NODE_NAME_UNKNOWN_" and
                ownership_evidence == "exact-process-pair-unique-topic"):
            pass
        else:
            raise ValueError("AOT driver observed endpoint owner is invalid")
        if endpoint.get("topic") != expected_topics[role] or endpoint.get(
                "node_namespace") != "/":
            raise ValueError("AOT driver endpoint topic or namespace is invalid")
        observed_namespace = endpoint.get("observed_node_namespace")
        if ownership_evidence == "middleware-graph-owner":
            if observed_namespace != "/":
                raise ValueError("AOT driver exact endpoint namespace is invalid")
        elif observed_namespace != "_NODE_NAMESPACE_UNKNOWN_":
            raise ValueError("AOT driver unknown endpoint namespace evidence is invalid")
        if any(endpoint.get(name) != value for name, value in QOS.items()):
            raise ValueError("AOT driver endpoint QoS is invalid")
    if observed_roles != list(ENDPOINT_ROLES):
        raise ValueError("AOT driver endpoint order or role matrix is invalid")
    latencies = driver.get("latency_ns")
    if not isinstance(latencies, list) or len(latencies) != messages or any(
            not _is_positive_int(value) for value in latencies):
        raise ValueError("AOT driver raw latency evidence is invalid")
    if not _is_positive_int(driver.get("elapsed_ns")) or not _is_nonnegative_int(
            driver.get("cpu_time_ns")):
        raise ValueError("AOT driver timing evidence is invalid")
    if driver.get("cpu_clock") != "CLOCK_PROCESS_CPUTIME_ID":
        raise ValueError("AOT driver process CPU clock is invalid")
    _validate_rss_guard(driver.get("rss_guard"), "driver")


def _validate_driver_teardown(teardown: dict, sample: dict) -> None:
    if not isinstance(teardown, dict) or teardown.get("schema") != DRIVER_SCHEMA or teardown.get(
            "event") != "teardown":
        raise ValueError("AOT driver teardown evidence is invalid")
    if teardown.get("run_token") != sample["run_token"] or teardown.get(
            "pid") != sample["driver_pid"] or teardown.get(
                "process_group_id") != sample["driver_pid"]:
        raise ValueError("AOT driver teardown identity is invalid")
    if teardown.get("teardown_clean") is not True:
        raise ValueError("AOT driver teardown was not clean")


def _validate_rss_guard(guard: dict, owner: str) -> None:
    if not isinstance(guard, dict) or guard.get(
            "kind") != "post-warmup-peak-rss-growth" or guard.get("unit") != "bytes":
        raise ValueError("%s RSS guard evidence is invalid" % owner)
    baseline = guard.get("baseline_peak_bytes")
    final = guard.get("final_peak_bytes")
    growth = guard.get("growth_bytes")
    if not _is_positive_int(baseline) or not _is_positive_int(final) or final < baseline:
        raise ValueError("%s RSS guard observations are invalid" % owner)
    if growth != final - baseline or guard.get("limit_bytes") != RSS_GUARD_LIMIT_BYTES:
        raise ValueError("%s RSS guard calculation is invalid" % owner)
    if guard.get("within_limit") is not True or growth > RSS_GUARD_LIMIT_BYTES:
        raise ValueError("%s exceeded the bounded RSS growth guard" % owner)


def validate_sample(sample: dict, parameters: dict, build: dict, cache: dict) -> None:
    if not isinstance(sample, dict) or sample.get("schema") != SAMPLE_SCHEMA:
        raise ValueError("unsupported relay-boundary sample schema")
    variant = sample.get("variant")
    if variant not in VARIANTS:
        raise ValueError("unknown relay-boundary variant")
    if not _is_positive_int(sample.get("repetition")):
        raise ValueError("relay-boundary repetition is invalid")
    token = sample.get("run_token")
    if not isinstance(token, str) or not re.fullmatch(r"run_[a-f0-9]{32}", token):
        raise ValueError("relay-boundary run token is invalid")
    domain = sample.get("ros_domain_id")
    if not _is_int(domain) or not 0 <= domain <= 232:
        raise ValueError("relay-boundary ROS domain is invalid")
    relay_pid = sample.get("relay_pid")
    driver_pid = sample.get("driver_pid")
    if not _is_positive_int(relay_pid) or not _is_positive_int(driver_pid) or relay_pid == driver_pid:
        raise ValueError("relay-boundary requires two distinct process identities")
    topology = sample.get("topology")
    if not isinstance(topology, dict) or topology.get("process_count") != 2 or topology.get(
            "fresh_process_groups") is not True:
        raise ValueError("relay-boundary process topology is invalid")
    if not all(isinstance(topology.get(field), str) and topology[field] for field in (
            "driver_node", "relay_node", "input_topic", "output_topic")):
        raise ValueError("relay-boundary graph identity is incomplete")
    requested_rmw = parameters["requested_rmw"]
    if sample.get("requested_rmw") != requested_rmw or sample.get("qos") != QOS:
        raise ValueError("relay-boundary RMW or QoS selection is invalid")
    ready = sample.get("relay_ready")
    report = sample.get("relay_report")
    driver = sample.get("driver_result")
    _validate_ready(ready, sample, requested_rmw, build, cache)
    _validate_armed(sample.get("relay_armed"), sample)
    _validate_report(
        report, sample, parameters["warmup_messages"], parameters["messages"])
    _validate_driver(
        driver, sample, requested_rmw,
        parameters["warmup_messages"], parameters["messages"])
    _validate_driver_teardown(sample.get("driver_teardown"), sample)
    timing = sample.get("timing")
    if not isinstance(timing, dict):
        raise ValueError("relay-boundary timing evidence is required")
    messages = parameters["messages"]
    latency = latency_summary(driver["latency_ns"])
    if timing.get("latency_ns") != latency:
        raise ValueError("relay-boundary latency summary contradicts raw observations")
    relay_cpu = timing.get("relay_cpu_time_ns")
    if not _is_nonnegative_int(relay_cpu) or timing.get(
            "relay_cpu_ns_per_message") != relay_cpu / messages:
        raise ValueError("relay-boundary relay CPU evidence is invalid")
    driver_cpu = driver["cpu_time_ns"]
    if timing.get("driver_cpu_time_ns") != driver_cpu or timing.get(
            "driver_cpu_ns_per_message") != driver_cpu / messages:
        raise ValueError("relay-boundary driver CPU evidence is invalid")
    if timing.get("elapsed_ns") != driver["elapsed_ns"]:
        raise ValueError("relay-boundary elapsed timing is invalid")
    expected_throughput = messages * 1e9 / driver["elapsed_ns"]
    if not math.isclose(
            timing.get("throughput_messages_per_second", -1),
            expected_throughput, rel_tol=1e-12):
        raise ValueError("relay-boundary throughput is invalid")
    if sample.get("backend_verified") is not True or sample.get(
            "correctness_verified") is not True or sample.get("teardown_verified") is not True:
        raise ValueError("relay-boundary acceptance flags are incomplete")


def _metric(sample: dict, name: str) -> float:
    timing = sample["timing"]
    paths = {
        "relay_cpu_ns_per_message": timing["relay_cpu_ns_per_message"],
        "latency_p50_ns": timing["latency_ns"]["p50"],
        "latency_p95_ns": timing["latency_ns"]["p95"],
        "latency_p99_ns": timing["latency_ns"]["p99"],
        "latency_max_ns": timing["latency_ns"]["max"],
        "throughput_messages_per_second": timing["throughput_messages_per_second"],
    }
    return paths[name]


def summarize(results: list[dict], variants: list[str]) -> dict:
    medians = {}
    by_variant_repetition = {}
    for variant in variants:
        rows = sorted(
            (row for row in results if row.get("variant") == variant),
            key=lambda row: row["repetition"],
        )
        by_variant_repetition[variant] = {row["repetition"]: row for row in rows}
        medians[variant] = {
            metric: statistics.median([_metric(row, metric) for row in rows])
            for metric in METRICS
        } if rows else {}
    def paired_to(reference_variant):
        paired = {}
        reference = by_variant_repetition.get(reference_variant, {})
        if not reference:
            return paired
        for variant in variants:
            candidate = by_variant_repetition.get(variant, {})
            repetitions = sorted(set(candidate) & set(reference))
            paired[variant] = {
                metric: [
                    (
                        _metric(candidate[repetition], metric) /
                        _metric(reference[repetition], metric)
                        if _metric(reference[repetition], metric) != 0 else None
                    )
                    for repetition in repetitions
                ]
                for metric in METRICS
            }
        return paired
    return {
        "raw_medians": medians,
        "paired_ratios_to_stock": paired_to("stock-rclpy"),
        "paired_ratios_to_aot": paired_to("aot-staged"),
        "interpretation_allowed": False,
        "note": (
            "Stock, compatible, publisher_cpp, and direct_cpp use one Python relay implementation. "
            "Compatible adds default activation while preserving stock publish authority; "
            "publisher_cpp opts into same-handle C++ publishing; direct_cpp uses actual C++ messages "
            "and entities with either one owning native callback copy or one shared ownership lease "
            "over the received allocation. All ratios are "
            "descriptive: lower is better for CPU and latency, higher is better for throughput. No "
            "threshold, ranking, or winner is selected."
        ),
    }


def build_document(
        *, repo_root: Path, mode: str, parameters: dict, isolation: dict,
        aot_build: dict, cache: dict, source_files: dict, results: list,
        failures: list, command: list[str] | None = None) -> dict:
    document = {
        "schema": SCHEMA_ID,
        "generated_at": datetime.datetime.now(
            datetime.timezone.utc).isoformat().replace("+00:00", "Z"),
        "command": list(command if command is not None else sys.argv),
        "environment": environment_metadata(repo_root),
        "benchmark": {
            "name": "controlled_ros_relay_boundary",
            "mode": mode,
            "performance_claims_allowed": False,
            "parameters": parameters,
            "isolation": isolation,
            "aot_build": aot_build,
            "cache": cache,
            "source_files": source_files,
            "statistics": {
                "workload": "fixed-work closed-loop UInt64 relay with one message outstanding",
                "timed_region": "post-discovery and post-warmup fixed message window",
                "latency": "same-process AOT driver steady-clock observations; nearest-rank percentiles",
                "relay_cpu": "relay-owned CLOCK_PROCESS_CPUTIME_ID delta",
                "driver_cpu": "driver-owned CLOCK_PROCESS_CPUTIME_ID delta",
                "memory": "post-warmup peak-RSS growth is a bounded guard only; it is never ranked",
                "comparison": "paired by repetition with rotating execution order",
            },
            "scope": {
                "steady_state_only": True,
                "maximum_rate_throughput_claim": False,
                "cross_machine_claim": False,
                "general_rclcpp_advantage_claim": False,
            },
        },
        "results": list(results),
        "comparison": summarize(results, parameters["variants"]),
        "failures": list(failures),
    }
    validate_document(document)
    return document


def validate_document(document: dict) -> None:
    if not isinstance(document, dict) or document.get("schema") != SCHEMA_ID:
        raise ValueError("unsupported relay-boundary benchmark schema")
    if not isinstance(document.get("generated_at"), str):
        raise ValueError("relay-boundary generated_at is required")
    _validate_source_environment(document.get("environment"))
    benchmark = document.get("benchmark")
    if not isinstance(benchmark, dict) or benchmark.get("name") != "controlled_ros_relay_boundary":
        raise ValueError("relay-boundary benchmark metadata is invalid")
    if benchmark.get("mode") not in ("smoke", "measurement"):
        raise ValueError("relay-boundary mode is invalid")
    if benchmark.get("performance_claims_allowed") is not False:
        raise ValueError("raw relay-boundary evidence cannot allow performance claims")
    parameters = benchmark.get("parameters")
    if not isinstance(parameters, dict):
        raise ValueError("relay-boundary parameters are required")
    variants = parameters.get("variants")
    if not isinstance(variants, list) or not variants or any(
            variant not in VARIANTS for variant in variants) or len(set(variants)) != len(variants):
        raise ValueError("relay-boundary variant matrix is invalid")
    if not _is_positive_int(parameters.get("messages")) or not _is_positive_int(
            parameters.get("warmup_messages")) or not _is_positive_int(
            parameters.get("repetitions")):
        raise ValueError("relay-boundary message/repetition counts are invalid")
    requested_rmw = parameters.get("requested_rmw")
    if not isinstance(requested_rmw, str) or not requested_rmw or parameters.get("qos") != QOS:
        raise ValueError("relay-boundary RMW and QoS parameters are invalid")
    _validate_build(benchmark.get("aot_build"))
    validate_cache_manifest(benchmark.get("cache"), requested_rmw)
    source_files = benchmark.get("source_files")
    if not isinstance(source_files, dict) or not source_files or any(
            not _is_sha256(value) for value in source_files.values()):
        raise ValueError("relay-boundary source-file identities are incomplete")
    isolation = benchmark.get("isolation")
    required_isolation = (
        "fresh_process_pair_per_sample", "two_process_groups_per_sample",
        "unique_topics_per_sample", "one_leased_domain_per_run",
        "rotating_variant_order",
    )
    if not isinstance(isolation, dict) or any(
            isolation.get(field) is not True for field in required_isolation):
        raise ValueError("relay-boundary isolation evidence is incomplete")
    domain = isolation.get("ros_domain_id")
    if not _is_int(domain) or not 0 <= domain <= 232:
        raise ValueError("relay-boundary leased domain is invalid")
    results = document.get("results")
    failures = document.get("failures")
    if not isinstance(results, list) or not isinstance(failures, list):
        raise ValueError("relay-boundary results and failures must be arrays")
    identities = set()
    pids = []
    tokens = []
    for sample in results:
        validate_sample(sample, parameters, benchmark["aot_build"], benchmark["cache"])
        identity = (sample["variant"], sample["repetition"])
        if identity in identities:
            raise ValueError("duplicate relay-boundary sample")
        identities.add(identity)
        pids.extend((sample["relay_pid"], sample["driver_pid"]))
        tokens.append(sample["run_token"])
        if sample["ros_domain_id"] != domain:
            raise ValueError("relay-boundary sample used an unleased domain")
    if len(pids) != len(set(pids)) or len(tokens) != len(set(tokens)):
        raise ValueError("relay-boundary samples must use fresh processes and unique topics")
    if not failures:
        expected = {
            (variant, repetition)
            for repetition in range(1, parameters["repetitions"] + 1)
            for variant in variants
        }
        if identities != expected:
            raise ValueError("relay-boundary successful result matrix is incomplete")
    comparison = document.get("comparison")
    if comparison != summarize(results, variants):
        raise ValueError("relay-boundary descriptive comparison is inconsistent")
    if comparison.get("interpretation_allowed") is not False:
        raise ValueError("relay-boundary raw comparison cannot permit interpretation")


def dumps(document: dict) -> str:
    validate_document(document)
    return json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n"


def write(document: dict, path: Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(dumps(document), encoding="utf-8")
    temporary.replace(path)


__all__ = [
    "DRIVER_SCHEMA", "QOS", "RELAY_SCHEMA", "SAMPLE_SCHEMA", "SCHEMA_ID",
    "VARIANTS", "build_document", "dumps", "expected_input_checksum",
    "expected_output_checksum", "latency_summary", "nearest_rank", "summarize",
    "transformed", "validate_document", "validate_prewarm", "validate_sample", "write",
]
