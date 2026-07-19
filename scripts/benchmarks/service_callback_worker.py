#!/usr/bin/env python3
"""Prebuild and run dynamic service-callback benchmark servers."""

from __future__ import annotations

import argparse
import faulthandler
import hashlib
import json
import os
from pathlib import Path
import resource
import signal
import sys
import time


PREFIX = "@@RCLCPPYY_SERVICE_CALLBACK_V1@@"
PREWARM_SCHEMA = "rclcppyy.service-callback-prewarm/v1"
SERVER_SCHEMA = "rclcppyy.service-callback-server-event/v1"
DIAGNOSTIC_SCHEMA = "rclcppyy.service-callback-diagnostic/v1"
BACKEND_SCHEMA = "rclcppyy.benchmark-backend/v1"
CPP_TYPE = "std_srvs::srv::SetBool"
HEADER = "std_srvs/srv/set_bool.hpp"
NATIVE_BODY = (
    "response->success = request->data; "
    'response->message = request->data ? "enabled" : "disabled";'
)
RSS_LIMIT_BYTES = 64 * 1024 * 1024


def _emit(document: dict) -> None:
    print(PREFIX + json.dumps(document, sort_keys=True, allow_nan=False), flush=True)


def _phase(args, name: str, metadata: dict | None = None) -> None:
    document = {
        "schema": DIAGNOSTIC_SCHEMA,
        "event": "phase",
        "phase": name,
        "variant": args.variant,
        "run_token": args.run_token,
        "pid": os.getpid(),
    }
    if metadata:
        document["metadata"] = metadata
    print(
        PREFIX + json.dumps(document, sort_keys=True, allow_nan=False),
        file=sys.stderr,
        flush=True,
    )


def _configure_fault_diagnostics() -> None:
    faulthandler.enable(file=sys.stderr, all_threads=True)
    faulthandler.register(signal.SIGUSR1, file=sys.stderr, all_threads=True)


def _sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _peak_rss_bytes() -> int:
    return int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) * 1024


def _rss_guard(baseline: int, final: int) -> dict:
    growth = max(0, final - baseline)
    return {
        "kind": "post-warmup-peak-rss-growth",
        "unit": "bytes",
        "baseline_peak_bytes": baseline,
        "final_peak_bytes": final,
        "growth_bytes": growth,
        "limit_bytes": RSS_LIMIT_BYTES,
        "within_limit": growth <= RSS_LIMIT_BYTES,
    }


def _loaded_rmw() -> str:
    from rclpy.utilities import get_rmw_implementation_identifier

    loaded = get_rmw_implementation_identifier()
    requested = os.environ.get("RMW_IMPLEMENTATION")
    if loaded != requested:
        raise RuntimeError("loaded RMW %r differs from requested %r" % (loaded, requested))
    return loaded


def _response_code(value: bool, success: bool, message: str) -> int:
    return (100 if success else 0) + (10 if value else 0) + len(message)


def _make_setbool_callback():
    total = 0
    true_total = 0
    checksum_total = 0

    def callback(request, response):
        nonlocal total, true_total, checksum_total
        value = bool(request.data)
        response.success = value
        response.message = "enabled" if value else "disabled"
        total += 1
        true_total += int(value)
        checksum_total += _response_code(value, response.success, response.message)
        return response

    def snapshot():
        return total, true_total, checksum_total

    return callback, snapshot


def _artifact(path: str | Path, cached: bool, reason: str) -> dict:
    value = Path(path).resolve()
    if not value.is_file():
        raise RuntimeError("generated service artifact is missing")
    return {
        "cached": bool(cached),
        "reason": reason,
        "path": str(value),
        "sha256": _sha256(value),
        "size_bytes": value.stat().st_size,
    }


def _bridge_compile() -> tuple[dict, object]:
    import cppyy
    import cppyy_kit
    from ament_index_python.packages import get_package_prefix
    from rclcpp_kit.bringup_rclcpp import get_ros2_lib_path, ros2_include_paths

    cppyy.add_include_path(os.path.join(
        get_package_prefix("std_srvs"), "include", "std_srvs"))
    cppyy.include(HEADER)
    callback_type = "std::function<bool(bool)>"
    declarations = """
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
namespace rclcppyy_service_benchmark {
class PythonServiceBridge {
public:
  virtual ~PythonServiceBridge() = default;
  virtual uint64_t total_requests() const = 0;
  virtual uint64_t measured_requests() const = 0;
  virtual uint64_t total_crossings() const = 0;
  virtual uint64_t measured_crossings() const = 0;
  virtual uint64_t exceptions() const = 0;
  virtual uint64_t true_total() const = 0;
  virtual uint64_t true_measured() const = 0;
  virtual uint64_t response_checksum() const = 0;
  virtual void arm() = 0;
  virtual void close() = 0;
};
std::shared_ptr<PythonServiceBridge> make_python_service_bridge(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& service_name,
  %(callback_type)s callback);
}
""" % {"callback_type": callback_type}
    code = declarations.replace(
        "std::shared_ptr<PythonServiceBridge> make_python_service_bridge(\n",
        """
class PythonServiceBridgeImpl final : public PythonServiceBridge {
public:
  PythonServiceBridgeImpl(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& service_name,
      %(callback_type)s callback)
  : callback_(std::move(callback))
  {
    const auto profile = rmw_qos_profile_services_default;
    const auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization::from_rmw(profile), profile);
    service_ = node->create_service<std_srvs::srv::SetBool>(
      service_name,
      [this](
          std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        total_requests_.fetch_add(1, std::memory_order_relaxed);
        true_total_.fetch_add(request->data ? 1 : 0, std::memory_order_relaxed);
        total_crossings_.fetch_add(1, std::memory_order_relaxed);
        const bool measured = armed_.load(std::memory_order_acquire);
        if (measured) {
          measured_requests_.fetch_add(1, std::memory_order_relaxed);
          true_measured_.fetch_add(request->data ? 1 : 0, std::memory_order_relaxed);
          measured_crossings_.fetch_add(1, std::memory_order_relaxed);
        }
        try {
          response->success = callback_(request->data);
          response->message = response->success ? "enabled" : "disabled";
          if (measured) {
            response_checksum_.fetch_add(
              (response->success ? 100ULL : 0ULL) +
              (request->data ? 10ULL : 0ULL) + response->message.size(),
              std::memory_order_relaxed);
          }
        } catch (...) {
          exceptions_.fetch_add(1, std::memory_order_relaxed);
          response->success = false;
          response->message = "exception";
        }
      }, qos);
  }
  ~PythonServiceBridgeImpl() override { close(); }
  uint64_t total_requests() const override { return total_requests_.load(); }
  uint64_t measured_requests() const override { return measured_requests_.load(); }
  uint64_t total_crossings() const override { return total_crossings_.load(); }
  uint64_t measured_crossings() const override { return measured_crossings_.load(); }
  uint64_t exceptions() const override { return exceptions_.load(); }
  uint64_t true_total() const override { return true_total_.load(); }
  uint64_t true_measured() const override { return true_measured_.load(); }
  uint64_t response_checksum() const override { return response_checksum_.load(); }
  void arm() override { armed_.store(true, std::memory_order_release); }
  void close() override { service_.reset(); }
private:
  %(callback_type)s callback_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  std::atomic<bool> armed_{false};
  std::atomic<uint64_t> total_requests_{0};
  std::atomic<uint64_t> measured_requests_{0};
  std::atomic<uint64_t> total_crossings_{0};
  std::atomic<uint64_t> measured_crossings_{0};
  std::atomic<uint64_t> exceptions_{0};
  std::atomic<uint64_t> true_total_{0};
  std::atomic<uint64_t> true_measured_{0};
  std::atomic<uint64_t> response_checksum_{0};
};
std::shared_ptr<PythonServiceBridge> make_python_service_bridge(
""" % {"callback_type": callback_type},
    )
    code = code.replace(
        "  %(callback_type)s callback);\n}\n" % {"callback_type": callback_type},
        """  %(callback_type)s callback)
{
  return std::make_shared<PythonServiceBridgeImpl>(
    std::move(node), service_name, std::move(callback));
}
}
""" % {"callback_type": callback_type},
    )
    cache_root = os.environ.get("XDG_CACHE_HOME") or os.path.join(
        os.path.expanduser("~"), ".cache")
    result = cppyy_kit.cppdef_cached(
        code,
        decls=declarations,
        name="rclcppyy_service_python_bridge_v1",
        include_paths=tuple(sorted(ros2_include_paths())),
        library_paths=(get_ros2_lib_path(),),
        libraries=("rclcpp", "std_srvs__rosidl_typesupport_cpp"),
        directory=os.path.join(cache_root, "cppyy_kit", "service-benchmark"),
    )
    return dict(result), getattr(
        cppyy.gbl.rclcppyy_service_benchmark, "make_python_service_bridge")


def _native_service_artifact(source_id: str) -> Path:
    root = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache")
    matches = sorted((root / "cppyy_kit" / "native-services").rglob(
        "*%s*.so" % source_id))
    if len(matches) != 1:
        raise RuntimeError("native service artifact is missing or ambiguous")
    return matches[0].resolve()


def _node_options(ros):
    options = ros.rclcpp.NodeOptions()
    options.start_parameter_services(False)
    options.start_parameter_event_publisher(False)
    options.enable_rosout(False)
    return options


def _prewarm() -> int:
    import cppyy
    from rclcpp_kit.native import native
    from std_srvs.srv import SetBool

    with native(["service-callback-prewarm"]) as ros:
        node = ros.create_node("service_callback_prewarm", options=_node_options(ros))
        bridge_result, factory = _bridge_compile()

        def callback(value):
            return bool(value)

        callback_type = cppyy.gbl.std.function["bool(bool)"]
        bridge = factory(node, "/service_callback/prewarm_bridge", callback_type(callback))
        def direct_callback(_request, response):
            return response

        direct_service = ros.create_python_service(
            node, SetBool, "/service_callback/prewarm_direct", direct_callback)
        before = set(Path(os.environ["XDG_CACHE_HOME"]).rglob("*.so"))
        native_service = ros.create_native_service(
            node, SetBool, "/service_callback/prewarm_native", NATIVE_BODY)
        native_path = _native_service_artifact(native_service.source_id)
        native_cached = native_path in {path.resolve() for path in before}
        bridge.close()
    bridge_path = bridge_result.get("so")
    artifacts = {
        "python_bridge": _artifact(
            bridge_path, bridge_result.get("cached", False),
            bridge_result.get("reason", "unknown")),
        "direct_cpp_python_service": _artifact(
            direct_service.compile_result["so"],
            direct_service.compile_result.get("cached", False),
            direct_service.compile_result.get("reason", "unknown")),
        "native_cpp_service": _artifact(
            native_path, native_cached, "hit" if native_cached else "miss-built"),
    }
    _emit({
        "schema": PREWARM_SCHEMA,
        "pid": os.getpid(),
        "loaded_rmw": _loaded_rmw(),
        "native_cpp_source_id": native_service.source_id,
        "artifacts": artifacts,
    })
    return 0


def _stock_marker(service) -> dict:
    return {
        "schema": BACKEND_SCHEMA,
        "role": "server",
        "backend": "python",
        "evidence": "stock_rclpy_entity",
        "metadata": {
            "entity_type": "%s.%s" % (
                type(service).__module__, type(service).__qualname__),
        },
    }


def _status_service_marker(snapshot: dict, service_name: str) -> dict:
    matches = [
        record for record in snapshot["entities"]
        if record["metadata"].get("entity_type") == "service"
        and record["metadata"].get("service_name") == service_name
    ]
    if len(matches) != 1:
        raise RuntimeError("compatible service authority evidence is ambiguous")
    record = matches[0]
    return {
        "schema": BACKEND_SCHEMA,
        "role": "server",
        "backend": record["backend"],
        "evidence": "rclcppyy_status_entity",
        "metadata": {
            "decision_id": record["id"],
            "reason": record["reason"],
            "policies": record["policies"],
            "entity_type": "service",
            "service_name": service_name,
        },
    }


def _cleanup_python_server(
        args, executor, thread, node, service, context) -> bool:
    errors = []

    def step(name, action, require_true=False):
        _phase(args, name + "-before")
        try:
            result = action()
            if require_true and result is not True:
                raise RuntimeError("cleanup returned %r" % (result,))
        except BaseException as exc:
            errors.append((name, type(exc).__name__, str(exc)))
            _phase(args, name + "-error", {
                "exception_type": type(exc).__name__, "message": str(exc)})
            return
        _phase(args, name + "-after")

    step("executor-remove-node", lambda: executor.remove_node(node))
    step("executor-shutdown", lambda: executor.shutdown(timeout_sec=2.0), True)
    thread.join(timeout=2.0)
    _phase(args, "executor-thread-joined", {"thread_alive": thread.is_alive()})
    step("service-destroy", lambda: node.destroy_service(service), True)
    step("node-destroy", node.destroy_node)
    if context.ok():
        step("context-shutdown", context.shutdown)
    if thread.is_alive():
        thread.join(timeout=2.0)
    return not errors and not thread.is_alive() and not context.ok()


def _armed(args) -> dict:
    return {
        "schema": SERVER_SCHEMA,
        "event": "armed",
        "variant": args.variant,
        "run_token": args.run_token,
        "pid": os.getpid(),
        "process_group_id": os.getpgrp(),
        "warmup_requests": args.warmup_requests,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
    }


def _run_python_server(args, activate: bool) -> tuple[dict, dict, bool]:
    rclcppyy = None
    if activate:
        import rclcppyy as active

        active.enable_cpp_acceleration()
        rclcppyy = active
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_services_default
    from std_srvs.srv import SetBool
    import threading

    callback, snapshot = _make_setbool_callback()

    context = Context()
    context.init(args=[])
    node = Node(
        args.node_name, context=context, enable_rosout=False,
        start_parameter_services=False)
    service = node.create_service(
        SetBool, args.service_name, callback,
        qos_profile=qos_profile_services_default)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=False)
    thread.start()
    marker = (
        _status_service_marker(rclcppyy.status(), service.service_name)
        if activate else _stock_marker(service)
    )
    ready = {
        "schema": SERVER_SCHEMA,
        "event": "ready",
        "variant": args.variant,
        "run_token": args.run_token,
        "pid": os.getpid(),
        "process_group_id": os.getpgrp(),
        "node_name": args.node_name,
        "loaded_rmw": _loaded_rmw(),
        "execution_model": (
            "same-python-service-compatible-activation" if activate
            else "same-python-service-stock-rclpy"),
        "cache": {
            "state": "not_applicable",
            "kind": "compatible-python-service" if activate else "stock-rclpy",
        },
        "entity_type": "%s.%s" % (
            type(service).__module__, type(service).__qualname__),
        "service_authority": "python",
        "backend_marker": marker,
    }
    teardown_clean = False
    try:
        _emit(ready)
        if sys.stdin.readline().rstrip("\n") != "START":
            raise RuntimeError("server expected START control")
        total, true_total, checksum_total = snapshot()
        if total != args.warmup_requests:
            raise RuntimeError("Python service warmup count is invalid")
        baseline_true = true_total
        baseline_checksum = checksum_total
        rss_baseline = _peak_rss_bytes()
        cpu_start = time.process_time_ns()
        _emit(_armed(args))
        if sys.stdin.readline().rstrip("\n") != "REPORT":
            raise RuntimeError("server expected REPORT control")
        cpu_time = time.process_time_ns() - cpu_start
        rss_final = _peak_rss_bytes()
        total, true_total, checksum_total = snapshot()
        measured = total - args.warmup_requests
        report = {
            "warmup_requests": args.warmup_requests,
            "total_requests": total,
            "measured_requests": measured,
            "true_total": true_total,
            "true_measured": true_total - baseline_true,
            "response_checksum": checksum_total - baseline_checksum,
            "python_callback_count_total": total,
            "python_boundary_crossings_measured": measured,
            "exceptions": 0,
            "pending_requests": 0,
            "cpu_time_ns": cpu_time,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "rss_guard": _rss_guard(rss_baseline, rss_final),
        }
    finally:
        teardown_clean = _cleanup_python_server(
            args, executor, thread, node, service, context)
    return ready, report, teardown_clean


def _run_direct_cpp(args) -> tuple[dict, dict, bool]:
    import rclcppyy

    rclcppyy.enable_cpp_acceleration(profile="direct_cpp")
    import cppyy
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import SetBool
    import threading

    if SetBool.Request is not cppyy.gbl.std_srvs.srv.SetBool.Request or (
            SetBool.Response is not cppyy.gbl.std_srvs.srv.SetBool.Response):
        raise RuntimeError("direct C++ service aliases are not actual C++ values")

    def forbidden_boundary(*_args, **_kwargs):
        raise RuntimeError("direct C++ service used a conversion or serialization bridge")

    import importlib
    bringup_rclcpp = importlib.import_module("rclcpp_kit.bringup_rclcpp")
    serialization = importlib.import_module("rclcpp_kit.serialization")
    bringup_rclcpp.convert_python_msg_to_cpp = forbidden_boundary
    serialization.serialize_message = forbidden_boundary
    serialization.deserialize_message = forbidden_boundary

    callback, snapshot = _make_setbool_callback()
    rclpy.init(args=[])
    node = Node(args.node_name)
    service = node.create_service(SetBool, args.service_name, callback)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=False)
    thread.start()
    status = rclcppyy.status()
    marker = _status_service_marker(status, service.service_name)
    ready = {
        "schema": SERVER_SCHEMA,
        "event": "ready",
        "variant": args.variant,
        "run_token": args.run_token,
        "pid": os.getpid(),
        "process_group_id": os.getpgrp(),
        "node_name": args.node_name,
        "loaded_rmw": _loaded_rmw(),
        "execution_model": "source-compatible-direct-cpp-service-python-callback",
        "cache": {
            **_artifact(
                service.compile_result["so"],
                service.compile_result.get("cached", False),
                service.compile_result.get("reason", "unknown")),
            "state": "prebuilt",
            "kind": "direct-cpp-python-service",
            "source_id": service.source_id,
        },
        "entity_type": "rclcpp::Service<std_srvs::srv::SetBool>",
        "service_authority": "cpp",
        "backend_marker": marker,
        "data_path": {
            "request_representation": "actual_cpp",
            "response_representation": "actual_cpp",
            "python_message_conversions": 0,
            "serialization_bridges": 0,
            "python_callback_crossings_per_request": 1,
            "request_cpp_copies_per_request": 1,
            "response_cpp_copies_per_request": 1,
            "type_alias_identity_verified": True,
            "conversion_guards_installed": True,
        },
    }
    teardown_clean = False
    try:
        _emit(ready)
        if sys.stdin.readline().rstrip("\n") != "START":
            raise RuntimeError("server expected START control")
        baseline = service.stats()
        total, baseline_true, baseline_checksum = snapshot()
        if baseline.requests != args.warmup_requests or total != args.warmup_requests:
            raise RuntimeError("direct C++ service warmup count is invalid")
        rss_baseline = _peak_rss_bytes()
        cpu_start = time.process_time_ns()
        _emit(_armed(args))
        if sys.stdin.readline().rstrip("\n") != "REPORT":
            raise RuntimeError("server expected REPORT control")
        cpu_time = time.process_time_ns() - cpu_start
        rss_final = _peak_rss_bytes()
        final = service.stats()
        total, true_total, checksum_total = snapshot()
        report = {
            "warmup_requests": args.warmup_requests,
            "total_requests": final.requests,
            "measured_requests": final.requests - baseline.requests,
            "true_total": true_total,
            "true_measured": true_total - baseline_true,
            "response_checksum": checksum_total - baseline_checksum,
            "python_callback_count_total": total,
            "python_boundary_crossings_measured": (
                final.python_callback_crossings - baseline.python_callback_crossings),
            "request_cpp_copies_measured": (
                final.request_cpp_copies - baseline.request_cpp_copies),
            "response_cpp_copies_measured": (
                final.response_cpp_copies - baseline.response_cpp_copies),
            "python_message_conversions_measured": 0,
            "serialization_bridges_measured": 0,
            "exceptions": final.exceptions,
            "pending_requests": 0,
            "cpu_time_ns": cpu_time,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "rss_guard": _rss_guard(rss_baseline, rss_final),
        }
    finally:
        errors = []
        try:
            node.destroy_service(service)
        except BaseException as exc:
            errors.append(exc)
        try:
            node.destroy_node()
        except BaseException as exc:
            errors.append(exc)
        try:
            rclpy.shutdown()
        except BaseException as exc:
            errors.append(exc)
        thread.join(timeout=2.0)
        teardown_clean = not errors and not thread.is_alive() and not rclpy.ok()
    return ready, report, teardown_clean


def _run_native_python(args) -> tuple[dict, dict, bool]:
    import cppyy
    from rclcpp_kit.native import native

    callback_count = 0
    session = native(["service-callback-native-python"])
    bridge = None
    with session as ros:
        node = ros.create_node(args.node_name, options=_node_options(ros))
        compile_result, factory = _bridge_compile()

        def callback(value):
            nonlocal callback_count
            callback_count += 1
            return bool(value)

        function = cppyy.gbl.std.function[
            "bool(bool)"
        ](callback)
        bridge = ros.register_resource(factory(
            node, args.service_name, function))
        executor = ros.create_executor("single_threaded", threads=1)
        executor.add_node(node)
        thread = ros.start_executor(executor)
        artifact = _artifact(
            compile_result["so"], compile_result.get("cached", False),
            compile_result.get("reason", "unknown"))
        ready = {
            "schema": SERVER_SCHEMA,
            "event": "ready",
            "variant": args.variant,
            "run_token": args.run_token,
            "pid": os.getpid(),
            "process_group_id": os.getpgrp(),
            "node_name": args.node_name,
            "loaded_rmw": _loaded_rmw(),
            "execution_model": "native-rclcpp-service-python-callback",
            "cache": {**artifact, "state": "prebuilt", "kind": "python-service-bridge"},
            "entity_type": "rclcpp::Service<std_srvs::srv::SetBool>",
            "service_authority": "cpp",
        }
        _emit(ready)
        if sys.stdin.readline().rstrip("\n") != "START":
            raise RuntimeError("server expected START control")
        if int(bridge.total_requests()) != args.warmup_requests:
            raise RuntimeError("native Python service warmup count is invalid")
        bridge.arm()
        rss_baseline = _peak_rss_bytes()
        cpu_start = time.process_time_ns()
        _emit(_armed(args))
        if sys.stdin.readline().rstrip("\n") != "REPORT":
            raise RuntimeError("server expected REPORT control")
        cpu_time = time.process_time_ns() - cpu_start
        rss_final = _peak_rss_bytes()
        report = {
            "warmup_requests": args.warmup_requests,
            "total_requests": int(bridge.total_requests()),
            "measured_requests": int(bridge.measured_requests()),
            "true_total": int(bridge.true_total()),
            "true_measured": int(bridge.true_measured()),
            "response_checksum": int(bridge.response_checksum()),
            "python_callback_count_total": callback_count,
            "python_boundary_crossings_measured": int(bridge.measured_crossings()),
            "exceptions": int(bridge.exceptions()) + int(thread.exceptions),
            "pending_requests": 0,
            "cpu_time_ns": cpu_time,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "rss_guard": _rss_guard(rss_baseline, rss_final),
        }
    return ready, report, bool(session.closed and bridge and thread.closed)


def _run_native_cpp(args) -> tuple[dict, dict, bool]:
    from rclcpp_kit.native import native
    from std_srvs.srv import SetBool

    session = native(["service-callback-native-cpp"])
    with session as ros:
        node = ros.create_node(args.node_name, options=_node_options(ros))
        service = ros.create_native_service(
            node, SetBool, args.service_name, NATIVE_BODY)
        artifact_path = _native_service_artifact(service.source_id)
        executor = ros.create_executor("single_threaded", threads=1)
        executor.add_node(node)
        thread = ros.start_executor(executor)
        ready = {
            "schema": SERVER_SCHEMA,
            "event": "ready",
            "variant": args.variant,
            "run_token": args.run_token,
            "pid": os.getpid(),
            "process_group_id": os.getpgrp(),
            "node_name": args.node_name,
            "loaded_rmw": _loaded_rmw(),
            "execution_model": "native-session-cpp-service-callback",
            "cache": {
                **_artifact(artifact_path, True, "hit"),
                "state": "prebuilt",
                "kind": "native-cpp-service",
                "source_id": service.source_id,
            },
            "entity_type": "rclcpp::Service<std_srvs::srv::SetBool>",
            "service_authority": "cpp",
        }
        _emit(ready)
        if sys.stdin.readline().rstrip("\n") != "START":
            raise RuntimeError("server expected START control")
        baseline = service.stats()
        if baseline.requests != args.warmup_requests:
            raise RuntimeError("native C++ service warmup count is invalid")
        rss_baseline = _peak_rss_bytes()
        cpu_start = time.process_time_ns()
        _emit(_armed(args))
        if sys.stdin.readline().rstrip("\n") != "REPORT":
            raise RuntimeError("server expected REPORT control")
        cpu_time = time.process_time_ns() - cpu_start
        rss_final = _peak_rss_bytes()
        final = service.stats()
        report = {
            "warmup_requests": args.warmup_requests,
            "total_requests": final.requests,
            "measured_requests": final.requests - baseline.requests,
            "true_total": None,
            "true_measured": None,
            "response_checksum": None,
            "python_callback_count_total": 0,
            "python_boundary_crossings_measured": 0,
            "exceptions": final.exceptions + int(thread.exceptions),
            "pending_requests": 0,
            "cpu_time_ns": cpu_time,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "rss_guard": _rss_guard(rss_baseline, rss_final),
        }
    return ready, report, bool(session.closed and service.closed and thread.closed)


def _run_server(args) -> int:
    if args.variant == "stock-rclpy":
        ready, report, teardown = _run_python_server(args, False)
    elif args.variant == "compatible-rclcppyy":
        ready, report, teardown = _run_python_server(args, True)
    elif args.variant == "native-python-callback":
        ready, report, teardown = _run_native_python(args)
    elif args.variant == "direct-cpp-rclcppyy":
        ready, report, teardown = _run_direct_cpp(args)
    else:
        ready, report, teardown = _run_native_cpp(args)
    correct = (
        report["total_requests"] == args.warmup_requests + args.messages
        and report["measured_requests"] == args.messages
        and report["exceptions"] == 0
        and report["pending_requests"] == 0
    )
    _emit({
        "schema": SERVER_SCHEMA,
        "event": "report",
        "variant": args.variant,
        "run_token": args.run_token,
        **report,
        "correct": correct,
        "teardown_clean": teardown,
    })
    return 0 if correct and teardown else 2


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prewarm", action="store_true")
    parser.add_argument("--variant", choices=(
        "stock-rclpy", "compatible-rclcppyy", "native-python-callback",
        "direct-cpp-rclcppyy", "native-cpp-callback"))
    parser.add_argument("--service-name")
    parser.add_argument("--node-name")
    parser.add_argument("--warmup-requests", type=int)
    parser.add_argument("--messages", type=int)
    parser.add_argument("--run-token")
    return parser


def main() -> int:
    _configure_fault_diagnostics()
    parser = _parser()
    args = parser.parse_args()
    if args.prewarm:
        return _prewarm()
    required = (
        "variant", "service_name", "node_name", "warmup_requests",
        "messages", "run_token")
    missing = [name for name in required if getattr(args, name) is None]
    if missing:
        parser.error("server mode requires: " + ", ".join(missing))
    if args.warmup_requests <= 0 or args.messages <= 0:
        parser.error("warmup and measured request counts must be positive")
    return _run_server(args)


if __name__ == "__main__":
    raise SystemExit(main())
