#!/usr/bin/env python3
"""Prebuild and run dynamic clients for the SetBool client benchmark."""

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


PREFIX = "@@RCLCPPYY_SERVICE_CLIENT_V1@@"
PREWARM_SCHEMA = "rclcppyy.service-client-prewarm/v1"
CLIENT_SCHEMA = "rclcppyy.service-client-client-event/v1"
BACKEND_SCHEMA = "rclcppyy.benchmark-backend/v1"
HEADER = "std_srvs/srv/set_bool.hpp"
RSS_LIMIT_BYTES = 64 * 1024 * 1024


def _emit(document: dict) -> None:
    print(PREFIX + json.dumps(document, sort_keys=True, allow_nan=False), flush=True)


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


def _validate_response(value: bool, response) -> int:
    success = bool(response.success)
    message = str(response.message)
    expected = "enabled" if value else "disabled"
    if success != value or message != expected:
        raise RuntimeError("SetBool response violated the benchmark contract")
    return _response_code(value, success, message)


def _artifact(path: str | Path, cached: bool, reason: str) -> dict:
    value = Path(path).resolve()
    if not value.is_file():
        raise RuntimeError("generated client artifact is missing")
    return {
        "cached": bool(cached),
        "reason": reason,
        "path": str(value),
        "sha256": _sha256(value),
        "size_bytes": value.stat().st_size,
    }


def _node_options(ros):
    options = ros.rclcpp.NodeOptions()
    options.start_parameter_services(False)
    options.start_parameter_event_publisher(False)
    options.enable_rosout(False)
    return options


def _state_compile() -> tuple[dict, object]:
    import cppyy
    import cppyy_kit
    from ament_index_python.packages import get_package_prefix
    from rclcpp_kit.bringup_rclcpp import get_ros2_lib_path, ros2_include_paths

    cppyy.add_include_path(os.path.join(
        get_package_prefix("std_srvs"), "include", "std_srvs"))
    cppyy.include(HEADER)
    declarations = """
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
namespace rclcppyy_service_client_benchmark {
class ClientStateMachine {
public:
  virtual ~ClientStateMachine() = default;
  virtual void verify_graph(const std::string&, const std::string&) = 0;
  virtual void warmup(uint64_t) = 0;
  virtual void run_measured(uint64_t) = 0;
  virtual uint64_t total_requests() const = 0;
  virtual uint64_t measured_requests() const = 0;
  virtual uint64_t true_measured() const = 0;
  virtual uint64_t response_checksum() const = 0;
  virtual uint64_t exceptions() const = 0;
  virtual uint64_t pending_requests() const = 0;
  virtual uint64_t cpu_time_ns() const = 0;
  virtual uint64_t elapsed_ns() const = 0;
  virtual const std::vector<uint64_t>& latencies() const = 0;
  virtual bool service_is_ready() const = 0;
  virtual void close() = 0;
};
std::shared_ptr<ClientStateMachine> make_client_state_machine(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& service_name);
}
"""
    code = declarations.replace(
        "std::shared_ptr<ClientStateMachine> make_client_state_machine(\n",
        """
class ClientStateMachineImpl final : public ClientStateMachine {
public:
  using Service = std_srvs::srv::SetBool;
  ClientStateMachineImpl(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& service_name)
  : node_(std::move(node))
  {
    client_ = node_->create_client<Service>(service_name, rclcpp::ServicesQoS());
  }
  ~ClientStateMachineImpl() override { close(); }
  void verify_graph(
      const std::string& server_name,
      const std::string& service_name) override
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (std::chrono::steady_clock::now() < deadline) {
      if (node_->count_services(service_name) == 1) {
        try {
          const auto services = node_->get_service_names_and_types_by_node(server_name, "/");
          const auto found = services.find(service_name);
          if (found != services.end() && found->second.size() == 1 &&
              found->second[0] == "std_srvs/srv/SetBool") {
            if (!client_->wait_for_service(std::chrono::seconds(1))) {
              throw std::runtime_error("verified service is not ready");
            }
            return;
          }
        } catch (const std::runtime_error&) {
        }
      } else if (node_->count_services(service_name) > 1) {
        throw std::runtime_error("service graph contains multiple benchmark servers");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    throw std::runtime_error("timed out verifying exact service graph");
  }
  void warmup(uint64_t count) override
  {
    if (total_requests_ != 0) {
      throw std::logic_error("warmup may run only once");
    }
    for (uint64_t sequence = 1; sequence <= count; ++sequence) {
      call(sequence % 2 == 1, false);
    }
    warmup_requests_ = count;
  }
  void run_measured(uint64_t count) override
  {
    latencies_.clear();
    latencies_.reserve(count);
    const auto cpu_start = process_cpu_ns();
    const auto wall_start = std::chrono::steady_clock::now();
    for (uint64_t offset = 1; offset <= count; ++offset) {
      const uint64_t sequence = warmup_requests_ + offset;
      const bool value = sequence % 2 == 1;
      const auto started = std::chrono::steady_clock::now();
      call(value, true);
      const auto stopped = std::chrono::steady_clock::now();
      latencies_.push_back(static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(stopped - started).count()));
    }
    const auto wall_stop = std::chrono::steady_clock::now();
    cpu_time_ns_ = process_cpu_ns() - cpu_start;
    elapsed_ns_ = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(wall_stop - wall_start).count());
  }
  uint64_t total_requests() const override { return total_requests_; }
  uint64_t measured_requests() const override { return measured_requests_; }
  uint64_t true_measured() const override { return true_measured_; }
  uint64_t response_checksum() const override { return response_checksum_; }
  uint64_t exceptions() const override { return exceptions_; }
  uint64_t pending_requests() const override { return 0; }
  uint64_t cpu_time_ns() const override { return cpu_time_ns_; }
  uint64_t elapsed_ns() const override { return elapsed_ns_; }
  const std::vector<uint64_t>& latencies() const override { return latencies_; }
  bool service_is_ready() const override
  {
    return client_ && client_->service_is_ready();
  }
  void close() override { client_.reset(); }
private:
  static uint64_t process_cpu_ns()
  {
    timespec value{};
    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &value) != 0) {
      throw std::runtime_error("CLOCK_PROCESS_CPUTIME_ID failed");
    }
    return static_cast<uint64_t>(value.tv_sec) * 1000000000ULL +
      static_cast<uint64_t>(value.tv_nsec);
  }
  void call(bool value, bool measured)
  {
    auto request = std::make_shared<Service::Request>();
    request->data = value;
    try {
      auto future = client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(
          node_, future, std::chrono::seconds(15)) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        throw std::runtime_error("SetBool response timed out");
      }
      const auto response = future.get();
      const std::string expected = value ? "enabled" : "disabled";
      if (response->success != value || response->message != expected) {
        throw std::runtime_error("SetBool response contract failed");
      }
      ++total_requests_;
      if (measured) {
        ++measured_requests_;
        true_measured_ += value ? 1 : 0;
        response_checksum_ += (response->success ? 100ULL : 0ULL) +
          (value ? 10ULL : 0ULL) + response->message.size();
      }
    } catch (...) {
      ++exceptions_;
      throw;
    }
  }
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Client<Service>::SharedPtr client_;
  uint64_t warmup_requests_{0};
  uint64_t total_requests_{0};
  uint64_t measured_requests_{0};
  uint64_t true_measured_{0};
  uint64_t response_checksum_{0};
  uint64_t exceptions_{0};
  uint64_t cpu_time_ns_{0};
  uint64_t elapsed_ns_{0};
  std::vector<uint64_t> latencies_;
};
std::shared_ptr<ClientStateMachine> make_client_state_machine(
""",
    )
    code = code.replace(
        "  const std::string& service_name);\n}\n",
        """  const std::string& service_name)
{
  return std::make_shared<ClientStateMachineImpl>(
    std::move(node), service_name);
}
}
""",
    )
    cache_root = os.environ.get("XDG_CACHE_HOME") or os.path.join(
        os.path.expanduser("~"), ".cache")
    result = cppyy_kit.cppdef_cached(
        code,
        decls=declarations,
        name="rclcppyy_service_client_state_machine_v1",
        include_paths=tuple(sorted(ros2_include_paths())),
        library_paths=(get_ros2_lib_path(),),
        libraries=("rclcpp", "std_srvs__rosidl_typesupport_cpp"),
        directory=os.path.join(cache_root, "cppyy_kit", "service-client-benchmark"),
    )
    factory = cppyy.gbl.rclcppyy_service_client_benchmark.make_client_state_machine
    return result, factory


def _prewarm() -> int:
    from rclcpp_kit.native import native
    from std_srvs.srv import SetBool

    with native(["service-client-prewarm"]) as ros:
        node = ros.create_node("service_client_prewarm", options=_node_options(ros))
        before = set(Path(os.environ["XDG_CACHE_HOME"]).rglob("*.so"))
        native_client = ros.create_native_client(
            node, SetBool, "/service_client/prewarm_native")
        native_result = dict(native_client.compile_result)
        state_result, factory = _state_compile()
        state = ros.register_resource(factory(
            node, "/service_client/prewarm_state"))
        state.close()
    native_path = Path(native_result["so"]).resolve()
    native_cached = bool(native_result.get("cached")) or native_path in {
        path.resolve() for path in before}
    artifacts = {
        "native_client": _artifact(
            native_path, native_cached,
            "hit" if native_cached else native_result.get("reason", "miss-built")),
        "cpp_state_machine": _artifact(
            state_result["so"], state_result.get("cached", False),
            state_result.get("reason", "unknown")),
    }
    _emit({
        "schema": PREWARM_SCHEMA,
        "pid": os.getpid(),
        "loaded_rmw": _loaded_rmw(),
        "native_client_source_id": native_client.source_id,
        "artifacts": artifacts,
    })
    return 0


def _stock_marker(client) -> dict:
    return {
        "schema": BACKEND_SCHEMA,
        "role": "client",
        "backend": "python",
        "evidence": "stock_rclpy_entity",
        "metadata": {
            "entity_type": "%s.%s" % (
                type(client).__module__, type(client).__qualname__),
        },
    }


def _status_client_marker(snapshot: dict, service_name: str) -> dict:
    matches = [
        record for record in snapshot["entities"]
        if record["metadata"].get("entity_type") == "client"
        and record["metadata"].get("service_name") == service_name
    ]
    if len(matches) != 1:
        raise RuntimeError("compatible client authority evidence is ambiguous")
    record = matches[0]
    return {
        "schema": BACKEND_SCHEMA,
        "role": "client",
        "backend": record["backend"],
        "evidence": "rclcppyy_status_entity",
        "metadata": {
            "decision_id": record["id"],
            "reason": record["reason"],
            "policies": record["policies"],
            "entity_type": "client",
            "service_name": service_name,
        },
    }


def _direct_cpp_client_proof(snapshot: dict, client, node, guards: dict) -> dict:
    from rclcppyy import direct_cpp

    runtime = direct_cpp._runtime()
    raw_client_type = str(getattr(type(client._native.raw_client), "__cpp_name__", ""))
    if raw_client_type != "rclcpp::Client<std_srvs::srv::SetBool>":
        raise RuntimeError("direct_cpp raw client C++ identity is invalid")
    if runtime.nodes != [node] or runtime.session.nodes != (node._direct_cpp_node,):
        raise RuntimeError("direct_cpp native node authority is invalid")
    matches = [
        record for record in snapshot["entities"]
        if record["metadata"].get("entity_type") == "client"
        and record["metadata"].get("service_name") == client.srv_name
        and "direct_cpp_service" in record["policies"]
    ]
    if len(matches) != 1:
        raise RuntimeError("direct_cpp client authority evidence is ambiguous")
    metadata = matches[0]["metadata"]
    expected = {
        "request_representation": "actual_cpp",
        "response_representation": "actual_cpp",
        "python_message_conversions": 0,
        "request_handoff": "one_native_cpp_value_copy",
        "response_handoff": "shared_cpp_response",
        "future_control": "per_operation_rclpy_task_future",
        "python_request_crossings_per_call": 1,
        "python_response_crossings_per_call": 1,
        "cpp_request_copies_per_call": 1,
    }
    if matches[0]["backend"] != "cpp" or any(
            metadata.get(name) != value for name, value in expected.items()):
        raise RuntimeError("direct_cpp client status evidence is invalid")
    return {
        "profile": "direct_cpp",
        "node_authority": "cpp",
        "client_authority": "cpp",
        "client_entity_type": raw_client_type,
        "runtime_facade_node_count": len(runtime.nodes),
        "native_session_node_count": len(runtime.session.nodes),
        "native_node_identity_verified": True,
        "request_representation": "actual_cpp",
        "response_representation": "actual_cpp",
        "future_type": "rclpy.task.Future",
        "future_control": "per_operation_rclpy_task_future",
        "request_handoff": "one_native_cpp_value_copy",
        "response_handoff": "shared_cpp_response",
        "python_request_crossings_per_call": 1,
        "python_response_crossings_per_call": 1,
        "python_message_conversions_per_call": 0,
        "cpp_request_copies_per_call": 1,
        **guards,
        "status_decision": matches[0],
    }


def _install_direct_cpp_boundary_guards() -> dict:
    import importlib

    bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
    native_client = importlib.import_module("rclcpp_kit.native_client")
    serialization = importlib.import_module("rclcpp_kit.serialization")

    def forbidden_boundary(*_args, **_kwargs):
        raise RuntimeError("direct_cpp used a forbidden conversion/serialization boundary")

    bringup.convert_python_msg_to_cpp = forbidden_boundary
    native_client.convert_python_msg_to_cpp = forbidden_boundary
    serialization.serialize_message = forbidden_boundary
    serialization.deserialize_message = forbidden_boundary
    return {
        "python_conversion_guard_installed": (
            bringup.convert_python_msg_to_cpp is forbidden_boundary
            and native_client.convert_python_msg_to_cpp is forbidden_boundary),
        "serialization_guards_installed": (
            serialization.serialize_message is forbidden_boundary
            and serialization.deserialize_message is forbidden_boundary),
    }


def _verify_python_graph(node, server_node: str, service_name: str) -> None:
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        count = node.count_services(service_name)
        if count == 1:
            try:
                services = dict(node.get_service_names_and_types_by_node(
                    server_node, "/"))
            except RuntimeError:
                services = {}
            if services.get(service_name) == ["std_srvs/srv/SetBool"]:
                return
        elif count > 1:
            raise RuntimeError("service graph contains multiple benchmark servers")
        time.sleep(0.002)
    raise RuntimeError("timed out verifying exact service graph")


def _verify_native_graph(node, server_node: str, service_name: str) -> None:
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        count = int(node.count_services(service_name))
        if count == 1:
            services = node.get_service_names_and_types_by_node(server_node, "/")
            try:
                types = list(services.at(service_name))
            except Exception:
                types = []
            if [str(value) for value in types] == ["std_srvs/srv/SetBool"]:
                return
        elif count > 1:
            raise RuntimeError("service graph contains multiple benchmark servers")
        time.sleep(0.002)
    raise RuntimeError("timed out verifying exact native service graph")


def _warmed(args, *, model: str, authority: str, cache: dict, entity_type: str,
            marker: dict | None = None) -> dict:
    value = {
        "schema": CLIENT_SCHEMA,
        "event": "warmed",
        "variant": args.variant,
        "run_token": args.run_token,
        "pid": os.getpid(),
        "process_group_id": os.getpgrp(),
        "node_name": args.node_name,
        "loaded_rmw": _loaded_rmw(),
        "execution_model": model,
        "client_authority": authority,
        "cache": cache,
        "entity_type": entity_type,
        "warmup_requests": args.warmup_requests,
        "topology_verified": True,
        "endpoint_count": 1,
        "server_node": args.server_node,
        "service_name": args.service_name,
        "service_type": "std_srvs/srv/SetBool",
        "qos_verified": True,
    }
    if marker is not None:
        value["backend_marker"] = marker
    return value


def _wait_control(expected: str) -> None:
    if sys.stdin.readline().rstrip("\n") != expected:
        raise RuntimeError("client expected %s control" % expected)


def _wait_python_endpoint_gone(node, client, service_name: str) -> bool:
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline and node.count_services(service_name) != 0:
        time.sleep(0.002)
    return node.count_services(service_name) == 0 and not client.service_is_ready()


def _run_python_client(args, activate: bool) -> tuple[dict, dict, bool]:
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

    context = Context()
    context.init(args=[])
    node = Node(
        args.node_name, context=context, enable_rosout=False,
        start_parameter_services=False)
    client = node.create_client(
        SetBool, args.service_name, qos_profile=qos_profile_services_default)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)

    def call(value: bool) -> int:
        future = client.call_async(SetBool.Request(data=value))
        executor.spin_until_future_complete(future, timeout_sec=15.0)
        if not future.done() or future.exception() is not None:
            raise RuntimeError("Python client response failed or timed out")
        return _validate_response(value, future.result())

    teardown_clean = False
    try:
        _verify_python_graph(node, args.server_node, args.service_name)
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("verified service is not ready")
        for sequence in range(1, args.warmup_requests + 1):
            call(sequence % 2 == 1)
        marker = (
            _status_client_marker(rclcppyy.status(), client.srv_name)
            if activate else _stock_marker(client)
        )
        ready = _warmed(
            args,
            model=(
                "same-python-client-compatible-activation" if activate
                else "same-python-client-stock-rclpy"),
            authority="python",
            cache={
                "state": "not_applicable",
                "kind": "compatible-python-client" if activate else "stock-rclpy",
            },
            entity_type="%s.%s" % (
                type(client).__module__, type(client).__qualname__),
            marker=marker,
        )
        _emit(ready)
        _wait_control("START")
        rss_baseline = _peak_rss_bytes()
        cpu_start = time.process_time_ns()
        wall_start = time.perf_counter_ns()
        latencies = []
        checksum = 0
        true_measured = 0
        for offset in range(1, args.messages + 1):
            sequence = args.warmup_requests + offset
            value = sequence % 2 == 1
            started = time.perf_counter_ns()
            checksum += call(value)
            latencies.append(time.perf_counter_ns() - started)
            true_measured += int(value)
        elapsed_ns = time.perf_counter_ns() - wall_start
        cpu_time_ns = time.process_time_ns() - cpu_start
        rss_final = _peak_rss_bytes()
        report = {
            "total_requests": args.warmup_requests + args.messages,
            "true_measured": true_measured,
            "response_checksum": checksum,
            "python_orchestration_requests_measured": args.messages,
            "python_request_crossings_measured": args.messages,
            "python_response_crossings_measured": args.messages,
            "python_message_conversions_measured": args.messages * 2,
            "exceptions": 0,
            "pending_requests": 0,
            "elapsed_ns": elapsed_ns,
            "cpu_time_ns": cpu_time_ns,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "rss_guard": _rss_guard(rss_baseline, rss_final),
            "latency_ns": latencies,
        }
        _emit({
            "schema": CLIENT_SCHEMA,
            "event": "measured",
            "variant": args.variant,
            "run_token": args.run_token,
            "pid": os.getpid(),
            "process_group_id": os.getpgrp(),
            "messages": args.messages,
            **report,
        })
        _wait_control("TEARDOWN")
        endpoint_gone = _wait_python_endpoint_gone(
            node, client, args.service_name)
    finally:
        try:
            executor.remove_node(node)
            executor.shutdown(timeout_sec=2.0)
            node.destroy_client(client)
            node.destroy_node()
            if context.ok():
                context.shutdown()
            teardown_clean = not context.ok()
        except Exception:
            teardown_clean = False
    return ready, report, bool(endpoint_gone and teardown_clean)


def _run_direct_cpp_client(args) -> tuple[dict, dict, dict]:
    import rclcppyy as active

    active.enable_cpp_acceleration(profile="direct_cpp")
    import cppyy
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_services_default
    from rclpy.task import Future
    from std_srvs.srv import SetBool

    if SetBool.Request is not cppyy.gbl.std_srvs.srv.SetBool.Request or (
            SetBool.Response is not cppyy.gbl.std_srvs.srv.SetBool.Response):
        raise RuntimeError("direct_cpp did not install actual C++ SetBool values")
    guards = _install_direct_cpp_boundary_guards()
    if not all(guards.values()):
        raise RuntimeError("direct_cpp boundary guards were not installed")

    rclpy.init(args=[])
    node = Node(args.node_name)
    raw_node = node._direct_cpp_node
    client = node.create_client(
        SetBool, args.service_name, qos_profile=qos_profile_services_default)
    endpoint_gone = False

    def call(value: bool) -> int:
        request = SetBool.Request(data=value)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
        if not future.done() or future.exception() is not None:
            raise RuntimeError("direct_cpp client response failed or timed out")
        response = future.result()
        return _validate_response(value, response)

    def verify_call(value: bool) -> int:
        request = SetBool.Request(data=value)
        if type(request) is not SetBool.Request:
            raise RuntimeError("direct_cpp request is not the actual C++ value")
        future = client.call_async(request)
        if type(future) is not Future:
            raise RuntimeError("direct_cpp did not return an rclpy.task.Future")
        rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
        if not future.done() or future.exception() is not None:
            raise RuntimeError("direct_cpp client response failed or timed out")
        response = future.result()
        if type(response) is not SetBool.Response:
            raise RuntimeError("direct_cpp response is not the actual C++ value")
        return _validate_response(value, response)

    try:
        _verify_native_graph(raw_node, args.server_node, args.service_name)
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("verified direct_cpp service is not ready")
        verify_call(True)
        for sequence in range(2, args.warmup_requests + 1):
            call(sequence % 2 == 1)
        baseline = client.stats()
        artifact = _artifact(
            client.compile_result["so"], client.compile_result.get("cached", False),
            client.compile_result.get("reason", "unknown"))
        ready = _warmed(
            args,
            model="direct-cpp-rclpy-call-shape-client",
            authority="cpp",
            cache={
                **artifact,
                "state": "prebuilt",
                "kind": "native-client",
                "source_id": client.source_id,
            },
            entity_type="rclcpp::Client<std_srvs::srv::SetBool>",
        )
        ready["direct_cpp_proof"] = _direct_cpp_client_proof(
            active.status(), client, node, guards)
        _emit(ready)
        _wait_control("START")
        rss_baseline = _peak_rss_bytes()
        cpu_start = time.process_time_ns()
        wall_start = time.perf_counter_ns()
        latencies = []
        checksum = 0
        true_measured = 0
        for offset in range(1, args.messages + 1):
            sequence = args.warmup_requests + offset
            value = sequence % 2 == 1
            started = time.perf_counter_ns()
            checksum += call(value)
            latencies.append(time.perf_counter_ns() - started)
            true_measured += int(value)
        elapsed_ns = time.perf_counter_ns() - wall_start
        cpu_time_ns = time.process_time_ns() - cpu_start
        rss_final = _peak_rss_bytes()
        final = client.stats()
        report = {
            "total_requests": final.requests_sent,
            "true_measured": true_measured,
            "response_checksum": checksum,
            "python_orchestration_requests_measured": args.messages,
            "python_request_crossings_measured": (
                final.python_request_crossings - baseline.python_request_crossings),
            "python_response_crossings_measured": (
                final.python_response_crossings - baseline.python_response_crossings),
            "python_message_conversions_measured": 0,
            "cpp_request_copies_measured": (
                final.cpp_request_copies - baseline.cpp_request_copies),
            "exceptions": final.exceptions,
            "pending_requests": final.pending_requests,
            "elapsed_ns": elapsed_ns,
            "cpu_time_ns": cpu_time_ns,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "rss_guard": _rss_guard(rss_baseline, rss_final),
            "latency_ns": latencies,
        }
        _emit({
            "schema": CLIENT_SCHEMA,
            "event": "measured",
            "variant": args.variant,
            "run_token": args.run_token,
            "pid": os.getpid(),
            "process_group_id": os.getpgrp(),
            "messages": args.messages,
            **report,
        })
        _wait_control("TEARDOWN")
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and int(raw_node.count_services(
                args.service_name)) != 0:
            time.sleep(0.002)
        endpoint_gone = (
            int(raw_node.count_services(args.service_name)) == 0
            and not client.service_is_ready())
    finally:
        try:
            from rclcppyy import direct_cpp

            runtime = direct_cpp._runtime()
            session = runtime.session
            node.destroy_client(client)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            teardown_evidence = {
                "endpoint_disappeared": bool(endpoint_gone),
                "client_closed": bool(client.closed),
                "node_destroyed": node._direct_cpp_node is None,
                "context_shutdown": not rclpy.ok(),
                "native_session_closed": bool(session.closed),
                "native_session_released": runtime.session is None,
                "native_executor_released": runtime.executor is None,
                "runtime_nodes_released": runtime.nodes == [],
            }
        except Exception:
            teardown_evidence = {
                "endpoint_disappeared": bool(endpoint_gone),
                "client_closed": False,
                "node_destroyed": False,
                "context_shutdown": False,
                "native_session_closed": False,
                "native_session_released": False,
                "native_executor_released": False,
                "runtime_nodes_released": False,
            }
    return ready, report, teardown_evidence


def _run_native_orchestrated(args) -> tuple[dict, dict, bool]:
    from rclcpp_kit.native import native
    from std_srvs.srv import SetBool

    session = native(["service-client-native-python-orchestrated"])
    endpoint_gone = False
    with session as ros:
        node = ros.create_node(args.node_name, options=_node_options(ros))
        executor = ros.create_executor("single_threaded", threads=1)
        executor.add_node(node)
        thread = ros.start_executor(executor)
        client = ros.create_native_client(node, SetBool, args.service_name)
        _verify_native_graph(node, args.server_node, args.service_name)
        if not client.wait_for_service(1.0):
            raise RuntimeError("verified native service is not ready")

        def call(value: bool) -> int:
            request = client.make_request()
            request.data = value
            token = client.send(request)
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline and not client.ready(token):
                time.sleep(0.0001)
            if not client.ready(token):
                client.cancel(token)
                raise RuntimeError("native orchestrated response timed out")
            return _validate_response(value, client.take(token))

        for sequence in range(1, args.warmup_requests + 1):
            call(sequence % 2 == 1)
        baseline = client.stats()
        artifact = _artifact(
            client.compile_result["so"], client.compile_result.get("cached", False),
            client.compile_result.get("reason", "unknown"))
        ready = _warmed(
            args,
            model="native-session-python-orchestrated-client",
            authority="cpp",
            cache={
                **artifact,
                "state": "prebuilt",
                "kind": "native-client",
                "source_id": client.source_id,
            },
            entity_type="rclcpp::Client<std_srvs::srv::SetBool>",
        )
        _emit(ready)
        _wait_control("START")
        rss_baseline = _peak_rss_bytes()
        cpu_start = time.process_time_ns()
        wall_start = time.perf_counter_ns()
        latencies = []
        checksum = 0
        true_measured = 0
        for offset in range(1, args.messages + 1):
            sequence = args.warmup_requests + offset
            value = sequence % 2 == 1
            started = time.perf_counter_ns()
            checksum += call(value)
            latencies.append(time.perf_counter_ns() - started)
            true_measured += int(value)
        elapsed_ns = time.perf_counter_ns() - wall_start
        cpu_time_ns = time.process_time_ns() - cpu_start
        rss_final = _peak_rss_bytes()
        final = client.stats()
        report = {
            "total_requests": final.requests_sent,
            "true_measured": true_measured,
            "response_checksum": checksum,
            "python_orchestration_requests_measured": args.messages,
            "python_request_crossings_measured": (
                final.python_request_crossings - baseline.python_request_crossings),
            "python_response_crossings_measured": (
                final.python_response_crossings - baseline.python_response_crossings),
            "python_message_conversions_measured": 0,
            "exceptions": final.exceptions + int(thread.exceptions),
            "pending_requests": final.pending_requests,
            "elapsed_ns": elapsed_ns,
            "cpu_time_ns": cpu_time_ns,
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "rss_guard": _rss_guard(rss_baseline, rss_final),
            "latency_ns": latencies,
        }
        _emit({
            "schema": CLIENT_SCHEMA,
            "event": "measured",
            "variant": args.variant,
            "run_token": args.run_token,
            "pid": os.getpid(),
            "process_group_id": os.getpgrp(),
            "messages": args.messages,
            **report,
        })
        _wait_control("TEARDOWN")
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and int(node.count_services(
                args.service_name)) != 0:
            time.sleep(0.002)
        endpoint_gone = (
            int(node.count_services(args.service_name)) == 0
            and not client.service_is_ready())
    return ready, report, bool(
        endpoint_gone and session.closed and client.closed and thread.closed)


def _run_native_state_machine(args) -> tuple[dict, dict, bool]:
    from rclcpp_kit.native import native

    session = native(["service-client-native-cpp-state-machine"])
    endpoint_gone = False
    with session as ros:
        node = ros.create_node(args.node_name, options=_node_options(ros))
        compile_result, factory = _state_compile()
        state = ros.register_resource(factory(node, args.service_name))
        state.verify_graph(args.server_node, args.service_name)
        state.warmup(args.warmup_requests)
        artifact = _artifact(
            compile_result["so"], compile_result.get("cached", False),
            compile_result.get("reason", "unknown"))
        ready = _warmed(
            args,
            model="native-content-addressed-cpp-client-state-machine",
            authority="cpp",
            cache={
                **artifact,
                "state": "prebuilt",
                "kind": "cpp-client-state-machine",
            },
            entity_type="rclcpp::Client<std_srvs::srv::SetBool>",
        )
        _emit(ready)
        _wait_control("START")
        rss_baseline = _peak_rss_bytes()
        state.run_measured(args.messages)
        rss_final = _peak_rss_bytes()
        report = {
            "total_requests": int(state.total_requests()),
            "true_measured": int(state.true_measured()),
            "response_checksum": int(state.response_checksum()),
            "python_orchestration_requests_measured": 0,
            "python_request_crossings_measured": 0,
            "python_response_crossings_measured": 0,
            "python_message_conversions_measured": 0,
            "exceptions": int(state.exceptions()),
            "pending_requests": int(state.pending_requests()),
            "elapsed_ns": int(state.elapsed_ns()),
            "cpu_time_ns": int(state.cpu_time_ns()),
            "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
            "rss_guard": _rss_guard(rss_baseline, rss_final),
            "latency_ns": [int(value) for value in state.latencies()],
        }
        _emit({
            "schema": CLIENT_SCHEMA,
            "event": "measured",
            "variant": args.variant,
            "run_token": args.run_token,
            "pid": os.getpid(),
            "process_group_id": os.getpgrp(),
            "messages": args.messages,
            **report,
        })
        _wait_control("TEARDOWN")
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and int(node.count_services(
                args.service_name)) != 0:
            time.sleep(0.002)
        endpoint_gone = (
            int(node.count_services(args.service_name)) == 0
            and not bool(state.service_is_ready()))
    return ready, report, bool(endpoint_gone and session.closed)


def _run_client(args) -> int:
    direct_teardown = None
    if args.variant == "stock-rclpy":
        _, report, teardown = _run_python_client(args, False)
    elif args.variant == "compatible-rclcppyy":
        _, report, teardown = _run_python_client(args, True)
    elif args.variant == "direct-cpp-rclcppyy":
        _, report, direct_teardown = _run_direct_cpp_client(args)
        teardown = all(direct_teardown.values())
    elif args.variant == "native-python-orchestrated":
        _, report, teardown = _run_native_orchestrated(args)
    else:
        _, report, teardown = _run_native_state_machine(args)
    correct = (
        report["total_requests"] == args.warmup_requests + args.messages
        and report["exceptions"] == 0
        and report["pending_requests"] == 0
    )
    teardown_event = {
        "schema": CLIENT_SCHEMA,
        "event": "teardown",
        "variant": args.variant,
        "run_token": args.run_token,
        "pid": os.getpid(),
        "process_group_id": os.getpgrp(),
        "endpoint_disappeared": teardown,
        "teardown_clean": teardown,
    }
    if direct_teardown is not None:
        teardown_event["direct_cpp_teardown"] = direct_teardown
    _emit(teardown_event)
    return 0 if correct and teardown else 2


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prewarm", action="store_true")
    parser.add_argument("--variant", choices=(
        "stock-rclpy", "compatible-rclcppyy", "direct-cpp-rclcppyy",
        "native-python-orchestrated", "native-cpp-state-machine"))
    parser.add_argument("--service-name")
    parser.add_argument("--node-name")
    parser.add_argument("--server-node")
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
        "variant", "service_name", "node_name", "server_node",
        "warmup_requests", "messages", "run_token")
    missing = [name for name in required if getattr(args, name) is None]
    if missing:
        parser.error("client mode requires: " + ", ".join(missing))
    if args.warmup_requests <= 0 or args.messages <= 0:
        parser.error("warmup and measured request counts must be positive")
    return _run_client(args)


if __name__ == "__main__":
    raise SystemExit(main())
