#!/usr/bin/env python3
"""Run one dynamic lane of the timer-executor benchmark."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import threading
import time

from _timer_executor_protocol import (
    EVENT_SCHEMA,
    MASK64,
    PERIOD_NS,
    PREWARM_SCHEMA,
    RECURRENCE_INCREMENT,
    RECURRENCE_MULTIPLIER,
    RECURRENCE_SEED,
    RMW,
    VARIANTS,
    consecutive_interval_errors,
    deadline_summary,
    max_phase_slip_periods,
)


PROTOCOL_PREFIX = "@@RCLCPPYY_TIMER_EXECUTOR_V1@@"
PROBE_SOURCE_ID = hashlib.sha256(
    ("timer-probe-v1:%d:%d:%d:%d" % (
        PERIOD_NS, RECURRENCE_SEED,
        RECURRENCE_MULTIPLIER, RECURRENCE_INCREMENT)).encode()
).hexdigest()[:16]


def _emit(document: dict) -> None:
    print(
        PROTOCOL_PREFIX + json.dumps(document, sort_keys=True, allow_nan=False),
        flush=True,
    )


def _sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _loaded_rmw() -> str:
    from rclpy.utilities import get_rmw_implementation_identifier

    loaded = get_rmw_implementation_identifier()
    if os.environ.get("RMW_IMPLEMENTATION") != RMW or loaded != RMW:
        raise RuntimeError("timer worker requires explicit %s" % RMW)
    return loaded


def _cpp_name(value) -> str:
    return str(
        getattr(type(value), "__cpp_name__", "")
        or getattr(value, "__cpp_name__", "")
    )


def _event(args, event: str, **values) -> dict:
    return {
        "schema": EVENT_SCHEMA,
        "event": event,
        "variant": args.variant,
        "run_token": args.run_token,
        "pid": os.getpid(),
        "process_group_id": os.getpgrp(),
        **values,
    }


def _armed(args) -> dict:
    return _event(
        args,
        "armed",
        cpu_clock="CLOCK_PROCESS_CPUTIME_ID",
        timer_reset=True,
        measurement_starts_after_emit=False,
    )


def _measurement_ready(args) -> dict:
    return _event(
        args,
        "armed",
        cpu_clock="CLOCK_PROCESS_CPUTIME_ID",
        timer_reset=False,
        measurement_starts_after_emit=True,
    )


def _advance(state: dict) -> None:
    state["value"] = (
        state["value"] * RECURRENCE_MULTIPLIER + RECURRENCE_INCREMENT
    ) & MASK64
    state["checksum"] = (state["checksum"] + state["value"]) & MASK64


def _report(
    args,
    *,
    cpu_time_ns: int,
    wall_duration_ns: int,
    errors: list[int],
    state: int,
    checksum: int,
    python_callbacks: int,
    measured_python_callbacks: int,
    python_crossings: int,
    post_cancel_firings: int,
    exceptions: int,
    teardown_clean: bool,
    executor_thread_joined: bool,
) -> dict:
    interval_errors = consecutive_interval_errors(errors)
    phase_slip_periods = max_phase_slip_periods(errors)
    return _event(
        args,
        "report",
        warmup_firings=args.warmup_firings,
        measured_firings=args.measured_firings,
        recurrence_state=state,
        checksum=checksum,
        python_callback_count=python_callbacks,
        measured_python_callback_count=measured_python_callbacks,
        python_boundary_crossings=python_crossings,
        post_cancel_firings=post_cancel_firings,
        exceptions=exceptions,
        cpu_time_ns=cpu_time_ns,
        cpu_clock="CLOCK_PROCESS_CPUTIME_ID",
        wall_duration_ns=wall_duration_ns,
        scheduled_deadline_error=deadline_summary(errors),
        first_rearm_error_ns=errors[0],
        consecutive_interval_observations=len(interval_errors),
        consecutive_interval_error=deadline_summary(interval_errors),
        missed_periods=phase_slip_periods,
        max_phase_slip_periods=phase_slip_periods,
        timer_canceled=True,
        teardown_clean=teardown_clean,
        executor_thread_joined=executor_thread_joined,
    )


def _python_authority(args, *, activate: bool) -> int:
    active_product = None
    if activate:
        import rclcppyy as active_product

        active_product.enable_cpp_acceleration(profile="compatible")

    from rclpy.clock import Clock, ClockType
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    context = Context()
    context.init(args=[])
    node = Node(args.node_name, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    clock = Clock(clock_type=ClockType.STEADY_TIME)
    phase = {"name": "warmup"}
    warmup = {"count": 0}
    measured = {"count": 0}
    recurrence = {"value": RECURRENCE_SEED, "checksum": 0}
    errors = []
    timing = {}
    timer = None

    def callback():
        if phase["name"] == "warmup":
            warmup["count"] += 1
            if warmup["count"] == args.warmup_firings:
                timer.cancel()
                phase["name"] = "idle"
            return
        if phase["name"] != "measured":
            return
        measured["count"] += 1
        actual_ns = time.monotonic_ns()
        expected_ns = timing["epoch_ns"] + measured["count"] * PERIOD_NS
        errors.append(actual_ns - expected_ns)
        _advance(recurrence)
        if measured["count"] == args.measured_firings:
            timer.cancel()
            timing["cpu_stop_ns"] = time.process_time_ns()
            timing["wall_stop_ns"] = time.monotonic_ns()
            phase["name"] = "done"

    timer = node.create_timer(
        PERIOD_NS / 1e9,
        callback,
        clock=clock,
        autostart=True,
    )
    while warmup["count"] < args.warmup_firings:
        executor.spin_once(timeout_sec=0.1)

    activation = None
    if activate:
        status = active_product.status()
        records = [
            record for record in status["entities"]
            if record["metadata"].get("entity_type") == "timer"
        ]
        if not records or records[-1]["backend"] != "python":
            raise RuntimeError("compatible activation did not retain Python timer authority")
        activation = {
            "profile": "compatible",
            "timer_status_backend": "python",
            "timer_decision_id": records[-1]["id"],
        }
    ready = _event(
        args,
        "ready",
        node_name=args.node_name,
        loaded_rmw=_loaded_rmw(),
        execution_model=VARIANTS[args.variant]["execution_model"],
        warmup_firings=warmup["count"],
        timer_canceled=bool(timer.is_canceled()),
        timer_marker={
            "authority": "python",
            "implementation": "%s.%s" % (
                type(timer).__module__, type(timer).__qualname__),
            "clock": "steady",
            "period_ns": PERIOD_NS,
            "callback_language": "python",
        },
        executor_marker={
            "authority": "python",
            "implementation": "%s.%s" % (
                type(executor).__module__, type(executor).__qualname__),
            "kind": "single_threaded",
            "threads": 1,
        },
        cache=(
            {"state": "activation-only", "kind": "activation-only"}
            if activate else
            {"state": "not_applicable", "kind": "stock-rclpy"}
        ),
        **({"activation": activation} if activation is not None else {}),
    )
    _emit(ready)
    if sys.stdin.readline().rstrip("\n") != "START":
        raise RuntimeError("timer worker expected START")

    measured["count"] = 0
    recurrence.update(value=RECURRENCE_SEED, checksum=0)
    errors.clear()
    _emit(_measurement_ready(args))
    phase["name"] = "measured"
    timing["epoch_ns"] = time.monotonic_ns()
    timing["cpu_start_ns"] = time.process_time_ns()
    timer.reset()
    while measured["count"] < args.measured_firings:
        executor.spin_once(timeout_sec=0.1)

    canceled_count = measured["count"]
    for _ in range(3):
        executor.spin_once(timeout_sec=0.001)
    post_cancel = measured["count"] - canceled_count
    executor.remove_node(node)
    executor.shutdown(timeout_sec=2.0)
    node.destroy_timer(timer)
    node.destroy_node()
    context.shutdown()
    report = _report(
        args,
        cpu_time_ns=timing["cpu_stop_ns"] - timing["cpu_start_ns"],
        wall_duration_ns=timing["wall_stop_ns"] - timing["epoch_ns"],
        errors=errors,
        state=recurrence["value"],
        checksum=recurrence["checksum"],
        python_callbacks=args.warmup_firings + args.measured_firings,
        measured_python_callbacks=args.measured_firings,
        python_crossings=args.warmup_firings + args.measured_firings,
        post_cancel_firings=post_cancel,
        exceptions=0,
        teardown_clean=not context.ok(),
        executor_thread_joined=True,
    )
    _emit(report)
    return 0


def _direct_cpp_python_callback(args) -> int:
    import rclcppyy as active_product

    active_product.enable_cpp_acceleration(profile="direct_cpp")

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclcppyy import direct_cpp

    rclpy.init(args=[])
    node = Node(args.node_name)
    runtime = direct_cpp._runtime()
    executor_surface = VARIANTS[args.variant]["executor_surface"]
    executor = None
    native_executor = None

    if args.variant == "direct-public-ste":
        executor = SingleThreadedExecutor(context=node.context)
        if not executor.add_node(node):
            raise RuntimeError("public direct STE did not acquire the benchmark node")
        native_executor = executor.native_executor

        def spin_once(timeout_sec):
            executor.spin_once(timeout_sec=timeout_sec)

    elif args.variant == "direct-raw-ste-control":
        import cppyy

        native_executor = runtime.session.create_executor("single_threaded")
        native_executor.add_node(node._direct_cpp_node)

        def spin_once(timeout_sec):
            duration = cppyy.gbl.std.chrono.nanoseconds(int(timeout_sec * 1e9))
            native_executor.spin_once(duration)
            node._poll_direct_entities()

    else:
        def spin_once(timeout_sec):
            rclpy.spin_once(node, timeout_sec=timeout_sec)

    phase = {"name": "warmup"}
    warmup = {"count": 0}
    measured = {"count": 0}
    recurrence = {"value": RECURRENCE_SEED, "checksum": 0}
    errors = []
    timing = {}
    timer = None

    def callback():
        if phase["name"] == "warmup":
            warmup["count"] += 1
            if warmup["count"] == args.warmup_firings:
                timer.cancel()
                phase["name"] = "idle"
            return
        if phase["name"] != "measured":
            return
        measured["count"] += 1
        actual_ns = time.monotonic_ns()
        expected_ns = timing["epoch_ns"] + measured["count"] * PERIOD_NS
        errors.append(actual_ns - expected_ns)
        _advance(recurrence)
        if measured["count"] == args.measured_firings:
            timer.cancel()
            timing["cpu_stop_ns"] = time.process_time_ns()
            timing["wall_stop_ns"] = time.monotonic_ns()
            phase["name"] = "done"

    timer = node.create_timer(PERIOD_NS / 1e9, callback, autostart=True)
    while warmup["count"] < args.warmup_firings:
        spin_once(0.1)

    if native_executor is None:
        global_executor = runtime._global_executor
        if global_executor is None:
            raise RuntimeError("direct global executor was not created during warmup")
        native_executor = global_executor.native_executor

    records = [
        record for record in active_product.status()["entities"]
        if record["metadata"].get("entity_type") == "timer"
    ]
    if not records or records[-1]["backend"] != "cpp":
        raise RuntimeError("direct timer did not record native authority")
    timer_record = records[-1]
    executor_type = _cpp_name(native_executor)
    activation = {
        "profile": "direct_cpp",
        "timer_status_backend": "cpp",
        "timer_decision_id": timer_record["id"],
        "timer_creation_route": timer_record["metadata"]["creation_route"],
        "callback_handoff": timer_record["metadata"]["callback_handoff"],
        "native_timer_type": timer_record["metadata"]["native_type"],
        "native_executor_type": executor_type,
        "executor_surface": executor_surface,
        "executor_session_owned": (
            runtime.session is not None
            and any(
                native_executor is candidate
                for candidate in runtime.session.executors
            )
        ),
    }
    _emit(_event(
        args,
        "ready",
        node_name=args.node_name,
        loaded_rmw=_loaded_rmw(),
        execution_model=VARIANTS[args.variant]["execution_model"],
        warmup_firings=warmup["count"],
        timer_canceled=timer.is_canceled(),
        timer_marker={
            "authority": "cpp",
            "implementation": timer.__cpp_name__,
            "clock": "steady",
            "period_ns": timer.timer_period_ns,
            "callback_language": "python",
        },
        executor_marker={
            "authority": "cpp",
            "implementation": executor_type,
            "kind": "single_threaded",
            "threads": 1,
        },
        cache={"state": "process_warm", "kind": "direct-rclcpp-runtime"},
        activation=activation,
    ))
    if sys.stdin.readline().rstrip("\n") != "START":
        raise RuntimeError("timer worker expected START")

    measured["count"] = 0
    recurrence.update(value=RECURRENCE_SEED, checksum=0)
    errors.clear()
    _emit(_measurement_ready(args))
    phase["name"] = "measured"
    timing["epoch_ns"] = time.monotonic_ns()
    timing["cpu_start_ns"] = time.process_time_ns()
    timer.reset()
    while measured["count"] < args.measured_firings:
        spin_once(0.1)

    canceled_count = measured["count"]
    for _ in range(3):
        spin_once(0.001)
    post_cancel = measured["count"] - canceled_count
    timer_canceled = timer.is_canceled()
    timer_destroyed = node.destroy_timer(timer)
    if args.variant == "direct-public-ste":
        executor.remove_node(node)
        executor_shutdown = executor.shutdown(timeout_sec=2.0)
    elif args.variant == "direct-raw-ste-control":
        native_executor.remove_node(node._direct_cpp_node)
        executor_shutdown = node.executor is None
    else:
        executor_shutdown = True
    node.destroy_node()
    rclpy.shutdown()
    teardown_clean = (
        timer_canceled
        and timer_destroyed
        and executor_shutdown
        and not rclpy.ok()
        and runtime.session is None
        and runtime.executor is None
        and not runtime.nodes
    )
    _emit(_report(
        args,
        cpu_time_ns=timing["cpu_stop_ns"] - timing["cpu_start_ns"],
        wall_duration_ns=timing["wall_stop_ns"] - timing["epoch_ns"],
        errors=errors,
        state=recurrence["value"],
        checksum=recurrence["checksum"],
        python_callbacks=args.warmup_firings + args.measured_firings,
        measured_python_callbacks=args.measured_firings,
        python_crossings=args.warmup_firings + args.measured_firings,
        post_cancel_firings=post_cancel,
        exceptions=0,
        teardown_clean=teardown_clean,
        executor_thread_joined=True,
    ))
    return 0


def _probe_sources() -> tuple[str, str]:
    declarations = r"""
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
namespace rclcppyy_timer_benchmark {
class Probe {
public:
  virtual ~Probe() = default;
  virtual bool wait_warmup(uint64_t timeout_ms) = 0;
  virtual void arm() = 0;
  virtual bool wait_measured(uint64_t timeout_ms) = 0;
  virtual uint64_t warmup_firings() const = 0;
  virtual uint64_t measured_firings() const = 0;
  virtual uint64_t recurrence_state() const = 0;
  virtual uint64_t checksum() const = 0;
  virtual uint64_t cpu_time_ns() const = 0;
  virtual uint64_t wall_duration_ns() const = 0;
  virtual uint64_t exceptions() const = 0;
  virtual uint64_t post_cancel_firings() const = 0;
  virtual bool timer_canceled() const = 0;
  virtual bool verify_no_post_cancel(uint64_t wait_ns) = 0;
  virtual std::vector<int64_t> deadline_errors() const = 0;
  virtual void close() = 0;
};
std::shared_ptr<Probe> make_probe(
  std::shared_ptr<rclcpp::Node> node,
  uint64_t period_ns,
  uint64_t warmup_firings,
  uint64_t measured_firings);
}
"""
    code = declarations + r"""
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace {
uint64_t process_cpu_ns()
{
  timespec value{};
  if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &value) != 0) {
    throw std::runtime_error("CLOCK_PROCESS_CPUTIME_ID failed");
  }
  return static_cast<uint64_t>(value.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(value.tv_nsec);
}

uint64_t steady_ns()
{
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}
}

namespace rclcppyy_timer_benchmark {
class ProbeImpl final : public Probe {
public:
  ProbeImpl(
    std::shared_ptr<rclcpp::Node> node,
    uint64_t period_ns,
    uint64_t warmup_target,
    uint64_t measured_target)
  : period_ns_(period_ns),
    warmup_target_(warmup_target),
    measured_target_(measured_target)
  {
    errors_.reserve(measured_target_);
    timer_ = node->create_wall_timer(
      std::chrono::nanoseconds(period_ns_), [this]() { on_timer(); });
  }

  ~ProbeImpl() override { close(); }

  bool wait_warmup(uint64_t timeout_ms) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(
      lock, std::chrono::milliseconds(timeout_ms), [this]() { return warmup_done_; });
  }

  void arm() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!warmup_done_ || !timer_ || !timer_->is_canceled()) {
      throw std::runtime_error("timer probe cannot arm before canceled warmup");
    }
    measured_.store(0);
    post_cancel_.store(0);
    exceptions_.store(0);
    state_ = %(seed)dULL;
    checksum_ = 0;
    errors_.clear();
    epoch_ns_ = steady_ns();
    cpu_start_ns_ = process_cpu_ns();
    phase_.store(1);
    timer_->reset();
  }

  bool wait_measured(uint64_t timeout_ms) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(
      lock, std::chrono::milliseconds(timeout_ms), [this]() { return measured_done_; });
  }

  uint64_t warmup_firings() const override { return warmup_.load(); }
  uint64_t measured_firings() const override { return measured_.load(); }
  uint64_t recurrence_state() const override { return state_; }
  uint64_t checksum() const override { return checksum_; }
  uint64_t cpu_time_ns() const override { return cpu_stop_ns_ - cpu_start_ns_; }
  uint64_t wall_duration_ns() const override { return wall_stop_ns_ - epoch_ns_; }
  uint64_t exceptions() const override { return exceptions_.load(); }
  uint64_t post_cancel_firings() const override { return post_cancel_.load(); }
  bool timer_canceled() const override { return timer_ && timer_->is_canceled(); }

  bool verify_no_post_cancel(uint64_t wait_ns) override
  {
    const auto before = measured_.load();
    std::this_thread::sleep_for(std::chrono::nanoseconds(wait_ns));
    return measured_.load() == before && post_cancel_.load() == 0;
  }

  std::vector<int64_t> deadline_errors() const override { return errors_; }

  void close() override
  {
    if (timer_) {
      timer_->cancel();
      timer_.reset();
    }
  }

private:
  void on_timer() noexcept
  {
    try {
      const auto phase = phase_.load();
      if (phase == 0) {
        const auto count = warmup_.fetch_add(1) + 1;
        if (count == warmup_target_) {
          timer_->cancel();
          std::lock_guard<std::mutex> lock(mutex_);
          warmup_done_ = true;
          condition_.notify_all();
        }
        return;
      }
      if (phase != 1) {
        post_cancel_.fetch_add(1);
        return;
      }
      const auto count = measured_.fetch_add(1) + 1;
      const auto actual_ns = steady_ns();
      const auto expected_ns = epoch_ns_ + count * period_ns_;
      errors_.push_back(
        static_cast<int64_t>(actual_ns) - static_cast<int64_t>(expected_ns));
      state_ = state_ * %(multiplier)dULL + %(increment)dULL;
      checksum_ += state_;
      if (count == measured_target_) {
        timer_->cancel();
        cpu_stop_ns_ = process_cpu_ns();
        wall_stop_ns_ = steady_ns();
        phase_.store(2);
        std::lock_guard<std::mutex> lock(mutex_);
        measured_done_ = true;
        condition_.notify_all();
      }
    } catch (...) {
      exceptions_.fetch_add(1);
      if (timer_) {
        timer_->cancel();
      }
      std::lock_guard<std::mutex> lock(mutex_);
      measured_done_ = true;
      condition_.notify_all();
    }
  }

  const uint64_t period_ns_;
  const uint64_t warmup_target_;
  const uint64_t measured_target_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<int> phase_{0};
  std::atomic<uint64_t> warmup_{0};
  std::atomic<uint64_t> measured_{0};
  std::atomic<uint64_t> exceptions_{0};
  std::atomic<uint64_t> post_cancel_{0};
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool warmup_done_{false};
  bool measured_done_{false};
  uint64_t epoch_ns_{0};
  uint64_t cpu_start_ns_{0};
  uint64_t cpu_stop_ns_{0};
  uint64_t wall_stop_ns_{0};
  uint64_t state_{%(seed)dULL};
  uint64_t checksum_{0};
  std::vector<int64_t> errors_;
};

std::shared_ptr<Probe> make_probe(
  std::shared_ptr<rclcpp::Node> node,
  uint64_t period_ns,
  uint64_t warmup_firings,
  uint64_t measured_firings)
{
  return std::make_shared<ProbeImpl>(
    std::move(node), period_ns, warmup_firings, measured_firings);
}
}
""" % {
        "seed": RECURRENCE_SEED,
        "multiplier": RECURRENCE_MULTIPLIER,
        "increment": RECURRENCE_INCREMENT,
    }
    return code, declarations


def _probe_cache_dir() -> str:
    root = os.environ.get("XDG_CACHE_HOME")
    if not root:
        raise RuntimeError("timer helper requires an isolated XDG_CACHE_HOME")
    return os.path.join(root, "rclcppyy", "timer-executor")


def _compile_probe() -> dict:
    import cppyy_kit
    from rclcpp_kit.bringup_rclcpp import get_ros2_lib_path, ros2_include_paths

    code, declarations = _probe_sources()
    return cppyy_kit.cppdef_cached(
        code,
        decls=declarations,
        name="rclcppyy_timer_probe_%s" % PROBE_SOURCE_ID,
        include_paths=tuple(sorted(ros2_include_paths())),
        library_paths=(get_ros2_lib_path(),),
        libraries=("rclcpp",),
        directory=_probe_cache_dir(),
    )


def _artifact(result: dict) -> dict:
    path = result.get("so")
    if not path or not Path(path).is_file():
        raise RuntimeError("timer helper produced no shared library")
    resolved = Path(path).resolve()
    return {
        "cached": bool(result.get("cached")),
        "path": str(resolved),
        "sha256": _sha256(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def _prewarm() -> int:
    from rclcpp_kit.native import native

    with native(["timer-executor-prewarm"]):
        result = _compile_probe()
        loaded = _loaded_rmw()
    _emit({
        "schema": PREWARM_SCHEMA,
        "pid": os.getpid(),
        "loaded_rmw": loaded,
        "source_id": PROBE_SOURCE_ID,
        "artifact": _artifact(result),
    })
    return 0


def _native_python_callback(args) -> int:
    from rclcpp_kit.native import native

    session = native(["timer-executor-native-python"])
    warmup_done = threading.Event()
    measured_done = threading.Event()
    phase = {"name": "warmup"}
    warmup = {"count": 0}
    measured = {"count": 0}
    recurrence = {"value": RECURRENCE_SEED, "checksum": 0}
    errors = []
    timing = {}
    timer = None
    executor_thread = None
    thread_exceptions = 0

    def callback():
        if phase["name"] == "warmup":
            warmup["count"] += 1
            if warmup["count"] == args.warmup_firings:
                timer.cancel()
                phase["name"] = "idle"
                warmup_done.set()
            return
        if phase["name"] != "measured":
            return
        measured["count"] += 1
        actual_ns = time.monotonic_ns()
        errors.append(actual_ns - (
            timing["epoch_ns"] + measured["count"] * PERIOD_NS))
        _advance(recurrence)
        if measured["count"] == args.measured_firings:
            timer.cancel()
            timing["cpu_stop_ns"] = time.process_time_ns()
            timing["wall_stop_ns"] = time.monotonic_ns()
            phase["name"] = "done"
            measured_done.set()

    session.open()
    node = session.create_node(args.node_name)
    timer = node.create_timer(PERIOD_NS / 1e9, callback)
    executor = session.create_executor("single_threaded", threads=1)
    executor.add_node(node)
    executor_thread = session.start_executor(executor)
    if not warmup_done.wait(timeout=30.0):
        raise RuntimeError("native Python timer warmup timed out")
    _emit(_event(
        args,
        "ready",
        node_name=args.node_name,
        loaded_rmw=_loaded_rmw(),
        execution_model=VARIANTS[args.variant]["execution_model"],
        warmup_firings=warmup["count"],
        timer_canceled=bool(timer.is_canceled()),
        timer_marker={
            "authority": "cpp",
            "implementation": _cpp_name(timer),
            "clock": "steady",
            "period_ns": PERIOD_NS,
            "callback_language": "python",
        },
        executor_marker={
            "authority": "cpp",
            "implementation": _cpp_name(executor),
            "kind": "single_threaded",
            "threads": 1,
        },
        cache={"state": "process_warm", "kind": "managed-rclcpp-runtime"},
    ))
    if sys.stdin.readline().rstrip("\n") != "START":
        raise RuntimeError("timer worker expected START")
    measured["count"] = 0
    recurrence.update(value=RECURRENCE_SEED, checksum=0)
    errors.clear()
    _emit(_measurement_ready(args))
    phase["name"] = "measured"
    timing["epoch_ns"] = time.monotonic_ns()
    timing["cpu_start_ns"] = time.process_time_ns()
    timer.reset()
    if not measured_done.wait(timeout=30.0):
        raise RuntimeError("native Python timer measurement timed out")
    canceled_count = measured["count"]
    time.sleep(0.003)
    post_cancel = measured["count"] - canceled_count
    thread_exceptions = int(executor_thread.exceptions)
    timer.cancel()
    session.close()
    joined = bool(executor_thread.closed)
    report = _report(
        args,
        cpu_time_ns=timing["cpu_stop_ns"] - timing["cpu_start_ns"],
        wall_duration_ns=timing["wall_stop_ns"] - timing["epoch_ns"],
        errors=errors,
        state=recurrence["value"],
        checksum=recurrence["checksum"],
        python_callbacks=args.warmup_firings + args.measured_firings,
        measured_python_callbacks=args.measured_firings,
        python_crossings=args.warmup_firings + args.measured_firings,
        post_cancel_firings=post_cancel,
        exceptions=thread_exceptions,
        teardown_clean=session.closed,
        executor_thread_joined=joined,
    )
    _emit(report)
    return 0


def _native_cpp_callback(args) -> int:
    import cppyy
    from rclcpp_kit.native import native

    session = native(["timer-executor-native-cpp"])
    session.open()
    result = _compile_probe()
    artifact = _artifact(result)
    if not artifact["cached"]:
        raise RuntimeError("native C++ timer helper was not prewarmed")
    node = session.create_node(args.node_name)
    factory = cppyy.gbl.rclcppyy_timer_benchmark.make_probe
    probe = factory(node, PERIOD_NS, args.warmup_firings, args.measured_firings)
    executor = session.create_executor("single_threaded", threads=1)
    executor.add_node(node)
    executor_thread = session.start_executor(executor)
    if not probe.wait_warmup(30_000):
        raise RuntimeError("native C++ timer warmup timed out")
    _emit(_event(
        args,
        "ready",
        node_name=args.node_name,
        loaded_rmw=_loaded_rmw(),
        execution_model=VARIANTS[args.variant]["execution_model"],
        warmup_firings=int(probe.warmup_firings()),
        timer_canceled=bool(probe.timer_canceled()),
        timer_marker={
            "authority": "cpp",
            "implementation": "rclcppyy_timer_benchmark::Probe",
            "clock": "steady",
            "period_ns": PERIOD_NS,
            "callback_language": "cpp",
        },
        executor_marker={
            "authority": "cpp",
            "implementation": _cpp_name(executor),
            "kind": "single_threaded",
            "threads": 1,
        },
        cache={
            "state": "prebuilt",
            "kind": "timer-probe-shared-library",
            "path": artifact["path"],
            "sha256": artifact["sha256"],
            "size_bytes": artifact["size_bytes"],
            "hit": True,
            "source_id": PROBE_SOURCE_ID,
        },
    ))
    if sys.stdin.readline().rstrip("\n") != "START":
        raise RuntimeError("timer worker expected START")
    probe.arm()
    _emit(_armed(args))
    if not probe.wait_measured(30_000):
        raise RuntimeError("native C++ timer measurement timed out")
    if not probe.verify_no_post_cancel(3 * PERIOD_NS):
        raise RuntimeError("native C++ timer fired after cancellation")
    errors = [int(value) for value in probe.deadline_errors()]
    values = {
        "cpu_time_ns": int(probe.cpu_time_ns()),
        "wall_duration_ns": int(probe.wall_duration_ns()),
        "state": int(probe.recurrence_state()),
        "checksum": int(probe.checksum()),
        "post_cancel": int(probe.post_cancel_firings()),
        "exceptions": int(probe.exceptions()) + int(executor_thread.exceptions),
    }
    probe.close()
    session.close()
    report = _report(
        args,
        cpu_time_ns=values["cpu_time_ns"],
        wall_duration_ns=values["wall_duration_ns"],
        errors=errors,
        state=values["state"],
        checksum=values["checksum"],
        python_callbacks=0,
        measured_python_callbacks=0,
        python_crossings=0,
        post_cancel_firings=values["post_cancel"],
        exceptions=values["exceptions"],
        teardown_clean=session.closed,
        executor_thread_joined=executor_thread.closed,
    )
    _emit(report)
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prewarm", action="store_true")
    parser.add_argument("--variant", choices=tuple(VARIANTS)[:-1])
    parser.add_argument("--node-name")
    parser.add_argument("--run-token")
    parser.add_argument("--warmup-firings", type=int)
    parser.add_argument("--measured-firings", type=int)
    return parser


def main() -> int:
    args = _parser().parse_args()
    if args.prewarm:
        return _prewarm()
    required = ("variant", "node_name", "run_token", "warmup_firings", "measured_firings")
    missing = [name for name in required if getattr(args, name) is None]
    if missing:
        raise SystemExit("timer worker requires: " + ", ".join(missing))
    if args.warmup_firings <= 0 or args.measured_firings < 2:
        raise SystemExit("timer requires positive warmup and at least two measured firings")
    if args.variant == "stock-rclpy":
        return _python_authority(args, activate=False)
    if args.variant == "compatible-rclcppyy":
        return _python_authority(args, activate=True)
    if args.variant in (
        "direct-cpp-rclcppyy",
        "direct-public-ste",
        "direct-raw-ste-control",
    ):
        return _direct_cpp_python_callback(args)
    if args.variant == "native-python-callback":
        return _native_python_callback(args)
    return _native_cpp_callback(args)


if __name__ == "__main__":
    raise SystemExit(main())
