#!/usr/bin/env python3
"""Run one dynamic lane of the controlled action-client benchmark."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import resource
import sys
import time

from _action_client_protocol import (
    ACTION_TYPE,
    CLIENT_SCHEMA,
    FEEDBACK_PER_GOAL,
    PREWARM_SCHEMA,
    QOS,
    RSS_LIMIT_BYTES,
    RMW,
    VARIANTS,
    endpoint_names,
    expected_checksum,
    expected_crossings,
    goal_strings,
    latency_summary,
)


PROTOCOL_PREFIX = "@@RCLCPPYY_ACTION_CLIENT_V1@@"
STATE_MACHINE_SOURCE_ID = hashlib.sha256(
    b"rclcppyy-action-state-machine-v1:tf2_msgs/LookupTransform"
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
        raise RuntimeError("action worker requires explicit %s" % RMW)
    return loaded


def _peak_rss_bytes() -> int:
    return int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) * 1024


def _rss_guard(baseline: int, final_value: int) -> dict:
    growth = max(0, final_value - baseline)
    return {
        "kind": "post-warmup-peak-rss-growth",
        "unit": "bytes",
        "baseline_peak_bytes": baseline,
        "final_peak_bytes": final_value,
        "growth_bytes": growth,
        "limit_bytes": RSS_LIMIT_BYTES,
        "within_limit": growth <= RSS_LIMIT_BYTES,
    }


def _artifact(result: dict) -> dict:
    path = result.get("so")
    if not path or not Path(path).is_file():
        raise RuntimeError("action cache produced no shared library")
    resolved = Path(path).resolve()
    return {
        "cached": bool(result.get("cached")),
        "path": str(resolved),
        "sha256": _sha256(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def _event(args, event: str, **values) -> dict:
    return {
        "schema": CLIENT_SCHEMA,
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
        measurement_reset=True,
    )


def _wait_until(predicate, timeout: float, label: str, polls: list[int]) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        polls[0] += 1
        if predicate():
            return
        time.sleep(0.0001)
    raise RuntimeError("timed out waiting for %s" % label)


def _cache_dir() -> str:
    root = os.environ.get("XDG_CACHE_HOME")
    if not root:
        raise RuntimeError("action benchmark requires isolated XDG_CACHE_HOME")
    return os.path.join(root, "rclcppyy", "action-client")


def _state_machine_sources() -> tuple[str, str]:
    interface = "StateMachine_%s" % STATE_MACHINE_SOURCE_ID
    factory = "make_state_machine_%s" % STATE_MACHINE_SOURCE_ID
    declarations = r"""
#include <cstdint>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
namespace rclcppyy_action_benchmark {
class %(interface)s {
public:
  virtual ~%(interface)s() = default;
  virtual bool wait_for_server(uint64_t timeout_ns) const = 0;
  virtual void run_warmup(uint64_t goals) = 0;
  virtual void arm(uint64_t goals) = 0;
  virtual void run_measured() = 0;
  virtual uint64_t goals_sent() const = 0;
  virtual uint64_t goals_accepted() const = 0;
  virtual uint64_t feedback_received() const = 0;
  virtual uint64_t results_received() const = 0;
  virtual uint64_t terminal_succeeded() const = 0;
  virtual uint64_t measured_checksum() const = 0;
  virtual uint64_t last_sequence() const = 0;
  virtual uint64_t exceptions() const = 0;
  virtual uint64_t active_goals() const = 0;
  virtual uint64_t cpu_time_ns() const = 0;
  virtual uint64_t wall_duration_ns() const = 0;
  virtual uint64_t rss_baseline() const = 0;
  virtual uint64_t rss_final() const = 0;
  virtual uint64_t accept_p50() const = 0;
  virtual uint64_t accept_p95() const = 0;
  virtual uint64_t accept_p99() const = 0;
  virtual uint64_t accept_max() const = 0;
  virtual uint64_t feedback_p50() const = 0;
  virtual uint64_t feedback_p95() const = 0;
  virtual uint64_t feedback_p99() const = 0;
  virtual uint64_t feedback_max() const = 0;
  virtual uint64_t result_p50() const = 0;
  virtual uint64_t result_p95() const = 0;
  virtual uint64_t result_p99() const = 0;
  virtual uint64_t result_max() const = 0;
  virtual void close() = 0;
};
std::shared_ptr<%(interface)s> %(factory)s(
  std::shared_ptr<rclcpp::Node> node, const std::string& action_name);
}
""" % {"interface": interface, "factory": factory}
    code = declarations + r"""
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_msgs/action/lookup_transform.hpp>
#include <tf2_msgs/msg/tf2_error.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <ctime>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>
#include <sys/resource.h>

namespace rclcppyy_action_benchmark {
namespace {
using Action = tf2_msgs::action::LookupTransform;
using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
constexpr uint64_t kFeedbackPerGoal = 3;

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

uint64_t peak_rss_bytes()
{
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0) {
    throw std::runtime_error("getrusage failed");
  }
  return static_cast<uint64_t>(usage.ru_maxrss) * 1024ULL;
}

std::string goal_base(const std::string& phase, uint64_t sequence)
{
  return "rclcppyy/action-benchmark/" + phase + "/" +
         std::to_string(sequence);
}

uint64_t nearest_rank(std::vector<uint64_t> values, int percentile)
{
  if (values.empty()) {
    throw std::runtime_error("action latency set is empty");
  }
  std::sort(values.begin(), values.end());
  const auto rank = std::max<std::size_t>(
    1, static_cast<std::size_t>(std::ceil(percentile / 100.0 * values.size())));
  return values[rank - 1];
}

struct Observation {
  std::atomic<uint64_t> feedback{0};
  std::atomic<uint64_t> first_feedback_ns{0};
  std::atomic<uint64_t> invalid_feedback{0};
};
}

class StateMachineImpl_%(source_id)s final : public %(interface)s {
public:
  StateMachineImpl_%(source_id)s(
      std::shared_ptr<rclcpp::Node> node, const std::string& action_name)
  {
    client_ = rclcpp_action::create_client<Action>(node, action_name);
  }

  ~StateMachineImpl_%(source_id)s() override { close(); }

  bool wait_for_server(uint64_t timeout_ns) const override
  {
    return client_ && client_->wait_for_action_server(
      std::chrono::nanoseconds(timeout_ns));
  }

  void run_warmup(uint64_t goals) override
  {
    run_phase("warmup", goals, false);
  }

  void arm(uint64_t goals) override
  {
    measured_target_ = goals;
    measured_checksum_ = 0;
    last_sequence_ = 0;
    accept_.clear();
    feedback_.clear();
    result_.clear();
    accept_.reserve(goals);
    feedback_.reserve(goals);
    result_.reserve(goals);
    rss_baseline_ = peak_rss_bytes();
    wall_start_ns_ = steady_ns();
    cpu_start_ns_ = process_cpu_ns();
  }

  void run_measured() override
  {
    run_phase("measured", measured_target_, true);
    cpu_stop_ns_ = process_cpu_ns();
    wall_stop_ns_ = steady_ns();
    rss_final_ = peak_rss_bytes();
  }

  uint64_t goals_sent() const override { return goals_sent_; }
  uint64_t goals_accepted() const override { return goals_accepted_; }
  uint64_t feedback_received() const override { return feedback_received_; }
  uint64_t results_received() const override { return results_received_; }
  uint64_t terminal_succeeded() const override { return terminal_succeeded_; }
  uint64_t measured_checksum() const override { return measured_checksum_; }
  uint64_t last_sequence() const override { return last_sequence_; }
  uint64_t exceptions() const override { return exceptions_; }
  uint64_t active_goals() const override { return active_goals_; }
  uint64_t cpu_time_ns() const override { return cpu_stop_ns_ - cpu_start_ns_; }
  uint64_t wall_duration_ns() const override { return wall_stop_ns_ - wall_start_ns_; }
  uint64_t rss_baseline() const override { return rss_baseline_; }
  uint64_t rss_final() const override { return rss_final_; }
  uint64_t accept_p50() const override { return nearest_rank(accept_, 50); }
  uint64_t accept_p95() const override { return nearest_rank(accept_, 95); }
  uint64_t accept_p99() const override { return nearest_rank(accept_, 99); }
  uint64_t accept_max() const override { return *std::max_element(accept_.begin(), accept_.end()); }
  uint64_t feedback_p50() const override { return nearest_rank(feedback_, 50); }
  uint64_t feedback_p95() const override { return nearest_rank(feedback_, 95); }
  uint64_t feedback_p99() const override { return nearest_rank(feedback_, 99); }
  uint64_t feedback_max() const override { return *std::max_element(feedback_.begin(), feedback_.end()); }
  uint64_t result_p50() const override { return nearest_rank(result_, 50); }
  uint64_t result_p95() const override { return nearest_rank(result_, 95); }
  uint64_t result_p99() const override { return nearest_rank(result_, 99); }
  uint64_t result_max() const override { return *std::max_element(result_.begin(), result_.end()); }

  void close() override { client_.reset(); }

private:
  void run_phase(const std::string& phase, uint64_t goals, bool measured)
  {
    for (uint64_t sequence = 1; sequence <= goals; ++sequence) {
      run_goal(phase, sequence, measured);
    }
  }

  void run_goal(const std::string& phase, uint64_t sequence, bool measured)
  {
    ++active_goals_;
    const auto base = goal_base(phase, sequence);
    auto goal = std::make_shared<Action::Goal>();
    goal->target_frame = base + "/target";
    goal->source_frame = base + "/source";
    auto observation = std::make_shared<Observation>();
    typename rclcpp_action::Client<Action>::SendGoalOptions options;
    options.feedback_callback =
      [observation](
        std::shared_ptr<GoalHandle>, const std::shared_ptr<const Action::Feedback> feedback) {
        const auto index = observation->feedback.fetch_add(1) + 1;
        if (index == 1) {
          observation->first_feedback_ns.store(steady_ns());
        }
        if (!feedback || feedback->structure_needs_at_least_one_member != 0) {
          observation->invalid_feedback.fetch_add(1);
        }
      };
    const auto send_ns = steady_ns();
    auto goal_future = client_->async_send_goal(*goal, options);
    ++goals_sent_;
    if (goal_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      ++exceptions_;
      --active_goals_;
      throw std::runtime_error("native C++ action goal response timed out");
    }
    const auto accept_ns = steady_ns();
    const auto handle = goal_future.get();
    if (!handle) {
      --active_goals_;
      throw std::runtime_error("native C++ action goal was rejected");
    }
    ++goals_accepted_;
    auto result_future = client_->async_get_result(handle);
    if (result_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      ++exceptions_;
      --active_goals_;
      throw std::runtime_error("native C++ action result timed out");
    }
    const auto result_ns = steady_ns();
    const auto feedback_deadline = std::chrono::steady_clock::now() +
      std::chrono::seconds(2);
    while (observation->feedback.load() < kFeedbackPerGoal &&
      std::chrono::steady_clock::now() < feedback_deadline)
    {
      std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
    if (observation->feedback.load() != kFeedbackPerGoal ||
        observation->invalid_feedback.load() != 0) {
      --active_goals_;
      throw std::runtime_error(
        "native C++ action feedback contract failed: feedback=" +
        std::to_string(observation->feedback.load()) + " invalid=" +
        std::to_string(observation->invalid_feedback.load()));
    }
    feedback_received_ += kFeedbackPerGoal;
    const auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
        wrapped.result->transform.header.frame_id != phase ||
        wrapped.result->transform.child_frame_id != std::to_string(sequence) ||
        wrapped.result->error.error != tf2_msgs::msg::TF2Error::NO_ERROR ||
        !wrapped.result->error.error_string.empty()) {
      --active_goals_;
      throw std::runtime_error("native C++ action terminal result contract failed");
    }
    ++results_received_;
    ++terminal_succeeded_;
    --active_goals_;
    if (measured) {
      accept_.push_back(accept_ns - send_ns);
      feedback_.push_back(observation->first_feedback_ns.load() - send_ns);
      result_.push_back(result_ns - send_ns);
      measured_checksum_ += sequence;
      last_sequence_ = sequence;
    }
  }

  rclcpp_action::Client<Action>::SharedPtr client_;
  uint64_t measured_target_{0};
  uint64_t goals_sent_{0};
  uint64_t goals_accepted_{0};
  uint64_t feedback_received_{0};
  uint64_t results_received_{0};
  uint64_t terminal_succeeded_{0};
  uint64_t measured_checksum_{0};
  uint64_t last_sequence_{0};
  uint64_t exceptions_{0};
  uint64_t active_goals_{0};
  uint64_t cpu_start_ns_{0};
  uint64_t cpu_stop_ns_{0};
  uint64_t wall_start_ns_{0};
  uint64_t wall_stop_ns_{0};
  uint64_t rss_baseline_{0};
  uint64_t rss_final_{0};
  std::vector<uint64_t> accept_;
  std::vector<uint64_t> feedback_;
  std::vector<uint64_t> result_;
};

std::shared_ptr<%(interface)s> %(factory)s(
  std::shared_ptr<rclcpp::Node> node, const std::string& action_name)
{
  return std::make_shared<StateMachineImpl_%(source_id)s>(
    std::move(node), action_name);
}
}
""" % {
        "source_id": STATE_MACHINE_SOURCE_ID,
        "interface": interface,
        "factory": factory,
    }
    return code, declarations


def _compile_state_machine() -> dict:
    import cppyy_kit
    from cppyy_kit.cache import artifact_paths
    from rclcpp_kit.bringup_rclcpp import get_ros2_lib_path, ros2_include_paths

    code, declarations = _state_machine_sources()
    options = {
        "decls": declarations,
        "name": "rclcppyy_action_state_machine_%s" % STATE_MACHINE_SOURCE_ID,
        "include_paths": tuple(sorted(ros2_include_paths())),
        "library_paths": (get_ros2_lib_path(),),
        "libraries": (
            "rclcpp_action", "rclcpp", "tf2_msgs__rosidl_typesupport_cpp"),
        "directory": _cache_dir(),
    }
    so_path = artifact_paths(
        code,
        declarations,
        options["name"],
        options["include_paths"],
        options["libraries"],
        directory=options["directory"],
    )[0]
    was_cached = os.path.exists(so_path)
    cppyy_kit.prebuild(code, **options)
    result = cppyy_kit.cppdef_cached(code, **options)
    if not was_cached:
        result = dict(result)
        result.update(cached=False, reason="prebuilt-miss")
    return result


def _ready(
    args,
    *,
    authority: str,
    goal_representation: str,
    executor_implementation: str,
    cache: dict,
    activation: dict | None = None,
) -> dict:
    document = _event(
        args,
        "ready",
        node_name=args.node_name,
        action_name=args.action_name,
        loaded_rmw=_loaded_rmw(),
        action_type=ACTION_TYPE,
        execution_model=VARIANTS[args.variant]["execution_model"],
        action_authority=authority,
        action_implementation=VARIANTS[args.variant]["action_implementation"],
        goal_representation=goal_representation,
        qos=QOS,
        endpoints=endpoint_names(args.action_name),
        executor={
            "authority": authority,
            "kind": "single_threaded",
            "threads": 1,
            "implementation": executor_implementation,
        },
        cache=cache,
        warmup_goals=args.warmup_goals,
        warmup_checksum=expected_checksum(args.warmup_goals),
        warmup_feedback=args.warmup_goals * FEEDBACK_PER_GOAL,
        warmup_results=args.warmup_goals,
        warmup_terminal_success=args.warmup_goals,
        active_goals=0,
        pending_operations=0,
    )
    if activation is not None:
        document["activation"] = activation
    return document


def _report(
    args,
    *,
    counters: dict,
    cpu_time_ns: int,
    wall_duration_ns: int,
    latency_ns: dict,
    rss_guard: dict,
    polls: int,
    teardown_clean: bool,
    executor_thread_joined: bool,
) -> dict:
    total = args.warmup_goals + args.measured_goals
    return _event(
        args,
        "report",
        warmup_goals=args.warmup_goals,
        measured_goals=args.measured_goals,
        goals_sent=counters["goals_sent"],
        goals_accepted=counters["goals_accepted"],
        goals_rejected=counters.get("goals_rejected", 0),
        feedback_received=counters["feedback_received"],
        feedback_dropped=counters.get("feedback_dropped", 0),
        results_received=counters["results_received"],
        terminal_succeeded=counters["terminal_succeeded"],
        sequence_checksum=counters["sequence_checksum"],
        last_sequence=counters["last_sequence"],
        active_goals=counters.get("active_goals", 0),
        pending_operations=counters.get("pending_operations", 0),
        exceptions=counters.get("exceptions", 0),
        python_crossings=expected_crossings(args.variant, total),
        no_python_message_conversion=(
            VARIANTS[args.variant]["no_python_message_conversion"]),
        cpu_time_ns=cpu_time_ns,
        cpu_clock="CLOCK_PROCESS_CPUTIME_ID",
        wall_duration_ns=wall_duration_ns,
        latency_ns=latency_ns,
        rss_guard=rss_guard,
        orchestration_poll_count=polls,
        teardown_clean=teardown_clean,
        executor_thread_joined=executor_thread_joined,
    )


def _spin_until(executor, predicate, timeout: float, label: str, polls: list[int]) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        polls[0] += 1
        if predicate():
            return
        executor.spin_once(timeout_sec=0.01)
    raise RuntimeError("timed out waiting for %s" % label)


def _rclpy_phase(client, executor, phase: str, count: int, measured: bool, state: dict) -> None:
    from action_msgs.msg import GoalStatus
    from tf2_msgs.action import LookupTransform

    for sequence in range(1, count + 1):
        target, source = goal_strings(phase, sequence)
        feedback_times = []
        invalid_feedback = [0]

        def feedback_callback(update):
            feedback_times.append(time.monotonic_ns())
            if state.get("require_cpp"):
                valid = (
                    type(update) is LookupTransform.Impl.FeedbackMessage
                    and type(update.feedback) is LookupTransform.Feedback
                )
            else:
                valid = isinstance(update.feedback, LookupTransform.Feedback)
            if not valid:
                invalid_feedback[0] += 1

        goal = LookupTransform.Goal(target_frame=target, source_frame=source)
        if state.get("require_cpp") and type(goal) is not LookupTransform.Goal:
            raise RuntimeError("direct action goal is not the generated C++ type")
        send_ns = time.monotonic_ns()
        goal_future = client.send_goal_async(goal, feedback_callback=feedback_callback)
        state["goals_sent"] += 1
        _spin_until(
            executor, goal_future.done, 10.0, "Python action goal response", state["polls"])
        accept_ns = time.monotonic_ns()
        handle = goal_future.result()
        if not handle.accepted:
            raise RuntimeError("Python action goal was rejected")
        state["goals_accepted"] += 1
        result_future = handle.get_result_async()
        _spin_until(
            executor,
            lambda: result_future.done() and len(feedback_times) >= FEEDBACK_PER_GOAL,
            10.0,
            "Python action feedback and result",
            state["polls"],
        )
        result_ns = time.monotonic_ns()
        if len(feedback_times) != FEEDBACK_PER_GOAL or invalid_feedback[0]:
            raise RuntimeError("Python action feedback contract failed")
        wrapped = result_future.result()
        if state.get("require_cpp") and (
            type(wrapped) is not LookupTransform.Impl.GetResultService.Response
            or type(wrapped.result) is not LookupTransform.Result
        ):
            raise RuntimeError("direct action result is not a generated C++ envelope")
        error_value = wrapped.result.error.error
        if isinstance(error_value, str):
            error_value = ord(error_value)
        status = wrapped.status
        if isinstance(status, str):
            status = ord(status)
        if (
            status != GoalStatus.STATUS_SUCCEEDED
            or wrapped.result.transform.header.frame_id != phase
            or wrapped.result.transform.child_frame_id != str(sequence)
            or error_value != 0
            or str(wrapped.result.error.error_string)
        ):
            raise RuntimeError("Python action result contract failed")
        state["feedback_received"] += FEEDBACK_PER_GOAL
        state["results_received"] += 1
        state["terminal_succeeded"] += 1
        if measured:
            state["accept"].append(accept_ns - send_ns)
            state["feedback"].append(feedback_times[0] - send_ns)
            state["result"].append(result_ns - send_ns)
            state["sequence_checksum"] += sequence
            state["last_sequence"] = sequence


def _python_lane(args, *, activate: bool) -> int:
    active_product = None
    if activate:
        import rclcppyy as active_product

        active_product.enable_cpp_acceleration(profile="compatible")

    from rclpy.action import ActionClient
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_system_default
    from tf2_msgs.action import LookupTransform

    context = Context()
    context.init(args=[])
    node = Node(args.node_name, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    client = ActionClient(
        node,
        LookupTransform,
        args.action_name,
        feedback_sub_qos_profile=qos_profile_system_default,
    )
    client_implementation = "%s.%s" % (
        type(client).__module__, type(client).__qualname__)
    if client_implementation != VARIANTS[args.variant]["action_implementation"]:
        raise RuntimeError("Python action authority marker changed")
    if not client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError("Python action server discovery timed out")
    state = {
        "goals_sent": 0,
        "goals_accepted": 0,
        "feedback_received": 0,
        "results_received": 0,
        "terminal_succeeded": 0,
        "sequence_checksum": 0,
        "last_sequence": 0,
        "accept": [],
        "feedback": [],
        "result": [],
        "polls": [0],
    }
    _rclpy_phase(client, executor, "warmup", args.warmup_goals, False, state)
    cache = (
        {"kind": "activation-only", "state": "activation-only"}
        if activate else
        {"kind": "stock-rclpy", "state": "not_applicable"}
    )
    _emit(_ready(
        args,
        authority="python",
        goal_representation="python-message",
        executor_implementation="rclpy.executors.SingleThreadedExecutor",
        cache=cache,
        activation=(
            {"profile": "compatible", "action_authority": "python"}
            if activate else None),
    ))
    if sys.stdin.readline().rstrip("\n") != "START":
        raise RuntimeError("action client expected START")
    state["sequence_checksum"] = 0
    state["last_sequence"] = 0
    state["accept"].clear()
    state["feedback"].clear()
    state["result"].clear()
    state["polls"][0] = 0
    rss_baseline = _peak_rss_bytes()
    wall_start = time.monotonic_ns()
    cpu_start = time.process_time_ns()
    _emit(_armed(args))
    _rclpy_phase(client, executor, "measured", args.measured_goals, True, state)
    cpu_stop = time.process_time_ns()
    wall_stop = time.monotonic_ns()
    rss_final = _peak_rss_bytes()
    counters = dict(state)
    client.destroy()
    executor.remove_node(node)
    executor.shutdown(timeout_sec=2.0)
    node.destroy_node()
    context.shutdown()
    _emit(_report(
        args,
        counters=counters,
        cpu_time_ns=cpu_stop - cpu_start,
        wall_duration_ns=wall_stop - wall_start,
        latency_ns={
            "send_to_accept": latency_summary(state["accept"]),
            "send_to_first_feedback": latency_summary(state["feedback"]),
            "send_to_result": latency_summary(state["result"]),
        },
        rss_guard=_rss_guard(rss_baseline, rss_final),
        polls=state["polls"][0],
        teardown_clean=not context.ok(),
        executor_thread_joined=True,
    ))
    return 0


def _direct_source_compatible_lane(args) -> int:
    import importlib
    import rclcppyy

    rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

    import cppyy
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from tf2_msgs.action import LookupTransform

    def forbidden_boundary(*_args, **_kwargs):
        raise RuntimeError("direct action benchmark entered a conversion boundary")

    bringup_module = importlib.import_module("rclcpp_kit.bringup_rclcpp")
    serialization_module = importlib.import_module("rclcpp_kit.serialization")

    bringup_module.convert_python_msg_to_cpp = forbidden_boundary
    serialization_module.serialize_message = forbidden_boundary
    serialization_module.deserialize_message = forbidden_boundary

    if LookupTransform.Goal is not cppyy.gbl.tf2_msgs.action.LookupTransform.Goal:
        raise RuntimeError("direct action benchmark did not install the C++ Goal alias")

    rclpy.init(args=[])
    node = Node(args.node_name)
    client = ActionClient(node, LookupTransform, args.action_name)

    class DirectExecutor:
        @staticmethod
        def spin_once(timeout_sec):
            rclpy.spin_once(node, timeout_sec=timeout_sec)

    executor = DirectExecutor()
    client_implementation = "%s.%s" % (
        type(client).__module__, type(client).__qualname__)
    if client_implementation != VARIANTS[args.variant]["action_implementation"]:
        raise RuntimeError("direct action authority marker changed")
    if not client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError("direct action server discovery timed out")
    state = {
        "goals_sent": 0,
        "goals_accepted": 0,
        "feedback_received": 0,
        "results_received": 0,
        "terminal_succeeded": 0,
        "sequence_checksum": 0,
        "last_sequence": 0,
        "accept": [],
        "feedback": [],
        "result": [],
        "polls": [0],
        "require_cpp": True,
    }
    _rclpy_phase(client, executor, "warmup", args.warmup_goals, False, state)
    artifact = _artifact(client.compile_result)
    if not artifact["cached"]:
        raise RuntimeError("direct action client helper was not prewarmed")
    _emit(_ready(
        args,
        authority="cpp",
        goal_representation="cpp-message",
        executor_implementation="rclcpp::executors::SingleThreadedExecutor",
        cache={
            "kind": "native-action-client-shared-library",
            "state": "prebuilt",
            "hit": True,
            "path": artifact["path"],
            "sha256": artifact["sha256"],
            "size_bytes": artifact["size_bytes"],
        },
        activation={
            "profile": "direct_cpp",
            "action_authority": "cpp",
            "representations": "actual_cpp",
        },
    ))
    if sys.stdin.readline().rstrip("\n") != "START":
        raise RuntimeError("action client expected START")
    state["sequence_checksum"] = 0
    state["last_sequence"] = 0
    state["accept"].clear()
    state["feedback"].clear()
    state["result"].clear()
    state["polls"][0] = 0
    rss_baseline = _peak_rss_bytes()
    wall_start = time.monotonic_ns()
    cpu_start = time.process_time_ns()
    _emit(_armed(args))
    _rclpy_phase(client, executor, "measured", args.measured_goals, True, state)
    cpu_stop = time.process_time_ns()
    wall_stop = time.monotonic_ns()
    rss_final = _peak_rss_bytes()

    total = args.warmup_goals + args.measured_goals
    stats = client.stats()
    actual_crossings = {
        "goal": stats.python_goal_crossings,
        "feedback": stats.python_feedback_crossings,
        "result": stats.python_result_crossings,
    }
    actual_crossings["total"] = sum(actual_crossings.values())
    if actual_crossings != expected_crossings(args.variant, total):
        raise RuntimeError("direct action crossing counters differ: %s" % actual_crossings)
    exact_cpp_evidence = {
        "goal_submissions": stats.cpp_goal_value_submissions,
        "goal_ids": stats.cpp_goal_id_materializations,
        "goal_responses": stats.cpp_goal_response_materializations,
        "feedback_envelopes": stats.cpp_feedback_message_materializations,
        "result_envelopes": stats.cpp_result_response_materializations,
    }
    expected_cpp_evidence = {
        "goal_submissions": total,
        "goal_ids": total,
        "goal_responses": total,
        "feedback_envelopes": total * FEEDBACK_PER_GOAL,
        "result_envelopes": total,
    }
    if exact_cpp_evidence != expected_cpp_evidence:
        raise RuntimeError(
            "direct action C++ representation counters differ: %s" %
            exact_cpp_evidence)
    if client.python_feedback_callbacks != total * FEEDBACK_PER_GOAL:
        raise RuntimeError("direct action Python feedback callback count differs")
    counters = {
        "goals_sent": stats.goals_sent,
        "goals_accepted": stats.goals_accepted,
        "goals_rejected": stats.goals_rejected,
        "feedback_received": stats.feedback_received,
        "feedback_dropped": stats.feedback_dropped,
        "results_received": stats.results_taken,
        "terminal_succeeded": stats.results_taken,
        "sequence_checksum": state["sequence_checksum"],
        "last_sequence": state["last_sequence"],
        "active_goals": stats.active_goals,
        "pending_operations": stats.active_goals,
        "exceptions": stats.exceptions,
    }
    client.destroy()
    node.destroy_node()
    rclpy.shutdown()
    _emit(_report(
        args,
        counters=counters,
        cpu_time_ns=cpu_stop - cpu_start,
        wall_duration_ns=wall_stop - wall_start,
        latency_ns={
            "send_to_accept": latency_summary(state["accept"]),
            "send_to_first_feedback": latency_summary(state["feedback"]),
            "send_to_result": latency_summary(state["result"]),
        },
        rss_guard=_rss_guard(rss_baseline, rss_final),
        polls=state["polls"][0],
        teardown_clean=not rclpy.ok(),
        executor_thread_joined=True,
    ))
    return 0


def _native_python_phase(client, phase: str, count: int, measured: bool, state: dict) -> None:
    from action_msgs.msg import GoalStatus

    for sequence in range(1, count + 1):
        target, source = goal_strings(phase, sequence)
        goal = client.make_goal()
        goal.target_frame = target
        goal.source_frame = source
        send_ns = time.monotonic_ns()
        token = client.send_goal(goal)
        _wait_until(
            lambda: client.goal_response_ready(token),
            10.0,
            "native action goal response",
            state["polls"],
        )
        accept_ns = time.monotonic_ns()
        if not client.goal_accepted(token):
            raise RuntimeError("native action goal was rejected")
        feedback_times = []
        result_ready = False
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            state["polls"][0] += 1
            while client.feedback_ready(token):
                feedback = client.take_feedback(token)
                feedback_times.append(time.monotonic_ns())
                placeholder = feedback.structure_needs_at_least_one_member
                if isinstance(placeholder, str):
                    placeholder = ord(placeholder)
                if placeholder != 0:
                    raise RuntimeError("native action feedback payload was not empty")
            result_ready = client.result_ready(token)
            if result_ready and len(feedback_times) >= FEEDBACK_PER_GOAL:
                break
            time.sleep(0.0001)
        if not result_ready or len(feedback_times) != FEEDBACK_PER_GOAL:
            raise RuntimeError(
                "native action feedback/result contract timed out: "
                "feedback=%d result_ready=%s stats=%s" % (
                    len(feedback_times), result_ready, client.stats().to_dict()))
        result_ns = time.monotonic_ns()
        wrapped = client.take_result(token)
        error_value = wrapped.result.error.error
        if isinstance(error_value, str):
            error_value = ord(error_value)
        if (
            wrapped.code != GoalStatus.STATUS_SUCCEEDED
            or wrapped.result.transform.header.frame_id != phase
            or wrapped.result.transform.child_frame_id != str(sequence)
            or error_value != 0
            or str(wrapped.result.error.error_string)
        ):
            raise RuntimeError("native action terminal result contract failed")
        if measured:
            state["accept"].append(accept_ns - send_ns)
            state["feedback"].append(feedback_times[0] - send_ns)
            state["result"].append(result_ns - send_ns)
            state["sequence_checksum"] += sequence
            state["last_sequence"] = sequence


def _native_python_lane(args) -> int:
    from rclcpp_kit.native import native
    from tf2_msgs.action import LookupTransform

    session = native(["action-client-native-python"])
    session.open()
    node = session.create_node(args.node_name)
    executor = session.create_executor("single_threaded", threads=1)
    executor.add_node(node)
    executor_thread = session.start_executor(executor)
    client = session.create_native_action_client(
        node, LookupTransform, args.action_name, feedback_capacity=FEEDBACK_PER_GOAL)
    client_implementation = "%s.%s" % (
        type(client).__module__, type(client).__qualname__)
    if client_implementation != VARIANTS[args.variant]["action_implementation"]:
        raise RuntimeError("native action authority marker changed")
    if not client.wait_for_server(10.0):
        raise RuntimeError("native action server discovery timed out")
    state = {
        "sequence_checksum": 0,
        "last_sequence": 0,
        "accept": [],
        "feedback": [],
        "result": [],
        "polls": [0],
    }
    _native_python_phase(client, "warmup", args.warmup_goals, False, state)
    artifact = _artifact(client.compile_result)
    if not artifact["cached"]:
        raise RuntimeError("native action client helper was not prewarmed")
    _emit(_ready(
        args,
        authority="cpp",
        goal_representation="cpp-message",
        executor_implementation=str(getattr(executor, "__cpp_name__", type(executor))),
        cache={
            "kind": "native-action-client-shared-library",
            "state": "prebuilt",
            "hit": True,
            "path": artifact["path"],
            "sha256": artifact["sha256"],
            "size_bytes": artifact["size_bytes"],
        },
    ))
    if sys.stdin.readline().rstrip("\n") != "START":
        raise RuntimeError("action client expected START")
    state["sequence_checksum"] = 0
    state["last_sequence"] = 0
    state["accept"].clear()
    state["feedback"].clear()
    state["result"].clear()
    state["polls"][0] = 0
    rss_baseline = _peak_rss_bytes()
    wall_start = time.monotonic_ns()
    cpu_start = time.process_time_ns()
    _emit(_armed(args))
    _native_python_phase(client, "measured", args.measured_goals, True, state)
    cpu_stop = time.process_time_ns()
    wall_stop = time.monotonic_ns()
    rss_final = _peak_rss_bytes()
    stats = client.stats()
    actual_crossings = {
        "goal": stats.python_goal_crossings,
        "feedback": stats.python_feedback_crossings,
        "result": stats.python_result_crossings,
    }
    actual_crossings["total"] = sum(actual_crossings.values())
    expected = expected_crossings(
        args.variant, args.warmup_goals + args.measured_goals)
    if actual_crossings != expected:
        raise RuntimeError(
            "native action crossing counters differ: %s" % actual_crossings)
    counters = {
        "goals_sent": stats.goals_sent,
        "goals_accepted": stats.goals_accepted,
        "goals_rejected": stats.goals_rejected,
        "feedback_received": stats.feedback_received,
        "feedback_dropped": stats.feedback_dropped,
        "results_received": stats.results_taken,
        "terminal_succeeded": stats.results_taken,
        "sequence_checksum": state["sequence_checksum"],
        "last_sequence": state["last_sequence"],
        "active_goals": stats.active_goals,
        "pending_operations": stats.active_goals,
        "exceptions": stats.exceptions + int(executor_thread.exceptions),
    }
    session.close()
    _emit(_report(
        args,
        counters=counters,
        cpu_time_ns=cpu_stop - cpu_start,
        wall_duration_ns=wall_stop - wall_start,
        latency_ns={
            "send_to_accept": latency_summary(state["accept"]),
            "send_to_first_feedback": latency_summary(state["feedback"]),
            "send_to_result": latency_summary(state["result"]),
        },
        rss_guard=_rss_guard(rss_baseline, rss_final),
        polls=state["polls"][0],
        teardown_clean=session.closed and client.closed,
        executor_thread_joined=executor_thread.closed,
    ))
    return 0


def _cpp_percentiles(probe, prefix: str) -> dict:
    return {
        "p50": int(getattr(probe, prefix + "_p50")()),
        "p95": int(getattr(probe, prefix + "_p95")()),
        "p99": int(getattr(probe, prefix + "_p99")()),
        "max": int(getattr(probe, prefix + "_max")()),
    }


def _native_cpp_lane(args) -> int:
    import cppyy
    from rclcpp_kit.native import native

    session = native(["action-client-native-cpp"])
    session.open()
    result = _compile_state_machine()
    artifact = _artifact(result)
    if not artifact["cached"]:
        raise RuntimeError("native C++ action state machine was not prewarmed")
    node = session.create_node(args.node_name)
    factory = getattr(
        cppyy.gbl.rclcppyy_action_benchmark,
        "make_state_machine_%s" % STATE_MACHINE_SOURCE_ID,
    )
    probe = factory(node, args.action_name)
    executor = session.create_executor("single_threaded", threads=1)
    executor.add_node(node)
    executor_thread = session.start_executor(executor)
    if not probe.wait_for_server(10_000_000_000):
        raise RuntimeError("native C++ action server discovery timed out")
    probe.run_warmup(args.warmup_goals)
    _emit(_ready(
        args,
        authority="cpp",
        goal_representation="cpp-message",
        executor_implementation=str(getattr(executor, "__cpp_name__", type(executor))),
        cache={
            "kind": "action-state-machine-shared-library",
            "state": "prebuilt",
            "hit": True,
            "path": artifact["path"],
            "sha256": artifact["sha256"],
            "size_bytes": artifact["size_bytes"],
        },
    ))
    if sys.stdin.readline().rstrip("\n") != "START":
        raise RuntimeError("action client expected START")
    probe.arm(args.measured_goals)
    _emit(_armed(args))
    probe.run_measured()
    values = {
        "goals_sent": int(probe.goals_sent()),
        "goals_accepted": int(probe.goals_accepted()),
        "feedback_received": int(probe.feedback_received()),
        "results_received": int(probe.results_received()),
        "terminal_succeeded": int(probe.terminal_succeeded()),
        "sequence_checksum": int(probe.measured_checksum()),
        "last_sequence": int(probe.last_sequence()),
        "active_goals": int(probe.active_goals()),
        "pending_operations": int(probe.active_goals()),
        "exceptions": int(probe.exceptions()) + int(executor_thread.exceptions),
    }
    cpu_time = int(probe.cpu_time_ns())
    wall_time = int(probe.wall_duration_ns())
    rss_baseline = int(probe.rss_baseline())
    rss_final = int(probe.rss_final())
    latencies = {
        "send_to_accept": _cpp_percentiles(probe, "accept"),
        "send_to_first_feedback": _cpp_percentiles(probe, "feedback"),
        "send_to_result": _cpp_percentiles(probe, "result"),
    }
    probe.close()
    session.close()
    _emit(_report(
        args,
        counters=values,
        cpu_time_ns=cpu_time,
        wall_duration_ns=wall_time,
        latency_ns=latencies,
        rss_guard=_rss_guard(rss_baseline, rss_final),
        polls=0,
        teardown_clean=session.closed,
        executor_thread_joined=executor_thread.closed,
    ))
    return 0


def _prewarm(args) -> int:
    from rclcpp_kit.native import native
    from tf2_msgs.action import LookupTransform

    session = native(["action-client-prewarm"])
    session.open()
    state_result = _compile_state_machine()
    node = session.create_node("action_client_prewarm_%d" % os.getpid())
    client = session.create_native_action_client(
        node,
        LookupTransform,
        "/rclcppyy/action_benchmark/prewarm",
        feedback_capacity=FEEDBACK_PER_GOAL,
    )
    client_result = dict(client.compile_result)
    loaded = _loaded_rmw()
    client.close()
    session.close()
    _emit({
        "schema": PREWARM_SCHEMA,
        "pid": os.getpid(),
        "loaded_rmw": loaded,
        "state_machine_source_id": STATE_MACHINE_SOURCE_ID,
        "artifacts": {
            "native_action_client": _artifact(client_result),
            "state_machine": _artifact(state_result),
        },
    })
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prewarm", action="store_true")
    parser.add_argument("--variant", choices=tuple(VARIANTS)[:-1])
    parser.add_argument("--node-name")
    parser.add_argument("--action-name")
    parser.add_argument("--run-token")
    parser.add_argument("--warmup-goals", type=int)
    parser.add_argument("--measured-goals", type=int)
    return parser


def main() -> int:
    args = _parser().parse_args()
    if args.prewarm:
        return _prewarm(args)
    required = (
        args.variant, args.node_name, args.action_name, args.run_token,
        args.warmup_goals, args.measured_goals,
    )
    if any(value is None for value in required):
        raise SystemExit("action worker requires all lane arguments")
    if args.warmup_goals <= 0 or args.measured_goals <= 0:
        raise SystemExit("action goal counts must be positive")
    if args.variant == "stock-rclpy":
        return _python_lane(args, activate=False)
    if args.variant == "compatible-rclcppyy":
        return _python_lane(args, activate=True)
    if args.variant == "direct-source-compatible":
        return _direct_source_compatible_lane(args)
    if args.variant == "native-python-orchestrated":
        return _native_python_lane(args)
    if args.variant == "native-cpp-state-machine":
        return _native_cpp_lane(args)
    raise SystemExit("unsupported dynamic action variant")


if __name__ == "__main__":
    raise SystemExit(main())
