#!/usr/bin/env python3
"""Run one dynamic lane of the CPU-first action-server benchmark."""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import os
from pathlib import Path
import resource
import select
import sys
import time

from _action_server_protocol import (
    ACTION_TYPE,
    BOUNDARY_TRIPWIRE_SURFACES,
    FEEDBACK_PER_GOAL,
    PREWARM_SCHEMA,
    QOS,
    RSS_LIMIT_BYTES,
    RMW,
    SERVER_SCHEMA,
    VARIANTS,
    endpoint_names,
    expected_cpp_operations,
    expected_python_crossings,
)


PROTOCOL_PREFIX = "@@RCLCPPYY_ACTION_CLIENT_V1@@"
STATE_MACHINE_SOURCE_ID = hashlib.sha256(
    b"rclcppyy-action-server-state-machine-v1:tf2_msgs/LookupTransform"
).hexdigest()[:16]


def _debug(message: str) -> None:
    if os.environ.get("RCLCPPYY_ACTION_SERVER_DEBUG") == "1":
        print("action-server-debug: " + message, file=sys.stderr, flush=True)


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


def _artifact(result: dict) -> dict:
    path = result.get("so")
    if not path or not Path(path).is_file():
        raise RuntimeError("action-server cache produced no shared library")
    resolved = Path(path).resolve()
    return {
        "cached": bool(result.get("cached")),
        "path": str(resolved),
        "sha256": _sha256(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def _loaded_rmw() -> str:
    from rclpy.utilities import get_rmw_implementation_identifier

    loaded = get_rmw_implementation_identifier()
    if os.environ.get("RMW_IMPLEMENTATION") != RMW or loaded != RMW:
        raise RuntimeError("action-server worker requires explicit %s" % RMW)
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


def _cache_dir() -> str:
    root = os.environ.get("XDG_CACHE_HOME")
    if not root:
        raise RuntimeError("action-server benchmark requires isolated XDG_CACHE_HOME")
    return os.path.join(root, "rclcppyy", "action-server")


def _spin_until_stop(spin_once) -> None:
    while True:
        readable, _, _ = select.select([sys.stdin], [], [], 0)
        if readable:
            if sys.stdin.readline().rstrip("\n") != "STOP":
                raise RuntimeError("action-server worker expected STOP")
            return
        spin_once()


def _forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("action-server data conversion/serialization is forbidden")


def _poison_boundaries() -> dict:
    counters = {
        "python_message_conversions": 0,
        "python_serialization_calls": 0,
        "adapter_cdr_roundtrips": 0,
    }

    def conversion(*_args, **_kwargs):
        counters["python_message_conversions"] += 1
        return _forbidden_boundary()

    def serialization_call(*_args, **_kwargs):
        counters["python_serialization_calls"] += 1
        return _forbidden_boundary()

    def cdr_call(*_args, **_kwargs):
        counters["adapter_cdr_roundtrips"] += 1
        return _forbidden_boundary()

    bindings = (
        ("rclcpp_kit", "convert_python_msg_to_cpp", conversion),
        ("rclcpp_kit.bringup_rclcpp", "convert_python_msg_to_cpp", conversion),
        ("rclcpp_kit.native_action", "convert_python_msg_to_cpp", conversion),
        ("rclcpp_kit.native_action_server", "convert_python_msg_to_cpp", conversion),
        ("rclcppyy.bringup_rclcpp", "convert_python_msg_to_cpp", conversion),
        ("rclcppyy.node", "convert_python_msg_to_cpp", conversion),
        ("rclcpp_kit.serialization", "serialize_message", serialization_call),
        ("rclcpp_kit.serialization", "deserialize_message", serialization_call),
        ("rclcppyy.serialization", "serialize_message", serialization_call),
        ("rclcppyy.serialization", "deserialize_message", serialization_call),
        ("rclpy.serialization", "serialize_message", serialization_call),
        ("rclpy.serialization", "deserialize_message", serialization_call),
        ("rclcpp_kit.serialization", "serialized_message_from_bytes", cdr_call),
        ("rclcpp_kit.serialization", "serialized_message_to_bytes", cdr_call),
        ("rclcppyy.serialization", "serialized_message_from_bytes", cdr_call),
        ("rclcppyy.serialization", "serialized_message_to_bytes", cdr_call),
    )
    surfaces = []
    for module_name, attribute, poison in bindings:
        module = importlib.import_module(module_name)
        setattr(module, attribute, poison)
        surfaces.append(module_name + "." + attribute)
    if tuple(surfaces) != BOUNDARY_TRIPWIRE_SURFACES:
        raise RuntimeError("action-server tripwire surface changed")
    return {"counters": counters, "surfaces": surfaces}


def _boundary_evidence(guard: dict | None, *, exact_cpp: bool) -> dict:
    if guard is None:
        return {
            "proof": "python-message-lane",
            "exact_generated_cpp": exact_cpp,
            "python_message_conversions": None,
            "python_serialization_calls": None,
            "adapter_cdr_roundtrips": None,
            "tripwires_armed": False,
            "tripwire_surfaces": [],
        }
    return {
        "proof": "counter-backed-poison",
        "exact_generated_cpp": exact_cpp,
        **guard["counters"],
        "tripwires_armed": True,
        "tripwire_surfaces": list(guard["surfaces"]),
    }


class State:
    def __init__(self, args, *, require_cpp):
        self.args = args
        self.require_cpp = require_cpp
        self.total = args.warmup_goals + args.measured_goals
        self.goals_received = 0
        self.goals_accepted = 0
        self.feedback_sent = 0
        self.results_sent = 0
        self.terminal_succeeded = 0
        self.warmup_checksum = 0
        self.measured_checksum = 0
        self.active_goals = 0
        self.exceptions = 0
        self.python_crossings = {
            "goal_decision": 0, "accepted_goal": 0, "execute": 0, "total": 0,
        }
        self.cpu_start = 0
        self.cpu_stop = 0
        self.rss_baseline = 0
        self.rss_final = 0
        self.feedback_matched = False

    def inspect_goal(self, goal) -> tuple[str, int]:
        self.goals_received += 1
        warmup = self.goals_received <= self.args.warmup_goals
        sequence = (
            self.goals_received if warmup
            else self.goals_received - self.args.warmup_goals)
        phase = "warmup" if warmup else "measured"
        base = "rclcppyy/action-benchmark/%s/%d" % (phase, sequence)
        exact = (
            str(goal.target_frame) == base + "/target"
            and str(goal.source_frame) == base + "/source"
            and int(goal.source_time.sec) == 0
            and int(goal.source_time.nanosec) == 0
            and int(goal.timeout.sec) == 0
            and int(goal.timeout.nanosec) == 0
            and int(goal.target_time.sec) == 0
            and int(goal.target_time.nanosec) == 0
            and not str(goal.fixed_frame)
            and not bool(goal.advanced)
            and self.goals_received <= self.total
        )
        if not exact:
            raise RuntimeError("action-server goal value contract failed")
        if phase == "measured" and sequence == 1:
            self.cpu_start = time.process_time_ns()
        return phase, sequence

    def finish_goal(self, phase: str, sequence: int) -> None:
        if phase == "warmup":
            self.warmup_checksum += sequence
        else:
            self.measured_checksum += sequence
        self.results_sent += 1
        self.terminal_succeeded += 1
        self.active_goals -= 1
        if self.results_sent == self.args.warmup_goals:
            self.rss_baseline = _peak_rss_bytes()
        if self.results_sent == self.total:
            self.cpu_stop = time.process_time_ns()
            self.rss_final = _peak_rss_bytes()


def _ready(args, *, artifact: dict | None = None) -> dict:
    spec = VARIANTS[args.variant]
    if args.variant == "stock-rclpy":
        cache = {"kind": "stock-rclpy", "state": "not_applicable"}
    else:
        if artifact is None or not artifact["cached"]:
            raise RuntimeError("native action-server lane was not prewarmed")
        cache = {
            "kind": spec["cache_kind"],
            "state": "prebuilt",
            "hit": True,
            "path": artifact["path"],
            "sha256": artifact["sha256"],
            "size_bytes": artifact["size_bytes"],
        }
    return {
        "schema": SERVER_SCHEMA,
        "event": "ready",
        "variant": args.variant,
        "run_token": args.run_token,
        "pid": os.getpid(),
        "process_group_id": os.getpgrp(),
        "node_name": args.node_name,
        "action_name": args.action_name,
        "loaded_rmw": _loaded_rmw(),
        "action_type": ACTION_TYPE,
        "execution_model": spec["execution_model"],
        "action_authority": spec["authority"],
        "action_implementation": spec["implementation"],
        "goal_representation": spec["representation"],
        "feedback_representation": spec["representation"],
        "result_representation": spec["representation"],
        "goal_id_representation": spec["representation"],
        "envelope_representation": spec["representation"],
        "qos": QOS,
        "endpoints": endpoint_names(args.action_name),
        "executor": {
            "authority": spec["authority"], "kind": "single_threaded", "threads": 1,
        },
        "cache": cache,
    }


def _report(
    args,
    state: State,
    *,
    cpp_operations: dict,
    teardown_clean: bool,
    boundary_guard: dict | None = None,
) -> dict:
    expected = expected_python_crossings(args.variant, state.total)
    if state.python_crossings != expected:
        raise RuntimeError(
            "action-server Python crossings differ: %s != %s" %
            (state.python_crossings, expected))
    expected_operations = expected_cpp_operations(args.variant, state.total)
    if args.shared_values:
        expected_operations = dict(expected_operations)
        expected_operations.update({
            "adapter_message_deep_copies": 0,
            "feedback_shared_handoffs": state.total * FEEDBACK_PER_GOAL,
            "result_shared_handoffs": state.total,
        })
    if cpp_operations != expected_operations:
        raise RuntimeError(
            "action-server C++ operations differ: %s != %s" %
            (cpp_operations, expected_operations))
    spec = VARIANTS[args.variant]
    return {
        "schema": SERVER_SCHEMA,
        "event": "report",
        "variant": args.variant,
        "run_token": args.run_token,
        "pid": os.getpid(),
        "process_group_id": os.getpgrp(),
        "warmup_goals": args.warmup_goals,
        "measured_goals": args.measured_goals,
        "goals_received": state.goals_received,
        "goals_accepted": state.goals_accepted,
        "goals_rejected": 0,
        "feedback_sent": state.feedback_sent,
        "results_sent": state.results_sent,
        "terminal_succeeded": state.terminal_succeeded,
        "warmup_checksum": state.warmup_checksum,
        "measured_checksum": state.measured_checksum,
        "active_goals": state.active_goals,
        "pending_operations": state.active_goals,
        "exceptions": state.exceptions,
        "cpu_time_ns": state.cpu_stop - state.cpu_start,
        "cpu_clock": "CLOCK_PROCESS_CPUTIME_ID",
        "cpu_role": "server_under_test",
        "rss_guard": _rss_guard(state.rss_baseline, state.rss_final),
        "python_crossings": state.python_crossings,
        "python_crossing_semantics": "callback_entries_only",
        "cpp_value_operations": cpp_operations,
        "boundary_evidence": _boundary_evidence(
            boundary_guard, exact_cpp=spec["exact_cpp"]),
        "teardown_clean": teardown_clean,
    }


def _wait_for_feedback_match(node, action_name: str, state: State) -> None:
    if state.feedback_matched:
        return
    topic = action_name + "/_action/feedback"
    deadline = time.monotonic() + 5.0
    while node.count_subscribers(topic) == 0 and time.monotonic() < deadline:
        time.sleep(0.001)
    if node.count_subscribers(topic) == 0:
        raise RuntimeError("action-server feedback subscriber discovery timed out")
    time.sleep(0.02)
    state.feedback_matched = True


def _source_compatible_lane(args, *, profile: str | None) -> int:
    if profile is not None:
        import rclcppyy

        rclcppyy.enable_cpp_acceleration(profile=profile)

    require_cpp = profile == "direct_cpp"
    boundary_guard = _poison_boundaries() if require_cpp else None

    import rclpy
    from rclpy.action import ActionServer, GoalResponse
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from tf2_msgs.action import LookupTransform
    from tf2_msgs.msg import TF2Error

    if require_cpp:
        import cppyy

        from unique_identifier_msgs.msg import UUID

        if LookupTransform.Goal is not cppyy.gbl.tf2_msgs.action.LookupTransform.Goal:
            raise RuntimeError("direct action-server Goal alias is not generated C++")
    state = State(args, require_cpp=require_cpp)

    rclpy.init(args=[])
    node = Node(args.node_name)
    context = node.context
    executor = SingleThreadedExecutor(context=context)
    if executor.context is not context:
        raise RuntimeError("source-compatible action-server executor changed context")
    if not executor.add_node(node):
        raise RuntimeError("source-compatible action-server executor did not add its node")

    def goal_callback(goal):
        _debug("source goal callback start type=%r" % type(goal))
        if require_cpp and state.goals_received == 0 and type(goal) is not LookupTransform.Goal:
            raise RuntimeError("direct action-server goal is not exact C++")
        state.python_crossings["goal_decision"] += 1
        state.python_crossings["total"] += 1
        try:
            state.inspect_goal(goal)
        except BaseException as error:
            _debug("direct goal callback error=%r" % error)
            raise
        _debug("source goal callback accept")
        return GoalResponse.ACCEPT

    def accepted_callback(handle):
        _debug("source accepted callback")
        if require_cpp and state.goals_accepted == 0 and (
                type(handle.request) is not LookupTransform.Goal or
                type(handle.goal_id) is not UUID):
            raise RuntimeError("direct action-server accepted values are not exact C++")
        state.python_crossings["accepted_goal"] += 1
        state.python_crossings["total"] += 1
        handle.execute()

    def execute_callback(handle):
        _debug("source execute callback start")
        state.python_crossings["execute"] += 1
        state.python_crossings["total"] += 1
        state.goals_accepted += 1
        state.active_goals += 1
        phase = "warmup" if state.results_sent < args.warmup_goals else "measured"
        sequence = (
            state.results_sent + 1 if phase == "warmup"
            else state.results_sent - args.warmup_goals + 1)
        graph_node = node._require_node() if require_cpp else node
        _wait_for_feedback_match(graph_node, args.action_name, state)
        time.sleep(0.02)
        prove_feedback_type = require_cpp and state.results_sent == 0
        for index in range(FEEDBACK_PER_GOAL):
            feedback = (
                handle.create_feedback_shared()
                if args.shared_values else LookupTransform.Feedback()
            )
            if prove_feedback_type and index == 0 and type(
                    feedback) is not LookupTransform.Feedback:
                raise RuntimeError("direct action-server feedback is not exact C++")
            if args.shared_values:
                handle.publish_feedback_shared(feedback)
            else:
                handle.publish_feedback(feedback)
            state.feedback_sent += 1
            time.sleep(0.01)
        time.sleep(0.02)
        result = (
            handle.create_result_shared()
            if args.shared_values else LookupTransform.Result()
        )
        result.transform.header.frame_id = phase
        result.transform.child_frame_id = str(sequence)
        result.error.error = TF2Error.NO_ERROR
        if args.shared_values:
            handle.succeed_shared(result)
        else:
            handle.succeed()
        state.finish_goal(phase, sequence)
        _debug("source execute callback complete %s %d" % (phase, sequence))
        return result

    server = ActionServer(
        node,
        LookupTransform,
        args.action_name,
        execute_callback,
        goal_callback=goal_callback,
        handle_accepted_callback=accepted_callback,
    )
    artifact = _artifact(server.compile_result) if require_cpp else None
    _emit(_ready(args, artifact=artifact))
    while state.results_sent < state.total:
        executor.spin_once(timeout_sec=0.002)
    _spin_until_stop(lambda: executor.spin_once(timeout_sec=0.002))
    if require_cpp:
        stats = server.stats()
        cpp_operations = {
            "known": True,
            "goal_shared_handoffs": int(stats.cpp_goal_shared_handoffs),
            "goal_id_materializations": int(stats.cpp_goal_id_materializations),
            "feedback_value_submissions": int(stats.cpp_feedback_value_submissions),
            "result_value_submissions": int(stats.cpp_result_value_submissions),
            "adapter_message_deep_copies": int(
                stats.cpp_feedback_adapter_copies +
                stats.cpp_result_adapter_copies),
        }
        if args.shared_values:
            cpp_operations.update({
                "feedback_shared_handoffs": int(
                    stats.cpp_feedback_shared_handoffs),
                "result_shared_handoffs": int(
                    stats.cpp_result_shared_handoffs),
            })
    else:
        cpp_operations = expected_cpp_operations(args.variant, state.total)
    server.destroy()
    executor.remove_node(node)
    executor.shutdown(timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()
    _emit(_report(
        args,
        state,
        cpp_operations=cpp_operations,
        teardown_clean=not context.ok(),
        boundary_guard=boundary_guard,
    ))
    return 0


def _raw_lane(args) -> int:
    boundary_guard = _poison_boundaries()
    import cppyy
    from rclcpp_kit.native import native
    from tf2_msgs.action import LookupTransform as LookupTransformDescriptor
    from tf2_msgs.msg import TF2Error

    state = State(args, require_cpp=True)
    cpp_goal = None
    cpp_feedback = None
    cpp_result = None
    cpp_uuid = None

    def goal_callback(goal):
        _debug("raw goal callback start type=%r expected=%r" % (
            type(goal), cpp_goal))
        if state.goals_received == 0 and type(goal) is not cpp_goal:
            raise RuntimeError("raw action-server goal is not exact C++")
        state.python_crossings["goal_decision"] += 1
        state.python_crossings["total"] += 1
        state.inspect_goal(goal)
        _debug("raw goal callback accept")
        return True

    session = native(["action-server-raw"])
    session.open()
    node = session.create_node(args.node_name)
    executor = session.create_executor("single_threaded", threads=1)
    executor.add_node(node)
    server = session.create_native_action_server(
        node,
        LookupTransformDescriptor,
        args.action_name,
        goal_callback=goal_callback,
    )
    cpp_action = cppyy.gbl.tf2_msgs.action.LookupTransform
    cpp_goal = cpp_action.Goal
    cpp_feedback = cpp_action.Feedback
    cpp_result = cpp_action.Result
    cpp_uuid = cppyy.gbl.unique_identifier_msgs.msg.UUID
    artifact = _artifact(server.compile_result)
    _emit(_ready(args, artifact=artifact))
    duration = cppyy.gbl.std.chrono.nanoseconds(2_000_000)
    while state.results_sent < state.total:
        executor.spin_once(duration)
        while server.accepted_ready_count():
            accepted = server.take_accepted()
            if state.goals_accepted == 0 and (
                    type(accepted.goal) is not cpp_goal or
                    type(accepted.goal_id) is not cpp_uuid):
                raise RuntimeError("raw action-server accepted values are not exact C++")
            state.python_crossings["accepted_goal"] += 1
            state.python_crossings["total"] += 1
            state.goals_accepted += 1
            state.active_goals += 1
            phase = "warmup" if state.results_sent < args.warmup_goals else "measured"
            sequence = (
                state.results_sent + 1 if phase == "warmup"
                else state.results_sent - args.warmup_goals + 1)
            server.execute(accepted.token)
            _wait_for_feedback_match(node, args.action_name, state)
            time.sleep(0.02)
            for _ in range(FEEDBACK_PER_GOAL):
                feedback = cpp_feedback()
                server.publish_feedback(accepted.token, feedback)
                state.feedback_sent += 1
                time.sleep(0.01)
            time.sleep(0.02)
            result = cpp_result()
            result.transform.header.frame_id = phase
            result.transform.child_frame_id = str(sequence)
            result.error.error = TF2Error.NO_ERROR
            server.succeed(accepted.token, result)
            state.finish_goal(phase, sequence)
    _spin_until_stop(lambda: executor.spin_once(duration))
    stats = server.stats()
    cpp_operations = {
        "known": True,
        "goal_shared_handoffs": int(stats.cpp_goal_shared_handoffs),
        "goal_id_materializations": int(stats.cpp_goal_id_materializations),
        "feedback_value_submissions": int(stats.cpp_feedback_value_submissions),
        "result_value_submissions": int(stats.cpp_result_value_submissions),
        "adapter_message_deep_copies": int(
            stats.cpp_feedback_adapter_copies + stats.cpp_result_adapter_copies),
    }
    server.close()
    session.close()
    _emit(_report(
        args,
        state,
        cpp_operations=cpp_operations,
        teardown_clean=session.closed,
        boundary_guard=boundary_guard,
    ))
    return 0


def _state_machine_sources() -> tuple[str, str]:
    interface = "StateMachine_%s" % STATE_MACHINE_SOURCE_ID
    factory = "make_state_machine_%s" % STATE_MACHINE_SOURCE_ID
    declarations = r"""
#include <cstdint>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
namespace rclcppyy_action_server_benchmark {
class %(interface)s {
public:
  virtual ~%(interface)s() = default;
  virtual bool complete() const = 0;
  virtual uint64_t goals_received() const = 0;
  virtual uint64_t goals_accepted() const = 0;
  virtual uint64_t feedback_sent() const = 0;
  virtual uint64_t results_sent() const = 0;
  virtual uint64_t warmup_checksum() const = 0;
  virtual uint64_t measured_checksum() const = 0;
  virtual uint64_t exceptions() const = 0;
  virtual uint64_t cpu_time_ns() const = 0;
  virtual uint64_t rss_baseline() const = 0;
  virtual uint64_t rss_final() const = 0;
  virtual void close() = 0;
};
std::shared_ptr<%(interface)s> %(factory)s(
  std::shared_ptr<rclcpp::Node> node, const std::string& action_name,
  uint64_t warmup_goals, uint64_t measured_goals);
}
""" % {"interface": interface, "factory": factory}
    code = declarations + r"""
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_msgs/action/lookup_transform.hpp>
#include <tf2_msgs/msg/tf2_error.hpp>
#include <atomic>
#include <chrono>
#include <ctime>
#include <stdexcept>
#include <thread>
#include <sys/resource.h>
namespace rclcppyy_action_server_benchmark {
namespace {
using namespace std::chrono_literals;
using Action = tf2_msgs::action::LookupTransform;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
uint64_t cpu_ns() {
  timespec value{};
  if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &value) != 0) throw std::runtime_error("cpu clock");
  return static_cast<uint64_t>(value.tv_sec) * 1000000000ULL + value.tv_nsec;
}
uint64_t rss_bytes() {
  rusage value{};
  if (getrusage(RUSAGE_SELF, &value) != 0) throw std::runtime_error("rss");
  return static_cast<uint64_t>(value.ru_maxrss) * 1024ULL;
}
std::string base(const std::string& phase, uint64_t sequence) {
  return "rclcppyy/action-benchmark/" + phase + "/" + std::to_string(sequence);
}
}
class StateMachineImpl_%(source_id)s final : public %(interface)s {
public:
  StateMachineImpl_%(source_id)s(
      std::shared_ptr<rclcpp::Node> node, const std::string& name,
      uint64_t warmup, uint64_t measured)
  : node_(std::move(node)), warmup_(warmup), measured_(measured),
    feedback_topic_(name + "/_action/feedback") {
    server_ = rclcpp_action::create_server<Action>(
      node_, name,
      [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal> goal) {
        return goal_callback(goal);
      },
      [](std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::REJECT; },
      [this](std::shared_ptr<GoalHandle> handle) { execute(handle); });
  }
  ~StateMachineImpl_%(source_id)s() override { close(); }
  bool complete() const override { return results_ == warmup_ + measured_; }
  uint64_t goals_received() const override { return goals_; }
  uint64_t goals_accepted() const override { return accepted_; }
  uint64_t feedback_sent() const override { return feedback_; }
  uint64_t results_sent() const override { return results_; }
  uint64_t warmup_checksum() const override { return warmup_checksum_; }
  uint64_t measured_checksum() const override { return measured_checksum_; }
  uint64_t exceptions() const override { return exceptions_; }
  uint64_t cpu_time_ns() const override { return cpu_stop_ - cpu_start_; }
  uint64_t rss_baseline() const override { return rss_baseline_; }
  uint64_t rss_final() const override { return rss_final_; }
  void close() override { server_.reset(); }
private:
  rclcpp_action::GoalResponse goal_callback(const std::shared_ptr<const Action::Goal>& goal) {
    ++goals_;
    const bool warmup = goals_ <= warmup_;
    const uint64_t sequence = warmup ? goals_ : goals_ - warmup_;
    const std::string phase = warmup ? "warmup" : "measured";
    const auto expected = base(phase, sequence);
    if (!goal || goal->target_frame != expected + "/target" ||
        goal->source_frame != expected + "/source" || goals_ > warmup_ + measured_) {
      ++exceptions_;
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!warmup && sequence == 1) cpu_start_ = cpu_ns();
    ++accepted_;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  void execute(const std::shared_ptr<GoalHandle>& handle) {
    const bool warmup = results_ < warmup_;
    const uint64_t sequence = warmup ? results_ + 1 : results_ - warmup_ + 1;
    const std::string phase = warmup ? "warmup" : "measured";
    if (!feedback_matched_) {
      const auto deadline = std::chrono::steady_clock::now() + 5s;
      while (node_->count_subscribers(feedback_topic_) == 0 &&
             std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(1ms);
      if (node_->count_subscribers(feedback_topic_) == 0) { ++exceptions_; return; }
      std::this_thread::sleep_for(20ms);
      feedback_matched_ = true;
    }
    std::this_thread::sleep_for(20ms);
    for (uint64_t index = 0; index < %(feedback_count)d; ++index) {
      handle->publish_feedback(std::make_shared<Action::Feedback>());
      ++feedback_;
      std::this_thread::sleep_for(10ms);
    }
    std::this_thread::sleep_for(20ms);
    auto result = std::make_shared<Action::Result>();
    result->transform.header.frame_id = phase;
    result->transform.child_frame_id = std::to_string(sequence);
    result->error.error = tf2_msgs::msg::TF2Error::NO_ERROR;
    handle->succeed(result);
    if (warmup) warmup_checksum_ += sequence; else measured_checksum_ += sequence;
    ++results_;
    if (results_ == warmup_) rss_baseline_ = rss_bytes();
    if (results_ == warmup_ + measured_) { cpu_stop_ = cpu_ns(); rss_final_ = rss_bytes(); }
  }
  std::shared_ptr<rclcpp::Node> node_;
  uint64_t warmup_, measured_;
  std::string feedback_topic_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  bool feedback_matched_{false};
  uint64_t goals_{0}, accepted_{0}, feedback_{0}, results_{0};
  uint64_t warmup_checksum_{0}, measured_checksum_{0}, exceptions_{0};
  uint64_t cpu_start_{0}, cpu_stop_{0}, rss_baseline_{0}, rss_final_{0};
};
std::shared_ptr<%(interface)s> %(factory)s(
    std::shared_ptr<rclcpp::Node> node, const std::string& action_name,
    uint64_t warmup, uint64_t measured) {
  return std::make_shared<StateMachineImpl_%(source_id)s>(
    std::move(node), action_name, warmup, measured);
}
}
""" % {
        "source_id": STATE_MACHINE_SOURCE_ID,
        "interface": interface,
        "factory": factory,
        "feedback_count": FEEDBACK_PER_GOAL,
    }
    return code, declarations


def _compile_state_machine() -> dict:
    import cppyy_kit
    from cppyy_kit.cache import artifact_paths
    from rclcpp_kit.bringup_rclcpp import get_ros2_lib_path, ros2_include_paths

    code, declarations = _state_machine_sources()
    options = {
        "decls": declarations,
        "name": "rclcppyy_action_server_state_machine_%s" % STATE_MACHINE_SOURCE_ID,
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
    result = dict(cppyy_kit.cppdef_cached(code, **options))
    if not was_cached:
        result.update(cached=False, reason="prebuilt-miss")
    return result


def _state_machine_lane(args) -> int:
    boundary_guard = _poison_boundaries()
    import cppyy
    from rclcpp_kit.native import native

    session = native(["action-server-cpp-state-machine"])
    session.open()
    artifact = _artifact(_compile_state_machine())
    node = session.create_node(args.node_name)
    factory = getattr(
        cppyy.gbl.rclcppyy_action_server_benchmark,
        "make_state_machine_%s" % STATE_MACHINE_SOURCE_ID,
    )
    machine = factory(
        node, args.action_name, args.warmup_goals, args.measured_goals)
    executor = session.create_executor("single_threaded", threads=1)
    executor.add_node(node)
    _emit(_ready(args, artifact=artifact))
    duration = cppyy.gbl.std.chrono.nanoseconds(2_000_000)
    while not machine.complete():
        executor.spin_once(duration)
    _spin_until_stop(lambda: executor.spin_once(duration))
    state = State(args, require_cpp=True)
    state.goals_received = int(machine.goals_received())
    state.goals_accepted = int(machine.goals_accepted())
    state.feedback_sent = int(machine.feedback_sent())
    state.results_sent = int(machine.results_sent())
    state.terminal_succeeded = state.results_sent
    state.warmup_checksum = int(machine.warmup_checksum())
    state.measured_checksum = int(machine.measured_checksum())
    state.exceptions = int(machine.exceptions())
    state.cpu_start = 0
    state.cpu_stop = int(machine.cpu_time_ns())
    state.rss_baseline = int(machine.rss_baseline())
    state.rss_final = int(machine.rss_final())
    machine.close()
    session.close()
    _emit(_report(
        args,
        state,
        cpp_operations=expected_cpp_operations(args.variant, state.total),
        teardown_clean=session.closed,
        boundary_guard=boundary_guard,
    ))
    return 0


def _prewarm() -> int:
    from rclcpp_kit.native import native
    from tf2_msgs.action import LookupTransform

    session = native(["action-server-prewarm"])
    session.open()
    state_machine = _compile_state_machine()
    node = session.create_node("action_server_prewarm_%d" % os.getpid())
    server = session.create_native_action_server(
        node,
        LookupTransform,
        "/rclcppyy/action_server_benchmark/prewarm",
        goal_callback=lambda _goal: True,
    )
    native_result = dict(server.compile_result)
    loaded = _loaded_rmw()
    server.close()
    session.close()
    _emit({
        "schema": PREWARM_SCHEMA,
        "pid": os.getpid(),
        "loaded_rmw": loaded,
        "state_machine_source_id": STATE_MACHINE_SOURCE_ID,
        "artifacts": {
            "native_action_server": _artifact(native_result),
            "state_machine": _artifact(state_machine),
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
    parser.add_argument("--shared-values", action="store_true")
    return parser


def main() -> int:
    args = _parser().parse_args()
    if args.prewarm:
        return _prewarm()
    required = (
        args.variant, args.node_name, args.action_name, args.run_token,
        args.warmup_goals, args.measured_goals,
    )
    if any(value is None for value in required):
        raise SystemExit("action-server worker requires all lane arguments")
    if args.warmup_goals <= 0 or args.measured_goals <= 0:
        raise SystemExit("action-server goal counts must be positive")
    if args.shared_values and args.variant != "direct-source-compatible":
        raise SystemExit(
            "--shared-values requires the direct-source-compatible variant")
    if args.variant == "stock-rclpy":
        return _source_compatible_lane(args, profile=None)
    if args.variant == "direct-source-compatible":
        return _source_compatible_lane(args, profile="direct_cpp")
    if args.variant == "native-python-orchestrated":
        return _raw_lane(args)
    if args.variant == "native-cpp-state-machine":
        return _state_machine_lane(args)
    raise SystemExit("unsupported dynamic action-server variant")


if __name__ == "__main__":
    raise SystemExit(main())
