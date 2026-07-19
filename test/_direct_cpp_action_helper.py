#!/usr/bin/env python3
"""Live source-shape proof for a direct C++ action client."""

import gc
import importlib
import inspect
import os
from pathlib import Path
import subprocess
import sys
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from action_msgs.srv import CancelGoal  # noqa: E402
from rclcppyy.policy import BackendUnavailableError  # noqa: E402
from rclpy.action import ActionClient, ActionServer  # noqa: E402
from rclpy.action.client import ClientGoalHandle  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile  # noqa: E402
from rclpy.task import Future  # noqa: E402
from tf2_msgs.action import (  # noqa: E402
    LookupTransform,
    LookupTransform_GetResult_Request,
    LookupTransform_GetResult_Response,
    LookupTransform_SendGoal_Request,
    LookupTransform_SendGoal_Response,
)
from unique_identifier_msgs.msg import UUID  # noqa: E402


def as_int8(value):
    return ord(value) if isinstance(value, str) else int(value)


def spin_until(node, predicate, timeout=15.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
    assert predicate()


def expect_failure(operation, exception_types=(BackendUnavailableError, TypeError, ValueError)):
    try:
        operation()
    except exception_types:
        return
    raise AssertionError("unsupported direct action operation succeeded")


cpp_action = cppyy.gbl.tf2_msgs.action.LookupTransform
assert LookupTransform.Goal is cpp_action.Goal
assert LookupTransform.Feedback is cpp_action.Feedback
assert LookupTransform.Result is cpp_action.Result
assert LookupTransform.Impl.FeedbackMessage is cpp_action.Impl.FeedbackMessage
assert LookupTransform.Impl.SendGoalService.Response is \
    cpp_action.Impl.SendGoalService.Response
assert LookupTransform.Impl.SendGoalService.Request is \
    cpp_action.Impl.SendGoalService.Request
assert LookupTransform.Impl.GetResultService.Response is \
    cpp_action.Impl.GetResultService.Response
assert LookupTransform.Impl.GetResultService.Request is \
    cpp_action.Impl.GetResultService.Request
assert LookupTransform_SendGoal_Request is \
    cpp_action.Impl.SendGoalService.Request
assert LookupTransform_SendGoal_Response is \
    cpp_action.Impl.SendGoalService.Response
assert LookupTransform_GetResult_Request is \
    cpp_action.Impl.GetResultService.Request
assert LookupTransform_GetResult_Response is \
    cpp_action.Impl.GetResultService.Response
assert UUID is cppyy.gbl.unique_identifier_msgs.msg.UUID
assert CancelGoal.Request is cppyy.gbl.action_msgs.srv.CancelGoal.Request
assert CancelGoal.Response is cppyy.gbl.action_msgs.srv.CancelGoal.Response
constructed = LookupTransform.Goal(target_frame="map", source_frame="base")
assert str(constructed.target_frame) == "map"
assert str(constructed.source_frame) == "base"
assert type(constructed.source_time) is cppyy.gbl.builtin_interfaces.msg.Time
try:
    LookupTransform.Goal(unknown=True)
except TypeError:
    pass
else:
    raise AssertionError("direct action Goal accepted an unknown field")
print("DIRECT_CPP_ACTION_CONSTRUCTORS_OK")


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("a Python-message conversion or serialization path ran")


bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
serialization = importlib.import_module("rclcpp_kit.serialization")
bringup.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary

action_name = "/direct_cpp/action/p%d" % os.getpid()
server_path = Path(__file__).with_name("_stock_action_server_helper.py")
server = subprocess.Popen(
    [sys.executable, str(server_path), action_name],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    text=True,
)

retained_feedback = []
retained_values = []
try:
    rclpy.init(args=[])
    node = Node("direct_action_%d" % os.getpid())

    before = len(node.action_clients)
    expect_failure(
        lambda: ActionServer(node, LookupTransform, action_name, lambda goal: None))
    assert len(node.action_clients) == before
    expect_failure(
        lambda: ActionClient(
            node, LookupTransform, action_name, callback_group=object()))
    expect_failure(
        lambda: ActionClient(
            node,
            LookupTransform,
            action_name,
            feedback_sub_qos_profile=QoSProfile(depth=1),
        ))
    assert len(node.action_clients) == before
    print("DIRECT_CPP_ACTION_FAIL_CLOSED_OK")

    client = ActionClient(node, LookupTransform, action_name)
    assert len(node.action_clients) == 1
    assert client.action_name == action_name
    assert client.wait_for_server(timeout_sec=15.0)
    assert server.poll() is None

    native_stats = client._native.stats

    def forbidden_hot_path_stats():
        raise AssertionError("full action stats materialized on the executor hot path")

    client._native.stats = forbidden_hot_path_stats

    def feedback_callback(message):
        assert type(message) is LookupTransform.Impl.FeedbackMessage
        assert type(message.feedback) is LookupTransform.Feedback
        retained_feedback.append(message)

    goal = LookupTransform.Goal(target_frame="map", source_frame="base")
    goal_future = client.send_goal_async(goal, feedback_callback=feedback_callback)
    assert type(goal_future) is Future
    goal_done = []
    goal_future.add_done_callback(goal_done.append)
    rclpy.spin_until_future_complete(node, goal_future, timeout_sec=15.0)
    assert goal_future.done() and not goal_future.cancelled()
    handle = goal_future.result()
    assert type(handle) is ClientGoalHandle
    assert handle.accepted
    assert type(handle.goal_id) is cppyy.gbl.unique_identifier_msgs.msg.UUID
    assert type(handle.stamp) is cppyy.gbl.builtin_interfaces.msg.Time
    assert any(int(value) for value in handle.goal_id.uuid)
    assert goal_done == [goal_future]

    spin_until(node, lambda: len(retained_feedback) == 3)
    result_future = handle.get_result_async()
    assert type(result_future) is Future
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=15.0)
    assert result_future.done() and result_future.exception() is None
    result_response = result_future.result()
    assert type(result_response) is LookupTransform.Impl.GetResultService.Response
    assert type(result_response.result) is LookupTransform.Result
    assert as_int8(result_response.status) == GoalStatus.STATUS_SUCCEEDED
    assert handle.status == GoalStatus.STATUS_SUCCEEDED
    assert str(result_response.result.transform.child_frame_id) == \
        "successful-stock-result"
    assert len(retained_feedback) == 3
    assert all(
        list(message.goal_id.uuid) == list(handle.goal_id.uuid)
        for message in retained_feedback
    )
    retained_values.extend((handle.goal_id, handle.stamp, result_response))
    client._native.stats = native_stats
    print("DIRECT_CPP_ACTION_SUCCESS_OK")

    rejected_future = client.send_goal_async(LookupTransform.Goal(
        target_frame="reject", source_frame="base"))
    rclpy.spin_until_future_complete(node, rejected_future, timeout_sec=15.0)
    rejected = rejected_future.result()
    assert not rejected.accepted
    assert type(rejected.goal_id) is cppyy.gbl.unique_identifier_msgs.msg.UUID
    assert not any(int(value) for value in rejected.goal_id.uuid)
    assert rejected.stamp.sec == 0 and rejected.stamp.nanosec == 0
    expect_failure(rejected.get_result_async)
    expect_failure(rejected.cancel_goal_async)
    print("DIRECT_CPP_ACTION_REJECTION_OK")

    cancel_feedback = []
    cancel_goal_future = client.send_goal_async(
        LookupTransform.Goal(target_frame="cancel", source_frame="base"),
        feedback_callback=cancel_feedback.append,
    )
    rclpy.spin_until_future_complete(node, cancel_goal_future, timeout_sec=15.0)
    cancel_handle = cancel_goal_future.result()
    assert cancel_handle.accepted
    spin_until(node, lambda: len(cancel_feedback) == 3)
    cancel_result_future = cancel_handle.get_result_async()
    cancel_future = cancel_handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=15.0)
    cancel_response = cancel_future.result()
    assert type(cancel_response) is cppyy.gbl.action_msgs.srv.CancelGoal.Response
    assert as_int8(cancel_response.return_code) == 0
    assert len(cancel_response.goals_canceling) == 1
    rclpy.spin_until_future_complete(node, cancel_result_future, timeout_sec=15.0)
    canceled_result = cancel_result_future.result()
    assert as_int8(canceled_result.status) == GoalStatus.STATUS_CANCELED
    assert cancel_handle.status == GoalStatus.STATUS_CANCELED
    assert str(canceled_result.result.transform.child_frame_id) == \
        "canceled-stock-result"
    assert len(cancel_feedback) == 3
    retained_values.extend((cancel_response, canceled_result))
    print("DIRECT_CPP_ACTION_CANCEL_OK")

    sent_before = client.stats().goals_sent
    expect_failure(
        lambda: client.send_goal_async(
            LookupTransform.Goal(), goal_uuid=cppyy.gbl.unique_identifier_msgs.msg.UUID()))
    assert client.stats().goals_sent == sent_before

    async def async_feedback(_message):
        return None

    assert inspect.iscoroutinefunction(async_feedback)
    expect_failure(
        lambda: client.send_goal_async(
            LookupTransform.Goal(), feedback_callback=async_feedback))
    assert client.stats().goals_sent == sent_before

    stats = client.stats()
    assert stats.goals_sent == 3
    assert stats.goals_accepted == 2
    assert stats.goals_rejected == 1
    assert stats.results_taken == 2
    assert stats.cancel_requests == 1
    assert stats.cancel_responses_taken == 1
    assert stats.feedback_received == 6
    assert stats.feedback_taken == 6
    assert stats.feedback_dropped == 0
    assert stats.active_goals == 0
    assert stats.python_goal_crossings == 3
    assert stats.python_feedback_crossings == 6
    assert stats.python_result_crossings == 2
    assert stats.cpp_goal_value_submissions == 3
    assert stats.cpp_goal_id_materializations == 3
    assert stats.cpp_goal_response_materializations == 3
    assert stats.cpp_feedback_message_materializations == 6
    assert stats.cpp_result_response_materializations == 2
    assert client.python_feedback_callbacks == 6

    records = [
        item for item in rclcppyy.status()["entities"]
        if "direct_cpp_action" in item["policies"]
    ]
    assert len(records) == 1
    evidence = records[0]["metadata"]
    assert records[0]["backend"] == "cpp"
    assert evidence["goal_representation"] == "actual_cpp"
    assert evidence["feedback_representation"] == "actual_cpp"
    assert evidence["result_representation"] == "actual_cpp"
    assert evidence["cancel_representation"] == "actual_cpp"
    assert evidence["python_message_conversions"] == 0
    assert evidence["python_serialization_calls"] == 0
    assert evidence["action_interface"] == \
        "tf2_msgs/action/LookupTransform"
    print("DIRECT_CPP_ACTION_EVIDENCE_OK")

    assert client.destroy() is None
    assert client.closed
    assert len(node.action_clients) == 0
    node.destroy_node()
    rclpy.shutdown()
    assert not rclpy.ok()

    gc.collect()
    assert any(int(value) for value in retained_values[0].uuid)
    assert str(retained_values[2].result.transform.child_frame_id) == \
        "successful-stock-result"
    assert str(retained_values[-1].result.transform.child_frame_id) == \
        "canceled-stock-result"
    assert all(type(message.feedback) is LookupTransform.Feedback
               for message in retained_feedback)

    rclpy.init(args=[])
    second = Node("direct_action_reinit_%d" % os.getpid())
    second_client = ActionClient(second, LookupTransform, action_name)
    assert second_client.wait_for_server(timeout_sec=15.0)
    second_client.destroy()
    second.destroy_node()
    rclpy.shutdown()
    print("DIRECT_CPP_ACTION_REINIT_TEARDOWN_OK")
finally:
    if rclpy.ok():
        rclpy.try_shutdown()
    server.terminate()
    try:
        server.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        server.kill()
        server.wait(timeout=5.0)
    if server.returncode not in (-15, -2, 0):
        stdout, stderr = server.communicate()
        raise AssertionError(
            "stock action server failed (%d):\n%s\n%s" %
            (server.returncode, stdout, stderr))
