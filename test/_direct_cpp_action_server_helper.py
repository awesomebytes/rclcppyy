#!/usr/bin/env python3
"""Live exact-C++ direct action-server proof on Jazzy with Cyclone DDS."""

import gc
import importlib
import inspect
import os
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from rclcppyy.policy import BackendUnavailableError  # noqa: E402
from rclpy.action import (  # noqa: E402
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.action.server import ServerGoalHandle  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile  # noqa: E402
from tf2_msgs.action import LookupTransform  # noqa: E402
from unique_identifier_msgs.msg import UUID  # noqa: E402


def as_int8(value):
    return ord(value) if isinstance(value, str) else int(value)


def expect_failure(operation, match):
    try:
        operation()
    except (BackendUnavailableError, TypeError, ValueError) as error:
        assert match in str(error), str(error)
        return
    raise AssertionError("unsupported direct action-server operation succeeded")


def main():
    assert os.environ.get("ROS_DISTRO") == "jazzy"
    assert os.environ.get("RMW_IMPLEMENTATION") == "rmw_cyclonedds_cpp"

    def forbidden_boundary(*_args, **_kwargs):
        raise AssertionError("a Python conversion or serialization boundary ran")

    native_action = importlib.import_module("rclcpp_kit.native_action")
    kit = importlib.import_module("rclcpp_kit")
    bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
    product_bringup = importlib.import_module("rclcppyy.bringup_rclcpp")
    product_node = importlib.import_module("rclcppyy.node")
    serialization = importlib.import_module("rclcpp_kit.serialization")
    product_serialization = importlib.import_module("rclcppyy.serialization")
    rclpy_serialization = importlib.import_module("rclpy.serialization")
    kit.convert_python_msg_to_cpp = forbidden_boundary
    native_action.convert_python_msg_to_cpp = forbidden_boundary
    bringup.convert_python_msg_to_cpp = forbidden_boundary
    product_bringup.convert_python_msg_to_cpp = forbidden_boundary
    product_node.convert_python_msg_to_cpp = forbidden_boundary
    for module in (serialization, product_serialization, rclpy_serialization):
        module.serialize_message = forbidden_boundary
        module.deserialize_message = forbidden_boundary
    for module in (serialization, product_serialization):
        module.serialized_message_from_bytes = forbidden_boundary
        module.serialized_message_to_bytes = forbidden_boundary

    retained = []
    feedback_messages = []
    deferred = {}
    decisions = []
    accepted_handles = []
    action_name = "/direct_cpp/action_server/p%d" % os.getpid()

    rclpy.init(args=[])
    node = Node("direct_action_server_%d" % os.getpid())

    async def async_execute(_handle):
        return LookupTransform.Result()

    assert inspect.iscoroutinefunction(async_execute)
    expect_failure(
        lambda: ActionServer(
            node, LookupTransform, action_name + "/async", async_execute),
        "synchronous",
    )
    expect_failure(
        lambda: ActionServer(
            node,
            LookupTransform,
            action_name + "/reentrant",
            lambda handle: LookupTransform.Result(),
            callback_group=ReentrantCallbackGroup(),
        ),
        "mutually-exclusive",
    )
    expect_failure(
        lambda: ActionServer(
            node,
            LookupTransform,
            action_name + "/qos",
            lambda handle: LookupTransform.Result(),
            feedback_pub_qos_profile=QoSProfile(depth=1),
        ),
        "default QoS",
    )
    expect_failure(MultiThreadedExecutor, "MultiThreadedExecutor")
    assert node.action_servers == []
    print("DIRECT_CPP_ACTION_SERVER_P0_FAIL_CLOSED_OK")

    def goal_callback(goal):
        assert type(goal) is LookupTransform.Goal
        assert not hasattr(goal, "get_fields_and_field_types")
        target = str(goal.target_frame)
        decisions.append(("goal", target))
        if target == "server-close":
            assert server.close() is False
            assert server.close_pending
            assert not server.closed
            assert node.action_servers == [server]
            return GoalResponse.REJECT
        if target in ("destroy-node", "shutdown"):
            native_node = node._direct_cpp_node
            active_executor = node.executor
            operation = node.destroy_node if target == "destroy-node" else rclpy.shutdown
            try:
                operation()
            except BackendUnavailableError as error:
                assert "active action-server callback" in str(error)
            else:
                raise AssertionError("%s inside callback was accepted" % target)
            assert node._direct_cpp_node is native_node
            assert node.executor is active_executor
            assert node.action_servers == [server]
            assert rclpy.ok()
            return GoalResponse.REJECT
        if target == "goal-error":
            raise RuntimeError("contained goal decision error")
        if target == "reject":
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(goal_handle):
        assert type(goal_handle) is ServerGoalHandle
        assert type(goal_handle.request) is LookupTransform.Goal
        decisions.append(("cancel", str(goal_handle.request.target_frame)))
        return CancelResponse.ACCEPT

    def handle_accepted_callback(goal_handle):
        assert type(goal_handle) is ServerGoalHandle
        assert type(goal_handle.request) is LookupTransform.Goal
        assert type(goal_handle.goal_id) is UUID
        accepted_handles.append(goal_handle)
        target = str(goal_handle.request.target_frame)
        if target in ("success", "cancel", "deferred"):
            deferred[target] = goal_handle
            return
        goal_handle.execute()

    def execute_callback(goal_handle):
        assert type(goal_handle) is ServerGoalHandle
        assert type(goal_handle.request) is LookupTransform.Goal
        assert type(goal_handle.goal_id) is UUID
        target = str(goal_handle.request.target_frame)
        if target == "execute-error":
            raise RuntimeError("contained execute error")
        result = LookupTransform.Result()
        result.transform.child_frame_id = target + "-result"
        if target == "deferred":
            goal_handle.succeed()
        elif target == "cancel":
            assert goal_handle.is_cancel_requested
            goal_handle.canceled()
        elif target != "default-abort":
            raise AssertionError("unexpected accepted goal: %s" % target)
        return result

    server = ActionServer(
        node,
        LookupTransform,
        action_name,
        execute_callback,
        goal_callback=goal_callback,
        handle_accepted_callback=handle_accepted_callback,
        cancel_callback=cancel_callback,
        result_timeout=5.0,
    )
    client = ActionClient(node, LookupTransform, action_name)
    assert type(server) is ActionServer
    assert server.action_type is LookupTransform
    assert server.action_name == action_name
    assert node.action_servers == [server]
    assert node.action_clients == [client]
    assert client.wait_for_server(timeout_sec=10.0)

    def spin_until(predicate, label, timeout=15.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert predicate(), "timed out waiting for %s" % label

    def send_goal(target, feedback_callback=None):
        goal = LookupTransform.Goal(target_frame=target, source_frame="base")
        future = client.send_goal_async(goal, feedback_callback=feedback_callback)
        spin_until(future.done, "%s goal response" % target)
        assert future.exception() is None
        return future.result()

    def get_result(goal_handle, target):
        future = goal_handle.get_result_async()
        spin_until(future.done, "%s result" % target)
        assert future.exception() is None
        return future.result()

    success_handle = send_goal("success", feedback_messages.append)
    assert success_handle.accepted
    assert type(success_handle.goal_id) is UUID
    success_result_future = success_handle.get_result_async()
    success_server_handle = deferred.pop("success")
    success_server_handle.executing()
    feedback = LookupTransform.Feedback()
    success_server_handle.publish_feedback(feedback)
    spin_until(lambda: len(feedback_messages) == 1, "success feedback 1")
    shared_feedback = success_server_handle.create_feedback_shared()
    assert type(shared_feedback) is LookupTransform.Feedback
    assert bool(shared_feedback.__smartptr__())
    expect_failure(
        lambda: success_server_handle.publish_feedback_shared(
            LookupTransform.Feedback()),
        "shared factory",
    )
    success_server_handle.publish_feedback_shared(shared_feedback)
    spin_until(lambda: len(feedback_messages) == 2, "success feedback 2")
    shared_result = success_server_handle.create_result_shared()
    assert type(shared_result) is LookupTransform.Result
    assert bool(shared_result.__smartptr__())
    shared_result.transform.child_frame_id = "success-result"
    expect_failure(
        lambda: success_server_handle.succeed_shared(LookupTransform.Result()),
        "create_result_shared",
    )
    success_server_handle.succeed_shared(shared_result)
    spin_until(success_result_future.done, "success result")
    success_response = success_result_future.result()
    spin_until(lambda: len(feedback_messages) == 2, "success feedback")
    assert as_int8(success_response.status) == GoalStatus.STATUS_SUCCEEDED
    assert type(success_response.result) is LookupTransform.Result
    assert str(success_response.result.transform.child_frame_id) == "success-result"
    assert all(
        type(message) is LookupTransform.Impl.FeedbackMessage
        and type(message.feedback) is LookupTransform.Feedback
        for message in feedback_messages
    )
    retained.extend((
        success_handle.goal_id,
        success_response,
        *feedback_messages,
        shared_feedback,
        shared_result,
    ))
    print("DIRECT_CPP_ACTION_SERVER_SUCCESS_FEEDBACK_OK")

    rejected = send_goal("reject")
    assert not rejected.accepted
    goal_error = send_goal("goal-error")
    assert not goal_error.accepted
    destroy_rejected = send_goal("destroy-node")
    assert not destroy_rejected.accepted
    shutdown_rejected = send_goal("shutdown")
    assert not shutdown_rejected.accepted
    assert server.callback_error_ready()
    goal_error_value = server.take_callback_error()
    assert type(goal_error_value) is RuntimeError
    assert "goal decision" in str(goal_error_value)
    print("DIRECT_CPP_ACTION_SERVER_REJECT_ERROR_OK")
    print("DIRECT_CPP_ACTION_SERVER_CALLBACK_TEARDOWN_GUARD_OK")

    cancel_client_handle = send_goal("cancel")
    assert cancel_client_handle.accepted
    assert deferred["cancel"].status == GoalStatus.STATUS_ACCEPTED
    cancel_result_future = cancel_client_handle.get_result_async()
    cancel_future = cancel_client_handle.cancel_goal_async()
    spin_until(cancel_future.done, "cancel response")
    cancel_response = cancel_future.result()
    assert len(cancel_response.goals_canceling) == 1
    assert deferred["cancel"].is_cancel_requested
    deferred["cancel"].execute()
    spin_until(cancel_result_future.done, "canceled result")
    canceled_response = cancel_result_future.result()
    assert as_int8(canceled_response.status) == GoalStatus.STATUS_CANCELED
    assert str(canceled_response.result.transform.child_frame_id) == "cancel-result"
    retained.extend((cancel_response, canceled_response))
    print("DIRECT_CPP_ACTION_SERVER_CANCEL_OK")

    execute_error_handle = send_goal("execute-error")
    execute_error_response = get_result(execute_error_handle, "execute-error")
    assert as_int8(execute_error_response.status) == GoalStatus.STATUS_ABORTED
    assert type(execute_error_response.result) is LookupTransform.Result
    assert server.callback_error_ready()
    execute_error = server.take_callback_error()
    assert type(execute_error) is RuntimeError
    assert "execute error" in str(execute_error)

    default_abort_handle = send_goal("default-abort")
    default_abort_response = get_result(default_abort_handle, "default-abort")
    assert as_int8(default_abort_response.status) == GoalStatus.STATUS_ABORTED
    assert str(default_abort_response.result.transform.child_frame_id) == \
        "default-abort-result"
    print("DIRECT_CPP_ACTION_SERVER_DEFAULT_ABORT_OK")

    deferred_client_handle = send_goal("deferred")
    deferred_server_handle = deferred["deferred"]
    retained.extend((deferred_server_handle.request, deferred_server_handle.goal_id))
    del deferred["deferred"]
    gc.collect()
    assert str(deferred_server_handle.request.target_frame) == "deferred"
    assert deferred_server_handle.status == GoalStatus.STATUS_ACCEPTED
    deferred_server_handle.execute()
    deferred_response = get_result(deferred_client_handle, "deferred")
    assert as_int8(deferred_response.status) == GoalStatus.STATUS_SUCCEEDED
    assert str(deferred_response.result.transform.child_frame_id) == "deferred-result"
    retained.append(deferred_response)
    print("DIRECT_CPP_ACTION_SERVER_DEFERRED_GC_OK")

    stats = server.stats()
    assert stats.goals_requested == 9
    assert stats.goals_accepted == 5
    assert stats.goals_rejected == 4
    assert stats.accepted_goals_taken == 5
    assert stats.cancel_requests == 1
    assert stats.cancels_accepted == 1
    assert stats.cancels_rejected == 0
    assert stats.execute_transitions == 4
    assert stats.feedback_published == 2
    assert stats.results_succeeded == 2
    assert stats.results_aborted == 2
    assert stats.results_canceled == 1
    assert stats.active_goals == 0
    assert stats.python_goal_decision_crossings == 9
    assert stats.python_cancel_decision_crossings == 1
    assert stats.python_accepted_goal_crossings == 5
    assert stats.cpp_goal_shared_handoffs == 5
    assert stats.cpp_feedback_value_submissions == 2
    assert stats.cpp_result_value_submissions == 5
    assert stats.cpp_feedback_adapter_copies == 1
    assert stats.cpp_result_adapter_copies == 4
    assert stats.cpp_feedback_shared_handoffs == 1
    assert stats.cpp_result_shared_handoffs == 1
    assert stats.python_message_conversions == 0
    assert stats.python_serialization_calls == 0
    assert not server.callback_error_ready()
    assert decisions.count(("cancel", "cancel")) == 1

    records = [
        item for item in rclcppyy.status()["entities"]
        if "direct_cpp_action_server" in item["policies"]
    ]
    assert len(records) == 1
    evidence = records[0]["metadata"]
    assert evidence["goal_representation"] == "actual_cpp"
    assert evidence["goal_id_representation"] == "actual_cpp"
    assert evidence["feedback_representation"] == "actual_cpp"
    assert evidence["result_representation"] == "actual_cpp"
    assert evidence["python_message_conversions"] == 0
    assert evidence["python_serialization_calls"] == 0
    print("DIRECT_CPP_ACTION_SERVER_EVIDENCE_OK")

    close_rejected = send_goal("server-close")
    assert not close_rejected.accepted
    assert server.closed
    assert not server.close_pending
    assert node.action_servers == []
    print("DIRECT_CPP_ACTION_SERVER_DEFERRED_CLOSE_OK")

    assert client.destroy() is None
    assert server.destroy() is None
    assert server.closed
    assert node.action_clients == []
    assert node.action_servers == []
    node.destroy_node()
    rclpy.shutdown()
    gc.collect()
    assert any(int(value) for value in retained[0].uuid)
    assert str(retained[1].result.transform.child_frame_id) == "success-result"
    assert bool(shared_feedback.__smartptr__())
    assert bool(shared_result.__smartptr__())
    assert str(shared_result.transform.child_frame_id) == "success-result"
    assert type(retained[-3]) is LookupTransform.Goal
    assert type(retained[-2]) is UUID
    assert str(retained[-1].result.transform.child_frame_id) == "deferred-result"

    rclpy.init(args=[])
    second = Node("direct_action_server_reinit_%d" % os.getpid())
    second_server = ActionServer(
        second,
        LookupTransform,
        action_name,
        lambda handle: LookupTransform.Result(),
    )
    second_client = ActionClient(second, LookupTransform, action_name)
    assert second_client.wait_for_server(timeout_sec=10.0)
    second.destroy_node()
    assert second_server.closed
    assert second_client.closed
    rclpy.shutdown()
    print("DIRECT_CPP_ACTION_SERVER_TEARDOWN_REINIT_OK")


if __name__ == "__main__":
    main()
