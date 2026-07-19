#!/usr/bin/env python3
"""Registered custom action client against an AOT rclcpp_action server."""

import gc
import importlib
import os
from pathlib import Path
import select
import signal
import subprocess
import sys

import rclcppyy


INTERFACE = "rclcppyy_test_interfaces/action/Accumulate"
if len(sys.argv) != 2:
    raise SystemExit("usage: helper.py PATH_TO_AOT_PEER")
peer_path = Path(sys.argv[1]).resolve()
assert peer_path.is_file() and os.access(peer_path, os.X_OK)

rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp", interfaces=(INTERFACE, INTERFACE))

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from action_msgs.srv import CancelGoal  # noqa: E402
from ament_index_python.packages import get_package_prefix  # noqa: E402
from rclpy.action import ActionClient, ActionServer  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclcppyy.policy import BackendUnavailableError  # noqa: E402
from rclcppyy_test_interfaces.action import (  # noqa: E402
    Accumulate,
    Accumulate_GetResult_Request,
    Accumulate_GetResult_Response,
    Accumulate_SendGoal_Request,
    Accumulate_SendGoal_Response,
)
from rclcppyy_test_interfaces.msg import StampedValue  # noqa: E402
from unique_identifier_msgs.msg import UUID  # noqa: E402


cpp_action = cppyy.gbl.rclcppyy_test_interfaces.action.Accumulate
assert Accumulate.Goal is cpp_action.Goal
assert Accumulate.Result is cpp_action.Result
assert Accumulate.Feedback is cpp_action.Feedback
assert Accumulate.Impl.FeedbackMessage is cpp_action.Impl.FeedbackMessage
assert Accumulate.Impl.SendGoalService.Request is \
    cpp_action.Impl.SendGoalService.Request
assert Accumulate.Impl.SendGoalService.Response is \
    cpp_action.Impl.SendGoalService.Response
assert Accumulate.Impl.GetResultService.Request is \
    cpp_action.Impl.GetResultService.Request
assert Accumulate.Impl.GetResultService.Response is \
    cpp_action.Impl.GetResultService.Response
assert Accumulate_SendGoal_Request is cpp_action.Impl.SendGoalService.Request
assert Accumulate_SendGoal_Response is cpp_action.Impl.SendGoalService.Response
assert Accumulate_GetResult_Request is cpp_action.Impl.GetResultService.Request
assert Accumulate_GetResult_Response is cpp_action.Impl.GetResultService.Response
assert UUID is cppyy.gbl.unique_identifier_msgs.msg.UUID
assert CancelGoal.Request is cppyy.gbl.action_msgs.srv.CancelGoal.Request
assert CancelGoal.Response is cppyy.gbl.action_msgs.srv.CancelGoal.Response
assert StampedValue is cppyy.gbl.rclcppyy_test_interfaces.msg.StampedValue
assert Accumulate.Goal(target=4).target == 4
try:
    Accumulate.Goal(unknown=True)
except TypeError:
    pass
else:
    raise AssertionError("custom C++ action Goal accepted an unknown field")
print("DIRECT_CPP_GENERIC_ACTION_ALIASES_OK")


def as_int8(value):
    return ord(value) if isinstance(value, str) else int(value)


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("a Python-message conversion or serialization path ran")


bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
serialization = importlib.import_module("rclcpp_kit.serialization")
bringup.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary

prefix = "/direct_cpp/custom_action/p%d" % os.getpid()
peer = subprocess.Popen(
    [str(peer_path), prefix],
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
    env=os.environ.copy(),
    start_new_session=True,
)
peer_output = ""
retained_feedback = []
retained_values = []
try:
    readable, _, _ = select.select([peer.stdout], [], [], 15.0)
    assert readable, "custom AOT peer did not report readiness"
    ready = peer.stdout.readline()
    peer_output += ready
    assert ready.strip() == "AOT_PEER_READY", ready

    rclpy.init(args=[])
    node = Node("direct_custom_action_%d" % os.getpid())
    before = len(node.action_clients)
    try:
        ActionServer(node, Accumulate, prefix + "/accumulate", lambda goal: None)
    except BackendUnavailableError:
        pass
    else:
        raise AssertionError("direct custom action server was accepted")
    assert len(node.action_clients) == before
    print("DIRECT_CPP_GENERIC_ACTION_SERVER_FAIL_CLOSED_OK")

    client = ActionClient(node, Accumulate, prefix + "/accumulate")
    assert client.wait_for_server(timeout_sec=15.0)

    def feedback_callback(message):
        assert type(message) is Accumulate.Impl.FeedbackMessage
        assert type(message.feedback) is Accumulate.Feedback
        retained_feedback.append(message)

    goal_future = client.send_goal_async(
        Accumulate.Goal(target=4), feedback_callback=feedback_callback)
    rclpy.spin_until_future_complete(node, goal_future, timeout_sec=15.0)
    assert goal_future.done() and goal_future.exception() is None
    handle = goal_future.result()
    assert handle.accepted
    assert type(handle.goal_id) is UUID
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=15.0)
    assert result_future.done() and result_future.exception() is None
    response = result_future.result()
    assert type(response) is Accumulate.Impl.GetResultService.Response
    assert type(response.result) is Accumulate.Result
    assert type(response.result.summary) is StampedValue
    assert as_int8(response.status) == GoalStatus.STATUS_SUCCEEDED
    assert response.result.total == 10
    assert str(response.result.summary.label) == "aot-action"
    assert list(response.result.summary.samples) == [4, 10]
    assert retained_feedback
    assert retained_feedback[-1].feedback.partial_total == 10
    retained_values.extend((handle.goal_id, response))

    stats = client.stats()
    assert stats.goals_sent == 1
    assert stats.goals_accepted == 1
    assert stats.python_goal_crossings == 1
    assert stats.cpp_goal_value_submissions == 1
    compiled = Path(client.compile_result["so"])
    assert compiled.is_file()
    dynamic = subprocess.run(
        ["readelf", "-d", str(compiled)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert dynamic.returncode == 0, dynamic.stderr
    package_lib = str(
        Path(get_package_prefix("rclcppyy_test_interfaces")) / "lib")
    assert package_lib in dynamic.stdout
    print("DIRECT_CPP_GENERIC_ACTION_RUNPATH_OK")
    records = [
        item for item in rclcppyy.status()["entities"]
        if item["metadata"].get("action_interface") == INTERFACE
    ]
    assert len(records) == 1
    assert records[0]["backend"] == "cpp"
    assert records[0]["metadata"]["python_message_conversions"] == 0
    assert records[0]["metadata"]["python_serialization_calls"] == 0
    operation = rclcppyy.status()["operations"][-1]["metadata"]
    assert operation["requested_action_interfaces"] == [INTERFACE]
    assert operation["action_types"] == [
        "tf2_msgs::action::LookupTransform",
        "rclcppyy_test_interfaces::action::Accumulate",
    ]
    print("DIRECT_CPP_GENERIC_ACTION_AOT_INTEROP_OK")
    print("DIRECT_CPP_GENERIC_ACTION_EVIDENCE_OK")

    client.destroy()
    node.destroy_node()
    rclpy.shutdown()
    gc.collect()
    assert any(int(value) for value in retained_values[0].uuid)
    assert retained_values[1].result.total == 10
    assert retained_feedback[-1].feedback.partial_total == 10
    print("DIRECT_CPP_GENERIC_ACTION_RETAINED_TEARDOWN_OK")
finally:
    if rclpy.ok():
        rclpy.try_shutdown()
    if peer.poll() is None:
        os.killpg(peer.pid, signal.SIGINT)
    try:
        remaining, _ = peer.communicate(timeout=10.0)
    except subprocess.TimeoutExpired:
        os.killpg(peer.pid, signal.SIGKILL)
        remaining, _ = peer.communicate(timeout=5.0)
    peer_output += remaining
    if peer.returncode not in (0, -signal.SIGINT):
        raise AssertionError(
            "custom AOT peer failed (%d):\n%s" %
            (peer.returncode, peer_output))
