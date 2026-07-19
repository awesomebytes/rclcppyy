#!/usr/bin/env python3
"""Direct action server receiving a payload-rich external stock client goal."""

import gc
import importlib
import os
from pathlib import Path
import subprocess
import sys
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from rclpy.action import ActionServer, GoalResponse  # noqa: E402
from rclpy.action.server import ServerGoalHandle  # noqa: E402
from rclpy.node import Node  # noqa: E402
from tf2_msgs.action import LookupTransform  # noqa: E402
from unique_identifier_msgs.msg import UUID  # noqa: E402


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError(
        "conversion, serialization, or CDR entered stock action interop")


bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
native_action = importlib.import_module("rclcpp_kit.native_action")
native_action_server = importlib.import_module("rclcpp_kit.native_action_server")
serialization = importlib.import_module("rclcpp_kit.serialization")
rclpy_serialization = importlib.import_module("rclpy.serialization")
bringup.convert_python_msg_to_cpp = forbidden_boundary
native_action.convert_python_msg_to_cpp = forbidden_boundary
native_action_server.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary
serialization.serialized_message_from_bytes = forbidden_boundary
serialization.serialized_message_to_bytes = forbidden_boundary
rclpy_serialization.serialize_message = forbidden_boundary
rclpy_serialization.deserialize_message = forbidden_boundary


rclpy.init(args=[])
node = Node("direct_server_stock_action_client_%d" % os.getpid())
action_name = "/direct_cpp/action_server_stock/p%d" % os.getpid()
received_goals = []
accepted_handles = []
published_feedback = []
returned_results = []


def goal_callback(goal):
    assert type(goal) is LookupTransform.Goal
    assert str(goal.target_frame) == "stock-client-target"
    assert str(goal.source_frame) == "stock-client-source"
    assert goal.source_time.sec == 17
    assert goal.source_time.nanosec == 23
    received_goals.append(goal)
    return GoalResponse.ACCEPT


def accepted_callback(handle):
    assert type(handle) is ServerGoalHandle
    assert type(handle.request) is LookupTransform.Goal
    assert type(handle.goal_id) is UUID
    accepted_handles.append(handle)
    handle.execute()


def execute_callback(handle):
    assert type(handle) is ServerGoalHandle
    assert handle is accepted_handles[0]
    for _ in range(2):
        feedback = LookupTransform.Feedback()
        assert type(feedback) is cppyy.gbl.tf2_msgs.action.LookupTransform.Feedback
        published_feedback.append(feedback)
        handle.publish_feedback(feedback)
        time.sleep(0.02)
    result = LookupTransform.Result()
    assert type(result) is cppyy.gbl.tf2_msgs.action.LookupTransform.Result
    result.transform.header.frame_id = str(handle.request.source_frame)
    result.transform.child_frame_id = "stock-client-result"
    result.transform.header.stamp.sec = handle.request.source_time.sec
    result.transform.header.stamp.nanosec = handle.request.source_time.nanosec
    returned_results.append(result)
    handle.succeed()
    return result


server = ActionServer(
    node,
    LookupTransform,
    action_name,
    execute_callback,
    goal_callback=goal_callback,
    handle_accepted_callback=accepted_callback,
)
peer_path = Path(__file__).with_name("_stock_action_client_peer.py")
peer = subprocess.Popen(
    [sys.executable, str(peer_path), action_name],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    text=True,
    env=os.environ.copy(),
)
deadline = time.monotonic() + 30.0
while peer.poll() is None and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.02)
if peer.poll() is None:
    peer.kill()
stdout, stderr = peer.communicate(timeout=5.0)
assert peer.returncode == 0, (
    "stock action client exit=%s\nstdout:\n%s\nstderr:\n%s" %
    (peer.returncode, stdout, stderr)
)
assert "STOCK_ACTION_CLIENT_DIRECT_SERVER_OK" in stdout
assert "STOCK_ACTION_CLIENT_TEARDOWN_OK" in stdout
assert len(received_goals) == 1
assert len(accepted_handles) == 1
assert len(published_feedback) == 2
assert len(returned_results) == 1
stats = server.stats()
assert stats.goals_requested == 1
assert stats.goals_accepted == 1
assert stats.goals_rejected == 0
assert stats.accepted_goals_taken == 1
assert stats.feedback_published == 2
assert stats.results_succeeded == 1
assert stats.python_message_conversions == 0
assert stats.python_serialization_calls == 0

records = [
    item for item in rclcppyy.status()["entities"]
    if "direct_cpp_action_server" in item["policies"]
]
assert len(records) == 1
metadata = records[0]["metadata"]
assert metadata["goal_representation"] == "actual_cpp"
assert metadata["feedback_representation"] == "actual_cpp"
assert metadata["result_representation"] == "actual_cpp"
assert metadata["python_message_conversions"] == 0
assert metadata["python_serialization_calls"] == 0
print("DIRECT_CPP_ACTION_SERVER_STOCK_EXACT_CPP_OK")

retained = (
    received_goals[0], accepted_handles[0].goal_id,
    published_feedback[0], returned_results[0],
)
server.destroy()
node.destroy_node()
rclpy.shutdown()
gc.collect()
assert str(retained[0].target_frame) == "stock-client-target"
assert type(retained[1]) is UUID
assert any(int(value) for value in retained[1].uuid)
assert type(retained[2]) is LookupTransform.Feedback
assert str(retained[3].transform.child_frame_id) == "stock-client-result"
print("DIRECT_CPP_ACTION_SERVER_STOCK_RETAINED_TEARDOWN_OK")
