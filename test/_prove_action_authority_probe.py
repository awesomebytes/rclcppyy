#!/usr/bin/env python3
"""Authority coverage tripwire for the batch-(a) action promotion set.

Enumerates the exact ledger rows the prove-authority lane promotes to
``exact_direct_cpp_authority`` for the action slice (PLAN-prove-authority
§2a/§3, uncommitted): the direct-C++ ``ActionClient``/``ClientGoalHandle``
(client side, against an external stock server peer) and
``ActionServer``/``ServerGoalHandle`` (server side, against an external stock
client peer), each exercised with converters/serializers poisoned and C++
representation asserted on goal/feedback/result/UUID payloads.

``ActionClient``/``ActionServer`` are each importable at two public module
paths (``rclpy.action.*`` and ``rclpy.action.{client,server}.*``) that name
the *same* class object -- proven once below, which is why exercising one
proves both ledger rows. ``send_goal``/``get_result``/``cancel_goal`` (the
synchronous variants) unconditionally fail-closed in this backend; they are
proven to fail-closed here specifically so the decline is evidence, not
assumption -- they are excluded from the promoted set.
"""

import gc
import importlib
import json
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
from rclcppyy.policy import BackendUnavailableError  # noqa: E402
import rclpy.action as action_pkg  # noqa: E402
import rclpy.action.client as client_module  # noqa: E402
import rclpy.action.server as server_module  # noqa: E402
from rclpy.node import Node  # noqa: E402
from tf2_msgs.action import LookupTransform  # noqa: E402
from unique_identifier_msgs.msg import UUID  # noqa: E402


PROBE_PREFIX = "RCLCPPYY_PROVE_ACTION_AUTHORITY_PROBE "

ACTION_CLIENT_PROMOTED = (
    "destroy", "send_goal_async", "server_is_ready", "wait_for_server")
CLIENT_GOAL_HANDLE_PROMOTED = (
    "__eq__", "__ne__", "__repr__", "accepted", "cancel_goal_async",
    "get_result_async", "goal_id", "stamp", "status")
ACTION_SERVER_PROMOTED = (
    "action_type", "destroy", "notify_execute", "notify_goal_done",
    "register_cancel_callback", "register_execute_callback",
    "register_goal_callback", "register_handle_accepted_callback")
SERVER_GOAL_HANDLE_PROMOTED = (
    "__eq__", "__ne__", "abort", "canceled", "destroy", "execute", "executing",
    "goal_id", "is_active", "is_cancel_requested", "publish_feedback",
    "request", "status", "succeed")

ACTION_CLIENT_FAIL_CLOSED = ("send_goal",)
CLIENT_GOAL_HANDLE_FAIL_CLOSED = ("get_result", "cancel_goal")


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("a promoted action operation used a conversion or serialization path")


def _poison_boundary():
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


def spin_until(node, predicate, timeout=15.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
    assert predicate()


def as_int8(value):
    return ord(value) if isinstance(value, str) else int(value)


def _client_side(proven):
    """Direct ActionClient/ClientGoalHandle against an external stock server."""
    assert action_pkg.ActionClient is client_module.ActionClient
    action_name = "/prove_action_authority/client/p%d" % os.getpid()
    server_path = Path(__file__).with_name("_stock_action_server_helper.py")
    server = subprocess.Popen(
        [sys.executable, str(server_path), action_name],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    retained = []
    try:
        rclpy.init(args=[])
        node = Node("prove_action_authority_client_%d" % os.getpid())
        client = action_pkg.ActionClient(node, LookupTransform, action_name)
        assert client.wait_for_server(timeout_sec=15.0)
        assert server.poll() is None
        proven["rclpy.action.ActionClient.wait_for_server"] = True
        proven["rclpy.action.client.ActionClient.wait_for_server"] = True
        proven["rclpy.action.ActionClient.server_is_ready"] = client.server_is_ready()
        proven["rclpy.action.client.ActionClient.server_is_ready"] = client.server_is_ready()

        feedback_messages = []
        goal = LookupTransform.Goal(target_frame="map", source_frame="base")
        goal_future = client.send_goal_async(goal, feedback_callback=feedback_messages.append)
        proven["rclpy.action.ActionClient.send_goal_async"] = True
        proven["rclpy.action.client.ActionClient.send_goal_async"] = True
        rclpy.spin_until_future_complete(node, goal_future, timeout_sec=15.0)
        handle = goal_future.result()
        assert type(handle) is client_module.ClientGoalHandle
        assert handle.accepted
        proven["rclpy.action.client.ClientGoalHandle.accepted"] = True
        assert type(handle.goal_id) is UUID
        proven["rclpy.action.client.ClientGoalHandle.goal_id"] = True
        assert type(handle.stamp) is cppyy.gbl.builtin_interfaces.msg.Time
        proven["rclpy.action.client.ClientGoalHandle.stamp"] = True
        assert handle == goal_future.result()
        proven["rclpy.action.client.ClientGoalHandle.__eq__"] = True
        assert not (handle != goal_future.result())
        proven["rclpy.action.client.ClientGoalHandle.__ne__"] = True
        assert "ClientGoalHandle" in repr(handle)
        proven["rclpy.action.client.ClientGoalHandle.__repr__"] = True

        spin_until(node, lambda: len(feedback_messages) == 3)
        result_future = handle.get_result_async()
        proven["rclpy.action.client.ClientGoalHandle.get_result_async"] = True
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=15.0)
        response = result_future.result()
        assert as_int8(response.status) == GoalStatus.STATUS_SUCCEEDED
        assert type(response.result) is LookupTransform.Result
        assert str(response.result.transform.child_frame_id) == "successful-stock-result"
        proven["rclpy.action.client.ClientGoalHandle.status"] = handle.status == GoalStatus.STATUS_SUCCEEDED

        # The synchronous variants unconditionally fail-closed: proof, not assumption.
        for name, operation in (
            ("send_goal", lambda: client.send_goal(goal)),
            ("get_result", handle.get_result),
        ):
            try:
                operation()
            except BackendUnavailableError:
                proven["fail_closed:%s" % name] = True
            else:
                raise AssertionError("%s unexpectedly succeeded" % name)

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
        proven["rclpy.action.client.ClientGoalHandle.cancel_goal_async"] = True
        rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=15.0)
        cancel_response = cancel_future.result()
        assert len(cancel_response.goals_canceling) == 1
        rclpy.spin_until_future_complete(node, cancel_result_future, timeout_sec=15.0)
        canceled_result = cancel_result_future.result()
        assert as_int8(canceled_result.status) == GoalStatus.STATUS_CANCELED

        try:
            cancel_handle.cancel_goal()
        except BackendUnavailableError:
            proven["fail_closed:cancel_goal"] = True
        else:
            raise AssertionError("cancel_goal unexpectedly succeeded")

        records = [
            item for item in rclcppyy.status()["entities"]
            if "direct_cpp_action" in item["policies"]
        ]
        assert len(records) == 1
        evidence = records[0]["metadata"]
        assert evidence["goal_representation"] == "actual_cpp"
        assert evidence["feedback_representation"] == "actual_cpp"
        assert evidence["result_representation"] == "actual_cpp"
        assert evidence["python_message_conversions"] == 0

        proven["rclpy.action.ActionClient.destroy"] = client.destroy() is None
        proven["rclpy.action.client.ActionClient.destroy"] = True
        retained.extend((handle.goal_id, response, canceled_result))
        node.destroy_node()
        rclpy.shutdown()
    finally:
        if rclpy.ok():
            rclpy.try_shutdown()
        server.terminate()
        try:
            server.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            server.kill()
            server.wait(timeout=5.0)
    gc.collect()
    return retained


def _server_side(proven):
    """Direct ActionServer/ServerGoalHandle against an external stock client."""
    assert action_pkg.ActionServer is server_module.ActionServer
    action_name = "/prove_action_authority/server/p%d" % os.getpid()
    rclpy.init(args=[])
    node = Node("prove_action_authority_server_%d" % os.getpid())
    accepted_handles = []
    published_feedback = []
    returned_results = []

    def goal_callback(goal):
        assert type(goal) is LookupTransform.Goal
        return action_pkg.GoalResponse.ACCEPT

    def accepted_callback(handle):
        assert type(handle) is server_module.ServerGoalHandle
        assert type(handle.goal_id) is UUID
        proven["rclpy.action.server.ServerGoalHandle.goal_id"] = True
        assert type(handle.request) is LookupTransform.Goal
        proven["rclpy.action.server.ServerGoalHandle.request"] = True
        assert handle.is_active
        proven["rclpy.action.server.ServerGoalHandle.is_active"] = True
        assert not handle.is_cancel_requested
        proven["rclpy.action.server.ServerGoalHandle.is_cancel_requested"] = True
        # Driven manually below (outside this callback) to prove executing()/
        # publish_feedback()/succeed() on the argument-taking path independent
        # of the auto-execute()/notify_execute() flow (proven separately, by
        # the manual abort/cancel pair further down).
        accepted_handles.append(handle)

    server = action_pkg.ActionServer(
        node, LookupTransform, action_name,
        lambda handle: LookupTransform.Result(),
        goal_callback=goal_callback, handle_accepted_callback=accepted_callback,
    )
    proven["rclpy.action.ActionServer.action_type"] = server.action_type is LookupTransform
    proven["rclpy.action.server.ActionServer.action_type"] = True
    proven["rclpy.action.ActionServer.register_goal_callback"] = True
    proven["rclpy.action.server.ActionServer.register_goal_callback"] = True
    proven["rclpy.action.ActionServer.register_handle_accepted_callback"] = True
    proven["rclpy.action.server.ActionServer.register_handle_accepted_callback"] = True
    proven["rclpy.action.ActionServer.register_execute_callback"] = True
    proven["rclpy.action.server.ActionServer.register_execute_callback"] = True
    server.register_cancel_callback(lambda _handle: action_pkg.CancelResponse.REJECT)
    proven["rclpy.action.ActionServer.register_cancel_callback"] = True
    proven["rclpy.action.server.ActionServer.register_cancel_callback"] = True
    assert server.notify_goal_done() is None
    proven["rclpy.action.ActionServer.notify_goal_done"] = True
    proven["rclpy.action.server.ActionServer.notify_goal_done"] = True

    peer_path = Path(__file__).with_name("_stock_action_client_peer.py")
    peer = subprocess.Popen(
        [sys.executable, str(peer_path), action_name],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        env=os.environ.copy(),
    )
    driven = False
    deadline = time.monotonic() + 30.0
    while peer.poll() is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if not driven and accepted_handles:
            driven = True
            handle = accepted_handles[0]
            handle.executing()
            proven["rclpy.action.server.ServerGoalHandle.executing"] = True
            assert handle.status == GoalStatus.STATUS_EXECUTING
            proven["rclpy.action.server.ServerGoalHandle.status"] = True
            for _ in range(2):
                feedback = LookupTransform.Feedback()
                assert type(feedback) is cppyy.gbl.tf2_msgs.action.LookupTransform.Feedback
                published_feedback.append(feedback)
                handle.publish_feedback(feedback)
            proven["rclpy.action.server.ServerGoalHandle.publish_feedback"] = True
            result = LookupTransform.Result()
            assert type(result) is cppyy.gbl.tf2_msgs.action.LookupTransform.Result
            result.transform.child_frame_id = "stock-client-result"
            returned_results.append(result)
            handle.succeed(result)
            proven["rclpy.action.server.ServerGoalHandle.succeed"] = True
    if peer.poll() is None:
        peer.kill()
    stdout, stderr = peer.communicate(timeout=5.0)
    assert peer.returncode == 0, (
        "stock action client exit=%s\nstdout:\n%s\nstderr:\n%s" %
        (peer.returncode, stdout, stderr))
    assert "STOCK_ACTION_CLIENT_DIRECT_SERVER_OK" in stdout
    assert driven
    assert len(accepted_handles) == 1
    handle = accepted_handles[0]
    assert handle == accepted_handles[0]
    proven["rclpy.action.server.ServerGoalHandle.__eq__"] = True
    assert not (handle != accepted_handles[0])
    proven["rclpy.action.server.ServerGoalHandle.__ne__"] = True

    # A second, manually-routed goal pair proves execute()/notify_execute()
    # (auto-execute flow) and abort()/canceled()/destroy(). "cancel-case" is
    # deferred (not auto-executed) until the client's cancel request is
    # accepted -- rcl_action rejects a direct EXECUTING -> CANCELED jump, so
    # canceled() must observe is_cancel_requested first, exactly like the
    # already-proven _direct_cpp_action_server_helper.py "cancel" scenario.
    manual_accepted = {}

    def manual_execute_callback(handle):
        target = str(handle.request.target_frame)
        result = LookupTransform.Result()
        if target == "abort-case":
            handle.abort(result)
            proven["rclpy.action.server.ServerGoalHandle.abort"] = True
            return result
        assert handle.is_cancel_requested
        handle.canceled()
        proven["rclpy.action.server.ServerGoalHandle.canceled"] = True
        return result

    def manual_accepted_callback(handle):
        target = str(handle.request.target_frame)
        manual_accepted[target] = handle
        if target == "abort-case":
            handle.execute()
            proven["rclpy.action.server.ServerGoalHandle.execute"] = True
            proven["rclpy.action.ActionServer.notify_execute"] = True
            proven["rclpy.action.server.ActionServer.notify_execute"] = True
        # cancel-case stays deferred; executed only once cancellation lands.

    manual_server = action_pkg.ActionServer(
        node, LookupTransform, action_name + "/manual", manual_execute_callback,
        handle_accepted_callback=manual_accepted_callback,
        cancel_callback=lambda _handle: action_pkg.CancelResponse.ACCEPT,
    )
    from rclpy.action import ActionClient as _LocalActionClient

    manual_client = _LocalActionClient(node, LookupTransform, action_name + "/manual")
    assert manual_client.wait_for_server(timeout_sec=15.0)

    manual_goal_future = manual_client.send_goal_async(
        LookupTransform.Goal(target_frame="abort-case"))
    rclpy.spin_until_future_complete(node, manual_goal_future, timeout_sec=15.0)
    spin_until(node, lambda: "abort-case" in manual_accepted)
    manual_handle = manual_accepted["abort-case"]
    spin_until(node, lambda: manual_handle.status == GoalStatus.STATUS_ABORTED)

    manual_goal_future_2 = manual_client.send_goal_async(
        LookupTransform.Goal(target_frame="cancel-case"))
    rclpy.spin_until_future_complete(node, manual_goal_future_2, timeout_sec=15.0)
    manual_client_handle_2 = manual_goal_future_2.result()
    spin_until(node, lambda: "cancel-case" in manual_accepted)
    manual_handle_2 = manual_accepted["cancel-case"]
    cancel_future = manual_client_handle_2.cancel_goal_async()
    rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=15.0)
    spin_until(node, lambda: manual_handle_2.is_cancel_requested)
    manual_handle_2.execute()
    spin_until(node, lambda: manual_handle_2.status == GoalStatus.STATUS_CANCELED)
    manual_handle.destroy()
    proven["rclpy.action.server.ServerGoalHandle.destroy"] = True

    proven["rclpy.action.ActionServer.destroy"] = server.destroy() is None
    proven["rclpy.action.server.ActionServer.destroy"] = True
    manual_client.destroy()
    manual_server.destroy()
    stats = server.stats()
    assert stats.python_message_conversions == 0
    assert stats.python_serialization_calls == 0
    retained = (published_feedback[0], returned_results[0])
    node.destroy_node()
    rclpy.shutdown()
    gc.collect()
    return retained


def main():
    proven = {}
    _poison_boundary()
    client_retained = _client_side(proven)
    server_retained = _server_side(proven)

    assert client_retained
    assert server_retained

    for name in ACTION_CLIENT_PROMOTED:
        for module in ("rclpy.action.ActionClient", "rclpy.action.client.ActionClient"):
            assert proven.get("%s.%s" % (module, name)) is True, "%s.%s" % (module, name)
    for name in CLIENT_GOAL_HANDLE_PROMOTED:
        path = "rclpy.action.client.ClientGoalHandle.%s" % name
        assert proven.get(path) is True, path
    for name in ACTION_SERVER_PROMOTED:
        for module in ("rclpy.action.ActionServer", "rclpy.action.server.ActionServer"):
            assert proven.get("%s.%s" % (module, name)) is True, "%s.%s" % (module, name)
    for name in SERVER_GOAL_HANDLE_PROMOTED:
        path = "rclpy.action.server.ServerGoalHandle.%s" % name
        assert proven.get(path) is True, path
    for name in ACTION_CLIENT_FAIL_CLOSED + CLIENT_GOAL_HANDLE_FAIL_CLOSED:
        assert proven.get("fail_closed:%s" % name) is True, name

    report = {
        "proven": {key: value for key, value in proven.items() if not key.startswith("fail_closed:")},
        "fail_closed": {
            key[len("fail_closed:"):]: value
            for key, value in proven.items() if key.startswith("fail_closed:")
        },
    }
    print(PROBE_PREFIX + json.dumps(report, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
