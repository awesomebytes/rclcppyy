#!/usr/bin/env python3
"""Run stock or compatible Python against the ahead-of-time C++ peer."""

import argparse
import json
import os
import platform
import queue
import signal
import subprocess
import threading
import time
from pathlib import Path


OPERATION_TIMEOUT_S = 20.0


class PeerProcess:
    def __init__(self, executable, prefix):
        self._lines = []
        self._queue = queue.Queue()
        self._process = subprocess.Popen(
            [executable, prefix],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=os.environ.copy(),
            start_new_session=True,
        )
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self):
        for raw_line in self._process.stdout:
            line = raw_line.rstrip()
            self._lines.append(line)
            self._queue.put(line)

    @property
    def lines(self):
        return tuple(self._lines)

    def wait_for(self, marker, timeout_s=OPERATION_TIMEOUT_S):
        deadline = time.monotonic() + timeout_s
        while marker not in self._lines:
            if self._process.poll() is not None:
                raise RuntimeError(
                    "AOT peer exited before %s: %s" % (marker, self._lines))
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    "timed out waiting for %s: %s" % (marker, self._lines))
            try:
                self._queue.get(timeout=min(0.2, remaining))
            except queue.Empty:
                pass

    def close(self):
        if self._process.poll() is None:
            os.killpg(self._process.pid, signal.SIGINT)
            try:
                self._process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self._process.pid, signal.SIGKILL)
                self._process.wait(timeout=5)
        self._reader.join(timeout=2)
        print("AOT_PEER_OUTPUT_BEGIN")
        print("\n".join(self._lines))
        print("AOT_PEER_OUTPUT_END")


def spin_until(executor, predicate, description, timeout_s=OPERATION_TIMEOUT_S,
               peer_executor=None):
    deadline = time.monotonic() + timeout_s
    while not predicate() and time.monotonic() < deadline:
        if peer_executor is not None:
            peer_executor.spin_some()
        executor.spin_once(timeout_sec=0.02)
    if not predicate():
        raise TimeoutError("timed out waiting for %s" % description)


def relevant_backend_records(status, topic):
    return [
        record for record in status["entities"]
        if record["metadata"].get("topic") == topic
    ]


def relevant_action_records(status, action_name):
    return [
        record for record in status["entities"]
        if record["metadata"].get("action_name") == action_name
    ]


def run_python_action_contract(context, action_type, prefix):
    import rclpy
    from action_msgs.msg import GoalStatus
    from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    action_name = prefix + "/python_accumulate"
    server_node = None
    client_node = None
    action_server = None
    action_client = None
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    callbacks = {"goals": [], "cancels": 0, "executions": []}
    success_feedback_complete = threading.Event()
    contract = None

    def goal_callback(goal_request):
        callbacks["goals"].append(goal_request.target)
        if goal_request.target < 1:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(_goal_handle):
        callbacks["cancels"] += 1
        return CancelResponse.ACCEPT

    def execute_callback(goal_handle):
        target = goal_handle.request.target
        callbacks["executions"].append(target)
        result = action_type.Result()
        feedback = action_type.Feedback()
        total = 0
        upper_bound = 1 if target == 100 else target
        for current in range(1, upper_bound + 1):
            total += current
            feedback.current = current
            feedback.partial_total = total
            goal_handle.publish_feedback(feedback)

        if target == 100:
            deadline = time.monotonic() + OPERATION_TIMEOUT_S
            while not goal_handle.is_cancel_requested and time.monotonic() < deadline:
                time.sleep(0.005)
            result.total = total
            result.summary.sequence = 1
            result.summary.label = "python-canceled"
            result.summary.samples = [1, total]
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result

        result.total = total
        result.summary.sequence = target
        result.summary.label = "python-succeeded"
        result.summary.samples = [target, total]
        if not success_feedback_complete.wait(timeout=OPERATION_TIMEOUT_S):
            result.summary.label = "python-feedback-timeout"
            goal_handle.abort()
            return result
        goal_handle.succeed()
        return result

    try:
        server_node = rclpy.create_node(
            "custom_action_server", context=context,
            start_parameter_services=False)
        client_node = rclpy.create_node(
            "custom_action_client", context=context,
            start_parameter_services=False)
        server_group = ReentrantCallbackGroup()
        client_group = ReentrantCallbackGroup()
        action_server = ActionServer(
            server_node,
            action_type,
            action_name,
            execute_callback=execute_callback,
            callback_group=server_group,
            goal_callback=goal_callback,
            cancel_callback=cancel_callback,
            result_timeout=5,
        )
        action_client = ActionClient(
            client_node,
            action_type,
            action_name,
            callback_group=client_group,
        )
        executor.add_node(server_node)
        executor.add_node(client_node)
        spin_until(
            executor,
            lambda: action_client.wait_for_server(timeout_sec=0.0),
            "Python action server discovery",
        )
        feedback_topic = action_name + "/_action/feedback"
        spin_until(
            executor,
            lambda: (
                client_node.count_publishers(feedback_topic) >= 1
                and server_node.count_subscribers(feedback_topic) >= 1
            ),
            "Python action feedback endpoint discovery",
        )

        success_feedback = []

        def record_success_feedback(update):
            success_feedback.append(update.feedback.partial_total)
            if update.feedback.partial_total == 10:
                success_feedback_complete.set()

        success_goal = action_type.Goal()
        success_goal.target = 4
        success_goal_future = action_client.send_goal_async(
            success_goal,
            feedback_callback=record_success_feedback,
        )
        spin_until(
            executor, success_goal_future.done, "Python action goal acceptance")
        success_handle = success_goal_future.result()
        assert success_handle.accepted
        success_result_future = success_handle.get_result_async()
        spin_until(
            executor, success_result_future.done, "Python action success result")
        try:
            spin_until(
                executor,
                lambda: success_feedback == [1, 3, 6, 10],
                "Python action success feedback",
            )
        except TimeoutError as exc:
            raise AssertionError(
                "Python action feedback mismatch: %r" % success_feedback) from exc
        success_result = success_result_future.result()
        assert success_result.status == GoalStatus.STATUS_SUCCEEDED
        assert success_result.result.total == 10
        assert success_result.result.summary.label == "python-succeeded"

        rejected_goal = action_type.Goal()
        rejected_goal.target = 0
        rejected_goal_future = action_client.send_goal_async(rejected_goal)
        spin_until(
            executor, rejected_goal_future.done, "Python action goal rejection")
        rejected_handle = rejected_goal_future.result()
        assert not rejected_handle.accepted

        cancel_feedback = []
        cancel_goal = action_type.Goal()
        cancel_goal.target = 100
        cancel_goal_future = action_client.send_goal_async(
            cancel_goal,
            feedback_callback=lambda update: cancel_feedback.append(
                update.feedback.partial_total),
        )
        spin_until(
            executor, cancel_goal_future.done, "Python action cancel goal acceptance")
        cancel_handle = cancel_goal_future.result()
        assert cancel_handle.accepted
        cancel_result_future = cancel_handle.get_result_async()
        spin_until(
            executor, lambda: cancel_feedback == [1],
            "Python action cancellation feedback")
        cancel_response_future = cancel_handle.cancel_goal_async()
        spin_until(
            executor, cancel_response_future.done,
            "Python action cancellation response")
        cancel_response = cancel_response_future.result()
        assert len(cancel_response.goals_canceling) == 1
        spin_until(
            executor, cancel_result_future.done, "Python action canceled result")
        cancel_result = cancel_result_future.result()
        assert cancel_result.status == GoalStatus.STATUS_CANCELED
        assert cancel_result.result.total == 1
        assert cancel_result.result.summary.label == "python-canceled"

        contract = {
            "action_name_suffix": "/python_accumulate",
            "identity": {
                "server_node_exact": type(server_node) is Node,
                "client_node_exact": type(client_node) is Node,
                "server_exact": type(action_server) is ActionServer,
                "client_exact": type(action_client) is ActionClient,
                "context_preserved": (
                    server_node.context is context and client_node.context is context),
                "server_group": server_group.has_entity(action_server),
                "client_group": client_group.has_entity(action_client),
            },
            "success": {
                "feedback": success_feedback,
                "status": success_result.status,
                "total": success_result.result.total,
                "summary": success_result.result.summary.label,
            },
            "rejected": not rejected_handle.accepted,
            "canceled": {
                "feedback": cancel_feedback,
                "goals_canceling": len(cancel_response.goals_canceling),
                "status": cancel_result.status,
                "total": cancel_result.result.total,
                "summary": cancel_result.result.summary.label,
            },
            "callbacks": callbacks,
        }
    finally:
        if action_client is not None:
            action_client.destroy()
        if action_server is not None:
            action_server.destroy()
        if client_node is not None:
            executor.remove_node(client_node)
        if server_node is not None:
            executor.remove_node(server_node)
        executor.shutdown(timeout_sec=2.0)
        if contract is not None:
            contract["teardown"] = {
                "client_waitable_removed": action_client not in client_node.waitables,
                "server_waitable_removed": action_server not in server_node.waitables,
            }
        if client_node is not None:
            client_node.destroy_node()
        if server_node is not None:
            server_node.destroy_node()
    if not all(contract["identity"].values()):
        raise AssertionError("Python action identity contract failed")
    if not all(contract["teardown"].values()):
        raise AssertionError("Python action teardown contract failed")
    return contract, action_name


def run(mode, peer_executable, evidence_path, reference_path=None):
    if mode == "activated":
        import rclcppyy
        rclcppyy.enable_cpp_acceleration()
    else:
        rclcppyy = None

    import rclpy
    from action_msgs.msg import GoalStatus
    from rclpy.action import ActionClient
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.publisher import Publisher
    from rclcppyy_test_interfaces.action import Accumulate
    from rclcppyy_test_interfaces.msg import StampedValue
    from rclcppyy_test_interfaces.srv import TransformValue

    prefix = "/rclcppyy_custom_%s_%d" % (mode, os.getpid())
    peer = PeerProcess(peer_executable, prefix)
    context = Context()
    node = None
    executor = None
    action_client = None
    publisher = None
    subscription = None
    service_client = None
    evidence = {
        "schema": "rclcppyy.custom-interface-interop/v1",
        "mode": mode,
        "architecture": platform.machine(),
        "interface_package": "rclcppyy_test_interfaces",
        "peer": "ahead-of-time-rclcpp",
        "outcomes": {},
        "backend": {"activated": mode == "activated"},
    }
    try:
        peer.wait_for("AOT_PEER_READY")
        context.init(args=[])
        node = rclpy.create_node("custom_interface_%s" % mode, context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        replies = []
        reply_topic = prefix + "/cpp_to_python"
        request_topic = prefix + "/python_to_cpp"
        subscription = node.create_subscription(
            StampedValue, reply_topic, replies.append, 10)
        publisher = node.create_publisher(StampedValue, request_topic, 10)
        assert type(node) is Node
        assert type(publisher) is Publisher
        spin_until(
            executor,
            lambda: publisher.get_subscription_count() >= 1,
            "AOT message subscriber discovery",
        )
        request = StampedValue()
        request.stamp.sec = 7
        request.stamp.nanosec = 11
        request.sequence = 23
        request.label = "python-%s" % mode
        request.samples = [2, 3, 5]
        deadline = time.monotonic() + OPERATION_TIMEOUT_S
        while not replies and time.monotonic() < deadline:
            publisher.publish(request)
            executor.spin_once(timeout_sec=0.1)
        if not replies:
            raise TimeoutError("timed out waiting for AOT custom-message reply")
        reply = replies[0]
        assert reply.sequence == 123
        assert reply.label == "aot:python-%s" % mode
        assert list(reply.samples) == [4, 6, 10]
        evidence["outcomes"]["message_bidirectional"] = True

        service_client = node.create_client(
            TransformValue, prefix + "/transform")
        spin_until(
            executor,
            service_client.service_is_ready,
            "AOT custom service discovery",
        )
        service_request = TransformValue.Request()
        service_request.input = request
        service_request.scale = 3
        service_future = service_client.call_async(service_request)
        spin_until(
            executor, service_future.done, "AOT custom service response")
        service_response = service_future.result()
        assert service_response.output.sequence == 24
        assert service_response.output.label == "aot-service:python-%s" % mode
        assert list(service_response.output.samples) == [6, 9, 15]
        evidence["outcomes"]["service_bidirectional"] = True

        feedback = []
        action_client = ActionClient(
            node, Accumulate, prefix + "/accumulate")
        spin_until(
            executor,
            lambda: action_client.wait_for_server(timeout_sec=0.0),
            "AOT custom action discovery",
        )
        goal = Accumulate.Goal()
        goal.target = 4
        goal_future = action_client.send_goal_async(
            goal,
            feedback_callback=lambda update: feedback.append(
                update.feedback.partial_total),
        )
        spin_until(executor, goal_future.done, "AOT action goal acceptance")
        goal_handle = goal_future.result()
        assert goal_handle.accepted
        result_future = goal_handle.get_result_async()
        spin_until(executor, result_future.done, "AOT action result")
        wrapped_result = result_future.result()
        assert wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
        assert wrapped_result.result.total == 10
        assert wrapped_result.result.summary.label == "aot-action"
        assert list(wrapped_result.result.summary.samples) == [4, 10]
        assert feedback and feedback[-1] == 10
        evidence["outcomes"]["action_bidirectional"] = True

        python_action_contract, python_action_name = run_python_action_contract(
            context, Accumulate, prefix)
        evidence["outcomes"]["python_action_client_server"] = True
        evidence["action_contract"] = python_action_contract

        peer.wait_for("AOT_MESSAGE_ROUNDTRIP_OK")
        peer.wait_for("AOT_SERVICE_ROUNDTRIP_OK")
        peer.wait_for("AOT_ACTION_ROUNDTRIP_OK")

        if rclcppyy is None:
            evidence["backend"].update({
                "publisher": "stock-rclpy",
                "subscription": "stock-rclpy",
                "action_client": "stock-rclpy",
                "action_server": "stock-rclpy",
                "verified": True,
            })
        else:
            status = rclcppyy.status()
            publisher_records = relevant_backend_records(status, request_topic)
            subscription_records = relevant_backend_records(status, reply_topic)
            aot_action_records = relevant_action_records(
                status, prefix + "/accumulate")
            python_action_records = relevant_action_records(
                status, python_action_name)
            assert any(record["backend"] == "python"
                       for record in publisher_records), status
            assert any(record["backend"] == "python"
                       for record in subscription_records), status
            assert any(
                record["backend"] == "python"
                and record["metadata"].get("entity_type") == "action_client"
                for record in aot_action_records
            ), status
            assert {
                "action_client", "action_server",
            } <= {
                record["metadata"].get("entity_type")
                for record in python_action_records
                if record["backend"] == "python"
                and "stock_action_authority" in record["policies"]
            }, status
            evidence["backend"].update({
                "publisher": "python",
                "subscription": "python",
                "action_client": "python",
                "action_server": "python",
                "verified": True,
                "publisher_records": publisher_records,
                "subscription_records": subscription_records,
                "aot_action_records": aot_action_records,
                "python_action_records": python_action_records,
            })
    finally:
        if action_client is not None:
            action_client.destroy()
        if node is not None and service_client is not None:
            node.destroy_client(service_client)
        if node is not None and publisher is not None:
            node.destroy_publisher(publisher)
        if node is not None and subscription is not None:
            node.destroy_subscription(subscription)
        if executor is not None and node is not None:
            executor.remove_node(node)
            executor.shutdown(timeout_sec=1.0)
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
        peer.close()

    if reference_path is not None:
        reference = json.loads(reference_path.read_text(encoding="utf-8"))
        if reference.get("mode") != "stock":
            raise AssertionError("custom-interface reference must use stock mode")
        for field in ("outcomes", "action_contract"):
            if evidence[field] != reference[field]:
                raise AssertionError(
                    "%s differs from stock reference" % field)
        evidence["differential"] = {
            "reference_mode": reference["mode"],
            "matches_stock": True,
        }

    evidence_path.parent.mkdir(parents=True, exist_ok=True)
    evidence_path.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n")
    print("CUSTOM_INTERFACE_INTEROP_RESULT=" + json.dumps(
        evidence, sort_keys=True))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("stock", "activated"), required=True)
    parser.add_argument("--peer", required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    parser.add_argument("--reference", type=Path)
    args = parser.parse_args()
    run(args.mode, args.peer, args.evidence, args.reference)


if __name__ == "__main__":
    main()
