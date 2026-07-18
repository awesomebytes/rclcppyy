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


def run(mode, peer_executable, evidence_path):
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

        peer.wait_for("AOT_MESSAGE_ROUNDTRIP_OK")
        peer.wait_for("AOT_SERVICE_ROUNDTRIP_OK")
        peer.wait_for("AOT_ACTION_ROUNDTRIP_OK")

        if rclcppyy is None:
            evidence["backend"].update({
                "publisher": "stock-rclpy",
                "subscription": "stock-rclpy",
                "verified": True,
            })
        else:
            status = rclcppyy.status()
            publisher_records = relevant_backend_records(status, request_topic)
            subscription_records = relevant_backend_records(status, reply_topic)
            assert any(record["backend"] == "cpp"
                       for record in publisher_records), status
            assert any(record["backend"] == "python"
                       for record in subscription_records), status
            evidence["backend"].update({
                "publisher": "cpp",
                "subscription": "python",
                "verified": True,
                "publisher_records": publisher_records,
                "subscription_records": subscription_records,
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

    evidence_path.parent.mkdir(parents=True, exist_ok=True)
    evidence_path.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n")
    print("CUSTOM_INTERFACE_INTEROP_RESULT=" + json.dumps(
        evidence, sort_keys=True))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("stock", "activated"), required=True)
    parser.add_argument("--peer", required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    args = parser.parse_args()
    run(args.mode, args.peer, args.evidence)


if __name__ == "__main__":
    main()
