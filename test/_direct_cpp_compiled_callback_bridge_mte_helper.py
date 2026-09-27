#!/usr/bin/env python3
"""Bounded two-worker overlap proof for compiled direct callbacks."""

import faulthandler
import os
import subprocess
import sys
import threading
import time


CALLBACK_TIMEOUT = 15.0


def client_main():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import UInt64
    from std_srvs.srv import SetBool

    rclpy.init(args=[])
    node = Node("compiled_callback_mte_peer_%d" % os.getpid())
    topic = "/direct_cpp/compiled_callback_mte/p%d/value" % os.getppid()
    service_name = "/direct_cpp/compiled_callback_mte/p%d/set_bool" % os.getppid()
    publisher = node.create_publisher(UInt64, topic, 10)
    client = node.create_client(SetBool, service_name)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    errors = []

    def spin_target():
        try:
            executor.spin()
        except BaseException as exc:  # captured to report a clean peer failure
            errors.append(exc)

    spin_thread = threading.Thread(target=spin_target, name="compiled-bridge-peer")
    spin_thread.start()
    try:
        deadline = time.monotonic() + CALLBACK_TIMEOUT
        while time.monotonic() < deadline:
            if publisher.get_subscription_count() and client.service_is_ready():
                break
            time.sleep(0.02)
        assert publisher.get_subscription_count(), "compiled bridge subscriber missing"
        assert client.service_is_ready(), "compiled bridge service missing"
        future = client.call_async(SetBool.Request(data=True))
        publisher.publish(UInt64(data=0xC11A))
        deadline = time.monotonic() + CALLBACK_TIMEOUT
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert future.done(), "SetBool response timed out"
        response = future.result()
        assert response.success is True
        assert response.message == "compiled-mte-ok"
    finally:
        assert executor.shutdown(timeout_sec=CALLBACK_TIMEOUT)
        spin_thread.join(timeout=CALLBACK_TIMEOUT)
        assert not spin_thread.is_alive(), "stock peer executor did not stop"
        assert not errors, "stock peer executor raised: %r" % errors
        node.destroy_node()
        rclpy.shutdown()
    return 0


def server_main():
    import rclcppyy

    rclcppyy.enable_cpp_acceleration(profile="direct_cpp")
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import UInt64
    from std_srvs.srv import SetBool

    rclpy.init(args=[])
    node = Node("compiled_callback_mte_server_%d" % os.getpid())
    topic = "/direct_cpp/compiled_callback_mte/p%d/value" % os.getpid()
    service_name = "/direct_cpp/compiled_callback_mte/p%d/set_bool" % os.getpid()
    group = ReentrantCallbackGroup()
    overlap = threading.Barrier(2)
    seen = {"subscription": threading.Event(), "service": threading.Event()}
    values = {}

    def on_message(message):
        overlap.wait(timeout=CALLBACK_TIMEOUT)
        assert type(message) is UInt64
        values["message"] = int(message.data)
        values["retained"] = message
        seen["subscription"].set()

    def on_service(request, response):
        overlap.wait(timeout=CALLBACK_TIMEOUT)
        assert type(request) is SetBool.Request
        assert type(response) is SetBool.Response
        values["request"] = bool(request.data)
        response.success = True
        response.message = "compiled-mte-ok"
        seen["service"].set()
        return response

    subscription = node.create_subscription(
        UInt64, topic, on_message, 10, callback_group=group)
    service = node.create_service(
        SetBool, service_name, on_service, callback_group=group)
    assert subscription._native.callback_handoff == "compiled_python_callback"
    assert service._native.callback_handoff == "compiled_python_callback"
    assert group.has_entity(subscription)
    assert group.has_entity(service)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_errors = []

    def spin_target():
        try:
            executor.spin()
        except BaseException as exc:  # captured so the test reports failure
            spin_errors.append(exc)

    spin_thread = threading.Thread(target=spin_target, name="compiled-bridge-mte")
    spin_thread.start()
    client_process = None
    try:
        client_process = subprocess.run(
            [sys.executable, os.path.abspath(__file__), "--client"],
            check=False,
            timeout=CALLBACK_TIMEOUT + 10.0,
            env=os.environ.copy(),
        )
        assert client_process.returncode == 0, (
            "stock peer exited %d" % client_process.returncode)
        assert seen["subscription"].wait(CALLBACK_TIMEOUT)
        assert seen["service"].wait(CALLBACK_TIMEOUT)
        assert values["message"] == 0xC11A
        assert type(values["retained"]) is UInt64
        assert values["retained"].data == 0xC11A
        assert values["request"] is True
        assert subscription._native.stats().callbacks == 1
        assert service._native.stats().requests == 1
        assert subscription._native.stats().bridge_errors == 0
        assert service._native.stats().exceptions == 0
    finally:
        if client_process is not None:
            assert client_process.returncode == 0
        assert executor.shutdown(timeout_sec=CALLBACK_TIMEOUT)
        spin_thread.join(timeout=CALLBACK_TIMEOUT)
        assert not spin_thread.is_alive(), "direct MTE executor did not stop"
        assert not spin_errors, "direct MTE executor raised: %r" % spin_errors
        assert node.destroy_service(service)
        assert node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    return 0


def main():
    faulthandler.dump_traceback_later(90.0, file=sys.stderr, exit=True)
    if sys.argv[1:] == ["--client"]:
        return client_main()
    return server_main()


if __name__ == "__main__":
    result = main()
    print("DIRECT_CPP_COMPILED_CALLBACK_BRIDGE_MTE_OK", flush=True)
    raise SystemExit(result)
