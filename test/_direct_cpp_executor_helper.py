#!/usr/bin/env python3
"""Live public-executor ownership proof for the direct-C++ profile."""

import importlib
import os
import threading
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from rclcppyy.policy import BackendUnavailableError  # noqa: E402
from rclpy.executors import (  # noqa: E402
    Executor,
    MultiThreadedExecutor,
    SingleThreadedExecutor,
)
from rclpy.node import Node  # noqa: E402
from rclpy.task import Future  # noqa: E402
from std_msgs.msg import String  # noqa: E402


EXPECTED_REJECTION = (BackendUnavailableError, TypeError, ValueError, RuntimeError)


def cpp_name(value):
    return str(
        getattr(type(value), "__cpp_name__", "")
        or getattr(value, "__cpp_name__", "")
    )


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("a Python-message conversion or serialization path ran")


bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
serialization = importlib.import_module("rclcpp_kit.serialization")
bringup.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary


assert String is cppyy.gbl.std_msgs.msg.String
assert issubclass(SingleThreadedExecutor, Executor)
assert issubclass(MultiThreadedExecutor, Executor)

rclpy.init(args=[])
runtime = importlib.import_module("rclcppyy.direct_cpp")._runtime()
node = Node("direct_executor_%d" % os.getpid())
assert node.executor is None

executor = SingleThreadedExecutor(context=node.context)
assert isinstance(executor, Executor)
assert executor.context is node.context
assert executor.get_nodes() == []
assert executor.add_node(node) is True
assert executor.add_node(node) is False
assert executor.get_nodes() == [node]
assert node.executor is executor

native_executors = runtime.session.executors
assert len(native_executors) == 1
assert "rclcpp::executors::SingleThreadedExecutor" in cpp_name(
    native_executors[0])
print("DIRECT_CPP_PUBLIC_EXECUTOR_OWNERSHIP_OK", flush=True)


global_node = Node("direct_executor_global_%d" % os.getpid())
global_executor = rclpy.get_global_executor()
global_firings = []
global_timer = None


def global_timer_callback():
    global_firings.append(time.monotonic_ns())
    if len(global_firings) == 3:
        global_timer.cancel()


global_timer = global_node.create_timer(0.01, global_timer_callback)
global_spin_calls = 0
global_deadline = time.monotonic() + 2.0
while len(global_firings) < 3 and time.monotonic() < global_deadline:
    rclpy.spin_once(global_node, timeout_sec=0.1)
    global_spin_calls += 1
    assert global_executor.get_nodes() == []
assert len(global_firings) == 3
assert global_spin_calls <= 8, global_spin_calls

global_replacement = SingleThreadedExecutor(context=global_node.context)
assert global_replacement.add_node(global_node) is True
assert global_executor.get_nodes() == []
assert global_replacement.get_nodes() == [global_node]
global_replacement.remove_node(global_node)
assert global_node.destroy_timer(global_timer)
global_node.destroy_node()
assert global_replacement.shutdown(timeout_sec=1.0) is True
print("DIRECT_CPP_GLOBAL_SPIN_ONCE_PARKING_OK", flush=True)


topic = "/direct_cpp/executor/p%d" % os.getpid()
received = []
completed = Future()


def receive(message):
    assert type(message) is String
    received.append(message)
    if not completed.done():
        completed.set_result(message)


publisher = node.create_publisher(String, topic, 10)
subscription = node.create_subscription(String, topic, receive, 10)
assert "rclcpp::Publisher" in cpp_name(publisher.native_entity)
assert "rclcpp::Subscription" in cpp_name(subscription.native_entity)

deadline = time.monotonic() + 10.0
while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
    rclpy.spin_once(node, executor=executor, timeout_sec=0.05)
assert publisher.get_subscription_count() == 1

publisher.publish(String(data="public-executor-cpp"))
rclpy.spin_until_future_complete(
    node, completed, executor=executor, timeout_sec=10.0)
assert completed.done() and not completed.cancelled()
assert completed.exception() is None
message = completed.result()
assert type(message) is String
assert str(message.data) == "public-executor-cpp"
assert received == [message]
assert executor.get_nodes() == [node]
assert node.executor is executor
print("DIRECT_CPP_PUBLIC_EXECUTOR_CPP_DATA_OK", flush=True)


replacement = SingleThreadedExecutor(context=node.context)
assert replacement.add_node(node) is True
assert executor.get_nodes() == []
assert replacement.get_nodes() == [node]
assert node.executor is replacement
replacement.remove_node(node)
replacement.remove_node(node)
assert replacement.get_nodes() == []
assert executor.add_node(node) is True
assert executor.get_nodes() == [node]
assert node.executor is executor
assert replacement.shutdown(timeout_sec=1.0) is True
print("DIRECT_CPP_PUBLIC_EXECUTOR_TRANSFER_OK", flush=True)


before_nodes = executor.get_nodes()
before_native_executors = runtime.session.executors
try:
    executor.add_node(object())
except EXPECTED_REJECTION:
    pass
else:
    raise AssertionError("direct executor accepted a foreign node")
assert executor.get_nodes() == before_nodes

try:
    SingleThreadedExecutor(context=object())
except EXPECTED_REJECTION:
    pass
else:
    raise AssertionError("direct executor accepted a foreign context")
assert runtime.session.executors == before_native_executors

boundary_node = Node("direct_executor_boundary_%d" % os.getpid())
assert boundary_node.executor is None
multi = None
try:
    multi = MultiThreadedExecutor(num_threads=2, context=boundary_node.context)
except EXPECTED_REJECTION:
    pass
else:
    try:
        multi.add_node(boundary_node)
    except EXPECTED_REJECTION:
        pass
    else:
        raise AssertionError("direct MultiThreadedExecutor accepted native membership")
    assert multi.get_nodes() == []
    assert multi.shutdown(timeout_sec=1.0) is True
assert boundary_node.executor is None
boundary_node.destroy_node()
print("DIRECT_CPP_PUBLIC_EXECUTOR_FAIL_CLOSED_OK", flush=True)


context_node = Node("direct_executor_context_%d" % os.getpid())
assert context_node.executor is None
with SingleThreadedExecutor(context=context_node.context) as managed:
    assert managed.add_node(context_node) is True
    assert managed.get_nodes() == [context_node]
    assert context_node.executor is managed
assert managed.get_nodes() == []
assert managed.is_spinning is False
context_node.destroy_node()
print("DIRECT_CPP_PUBLIC_EXECUTOR_CONTEXT_OK", flush=True)


blocked_node = Node("direct_executor_blocked_%d" % os.getpid())
blocked = SingleThreadedExecutor(context=blocked_node.context)
assert blocked.add_node(blocked_node) is True
entered_spin = threading.Event()
spin_done = threading.Event()
spin_errors = []


def spin_blocked():
    entered_spin.set()
    try:
        blocked.spin()
    except BaseException as exc:
        spin_errors.append(exc)
    finally:
        spin_done.set()


thread = threading.Thread(target=spin_blocked, name="direct-public-executor-spin")
thread.start()
assert entered_spin.wait(timeout=1.0)
deadline = time.monotonic() + 2.0
while not blocked.is_spinning and time.monotonic() < deadline:
    time.sleep(0.005)
assert blocked.is_spinning
assert blocked.shutdown(timeout_sec=2.0) is True
thread.join(timeout=2.0)
assert spin_done.is_set()
assert not thread.is_alive()
assert spin_errors == [], repr(spin_errors)
assert blocked.is_spinning is False
assert blocked.get_nodes() == []
blocked_node.destroy_node()
print("DIRECT_CPP_PUBLIC_EXECUTOR_SHUTDOWN_WAKE_OK", flush=True)


destroy_node = Node("direct_executor_destroy_%d" % os.getpid())
destroy_executor = SingleThreadedExecutor(context=destroy_node.context)
assert destroy_executor.add_node(destroy_node) is True
assert destroy_executor.get_nodes() == [destroy_node]
destroy_node.destroy_node()
assert destroy_executor.get_nodes() == []
assert destroy_node.executor is None
assert destroy_executor.shutdown(timeout_sec=1.0) is True
print("DIRECT_CPP_PUBLIC_EXECUTOR_NODE_DESTROY_OK", flush=True)


assert node.destroy_subscription(subscription)
assert node.destroy_publisher(publisher)
node.destroy_node()
assert executor.get_nodes() == []
assert node.executor is None
assert executor.shutdown(timeout_sec=1.0) is True
rclpy.shutdown()
assert not rclpy.ok()
assert runtime.session is None
print("DIRECT_CPP_PUBLIC_EXECUTOR_TEARDOWN_OK", flush=True)
