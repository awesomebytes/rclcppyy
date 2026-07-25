#!/usr/bin/env python3
"""Correctness proof for DirectLifecycleNode (PLAN-lifecycle.md P1).

Construction, the native state machine driven both via a stock client's
/change_state service call and via trigger_*, transition-callback previous-
state fidelity, the invalid-transition differential against stock, and
enable_communication_interface=False.
"""
import os
import threading

import rclcppyy


rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp", interfaces=["lifecycle_msgs/srv/ChangeState"])

import rclpy  # noqa: E402
from lifecycle_msgs.msg import Transition  # noqa: E402
from lifecycle_msgs.srv import ChangeState  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.impl.implementation_singleton import (  # noqa: E402
    rclpy_implementation as _rclpy)
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn  # noqa: E402
from rclcppyy.direct_lifecycle import DirectLifecycleNode  # noqa: E402


class RecordingLifecycleNode(LifecycleNode):
    def __init__(self, name):
        super().__init__(name)
        self.events = []

    def on_configure(self, state):
        self.events.append(("configure", state))
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.events.append(("activate", state))
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.events.append(("deactivate", state))
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.events.append(("cleanup", state))
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.events.append(("shutdown", state))
        return TransitionCallbackReturn.SUCCESS


def main():
    rclpy.init(args=None)
    pid = os.getpid()

    assert LifecycleNode is DirectLifecycleNode
    print("DIRECT_CPP_LIFECYCLE_REBOUND_OK", flush=True)

    node = RecordingLifecycleNode("direct_lifecycle_%d" % pid)
    assert node._current_state.label == "unconfigured"
    assert node._current_state.state_id == 1
    assert node._initialized
    print("DIRECT_CPP_LIFECYCLE_CONSTRUCT_OK", flush=True)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop_spin = threading.Event()

    def _spin_node():
        while not stop_spin.is_set():
            executor.spin_once(timeout_sec=0.05)

    spin_thread = threading.Thread(target=_spin_node, name="lifecycle-node-spin")
    spin_thread.start()

    # --- drive configure -> activate via trigger_* ---
    result = node.trigger_configure()
    assert result == TransitionCallbackReturn.SUCCESS
    assert node._current_state.label == "inactive"
    assert node.events[-1] == (
        "configure", node.events[-1][1])
    assert node.events[-1][1].label == "unconfigured"
    assert node.events[-1][1].state_id == 1

    result = node.trigger_activate()
    assert result == TransitionCallbackReturn.SUCCESS
    assert node._current_state.label == "active"
    assert node.events[-1][0] == "activate"
    assert node.events[-1][1].label == "inactive"
    print("DIRECT_CPP_LIFECYCLE_TRIGGER_STAR_OK", flush=True)

    # --- invalid transition: differential vs stock (type + message) ---
    try:
        node.trigger_configure()
    except _rclpy.RCLError as exc:
        message = str(exc)
        assert "Transition is not registered" in message, message
    else:
        raise AssertionError("expected trigger_configure to raise from active")
    print("DIRECT_CPP_LIFECYCLE_INVALID_TRANSITION_DIFFERENTIAL_OK", flush=True)

    # --- drive deactivate -> cleanup -> shutdown via a stock rclpy client ---
    client_node = rclpy.create_node("direct_lifecycle_client_%d" % pid)
    client_executor = SingleThreadedExecutor()
    client_executor.add_node(client_node)
    client = client_node.create_client(
        ChangeState, "/%s/change_state" % node.get_name())
    assert client.wait_for_service(timeout_sec=15.0), "service not available"

    def change_state(transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        client_executor.spin_until_future_complete(future, timeout_sec=15.0)
        response = future.result()
        assert response is not None, "no response from /change_state"
        return response.success

    assert change_state(Transition.TRANSITION_DEACTIVATE)
    assert node._current_state.label == "inactive"
    assert node.events[-1][0] == "deactivate"
    assert node.events[-1][1].label == "active"

    assert change_state(Transition.TRANSITION_CLEANUP)
    assert node._current_state.label == "unconfigured"
    assert node.events[-1][0] == "cleanup"
    assert node.events[-1][1].label == "inactive"

    assert change_state(Transition.TRANSITION_UNCONFIGURED_SHUTDOWN)
    assert node._current_state.label == "finalized"
    assert node.events[-1][0] == "shutdown"
    assert node.events[-1][1].label == "unconfigured"
    print("DIRECT_CPP_LIFECYCLE_STOCK_CLIENT_OK", flush=True)

    # --- shutdown-from-finalized: exact stock message ---
    try:
        node.trigger_shutdown()
    except _rclpy.RCLError as exc:
        assert str(exc) == "Shutdown transition not possible", str(exc)
    else:
        raise AssertionError("expected trigger_shutdown to raise when finalized")
    print("DIRECT_CPP_LIFECYCLE_SHUTDOWN_DIFFERENTIAL_OK", flush=True)

    client_executor.remove_node(client_node)
    client_node.destroy_node()
    stop_spin.set()
    spin_thread.join(timeout=15.0)
    assert not spin_thread.is_alive(), "node spin thread hung after teardown"
    executor.remove_node(node)
    node.destroy_node()
    print("DIRECT_CPP_LIFECYCLE_TEARDOWN_OK", flush=True)

    # --- enable_communication_interface=False: no services, trigger_* works ---
    bare_node = LifecycleNode(
        "direct_lifecycle_bare_%d" % pid,
        enable_communication_interface=False)
    assert bare_node._current_state.label == "unconfigured"
    result = bare_node.trigger_configure()
    assert result == TransitionCallbackReturn.SUCCESS
    assert bare_node._current_state.label == "inactive"

    bare_client_node = rclpy.create_node("direct_lifecycle_bare_client_%d" % pid)
    bare_client_executor = SingleThreadedExecutor()
    bare_client_executor.add_node(bare_client_node)
    bare_client = bare_client_node.create_client(
        ChangeState, "/%s/change_state" % bare_node.get_name())
    assert not bare_client.wait_for_service(timeout_sec=2.0), (
        "change_state service must not exist when "
        "enable_communication_interface=False")
    bare_client_executor.remove_node(bare_client_node)
    bare_client_node.destroy_node()
    bare_node.destroy_node()
    print("DIRECT_CPP_LIFECYCLE_NO_COMMUNICATION_INTERFACE_OK", flush=True)

    rclpy.shutdown()
    assert not rclpy.ok()
    print("DIRECT_CPP_LIFECYCLE_ALL_OK", flush=True)


if __name__ == "__main__":
    main()
