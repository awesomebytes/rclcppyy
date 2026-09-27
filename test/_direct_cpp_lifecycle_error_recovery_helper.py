#!/usr/bin/env python3
"""A raising transition callback maps to ERROR and recovers (PLAN-lifecycle.md
P1): stock's own ``__execute_callback`` (node.py:315-320) swallows a raising
transition callback rather than re-raising it; the native
``rclcpp_lifecycle`` error-processing path then runs ``on_error`` and
transitions to ``unconfigured``. Verifies the direct backend matches --
contained, no ``std::terminate``, the in-flight counter still bumped."""
import os

import rclcppyy


rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp", interfaces=["lifecycle_msgs/srv/ChangeState"])

import rclpy  # noqa: E402
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn  # noqa: E402


class RaisingConfigureNode(LifecycleNode):
    def __init__(self, name):
        super().__init__(name)
        self.configure_calls = 0
        self.error_calls = []

    def on_configure(self, state):
        self.configure_calls += 1
        raise RuntimeError("boom from on_configure")

    def on_error(self, state):
        self.error_calls.append(state)
        return TransitionCallbackReturn.SUCCESS


def main():
    rclpy.init(args=None)
    node = RaisingConfigureNode("direct_lifecycle_error_recovery_%d" % os.getpid())

    assert node._current_state.label == "unconfigured"

    # trigger_configure() itself must not raise: the exception is swallowed
    # inside the native dispatch, mapped to ERROR, and error-processing
    # recovers the state machine -- never propagated to the caller.
    result = node.trigger_configure()
    print("DIRECT_CPP_LIFECYCLE_RAISE_CONTAINED_OK", result, flush=True)

    assert node.configure_calls == 1
    assert len(node.error_calls) == 1
    # on_error receives the SAME previous state on_configure did (the state
    # before the whole transition sequence began) -- matches stock exactly:
    # neither rclcpp_lifecycle's change_state() nor stock's __change_state()
    # ever recomputes "initial_state" between the primary and error-recovery
    # callback invocations.
    assert node.error_calls[0].label == "unconfigured"
    assert node._current_state.label == "unconfigured", node._current_state
    print("DIRECT_CPP_LIFECYCLE_ERROR_RECOVERY_OK", flush=True)

    # The state machine must still be fully usable afterward -- a contained
    # raise must not leave the native node in a wedged state.
    node.configure_calls = 0
    node.on_configure = lambda state: TransitionCallbackReturn.SUCCESS
    result = node.trigger_configure()
    assert result == TransitionCallbackReturn.SUCCESS
    assert node._current_state.label == "inactive"
    print("DIRECT_CPP_LIFECYCLE_POST_ERROR_USABLE_OK", flush=True)

    node.destroy_node()
    rclpy.shutdown()
    assert not rclpy.ok()
    print("DIRECT_CPP_LIFECYCLE_ERROR_RECOVERY_ALL_OK", flush=True)


if __name__ == "__main__":
    main()
