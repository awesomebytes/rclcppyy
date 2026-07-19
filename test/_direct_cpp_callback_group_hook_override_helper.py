#!/usr/bin/env python3
"""A subclassed callback group with an overridden dispatch hook fails
closed at group-attachment time, rather than being silently accepted with
the override never invoked.

Native rclcpp dispatch enforces callback-group exclusion entirely in C++
and never calls back into DirectCallbackGroup's can_execute/
beginning_execution/ending_execution during real dispatch (they are
advisory only). Stock rclpy's own Executor._make_handler DOES call these
hooks, so a stock user's subclass that overrides one, expecting it to be
honored, would previously be accepted and silently ignored under
direct_cpp -- a semantic divergence, not an error. This proves the guard:
a harmless subclass (no hook override) is still accepted; a subclass that
overrides a hook is rejected before it ever reaches native dispatch.
"""

import os
import time

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclcppyy.policy import BackendUnavailableError  # noqa: E402
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import UInt64  # noqa: E402


class HarmlessSubclass(MutuallyExclusiveCallbackGroup):
    """Adds bookkeeping but never touches a dispatch hook."""

    def __init__(self):
        super().__init__()
        self.extra_state = []


class OverridesCanExecute(MutuallyExclusiveCallbackGroup):
    def __init__(self):
        super().__init__()
        self.calls = 0

    def can_execute(self, entity):
        self.calls += 1
        return super().can_execute(entity)


class OverridesBeginningExecution(MutuallyExclusiveCallbackGroup):
    def beginning_execution(self, entity):
        return super().beginning_execution(entity)


class OverridesEndingExecution(MutuallyExclusiveCallbackGroup):
    def ending_execution(self, entity):
        super().ending_execution(entity)


rclpy.init(args=[])
node = Node("direct_callback_group_hook_override_%d" % os.getpid())
executor = SingleThreadedExecutor()
executor.add_node(node)
prefix = "/direct_cpp/callback_group_hook_override/p%d" % os.getpid()


# --- harmless subclass: accepted, dispatches normally ---------------------
harmless = HarmlessSubclass()
topic = prefix + "/harmless"
pub = node.create_publisher(UInt64, topic, 10)
received = []
node.create_subscription(
    UInt64, topic, lambda m: received.append(m), 10, callback_group=harmless)

deadline = time.monotonic() + 15.0
while pub.get_subscription_count() < 1 and time.monotonic() < deadline:
    executor.spin_once(timeout_sec=0.05)
assert pub.get_subscription_count() == 1
pub.publish(UInt64(data=1))
deadline = time.monotonic() + 10.0
while not received and time.monotonic() < deadline:
    executor.spin_once(timeout_sec=0.05)
assert len(received) == 1
print("DIRECT_CPP_CALLBACK_GROUP_HOOK_OVERRIDE_HARMLESS_OK", flush=True)


# --- overridden hooks: rejected at attachment time, before any dispatch ---
for cls, hook_name in (
    (OverridesCanExecute, "can_execute"),
    (OverridesBeginningExecution, "beginning_execution"),
    (OverridesEndingExecution, "ending_execution"),
):
    group = cls()
    before = pub.get_subscription_count()
    try:
        node.create_subscription(
            UInt64, prefix + "/" + hook_name, lambda m: None, 10,
            callback_group=group)
    except BackendUnavailableError as error:
        assert hook_name in str(error), str(error)
    else:
        raise AssertionError(
            "a callback group overriding %s was silently accepted" % hook_name)
    # No entity or native group was created by the rejected attempt.
    assert group.node is None
    assert group.entities == set()

print("DIRECT_CPP_CALLBACK_GROUP_HOOK_OVERRIDE_REJECTED_OK", flush=True)


node.destroy_node()
executor.shutdown(timeout_sec=2.0)
rclpy.shutdown()
assert not rclpy.ok()
print("DIRECT_CPP_CALLBACK_GROUP_HOOK_OVERRIDE_TEARDOWN_OK", flush=True)
