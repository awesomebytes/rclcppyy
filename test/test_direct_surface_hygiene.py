"""Direct public-surface hygiene: the leak-fix stays hidden from dir()."""

import os
import subprocess
import sys


_CODE = """
import os

import rclcppyy

rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy
import rclpy.lifecycle.node as lifecycle_node_module
import rclpy.lifecycle.publisher as lifecycle_publisher_module
from rclpy.node import Node
from std_msgs.msg import String

from rclcppyy.direct_cpp import DirectNode, DirectPublisher

HIDDEN_NODE_NAMES = frozenset({
    "direct_cpp_parameter_cache_stats",
    "callback_groups",
    "action_clients",
    "action_servers",
})
HIDDEN_PUBLISHER_NAMES = frozenset({"closed", "native_entity"})

for owner, cls in (
    ("DirectNode", DirectNode),
    ("LifecycleNode", lifecycle_node_module.LifecycleNode),
):
    leaked = HIDDEN_NODE_NAMES & set(dir(cls))
    assert not leaked, "%s still exposes %s" % (owner, sorted(leaked))

for owner, cls in (
    ("DirectPublisher", DirectPublisher),
    ("LifecyclePublisher", lifecycle_publisher_module.LifecyclePublisher),
):
    leaked = HIDDEN_PUBLISHER_NAMES & set(dir(cls))
    assert not leaked, "%s still exposes %s" % (owner, sorted(leaked))

assert Node is DirectNode

rclpy.init(args=[])
node = Node("direct_surface_hygiene_%d" % os.getpid())
publisher = node.create_publisher(String, "/direct_cpp/surface_hygiene", 10)

# Instance access to the hidden-from-dir() members still works: the metaclass
# only curates dir(), it never removes the attributes themselves.
assert publisher.closed is False
assert publisher.native_entity is not None

# The renamed accessors are reachable under their new, internalized names.
assert node._callback_groups == (node.default_callback_group,)
assert node._action_clients == []
assert node._action_servers == []
stats = node._direct_cpp_parameter_cache_stats()
assert isinstance(stats, dict)

publisher.destroy()
assert publisher.closed is True

node.destroy_node()
rclpy.shutdown()
print("DIRECT_SURFACE_HYGIENE_OK")
"""


def test_direct_surface_hygiene_hides_leaked_members_but_keeps_instance_access():
    result = subprocess.run(
        [sys.executable, "-c", _CODE],
        cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )
    diagnostics = (
        f"\n--- exit code: {result.returncode} ---"
        f"\n--- stdout ---\n{result.stdout}"
        f"\n--- stderr ---\n{result.stderr}"
    )
    assert result.returncode == 0, diagnostics
    assert "DIRECT_SURFACE_HYGIENE_OK" in result.stdout, diagnostics


# Extends the leak-fix proof to the six action/executor/callback-group/
# subscription classes hidden via the shared rclcppyy._surface._DirectSurface
# metaclass (surface-hygiene slice, pattern (a)). Runs on ROS_DOMAIN_ID 72,
# exclusive to this lane, per the standing rule for any live run here.
_EXTENDED_CODE = """
import os

import rclcppyy

rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy
import rclpy.action.client
import rclpy.action.server
import rclpy.callback_groups
import rclpy.executors
import rclpy.lifecycle.node
import rclpy.parameter_client
import rclpy.parameter_event_handler
import rclpy.subscription
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.action import LookupTransform

from rclcppyy.direct_actions import (
    DirectActionClient,
    DirectActionServer,
    DirectServerGoalHandle,
)
from rclcppyy.direct_callback_groups import (
    DirectCallbackGroup,
    DirectMutuallyExclusiveCallbackGroup,
    DirectReentrantCallbackGroup,
)
from rclcppyy.direct_cpp import DirectSubscription
from rclcppyy.direct_executors import (
    DirectExecutor,
    DirectMultiThreadedExecutor,
    DirectSingleThreadedExecutor,
)

# Every class this slice hides members on, plus every bound rclpy alias path
# and every subclass -- dir() must be clean on all of them (7cc250c's own
# rule: the metaclass and its frozenset are inherited through the MRO).
CLASSES_AND_PATHS = (
    ("DirectActionClient", DirectActionClient),
    ("rclpy.action.ActionClient", ActionClient),
    ("rclpy.action.client.ActionClient", rclpy.action.client.ActionClient),
    ("DirectActionServer", DirectActionServer),
    ("rclpy.action.ActionServer", ActionServer),
    ("rclpy.action.server.ActionServer", rclpy.action.server.ActionServer),
    ("DirectServerGoalHandle", DirectServerGoalHandle),
    ("rclpy.action.server.ServerGoalHandle", rclpy.action.server.ServerGoalHandle),
    ("DirectCallbackGroup", DirectCallbackGroup),
    ("DirectReentrantCallbackGroup", DirectReentrantCallbackGroup),
    ("DirectMutuallyExclusiveCallbackGroup", DirectMutuallyExclusiveCallbackGroup),
    ("rclpy.callback_groups.CallbackGroup", rclpy.callback_groups.CallbackGroup),
    (
        "rclpy.callback_groups.MutuallyExclusiveCallbackGroup",
        rclpy.callback_groups.MutuallyExclusiveCallbackGroup,
    ),
    (
        "rclpy.callback_groups.ReentrantCallbackGroup",
        rclpy.callback_groups.ReentrantCallbackGroup,
    ),
    ("rclpy.lifecycle.node.CallbackGroup", rclpy.lifecycle.node.CallbackGroup),
    ("rclpy.parameter_client.CallbackGroup", rclpy.parameter_client.CallbackGroup),
    (
        "rclpy.parameter_event_handler.CallbackGroup",
        rclpy.parameter_event_handler.CallbackGroup,
    ),
    ("DirectExecutor", DirectExecutor),
    ("DirectSingleThreadedExecutor", DirectSingleThreadedExecutor),
    ("DirectMultiThreadedExecutor", DirectMultiThreadedExecutor),
    ("rclpy.executors.Executor", rclpy.executors.Executor),
    ("rclpy.executors.MultiThreadedExecutor", rclpy.executors.MultiThreadedExecutor),
    ("rclpy.executors.SingleThreadedExecutor", rclpy.executors.SingleThreadedExecutor),
    ("DirectSubscription", DirectSubscription),
    ("rclpy.subscription.Subscription", rclpy.subscription.Subscription),
    ("rclpy.parameter_client.Subscription", rclpy.parameter_client.Subscription),
)

for label, cls in CLASSES_AND_PATHS:
    hidden = frozenset(getattr(cls, "_PARITY_HIDDEN", ()))
    assert hidden, "%s: expected a non-empty _PARITY_HIDDEN" % label
    leaked = hidden & set(dir(cls))
    assert not leaked, "%s still exposes %s" % (label, sorted(leaked))

assert ActionClient is DirectActionClient
assert ActionServer is DirectActionServer
assert rclpy.action.server.ServerGoalHandle is DirectServerGoalHandle
assert rclpy.subscription.Subscription is DirectSubscription

rclpy.init(args=[])
node = Node("direct_surface_hygiene_extended_%d" % os.getpid())

# Callback group: bind by attaching it to an entity, then check the hidden
# native_group escape hatch still works by exact instance access.
group = ReentrantCallbackGroup()
subscription = node.create_subscription(
    String, "/direct_cpp/surface_hygiene_extended", lambda _msg: None, 10,
    callback_group=group,
)
assert subscription.closed is False
assert group.native_group is not None
subscription.destroy()
assert subscription.closed is True

# Executor: standalone construction is enough to exercise the hidden escape.
executor = SingleThreadedExecutor()
assert executor.native_executor is not None
executor.shutdown()

# Action client/server + a real accepted goal handle: exercises action_name
# and the shared-pointer terminal methods from within an active execute
# callback (execute_callback_depth > 0), and proves hiding __repr__ does not
# break repr() -- only dir() introspection omits it (risk 5 of the plan).
action_name = "/direct_cpp/surface_hygiene_extended/action/p%d" % os.getpid()
captured_handles = []


def execute_callback(goal_handle):
    captured_handles.append(goal_handle)
    shared_result = goal_handle.create_result_shared()
    shared_result.transform.child_frame_id = "surface-hygiene-result"
    goal_handle.succeed_shared(shared_result)
    return LookupTransform.Result()


server = ActionServer(node, LookupTransform, action_name, execute_callback)
client = ActionClient(node, LookupTransform, action_name)
assert client.action_name == action_name
assert client.wait_for_server(timeout_sec=15.0)

goal = LookupTransform.Goal(target_frame="hygiene", source_frame="base")
goal_future = client.send_goal_async(goal)


def spin_until(predicate, timeout=15.0):
    import time
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
    assert predicate()


spin_until(goal_future.done)
handle = goal_future.result()
assert handle.accepted

result_future = handle.get_result_async()
spin_until(result_future.done)
response = result_future.result()
assert str(response.result.transform.child_frame_id) == "surface-hygiene-result"

server_handle = captured_handles[0]
assert isinstance(server_handle, DirectServerGoalHandle)
assert repr(server_handle).startswith("ServerGoalHandle <id=")

client.destroy()
server.destroy()
node.destroy_node()
rclpy.shutdown()
print("DIRECT_SURFACE_HYGIENE_EXTENDED_OK")
"""


def test_direct_surface_hygiene_extended_classes_hide_but_keep_instance_access():
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "72"
    result = subprocess.run(
        [sys.executable, "-c", _EXTENDED_CODE],
        cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
        env=env,
    )
    diagnostics = (
        f"\n--- exit code: {result.returncode} ---"
        f"\n--- stdout ---\n{result.stdout}"
        f"\n--- stderr ---\n{result.stderr}"
    )
    assert result.returncode == 0, diagnostics
    assert "DIRECT_SURFACE_HYGIENE_EXTENDED_OK" in result.stdout, diagnostics
