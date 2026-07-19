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
