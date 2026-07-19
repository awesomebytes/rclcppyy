#!/usr/bin/env python3
"""LifecycleNode is not constructable under direct_cpp -- block it precisely.

LifecycleNode(LifecycleNodeMixin, Node) subclasses whatever Node currently is;
direct_cpp rebinds rclpy.node.Node to DirectNode, which owns a native rclcpp
node and has no ``handle`` attribute at all. LifecycleNodeMixin.__init__ needs
``self.handle`` to construct ``_rclpy.LifecycleStateMachine`` -- a stock
pybind node handle the direct backend does not and cannot own. Even after the
type-support crash over the four lifecycle service types is fixed, construction
hits that second, deeper wall as a raw AttributeError. This guard makes
LifecycleNode.__init__ raise a precise BackendUnavailableError instead, before
any node work.
"""

import os

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclcppyy.policy import BackendUnavailableError  # noqa: E402
from rclpy.lifecycle import LifecycleNode  # noqa: E402


rclpy.init(args=None)

try:
    LifecycleNode("direct_lifecycle_%d" % os.getpid())
except BackendUnavailableError as exc:
    assert "lifecycle nodes are not yet supported under the direct_cpp profile" in str(exc)
except AttributeError as exc:
    raise AssertionError(
        "LifecycleNode construction raised the raw AttributeError instead "
        "of a precise BackendUnavailableError: %s" % exc
    ) from exc
else:
    raise AssertionError("LifecycleNode constructed under direct_cpp")
print("DIRECT_CPP_LIFECYCLE_BLOCK_PRECISE_ERROR_OK", flush=True)

# The guard must fire before any node work -- no runtime node, no context
# node registration.
direct_module_name = "rclcppyy.direct_cpp"
import importlib  # noqa: E402
direct_module = importlib.import_module(direct_module_name)
assert direct_module._runtime().nodes == []
print("DIRECT_CPP_LIFECYCLE_BLOCK_NO_PARTIAL_NODE_OK", flush=True)

rclpy.shutdown()
assert not rclpy.ok()
print("DIRECT_CPP_LIFECYCLE_BLOCK_TEARDOWN_OK", flush=True)
