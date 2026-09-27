#!/usr/bin/env python3
"""Live proof: the Node patch-list gap on action.graph/wait_for_message is closed.

rclpy.action.graph and rclpy.wait_for_message each import Node into their own
module namespace at their own module top, before activate()'s Node swap used
to reach them -- leaving their local Node bindings stuck on stock. This closes
that gap; after activation, every alias must observe the same DirectNode.
"""

import rclcppyy


rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy.action.graph as graph_module  # noqa: E402
import rclpy.node as node_module  # noqa: E402
import rclpy.wait_for_message as wait_for_message_module  # noqa: E402
from rclcppyy.direct_cpp import DirectNode  # noqa: E402


assert node_module.Node is DirectNode
assert graph_module.Node is DirectNode
assert wait_for_message_module.Node is DirectNode
assert graph_module.Node is node_module.Node is wait_for_message_module.Node

print("DIRECT_CPP_NODE_PATCH_LIST_OK")
