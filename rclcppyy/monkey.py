"""
This file handles monkey-patching rclpy to use rclcpp implementations.
"""

import rclpy
from rclpy.node import Node
from rclcppyy.node import RclcppyyNode
from rclcppyy.monkeypatch_messages import install_ros_message_hook, convert_already_imported_python_msgs_to_cpp
from rclcppyy.bringup_rclcpp import bringup_rclcpp

# Store original functions
_original_create_node = rclpy.create_node
_original_node_create_subscription = Node.create_subscription
_original_spin_once = rclpy.spin_once

def patch_ros2():
    """
    Monkey-patch ROS2 Python API to use C++ implementations.
    This makes existing Python ROS2 code use C++ under the hood.
    """
    # Initialize rclcpp
    bringup_rclcpp()

    # Set up automatic message conversion
    install_ros_message_hook()
    
    # Convert already imported Python messages to C++
    convert_already_imported_python_msgs_to_cpp()
    
    # Monkey-patch rclpy.create_node
    rclpy.create_node = _create_node_wrapper

    # Monkey-patch rclpy.spin
    rclpy.spin = _spin_wrapper

    # Monkey-patch rclpy.spin_once. Tools that drive their own loop -- notably the
    # ros2 CLI (ros2cli's DirectNode discovery loop and `ros2 topic hz`) -- call
    # rclpy.spin_once(node) rather than rclpy.spin(node). The stock rclpy executor
    # cannot service an rclcpp-backed node's timers/subscriptions, so without this
    # the discovery loop spins forever. Delegate to an rclcpp executor instead.
    rclpy.spin_once = _spin_once_wrapper

    # Monkey-patch Node.create_subscription
    Node.create_subscription = _create_subscription_wrapper
    
    print("ROS2 C++ acceleration enabled!")
    return True

def _create_node_wrapper(*args, **kwargs):
    """
    Wrapper for rclpy.create_node that returns RclcppyyNode instead.
    This maintains the same API but uses our C++-backed node.
    """
    return RclcppyyNode(*args, **kwargs)

def _spin_wrapper(*args, **kwargs):
    """
    Wrapper for rclpy.spin that uses rclcpp.spin
    """
    rclcpp = bringup_rclcpp()
    node = args[0]
    rclcpp.spin(node._rclcpp_node)


def _get_spin_executor(node):
    """A persistent rclcpp SingleThreadedExecutor bound to this node.

    Created once and cached on the node so repeated spin_once calls reuse it (and
    so the node is never added to two executors). The executor picks up entities
    the node creates later -- e.g. the hz verb's subscription after DirectNode's
    discovery timer -- via the node's guard condition.
    """
    executor = getattr(node, "_rclcppyy_spin_executor", None)
    if executor is None:
        rclcpp = bringup_rclcpp()
        executor = rclcpp.executors.SingleThreadedExecutor()
        executor.add_node(node._rclcpp_node)
        node._rclcppyy_spin_executor = executor
    return executor


def _spin_once_wrapper(node, *, executor=None, timeout_sec=None):
    """
    Wrapper for rclpy.spin_once that drives an rclcpp executor for accelerated
    nodes (rclpy's executor cannot service rclcpp-backed entities). Non-accelerated
    nodes fall back to the original rclpy.spin_once unchanged.

    Matches rclpy semantics: timeout_sec=None blocks until one piece of work is
    ready; a finite timeout waits at most that long (rclcpp uses -1ns for "block").
    """
    if not isinstance(node, RclcppyyNode):
        return _original_spin_once(node, executor=executor, timeout_sec=timeout_sec)
    import cppyy
    _get_spin_executor(node)  # ensure created + node added
    ex = node._rclcppyy_spin_executor
    if timeout_sec is None:
        ex.spin_once()
    else:
        ex.spin_once(cppyy.gbl.std.chrono.nanoseconds(int(timeout_sec * 1e9)))

def _create_subscription_wrapper(self, *args, **kwargs):
    """
    Wrapper for Node.create_subscription that uses RclcppyyNode implementation
    when the node is an RclcppyyNode, otherwise falls back to original.
    """
    if isinstance(self, RclcppyyNode):
        return self.create_subscription(*args, **kwargs)
    else:
        return _original_node_create_subscription(self, *args, **kwargs)

def patch_node_class():
    """
    Monkey-patch the Node class so that any direct instantiations
    of rclpy.node.Node get our RclcppyyNode instead.
    """
    # Replace the Node class with RclcppyyNode
    # This is a more aggressive approach and might cause issues
    # so we make it optional
    rclpy.node.Node = RclcppyyNode
    return True 