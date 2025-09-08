"""
This file handles monkey-patching rclpy to use rclcpp implementations.
"""

import rclpy
import sys
from rclpy.node import Node
from rclcppyy.node import RclcppyyNode
from rclcppyy.monkeypatch_messages import install_ros_message_hook, convert_already_imported_python_msgs_to_cpp
from rclcppyy.bringup_rclcpp import bringup_rclcpp

# Store original functions
_original_create_node = rclpy.create_node
_original_node_create_subscription = Node.create_subscription

def patch_ros2():
    """
    Monkey-patch ROS2 Python API to use C++ implementations.
    This makes existing Python ROS2 code use C++ under the hood.
    """
    # Initialize rclcpp
    rclcpp = bringup_rclcpp()
    
    # Set up automatic message conversion
    install_ros_message_hook()
    
    # Convert already imported Python messages to C++
    convert_already_imported_python_msgs_to_cpp()
    
    # Monkey-patch rclpy.create_node
    rclpy.create_node = _create_node_wrapper

    # Monkey-patch rclpy.spin
    rclpy.spin = _spin_wrapper
    
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