"""
rclcppyy - High-performance ROS2 with Python API and C++ internals

This package allows ROS2 Python code to use C++ implementations
under the hood for improved performance without requiring changes
to existing code.
"""

# Public API
from rclcppyy.bringup_rclcpp import bringup_rclcpp
from rclcppyy import rosbag2_cpp
from rclcppyy import serialization
from rclcppyy.node import RclcppyyNode
from rclcppyy.monkey import patch_ros2, patch_node_class

def enable_cpp_acceleration(patch_node=True):
    """
    Enable C++ acceleration for ROS2 Python code.
    
    This will:
    1. Set up automatic message conversion from Python to C++
    2. Monkey-patch rclpy.create_node to return RclcppyyNode 
    3. Monkey-patch ROS2 message imports to use C++ versions
    
    Args:
        patch_node (bool): If True, also monkey-patch rclpy.node.Node class directly.
                          This is more aggressive but ensures all nodes use C++.
    
    Returns:
        bool: True if successful
    
    Example:
        ```python
        import rclpy
        from std_msgs.msg import String
        
        # Add this single line to your existing ROS2 Python code:
        import rclcppyy; rclcppyy.enable_cpp_acceleration()
        
        # Rest of your code remains unchanged
        rclpy.init()
        node = rclpy.create_node('my_node')  # Actually returns a RclcppyyNode
        ```
    """
    # Apply monkey patching
    result = patch_ros2()
    
    # Optionally patch the Node class directly
    if patch_node:
        patch_node_class()
        
    return result

__all__ = [
    'bringup_rclcpp',
    'RclcppyyNode',
    'enable_cpp_acceleration',
    'patch_ros2',
    'patch_node_class',
    'rosbag2_cpp',
    'serialization'
]