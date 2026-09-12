"""Example: accelerate every node in a launch file with rclcppyy, unedited.

Prepend the actions from ``rclcppyy_env_actions()`` to the ``LaunchDescription``;
every process this launch file spawns afterwards picks up the C++ backend via
``RCLCPPYY_ENABLE_HOOK`` the instant it imports ``rclpy`` -- no changes to the
nodes themselves. See ``rclcppyy.launch`` and ``rclcppyy.hook`` for the mechanism.

    ros2 launch rclcppyy example_accelerated.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node

from rclcppyy.launch import rclcppyy_env_actions


def generate_launch_description():
    talker = Node(
        package="demo_nodes_py",
        executable="talker",
        output="screen",
    )
    return LaunchDescription([
        *rclcppyy_env_actions(),
        talker,
    ])
