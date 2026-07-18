"""Launch the zero-edit startup-hook contract probe and stop on completion."""

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    probe = Node(
        package="rclcppyy",
        executable="hook_launch_probe",
        output="screen",
    )
    stop = RegisterEventHandler(OnProcessExit(
        target_action=probe,
        on_exit=[EmitEvent(event=Shutdown(reason="hook probe completed"))],
    ))
    return LaunchDescription([probe, stop])
