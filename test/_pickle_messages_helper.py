#!/usr/bin/env python3
"""Pickle round-trip proof for every C++-backed message under ``direct_cpp``."""

import pickle

import rclcppyy

rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp",
    interfaces=(
        "geometry_msgs/msg/Twist",
        "geometry_msgs/msg/Quaternion",
        "geometry_msgs/msg/PoseStamped",
        "sensor_msgs/msg/LaserScan",
        "visualization_msgs/msg/Marker",
    ),
)

from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402


def _assert_cpp_backed(message):
    assert type(message).__module__.startswith("cppyy."), (
        "expected a C++-backed message, got %r" % (type(message),))


def _roundtrip(message):
    _assert_cpp_backed(message)
    restored = pickle.loads(pickle.dumps(message))
    _assert_cpp_backed(restored)
    return restored


def check_twist():
    original = Twist()
    original.linear.x = 1.5
    original.linear.y = -0.25
    original.angular.z = 3.0
    restored = _roundtrip(original)
    assert (restored.linear.x, restored.linear.y, restored.linear.z) == (1.5, -0.25, 0.0)
    assert (restored.angular.x, restored.angular.y, restored.angular.z) == (0.0, 0.0, 3.0)


def check_quaternion():
    original = Quaternion(x=0.1, y=0.2, z=0.3, w=0.9)
    restored = _roundtrip(original)
    assert (restored.x, restored.y, restored.z, restored.w) == (0.1, 0.2, 0.3, 0.9)


def check_pose_stamped():
    original = PoseStamped()
    original.header.frame_id = "map"
    original.header.stamp.sec = 42
    original.header.stamp.nanosec = 7
    original.pose.position.x = 3.0
    original.pose.position.y = -4.0
    original.pose.orientation.w = 1.0
    restored = _roundtrip(original)
    assert str(restored.header.frame_id) == "map"
    assert int(restored.header.stamp.sec) == 42
    assert int(restored.header.stamp.nanosec) == 7
    assert restored.pose.position.x == 3.0
    assert restored.pose.position.y == -4.0
    assert restored.pose.orientation.w == 1.0


def check_laser_scan():
    original = LaserScan()
    original.header.frame_id = "laser"
    original.angle_min = -1.0
    original.angle_max = 1.0
    original.ranges = [1.0, 2.0, float("inf"), 3.5]
    original.intensities = [0.1, 0.2, 0.3]
    restored = _roundtrip(original)
    assert str(restored.header.frame_id) == "laser"
    assert restored.angle_min == -1.0
    ranges = list(restored.ranges)
    assert ranges[:2] == [1.0, 2.0]
    assert ranges[2] == float("inf")
    assert ranges[3] == 3.5
    intensities = list(restored.intensities)
    assert len(intensities) == 3
    assert abs(intensities[0] - 0.1) < 1e-6
    assert abs(intensities[1] - 0.2) < 1e-6
    assert abs(intensities[2] - 0.3) < 1e-6


def check_marker():
    original = Marker()
    original.ns = "demo"
    original.id = 7
    original.text = "hello world"
    original.pose.position.x = 5.0
    original.pose.orientation.w = 1.0
    point_a = Point()
    point_a.x, point_a.y, point_a.z = 1.0, 2.0, 3.0
    point_b = Point()
    point_b.x, point_b.y, point_b.z = -1.0, -2.0, -3.0
    original.points = [point_a, point_b]
    restored = _roundtrip(original)
    assert str(restored.ns) == "demo"
    assert int(restored.id) == 7
    assert str(restored.text) == "hello world"
    assert restored.pose.position.x == 5.0
    points = [(p.x, p.y, p.z) for p in restored.points]
    assert points == [(1.0, 2.0, 3.0), (-1.0, -2.0, -3.0)]


def main():
    check_twist()
    check_quaternion()
    check_pose_stamped()
    check_laser_scan()
    check_marker()
    print("PICKLE_MESSAGES_ROUNDTRIP_OK", flush=True)


if __name__ == "__main__":
    main()
