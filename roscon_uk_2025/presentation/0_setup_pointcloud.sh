#!/usr/bin/env bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Setup cleanup trap
trap 'kill $(jobs -p)' EXIT

echo "Playing pointcloud bag..."
ros2 bag play --loop ../pointcloud_lexus3-2024-04-05-gyor.mcap --qos-profile-overrides-path ../reliability_override.yaml &

echo "Running Rviz2..."
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
/opt/ros/jazzy/bin/rviz2 -d ../pointcloud.rviz

# Wait for all background processes
wait