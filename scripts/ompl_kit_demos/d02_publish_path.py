#!/usr/bin/env python
"""
ompl_kit + rclcppyy showcase: plan a 2D path with OMPL, then publish it as a
``nav_msgs/Path`` on a real ROS 2 topic -- so the plan is directly consumable by
RViz / any ROS navigation stack.

The plan is the d01 problem (unit square, circular obstacle, RRTConnect, Python
validity checker). The only new part is turning the OMPL ``PathGeometric`` into a
C++ ``nav_msgs::msg::Path`` and publishing it via rclcppyy -- the message is a C++
message end to end; Python only fills the handful of waypoint poses.

Run: pixi run -e ompl demo-ompl-path
"""
import argparse
import os
import time

os.environ.setdefault("ROS_DOMAIN_ID", "45")

from rclcppyy.bringup_rclcpp import bringup_rclcpp  # noqa: E402
from rclcppyy.kits import ompl_kit                  # noqa: E402

TOPIC = "ompl_kit/plan"
OBSTACLE = (0.5, 0.5, 0.25)


def is_state_valid(state):
    cx, cy, r = OBSTACLE
    return (state[0] - cx) ** 2 + (state[1] - cy) ** 2 > r ** 2


def plan_waypoints():
    ob, og = ompl_kit.bringup_ompl()
    ompl_kit.set_seed(42)
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0.0)
    bounds.setHigh(1.0)
    space.setBounds(bounds)
    ss = og.SimpleSetup(ob.StateSpacePtr(space))
    ss.setStateValidityChecker(ompl_kit.validity_checker(is_state_valid, owner=ss))
    start = ob.ScopedState[ob.RealVectorStateSpace](ss.getStateSpace())
    start[0], start[1] = 0.1, 0.1
    goal = ob.ScopedState[ob.RealVectorStateSpace](ss.getStateSpace())
    goal[0], goal[1] = 0.9, 0.9
    ss.setStartAndGoalStates(start, goal)
    ss.setPlanner(ob.PlannerPtr(og.RRTConnect(ss.getSpaceInformation())))
    if not ss.solve(1.0):
        return None
    ss.simplifySolution()
    return ompl_kit.path_to_list(ss.getSolutionPath(), dim=2)


def build_path_msg(waypoints):
    """OMPL waypoints -> a C++ nav_msgs::msg::Path (Python fills a few poses).

    Built as the real C++ message (cppyy.gbl.nav_msgs.msg.Path) so ``poses`` is a
    std::vector and the message crosses to the publisher without a Python-message
    conversion -- same approach as pcl_kit.msg_from_cloud."""
    import cppyy
    from rclcppyy.bringup_rclcpp import add_ros2_include_paths
    add_ros2_include_paths()
    cppyy.include("nav_msgs/msg/path.hpp")
    cppyy.include("geometry_msgs/msg/pose_stamped.hpp")
    msg = cppyy.gbl.nav_msgs.msg.Path()
    msg.header.frame_id = "map"
    for x, y in waypoints:
        pose = cppyy.gbl.geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        msg.poses.push_back(pose)
    return msg


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--count", type=int, default=3, help="how many times to publish")
    ap.add_argument("--rate", type=float, default=2.0, help="publish rate (Hz)")
    args = ap.parse_args()

    waypoints = plan_waypoints()
    if waypoints is None:
        print("No solution found.")
        return
    print(f"Planned {len(waypoints)} waypoints: "
          + " -> ".join(f"({x:.2f},{y:.2f})" for x, y in waypoints))

    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    from nav_msgs.msg import Path

    node = rclcpp.Node("ompl_kit_planner")
    pub = node.create_publisher(Path, TOPIC, 10)   # Python type registers the topic
    msg = build_path_msg(waypoints)                # C++ message object to publish

    for i in range(args.count):
        pub.publish(msg)
        print(f"STAT published nav_msgs/Path seq={i + 1} "
              f"poses={msg.poses.size()} on '{TOPIC}'", flush=True)
        time.sleep(1.0 / args.rate)
    print("SUMMARY done")


if __name__ == "__main__":
    main()
