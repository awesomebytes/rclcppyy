#!/usr/bin/env python
"""
moveit_kit SHOWCASE (d02): plan the Panda arm to a Cartesian pose goal with MoveIt's
REAL OMPL planning pipeline -- loaded in-process from Python via pluginlib -- then
publish the result as a moveit_msgs/DisplayTrajectory on /display_planned_path so
RViz's MotionPlanning display renders it. A joint-trajectory summary is printed.

This is the full MoveIt C++ motion-planning stack (RobotModel + PlanningScene/FCL +
KDL kinematics + OMPL PlannerManager) driven from ~30 lines of Python -- the API
moveit_py exposes only a curated slice of. Nothing is generated; the plugins are the
same .so files move_group loads.

Run:  pixi run -e moveit demo-moveit-plan
View: rviz2, add a MotionPlanning or Trajectory display on /display_planned_path
      (fixed frame 'panda_link0'), with the panda robot_description.
"""
import argparse
import os
import time

os.environ.setdefault("ROS_DOMAIN_ID", "48")

import cppyy                                          # noqa: E402
from rclcppyy.bringup_rclcpp import bringup_rclcpp    # noqa: E402
from rclcppyy.kits import moveit_kit                  # noqa: E402

GROUP = "panda_arm"
TIP = "panda_link8"
TOPIC = "display_planned_path"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--count", type=int, default=3, help="times to publish the plan")
    ap.add_argument("--rate", type=float, default=1.0, help="publish rate (Hz)")
    ap.add_argument("--time", type=float, default=5.0, help="allowed planning time (s)")
    args = ap.parse_args()

    # Full stack: parse + KDL kinematics (OMPL samples pose goals via IK) + OMPL.
    moveit_kit.bringup_moveit(with_kinematics=True, with_planning=True)
    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()

    cfg = moveit_kit.panda_config()
    model = moveit_kit.build_robot_model(cfg.urdf, cfg.srdf)
    scene = moveit_kit.planning_scene(model)
    jmg = model.getJointModelGroup(GROUP)

    # Start at the named 'ready' state.
    start = scene.getCurrentStateNonConst()
    start.setToDefaultValues(jmg, "ready")
    start.update()

    # A single node carries BOTH the OMPL params (namespace 'ompl', assembled from
    # ompl_planning.yaml) -- the plugins read their whole config from node params.
    node = moveit_kit.make_node("moveit_kit_d02",
                                moveit_kit.parameter_overrides(cfg.ompl, "ompl"))
    moveit_kit.load_kinematics_solver(node, model, GROUP)
    planner = moveit_kit.load_planner(node, model)

    # Pose goal: shift the 'ready' end-effector pose in x/z (keep its orientation).
    tf = start.getGlobalLinkTransform(TIP)
    p = tf.translation()
    q = cppyy.gbl.Eigen.Quaterniond(tf.rotation())
    target = (p[0] - 0.15, p[1] + 0.20, p[2] - 0.15, q.x(), q.y(), q.z(), q.w())
    print("Planning %s -> pose (%.3f, %.3f, %.3f) with OMPL (%s) ..."
          % (TIP, target[0], target[1], target[2],
             planner.getDescription()))

    t0 = time.perf_counter()
    result = moveit_kit.plan_pose_goal(planner, scene, GROUP, TIP, target,
                                       allowed_planning_time=args.time)
    wall = (time.perf_counter() - t0) * 1000.0
    if not result.ok:
        print("Planning failed (error code %d)." % result.error_code)
        return

    traj = result.trajectory
    n = traj.getWayPointCount()
    print("Solved: %d waypoints, duration %.2fs, planner time %.3fs (wall %.0fms)"
          % (n, traj.getDuration(), result.planning_time, wall))
    _print_summary(traj, jmg)

    # Publish an rviz-compatible DisplayTrajectory (C++ message, end to end).
    from moveit_msgs.msg import DisplayTrajectory   # Python type registers the topic
    pub = node.create_publisher(DisplayTrajectory, TOPIC, 10)
    msg = moveit_kit.display_trajectory(result, scene, model_id="panda")
    for i in range(args.count):
        pub.publish(msg)                             # rclcppyy publishes the C++ msg
        print("STAT published DisplayTrajectory seq=%d on '/%s'" % (i + 1, TOPIC),
              flush=True)
        time.sleep(1.0 / args.rate)
    print("SUMMARY done -- open rviz2 on /%s to view the motion." % TOPIC)


def _print_summary(traj, jmg):
    names = [str(n) for n in jmg.getVariableNames()]
    first = _wp(traj, 0, jmg)
    last = _wp(traj, traj.getWayPointCount() - 1, jmg)
    print("  joint          start -> goal")
    for name, a, b in zip(names, first, last):
        print("    %-13s %7.3f -> %7.3f" % (name, a, b))


def _wp(traj, i, jmg):
    vals = cppyy.gbl.std.vector["double"]()
    traj.getWayPoint(i).copyJointGroupPositions(jmg, vals)
    return list(vals)


if __name__ == "__main__":
    main()
