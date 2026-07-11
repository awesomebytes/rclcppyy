#!/usr/bin/env python
"""
moveit_kit d01: the MoveIt RobotModel / RobotState basics from Python -- build the
Panda model from its URDF+SRDF, do forward kinematics, then inverse kinematics via
the real KDL plugin. Mirrors MoveIt's C++ RobotState tutorial; the kit only hides
the cppyy bringup + the plugin-loading friction.

Runs with the full MoveIt C++ API against the installed MoveIt 2 -- no moveit_py.

Run: pixi run -e moveit demo-moveit-state
"""
import os

os.environ.setdefault("ROS_DOMAIN_ID", "48")

from rclcppyy.bringup_rclcpp import bringup_rclcpp   # noqa: E402
from rclcppyy.kits import moveit_kit                 # noqa: E402

GROUP = "panda_arm"
TIP = "panda_link8"


def main():
    # PARSE layer: RobotModel + RobotState (no node, no plugins).
    moveit = moveit_kit.bringup_moveit()
    cfg = moveit_kit.panda_config()
    model = moveit_kit.build_robot_model(cfg.urdf, cfg.srdf)
    jmg = model.getJointModelGroup(GROUP)
    print("Loaded RobotModel '%s': %d-DOF group '%s', tip '%s'"
          % (model.getName(), jmg.getVariableCount(), GROUP, TIP))

    # Forward kinematics at the named 'ready' state.
    state = moveit.core.RobotState(model)
    state.setToDefaultValues(jmg, "ready")
    state.update()
    tf = state.getGlobalLinkTransform(TIP)
    p = tf.translation()
    print("\nFK at 'ready':  %s = (%.4f, %.4f, %.4f)" % (TIP, p[0], p[1], p[2]))

    # Kinematics layer: load the KDL plugin via pluginlib and solve IK back to that
    # pose from a random seed. Needs rclcpp (the plugin initializes against a Node).
    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    moveit_kit.bringup_moveit(with_kinematics=True)
    node = moveit_kit.make_node("moveit_kit_d01")
    if not moveit_kit.load_kinematics_solver(node, model, GROUP):
        print("Failed to load the KDL kinematics solver.")
        return

    target = moveit_kit.pose(p[0], p[1], p[2],
                             *_quat(state.getGlobalLinkTransform(TIP)))
    state.setToRandomPositions(jmg)
    state.update()
    if state.setFromIK(jmg, target, 0.1):
        state.update()
        got = state.getGlobalLinkTransform(TIP).translation()
        err = sum((got[i] - p[i]) ** 2 for i in range(3)) ** 0.5
        jvals = _joint_values(state, jmg)
        print("IK solved:      pos error %.2e m" % err)
        print("IK joints:      [%s]" % ", ".join("%.4f" % v for v in jvals))
    else:
        print("IK failed.")
    print("\nSame MoveIt C++ RobotModel/RobotState/KDL used by move_group -- from Python.")


def _quat(isometry):
    import cppyy
    q = cppyy.gbl.Eigen.Quaterniond(isometry.rotation())
    return q.x(), q.y(), q.z(), q.w()


def _joint_values(state, jmg):
    import cppyy
    vals = cppyy.gbl.std.vector["double"]()
    state.copyJointGroupPositions(jmg, vals)
    return list(vals)


if __name__ == "__main__":
    main()
