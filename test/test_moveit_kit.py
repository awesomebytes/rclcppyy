#!/usr/bin/env python3
"""Tests for rclcppyy.kits.moveit_kit (the full MoveIt C++ API via cppyy).

MoveIt is an optional dependency (the pixi `moveit` env), absent from the default
env. The whole module auto-skips when the MoveIt headers are not installed, so the
default `pixi run test` is unaffected. Run the real thing with
`pixi run -e moveit test-moveit`.

All tests share one process; bringup_moveit() is idempotent. The parse layer needs
no rclcpp; the kinematics/planning layers load pluginlib plugins against a Node, so
those tests bring up rclcpp once. Planning is randomized -- tests assert success +
structure, not exact trajectories. A benign `class_loader` "SEVERE WARNING ... will
NOT be unloaded" may print at teardown; it is not an error (see the kit docstring).
"""
import os

import pytest

_HAVE_MOVEIT = os.path.isdir(
    os.path.join(os.environ.get("CONDA_PREFIX", ""), "include", "moveit_core"))

pytestmark = pytest.mark.skipif(
    not _HAVE_MOVEIT, reason="MoveIt not installed (use the moveit env)")

if _HAVE_MOVEIT:
    import cppyy
    from rclcppyy.bringup_rclcpp import bringup_rclcpp
    from rclcppyy.kits import moveit_kit

GROUP = "panda_arm"
TIP = "panda_link8"


@pytest.fixture(scope="module")
def config():
    return moveit_kit.panda_config()


@pytest.fixture(scope="module")
def model(config):
    moveit_kit.bringup_moveit()
    return moveit_kit.build_robot_model(config.urdf, config.srdf)


@pytest.fixture(scope="module")
def ros():
    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    return rclcpp


def test_bringup_idempotent(model):
    moveit = moveit_kit.bringup_moveit()
    assert moveit is moveit_kit.bringup_moveit()
    assert hasattr(moveit.core, "RobotState")


def test_build_robot_model_and_groups(model):
    assert str(model.getName()) == "panda"
    groups = [str(g) for g in model.getJointModelGroupNames()]
    assert GROUP in groups
    assert model.getJointModelGroup(GROUP).getVariableCount() == 7


def test_forward_kinematics_ready(model):
    """Probe A: FK of the named 'ready' state (parse-only, no node/plugins)."""
    moveit = moveit_kit.bringup_moveit()
    jmg = model.getJointModelGroup(GROUP)
    state = moveit.core.RobotState(model)
    state.setToDefaultValues(jmg, "ready")
    state.update()
    p = state.getGlobalLinkTransform(TIP).translation()
    # The panda 'ready' end-effector sits ~0.31 m forward, ~0.59 m up.
    assert p[0] == pytest.approx(0.307, abs=1e-2)
    assert p[2] == pytest.approx(0.590, abs=1e-2)


def test_self_collision_detection(model):
    """Probe C: PlanningScene + FCL self-collision (ready clear, zeros colliding)."""
    scene = moveit_kit.planning_scene(model)
    assert str(scene.getCollisionDetectorName()) == "FCL"
    cd = cppyy.gbl.collision_detection
    jmg = model.getJointModelGroup(GROUP)
    state = scene.getCurrentStateNonConst()

    state.setToDefaultValues(jmg, "ready")
    state.update()
    res = cd.CollisionResult()
    scene.checkSelfCollision(cd.CollisionRequest(), res)
    assert not res.collision

    zeros = cppyy.gbl.std.vector["double"]([0.0] * 7)
    state.setJointGroupPositions(jmg, zeros)
    state.update()
    res2 = cd.CollisionResult()
    scene.checkSelfCollision(cd.CollisionRequest(), res2)
    assert res2.collision


def test_world_object_collision(model):
    """Probe C: a world box in the arm's path is detected by full collision check."""
    scene = moveit_kit.planning_scene(model)
    jmg = model.getJointModelGroup(GROUP)
    state = scene.getCurrentStateNonConst()
    state.setToDefaultValues(jmg, "ready")
    state.update()
    cd = cppyy.gbl.collision_detection
    clean = cd.CollisionResult()
    scene.checkCollision(cd.CollisionRequest(), clean)
    assert not clean.collision
    box = cppyy.gbl.std.make_shared["shapes::Box"](0.4, 0.4, 0.9)
    scene.getWorldNonConst().addToObject("obstacle", box, moveit_kit.pose(0.3, 0.0, 0.5))
    hit = cd.CollisionResult()
    scene.checkCollision(cd.CollisionRequest(), hit)
    assert hit.collision
    scene.getWorldNonConst().removeObject("obstacle")


def test_parameter_overrides_flatten(config):
    """The panda-config param-assembly helper flattens nested YAML to dotted names."""
    params = moveit_kit.parameter_overrides(config.ompl, "ompl")
    names = {str(p.get_name()) for p in params}
    assert "ompl.planner_configs.RRTConnectkConfigDefault.type" in names
    assert "ompl.panda_arm.planner_configs" in names
    assert params.size() > 50


def test_kinematics_ik_via_kdl_plugin(model, ros):
    """Probe B: the KDL kinematics plugin loads in-process via pluginlib and solves
    IK back to a reachable pose (pos error ~0)."""
    moveit = moveit_kit.bringup_moveit(with_kinematics=True)
    jmg = model.getJointModelGroup(GROUP)
    node = moveit_kit.make_node("test_moveit_kit_ik")
    assert moveit_kit.load_kinematics_solver(node, model, GROUP)
    assert bool(jmg.getSolverInstance())

    state = moveit.core.RobotState(model)
    state.setToDefaultValues(jmg, "ready")
    state.update()
    target = cppyy.gbl.Eigen.Isometry3d(state.getGlobalLinkTransform(TIP))
    tr = target.translation()
    state.setToRandomPositions(jmg)
    state.update()
    assert state.setFromIK(jmg, target, 0.2)
    state.update()
    got = state.getGlobalLinkTransform(TIP).translation()
    err = sum((got[i] - tr[i]) ** 2 for i in range(3)) ** 0.5
    assert err < 1e-3


def test_state_validity_callback_invoked_during_ik(model, ros):
    """The Python GroupStateValidityCallbackFn is called by the C++ IK solver -- the
    collision-aware-IK capability moveit_py's set_from_ik cannot express."""
    moveit = moveit_kit.bringup_moveit(with_kinematics=True)
    jmg = model.getJointModelGroup(GROUP)
    node = moveit_kit.make_node("test_moveit_kit_ik_cb")
    moveit_kit.load_kinematics_solver(node, model, GROUP)
    scene = moveit_kit.planning_scene(model)
    state = moveit.core.RobotState(model)
    state.setToDefaultValues(jmg, "ready")
    state.update()
    target = cppyy.gbl.Eigen.Isometry3d(state.getGlobalLinkTransform(TIP))
    calls = {"n": 0}

    def validity(rs, group, values):
        calls["n"] += 1
        rs.setJointGroupPositions(group, values)
        rs.update()
        return not scene.isStateColliding(rs, group.getName(), False)

    cb = moveit_kit.state_validity_callback(validity, owner=state)
    state.setToRandomPositions(jmg)
    state.update()
    state.setFromIK(jmg, target, 0.2, cb)
    assert calls["n"] > 0                       # the C++ solver called Python


def test_planning_ompl_joint_goal(model, config, ros):
    """Probe D: MoveIt's real OMPL PlannerManager plugin plans a joint-space goal."""
    moveit = moveit_kit.bringup_moveit(with_planning=True)
    jmg = model.getJointModelGroup(GROUP)
    scene = moveit_kit.planning_scene(model)
    scene.getCurrentStateNonConst().setToDefaultValues(jmg, "ready")
    scene.getCurrentStateNonConst().update()
    node = moveit_kit.make_node("test_moveit_kit_plan",
                                moveit_kit.parameter_overrides(config.ompl, "ompl"))
    planner = moveit_kit.load_planner(node, model)
    assert str(planner.getDescription()) == "OMPL"

    goal = moveit.core.RobotState(model)
    goal.setToDefaultValues(jmg, "ready")
    gv = cppyy.gbl.std.vector["double"]()
    goal.copyJointGroupPositions(jmg, gv)
    gv[0] = gv[0] + 1.0
    goal.setJointGroupPositions(jmg, gv)
    goal.update()
    result = moveit_kit.plan_joint_goal(planner, scene, model, GROUP, goal,
                                        allowed_planning_time=5.0)
    assert result.ok
    assert result.trajectory.getWayPointCount() >= 2


def test_planning_ompl_pose_goal_and_display(model, config, ros):
    """Probe D: pose-goal planning (needs the IK solver) + DisplayTrajectory build."""
    moveit_kit.bringup_moveit(with_kinematics=True, with_planning=True)
    jmg = model.getJointModelGroup(GROUP)
    scene = moveit_kit.planning_scene(model)
    start = scene.getCurrentStateNonConst()
    start.setToDefaultValues(jmg, "ready")
    start.update()
    node = moveit_kit.make_node("test_moveit_kit_pose",
                                moveit_kit.parameter_overrides(config.ompl, "ompl"))
    moveit_kit.load_kinematics_solver(node, model, GROUP)
    planner = moveit_kit.load_planner(node, model)

    tf = start.getGlobalLinkTransform(TIP)
    p = tf.translation()
    q = cppyy.gbl.Eigen.Quaterniond(tf.rotation())
    target = (p[0] - 0.1, p[1] + 0.15, p[2] - 0.1, q.x(), q.y(), q.z(), q.w())
    result = moveit_kit.plan_pose_goal(planner, scene, GROUP, TIP, target,
                                       allowed_planning_time=5.0)
    assert result.ok
    dt = moveit_kit.display_trajectory(result, scene, model_id="panda")
    assert str(dt.model_id) == "panda"
    assert dt.trajectory.size() == 1


def test_pose_helper(model):
    """pose() builds an Eigen::Isometry3d with the given translation."""
    iso = moveit_kit.pose(0.1, 0.2, 0.3)
    t = iso.translation()
    assert (t[0], t[1], t[2]) == pytest.approx((0.1, 0.2, 0.3))
