# moveit_kit — cheat sheet for a coding agent

You are writing Python that drives **MoveIt 2** — the C++ motion-planning framework —
through `rclcppyy.kits.moveit_kit`. The kit **mirrors MoveIt's C++ API**: it returns the
real `moveit` namespace and you use `moveit.core.RobotState`,
`planning_scene::PlanningScene`, `RobotState::setFromIK`, a real OMPL `PlannerManager`
exactly as in the MoveIt C++ tutorials. The kit removes the cppyy friction: staged
bringup, the plugin/parameter bootstrap (loading the KDL/OMPL plugins via pluginlib and
assembling node parameters from config YAMLs), an Eigen pose helper, and ordered
teardown. You do **not** need to know cppyy.

(For *why* this exists and the moveit_py contrast, see [WHY.md](WHY.md); for the plugin/
parameter bring-up mechanics and benchmarks, see [REPORT.md](REPORT.md).)

**Requires** the `moveit` pixi env: `pixi run -e moveit python your_script.py`. The kit
is built around the **panda** test model (`moveit_resources_panda_*`).

**Golden rules**
- Bringup is **staged and idempotent**. `moveit_kit.bringup_moveit()` = the *parse* layer
  (RobotModel/RobotState/PlanningScene/FCL — no node, no plugins). Add
  `with_kinematics=True` for KDL IK, `with_planning=True` for OMPL planning. The heavy
  stages are gated (JIT + plugin teardown), so a parse-only user skips them.
- The **kinematics and planning layers need rclcpp initialized** (the plugins load against
  a Node): `rclcpp = bringup_rclcpp(); rclcpp.ok() or rclcpp.init()`.
- **Build the model from strings:** `model = build_robot_model(cfg.urdf, cfg.srdf)` where
  `cfg = panda_config()`. No parameter server needed for the model.
- **Plugins read node parameters.** The OMPL planner needs the config: build a node with
  `make_node(name, parameter_overrides(cfg.ompl, "ompl"))`. The KDL plugin declares its own
  defaults, so a plain `make_node(name)` is enough for IK.
- **Pin nothing manually for the plugins** — the kit registers their teardown. A benign
  `class_loader` "SEVERE WARNING … will NOT be unloaded" may print at exit; ignore it.
- Domain: set `ROS_DOMAIN_ID` if you publish (the demos use 48).

---

## Pattern 1 — RobotModel + forward kinematics  (parse layer, no node)
*Use for:* loading a robot, reading link transforms. No rclcpp needed.

```python
from rclcppyy.kits import moveit_kit
moveit = moveit_kit.bringup_moveit()
cfg = moveit_kit.panda_config()
model = moveit_kit.build_robot_model(cfg.urdf, cfg.srdf)
jmg = model.getJointModelGroup("panda_arm")

state = moveit.core.RobotState(model)
state.setToDefaultValues(jmg, "ready")     # a named SRDF group state
state.update()
p = state.getGlobalLinkTransform("panda_link8").translation()   # FK: p[0], p[1], p[2]
```

---

## Pattern 2 — inverse kinematics with the real KDL plugin
*Use for:* solving joint angles for an end-effector pose. Loads the KDL plugin via
pluginlib in-process.

```python
from rclcppyy.bringup_rclcpp import bringup_rclcpp
rclcpp = bringup_rclcpp(); rclcpp.ok() or rclcpp.init()
moveit = moveit_kit.bringup_moveit(with_kinematics=True)

node = moveit_kit.make_node("ik")                     # plain node: KDL uses its defaults
moveit_kit.load_kinematics_solver(node, model, "panda_arm")   # attaches KDL to the group

target = moveit_kit.pose(0.3, 0.0, 0.6)               # Eigen::Isometry3d (see Pattern 6)
state = moveit.core.RobotState(model)
state.setToRandomPositions(jmg); state.update()
if state.setFromIK(jmg, target, 0.1):                 # MoveIt's own setFromIK
    state.update()
    print(state.getGlobalLinkTransform("panda_link8").translation()[0])
```

---

## Pattern 3 — collision-aware IK with a Python validity callback  (moveit_py can't)
*Use for:* IK that rejects in-collision solutions. The C++ solver calls your Python check
per candidate. This overload has no moveit_py equivalent.

```python
scene = moveit_kit.planning_scene(model)
def ok(rs, group, values):                            # rs: RobotState*, values: joints
    rs.setJointGroupPositions(group, values); rs.update()
    return not scene.isStateColliding(rs, group.getName(), False)
cb = moveit_kit.state_validity_callback(ok, owner=state)
state.setFromIK(jmg, target, 0.2, cb)                 # solver invokes ok() per candidate
```

---

## Pattern 4 — PlanningScene + collision checking (FCL)
*Use for:* self-collision, world-object collision, state validity.

```python
import cppyy
scene = moveit_kit.planning_scene(model)              # FCL is the default detector
cd = cppyy.gbl.collision_detection
state = scene.getCurrentStateNonConst()
state.setToDefaultValues(jmg, "ready"); state.update()

res = cd.CollisionResult()
scene.checkSelfCollision(cd.CollisionRequest(), res)
print("self-collision:", bool(res.collision))

# add a world obstacle and check
box = cppyy.gbl.std.make_shared["shapes::Box"](0.3, 0.3, 0.3)
scene.getWorldNonConst().addToObject("box", box, moveit_kit.pose(0.3, 0.0, 0.5))
hit = cd.CollisionResult()
scene.checkCollision(cd.CollisionRequest(), hit)      # self + world
print("colliding:", bool(hit.collision))
```

---

## Pattern 5 — plan a motion with the real OMPL pipeline
*Use for:* motion planning. Loads MoveIt's OMPL `PlannerManager` plugin; params come from
`ompl_planning.yaml`.

```python
from rclcppyy.bringup_rclcpp import bringup_rclcpp
rclcpp = bringup_rclcpp(); rclcpp.ok() or rclcpp.init()
moveit = moveit_kit.bringup_moveit(with_kinematics=True, with_planning=True)

scene = moveit_kit.planning_scene(model)
scene.getCurrentStateNonConst().setToDefaultValues(jmg, "ready")
scene.getCurrentStateNonConst().update()

node = moveit_kit.make_node("plan", moveit_kit.parameter_overrides(cfg.ompl, "ompl"))
moveit_kit.load_kinematics_solver(node, model, "panda_arm")   # needed for POSE goals
planner = moveit_kit.load_planner(node, model)                # OMPL PlannerManager

# (a) joint-space goal
goal = moveit.core.RobotState(model); goal.setToDefaultValues(jmg, "ready")
gv = cppyy.gbl.std.vector["double"](); goal.copyJointGroupPositions(jmg, gv)
gv[0] += 1.0; goal.setJointGroupPositions(jmg, gv); goal.update()
r = moveit_kit.plan_joint_goal(planner, scene, model, "panda_arm", goal)

# (b) Cartesian pose goal (needs load_kinematics_solver above)
r = moveit_kit.plan_pose_goal(planner, scene, "panda_arm", "panda_link8",
                              (0.3, 0.1, 0.5, 0, 0, 0, 1))
print(r.ok, r.error_code, r.trajectory.getWayPointCount())
```
`PlanResult` fields: `ok`, `error_code`, `planning_time`, `trajectory`
(`robot_trajectory::RobotTrajectoryPtr`).

---

## Pattern 6 — publish a plan to RViz (DisplayTrajectory)
*Use for:* visualizing a plan. Build the rviz-compatible message and publish via rclcppyy.

```python
from moveit_msgs.msg import DisplayTrajectory          # Python type registers the topic
pub = node.create_publisher(DisplayTrajectory, "display_planned_path", 10)
msg = moveit_kit.display_trajectory(r, scene, model_id="panda")   # C++ message
pub.publish(msg)                                        # rclcppyy publishes it directly
```
`scripts/moveit_kit_demos/d02_plan_pose_goal.py` is the full showcase
(`pixi run -e moveit demo-moveit-plan`); open `rviz2` on `/display_planned_path`.

---

## Pattern 7 — Eigen poses
The `pose()` helper builds an `Eigen::Isometry3d` (Eigen block assignment does not cross
cppyy, so build poses this way, not `iso.translation()[i] = v`):

```python
p = moveit_kit.pose(x, y, z)                            # translation, identity rotation
p = moveit_kit.pose(x, y, z, qx, qy, qz, qw)            # with a quaternion
```

---

## Gotchas (short version)
- **Kinematics/planning need rclcpp** (`bringup_rclcpp(); init()`); parse/FK/collision do
  not.
- **Pose-goal planning needs `load_kinematics_solver`** (OMPL samples the goal via IK);
  joint goals do not.
- **The OMPL planner needs its params**: `make_node(name, parameter_overrides(cfg.ompl,
  "ompl"))`. The KDL plugin uses its own defaults (a plain node is fine).
- **Do NOT `cppyy.include` MoveIt's convenience headers** (`robot_model_loader.hpp`,
  `moveit_cpp.hpp`, `planning_pipeline.hpp`) — they pull `generate_parameter_library`
  headers that crash Cling. The kit loads the plugins directly instead (REPORT.md §2).
- **Trajectories are geometric** (no timing / `getDuration()` = 0): the direct planner
  plugin skips the time-parameterization adapter. Add
  `trajectory_processing::TimeOptimalTrajectoryGeneration` if you need velocities.
- **Build Eigen objects in C++** (`moveit_kit.pose`), not by item-assigning Eigen blocks.
- **Benign `class_loader` "SEVERE WARNING" at exit** — cosmetic; the process exits clean.
- **Panda-specific**: `panda_config()` and the `panda_arm`/`panda_link8` names are for the
  panda test model; another robot needs its own URDF/SRDF + kinematics/OMPL YAMLs.
- Call `moveit_kit.warmup()` once during init to move the plugin-load first-use JIT off
  your first live call.
