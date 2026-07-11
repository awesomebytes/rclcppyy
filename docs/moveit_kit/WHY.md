# Why moveit_kit — the full MoveIt 2 C++ API from Python via cppyy

`moveit_kit` lets you drive [MoveIt 2](https://moveit.ai) — the standard C++ motion-
planning framework for robot arms — from Python: the real C++ code owns the robot
model, the planning scene (FCL collision), the KDL kinematics solver and the OMPL
planner, while your Python composes the problem and reads the result. It does this
against the MoveIt that is already installed, with **no code generation and no build
step**, using MoveIt's **own C++ API names** (`RobotModel`, `RobotState::setFromIK`,
`PlanningScene::checkCollision`, a real OMPL `PlannerManager`).

The point isn't "MoveIt has no Python binding" — it does, `moveit_py`. The point is
that `moveit_py` is an explicit, hand-curated **subset**, and cppyy gives you the
**whole** C++ surface. For the API cheat sheet, see [MOVEIT_KIT.md](MOVEIT_KIT.md); for
the feasibility evidence, the plugin/parameter bring-up mechanics, benchmarks, and gaps,
see [REPORT.md](REPORT.md).

---

## The thing moveit_py makes you do without

`moveit_py` (the official pybind11 binding) wraps `MoveItPy` / `PlanningComponent` and a
curated slice of core (`RobotState`, `PlanningScene`, `AllowedCollisionMatrix`, a few
`construct_*` constraint helpers). It is a good high-level API — and a **fixed surface**:
only the methods someone chose to bind, in the shapes they chose. When you need a C++
method that wasn't wrapped, or a callback slot the binding omitted, you are stuck writing
and compiling a C++ node.

Concretely, and **verified in this spike** (REPORT.md §4): MoveIt C++
`RobotState::setFromIK` has an overload that takes a *validity callback*
(`GroupStateValidityCallbackFn`) which the solver invokes for every IK candidate — this
is how you get **collision-aware IK**. moveit_py's binding is
`set_from_ik(group_name, pose, timeout)`: **no callback parameter**. So moveit_py hands
you back whatever the solver finds, in collision or not, with no way to reject it from
Python.

With cppyy that overload is just *there*, because cppyy reads MoveIt's headers:

```python
cb = moveit_kit.state_validity_callback(
    lambda rs, group, values: not scene.isStateColliding(rs, group.getName()))
state.setFromIK(jmg, target_pose, 0.2, cb)   # the C++ KDL solver calls your Python
```

In the spike the C++ solver called this Python collision check **70–122 times** inside a
single `setFromIK`, rejecting in-collision candidates — a capability moveit_py's API
cannot express regardless of how you set it up.

---

## Side by side: build a model, do FK + IK, C++ vs Python

On the left, the shape of a MoveIt C++ program (RobotModelLoader + KDL IK) **and its
build system**. On the right, the runnable file this repo ships,
`scripts/moveit_kit_demos/d01_robot_state.py`.

### C++ — `robot_state.cpp` + `CMakeLists.txt` (+ ament package)

```cpp
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ik");   // needs robot_description +
                                                       // robot_description_kinematics
                                                       // params (launch file / YAML)
  robot_model_loader::RobotModelLoader loader(node);   // reads params, loads KDL plugin
  auto model = loader.getModel();
  moveit::core::RobotState state(model);
  const auto* jmg = model->getJointModelGroup("panda_arm");
  state.setToDefaultValues(jmg, "ready");
  state.update();
  Eigen::Isometry3d target = state.getGlobalLinkTransform("panda_link8");
  state.setToRandomPositions(jmg);
  bool ok = state.setFromIK(jmg, target, 0.1);
  RCLCPP_INFO(node->get_logger(), "IK %s", ok ? "solved" : "failed");
}
```

```cmake
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
add_executable(ik robot_state.cpp)
ament_target_dependencies(ik moveit_core moveit_ros_planning rclcpp)
```

…and this does not run yet: you need the `CMakeLists.txt`, a `package.xml`, `colcon
build`, a launch file that puts `robot_description` + `robot_description_kinematics` on
the node's parameter server, and a rebuild on every edit.

### Python — `d01_robot_state.py` (moveit_kit, shipped in this repo)

```python
from rclcppyy.bringup_rclcpp import bringup_rclcpp
from rclcppyy.kits import moveit_kit

moveit = moveit_kit.bringup_moveit(with_kinematics=True)   # parse + KDL plugin
cfg = moveit_kit.panda_config()
model = moveit_kit.build_robot_model(cfg.urdf, cfg.srdf)   # from URDF+SRDF strings
jmg = model.getJointModelGroup("panda_arm")

state = moveit.core.RobotState(model)
state.setToDefaultValues(jmg, "ready"); state.update()
target = state.getGlobalLinkTransform("panda_link8")       # FK

rclcpp = bringup_rclcpp(); rclcpp.ok() or rclcpp.init()
node = moveit_kit.make_node("ik")
moveit_kit.load_kinematics_solver(node, model, "panda_arm")  # KDL via pluginlib
state.setToRandomPositions(jmg); state.update()
print("IK", "solved" if state.setFromIK(jmg, target, 0.1) else "failed")   # IK
```

Run it directly: `pixi run -e moveit demo-moveit-state`. `RobotState`, `setFromIK`,
`getGlobalLinkTransform` are MoveIt's **own** C++ methods — the kit only assembles the
node parameters and loads the KDL plugin for you (the plugin/parameter bootstrap that
would otherwise be a launch file; see REPORT.md §2).

### What we gain

- **No compile step, no CMake, no launch file.** `python x.py` is the workflow. The
  model comes from URDF+SRDF *strings*, and the kit boots MoveIt's plugin/parameter stack
  in-process (the bit that normally forces a launch file).
- **The full C++ surface, header-following.** Anything in MoveIt's headers is reachable —
  including the `setFromIK` validity-callback overload moveit_py omits. cppyy reads the
  installed 2.12.4 headers, so the kit tracks whatever MoveIt is installed.
- **Python in the loop where it helps.** A collision check, a custom constraint, a
  heuristic can be a plain Python function the C++ solver calls — with a breakpoint and
  `print` in it. (See REPORT.md §3 for what that costs; collision checking itself runs at
  moveit_py's own speed — ~129k checks/sec.)

**What the C++ version buys that this one doesn't.** A compiled binary pays no JIT at
startup and gets static type-checking; the full `PlanningPipeline` (with time-
parameterization and the request/response adapters) is available in C++ but its header
crashes cppyy's parser, so the kit uses the planner plugin directly (geometric trajectory,
no timing — REPORT.md §5).

---

## The showcase: plan the Panda with MoveIt's real OMPL pipeline, from Python

`scripts/moveit_kit_demos/d02_plan_pose_goal.py` plans the Panda arm to a Cartesian pose
goal using MoveIt's **real OMPL `PlannerManager` plugin** — the same `.so` `move_group`
loads — configured from `ompl_planning.yaml`, then publishes the result as a
`moveit_msgs/DisplayTrajectory` on `/display_planned_path` so RViz renders it:

```python
moveit_kit.bringup_moveit(with_kinematics=True, with_planning=True)
node = moveit_kit.make_node("plan", moveit_kit.parameter_overrides(cfg.ompl, "ompl"))
moveit_kit.load_kinematics_solver(node, model, "panda_arm")
planner = moveit_kit.load_planner(node, model)                # OMPL, via pluginlib
result = moveit_kit.plan_pose_goal(planner, scene, "panda_arm", "panda_link8", target)
pub.publish(moveit_kit.display_trajectory(result, scene))     # rviz-compatible
```

The planner selects `geometric::RRTConnect` from the panda config and solves in a few
milliseconds. `pixi run -e moveit demo-moveit-plan`. This is the full MoveIt planning
stack — RobotModel + PlanningScene/FCL + KDL kinematics + OMPL — driven from ~30 lines of
Python, no code generation.

---

## Advantages of the cppyy approach

Grounded in the spike's measured numbers (see [REPORT.md](REPORT.md)):

- **The whole C++ API, not a curated subset.** The `setFromIK` validity-callback, direct
  `PlanningScene`/ACM/world manipulation, the planner plugin — all reachable; moveit_py
  binds a slice.
- **No codegen, no build, header-following.** `pixi install -e moveit` and `python x.py`;
  the kit tracks the installed MoveIt.
- **In-process plugin/parameter bootstrap.** The kit boots MoveIt's pluginlib + node-
  parameter stack from a Python-created node (REPORT.md §2) — the mechanic that normally
  requires `move_group` + a launch file — and the *next* kit (ros2_control) reuses it.
- **No speed tax on the hot path.** Collision checking runs at moveit_py's own throughput
  (~129k checks/sec); the full API is reachable at the subset's speed.

---

## Limits

moveit_kit is a v0 spike focused on **planning**, not execution. It plans but does not
drive controllers (that is the ros2_control kit); the trajectory is geometric (no time
parameterization, because the full `PlanningPipeline` header crashes Cling and only the
planner plugin is used); pose-goal planning needs the kinematics solver loaded; the
config helper is panda-specific; and MoveItServo / CHOMP/STOMP/Pilz are reachable via the
same pluginlib mechanic but not surfaced. The full, honest list is in
[REPORT.md](REPORT.md) §5.
