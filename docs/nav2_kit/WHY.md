# Why nav2_kit — your own Nav stack from Nav2's cores, in Python via cppyy

`nav2_kit` lets you build a navigation stack by driving [Nav2](https://nav2.org)'s
**algorithm cores directly** from Python: the real C++ code owns the costmap grid and
runs the planner (NavFn), while your Python owns the loop, the world, and the
controller. It does this against the Nav2 that is already installed, with **no
lifecycle servers, no pluginlib, and no tf** — and with no code generation and no
build step.

That framing is the whole point. Nav2 is a superb, production navigation system — but
its Python surface is deliberately *client-side*: you configure C++ servers with YAML
and send them goals. This doc shows what cppyy gives you when you want the opposite:
to compose your own miniature stack from Nav2's building blocks. For the API, see
[NAV2_KIT.md](NAV2_KIT.md); for the feasibility evidence, the honest coupling
boundary, and benchmarks, see [REPORT.md](REPORT.md).

---

## The thing stock Nav2 makes heavy: a *custom* planning loop

Suppose you just want to try your own idea: "take this occupancy grid, plan across it
with NavFn, and drive along the result." In stock Nav2, the supported way to run a
*custom* planner or controller is to make it a **C++ pluginlib plugin inside a
lifecycle server**. Concretely, per the
[Nav2 "writing a new planner plugin" docs](https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html):

- **Write a C++ class** deriving `nav2_core::GlobalPlanner`, implementing
  `configure() / activate() / deactivate() / cleanup() / createPlan()`, taking a
  `LifecycleNode`, a `tf2_ros::Buffer`, and a `Costmap2DROS`.
- **Export it as a plugin** — `PLUGINLIB_EXPORT_CLASS`, a `plugins.xml`, `ament`
  registration, and a `CMakeLists.txt` that builds a shared library.
- **Wire the lifecycle bringup** — a `planner_server` with a params YAML naming your
  plugin, then launch the lifecycle manager to `configure`→`activate` it.
- **Provide tf + a costmap** — the `Costmap2DROS` needs a transform tree
  (`map`→`odom`→`base_link`) and sensor/static layers to populate the grid.

That is: `colcon build`, a plugin XML, a YAML config, a launch file, a lifecycle
manager, and a tf tree — before you can call your planner once. It is the right
architecture for a fleet in production; it is a lot of ceremony for "try my idea on a
grid."

Contrast the cppyy "after": `pixi install -e nav2`, then `python your_plan.py`,
JIT-including the installed Nav2 headers in ~70 ms at startup.

---

## Side by side: a custom planning loop, stock Nav2 vs nav2_kit

### Stock Nav2 — the shape of a custom global planner

```cpp
// my_planner.hpp / .cpp — a nav2_core::GlobalPlanner plugin
class MyPlanner : public nav2_core::GlobalPlanner {
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                 std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void activate() override; void deactivate() override; void cleanup() override;
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
                                 const geometry_msgs::msg::PoseStamped & goal, ...) override;
};
PLUGINLIB_EXPORT_CLASS(MyPlanner, nav2_core::GlobalPlanner)
```
```xml
<!-- my_planner_plugin.xml -->
<library path="my_planner"><class type="MyPlanner" base_class_type="nav2_core::GlobalPlanner"/></library>
```
```yaml
# nav2_params.yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased: {plugin: "MyPlanner"}
```
…plus a `CMakeLists.txt` building the plugin, a launch file bringing up the
`planner_server` + lifecycle manager, and a tf tree feeding a `Costmap2DROS`. Then a
`colcon build` and a lifecycle bringup — before the planner runs once.

### nav2_kit — the complete runnable file this repo ships

```python
#!/usr/bin/env python
import numpy as np
from rclcppyy.kits import nav2_kit
nav2_kit.bringup_nav2()

grid = np.zeros((100, 100), dtype=np.uint8)                 # your world
grid[:, 50] = nav2_kit.LETHAL_OBSTACLE                      # a wall
grid[44:56, 50] = nav2_kit.FREE_SPACE                       # ... with a doorway
costmap = nav2_kit.costmap_from_numpy(grid, resolution=0.05)

path = nav2_kit.plan_navfn(costmap, start=(20, 50), goal=(80, 50))  # NavFn (C++)
print(f"Planned {len(path)} waypoints from {tuple(path[0])} to {tuple(path[-1])}")
```

Run it: `pixi run -e nav2 demo-nav2-plan`. It plans across the grid with **Nav2's
real NavFn algorithm** — the same C++ `nav2_navfn_planner::NavFn` the
`planner_server` runs — and prints the path, with no server, no plugin XML, no YAML,
no tf, no build.

### What we gain (from the comparison above)

- **No plugin/lifecycle/YAML/tf ceremony, no build.** The stock path needs a C++
  plugin, `plugins.xml`, params YAML, a launch file + lifecycle manager, and a tf
  tree; nav2_kit runs the moment you invoke it (~70 ms one-time cppyy bringup).
- **The world and the loop are just Python.** The occupancy grid is a NumPy array;
  the follow controller is a Python function you can breakpoint and edit. You iterate
  in seconds, not `colcon build` cycles.
- **It is the same `libnav2_*.so`.** `Costmap2D` and `NavFn` are Nav2's own classes,
  header-following, so nav2_kit tracks whatever Nav2 is installed — no binding to fall
  behind.
- **A prototype-to-native path.** As with the other kits, this is the L0 rung:
  prototype the stack with cppyy JIT today; the same calls lower to a compiled Nav2
  plugin when you want to deploy inside the real servers.

**What stock Nav2 buys that this does not.** A production stack: lifecycle
management, dynamic costmap layers from live sensors, tf/localization, recovery
behaviors, the full planner/controller/behavior-tree ecosystem, and the operational
maturity of the servers. nav2_kit is for *composing and prototyping from the cores*,
not for running a robot in production.

---

## The honest part: what is a clean core, and what is not

The thesis only works for Nav2 classes that are genuinely **node-free algorithms**.
Nav2 has clean examples of both sides, and nav2_kit draws the line where the evidence
does (full detail in [REPORT.md](REPORT.md)):

- **Separable cores (surfaced): `Costmap2D`, `NavFn`.** Plain classes — `Costmap2D(w,
  h, res, ox, oy)`, `NavFn(nx, ny)` operating on a raw `unsigned char*` cost array.
  No node, no tf, no pluginlib. Directly drivable.
- **Lifecycle-coupled (NOT surfaced): Smac Hybrid-A\*, the RegulatedPurePursuit
  controller.** Smac's collision checker only exposes a constructor taking a
  `Costmap2DROS` + a `LifecycleNode` (and its header transitively needs OMPL); the RPP
  controller's `configure()` takes a `LifecycleNode`. You cannot build these without
  the lifecycle machinery the thesis avoids — so nav2_kit does not pretend to, and the
  showcase's follow controller is **~30 lines of honest Python pure-pursuit**, stated
  as such. (RPP's header-only regulation math *is* separable and callable — a small
  bonus noted in the REPORT.)

This honesty is the point: the value is a real, working core road with a clearly
marked boundary — not a claim that all of Nav2 is "just Python".

---

## Two ways to use it

### Mode A — plan from Python on your own grid
Synthesize or load an occupancy grid, build a `Costmap2D`, plan with `NavFn`, and use
the path however you like (`d01_plan_grid.py`). Good for planner experiments,
map-based reasoning, and dataset generation where edit-run speed matters.

### Mode B — a whole miniature nav stack, live to rviz2
`scripts/nav2_kit_demos/d02_own_nav_stack.py` (the showcase) plans with NavFn, follows
with a pure-pursuit loop over simulated diff-drive kinematics, and publishes a live
`nav_msgs/OccupancyGrid` + `nav_msgs/Path` + `geometry_msgs/TwistStamped` via
rclcppyy — so an rviz2 (Fixed Frame `map`) shows the map, the plan, and the commanded
velocity as the simulated robot drives to the goal. One self-contained Python file,
planner in C++, controller in Python.

---

## Advantages of the cppyy approach

Grounded in the spike's measured numbers (see [REPORT.md](REPORT.md)):

- **No plugin/YAML/lifecycle/build ceremony.** `python x.py` is the workflow; bringup
  is a one-time ~70 ms JIT.
- **Header-following, tracks the installed Nav2.** No hand-maintained binding.
- **Bulk data stays fast.** A NumPy grid → `Costmap2D` is a single `memcpy`
  (~600–3600× a per-cell Python loop); the plan never leaves C++ (NavFn on 1024² in
  tens of ms vs ~2 s for a pure-Python A\* — the orchestration story).
- **A prototype-to-native lowering path**, as with bt_kit / pcl_kit / ompl_kit: the
  same calls become a compiled Nav2 plugin when you deploy.

---

## Limits

nav2_kit is a v0 spike, and deliberately **not a Nav2 stack**: no lifecycle servers,
no pluginlib, no tf, no dynamic obstacle/inflation layers, no recovery behaviors. Only
`Costmap2D` + `NavFn` are surfaced; Smac and the RPP controller are lifecycle-coupled
and out of scope (their separable pieces are noted). The complementary direction —
running a **Python planner/controller plugin *inside* a real Nav2 server** via a
pluginlib bridge — is a separate planned spike, not this one. The full, honest list is
in [REPORT.md](REPORT.md) §6.
