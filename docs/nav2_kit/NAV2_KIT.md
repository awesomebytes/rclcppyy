# nav2_kit — cheat sheet for a coding agent

You are writing Python that composes **your own navigation stack from Nav2's
algorithm cores** through `rclcppyy.kits.nav2_kit` — **no lifecycle servers, no
pluginlib, no tf**. Python owns the loop; Nav2's C++ owns the math. The kit
**mirrors Nav2's own C++ API**: `bringup_nav2()` returns the real `nav2_costmap_2d`
and `nav2_navfn_planner` namespaces, and you use `Costmap2D`, `NavFn` as in the C++.
The kit only removes the cppyy friction (bringup, the NumPy↔charmap memcpy, NavFn's
raw-pointer I/O). You do **not** need to know cppyy.

(For *why* this exists and a stock-Nav2 comparison, see [WHY.md](WHY.md); for the
feasibility matrix, the honest Smac/RPP boundary, and benchmarks, see
[REPORT.md](REPORT.md).)

**Requires** the `nav2` pixi env: `pixi run -e nav2 python your_script.py`.

**Golden rules**
- Call `nav2_kit.bringup_nav2()` once; it returns `(nav2_costmap_2d,
  nav2_navfn_planner)`. Idempotent (~70 ms, once). Call `nav2_kit.warmup()` during
  init to move the one-time first-use JIT off your first real call.
- A NumPy grid is `(H, W)` = `(rows=y, cols=x)`, indexed `grid[y, x]`; a costmap cell
  is `(mx=x, my=y)`; NavFn path coords are `(x, y)` in cells. Same row-major layout as
  a `nav_msgs/OccupancyGrid` (`data[y*W + x]`), so a plan lines up with a published
  grid with no flip.
- Costs are `nav2_costmap_2d` values, exposed as plain ints:
  `nav2_kit.FREE_SPACE` (0), `LETHAL_OBSTACLE` (254),
  `INSCRIBED_INFLATED_OBSTACLE` (253), `MAX_NON_OBSTACLE` (252),
  `NO_INFORMATION` (255).
- **`unsigned char` gotcha:** `costmap.getCost(mx, my)` returns a **1-char Python
  `str`**, not an int (cppyy maps `unsigned char` to `str`). Compare with
  `ord(costmap.getCost(mx, my))`. The bulk `costmap_to_numpy` path avoids this.

---

## Pattern 1 — plan on a synthetic grid  (the minimal path)
*Use for:* global planning where you have (or synthesize) an occupancy grid.

```python
import numpy as np
from rclcppyy.kits import nav2_kit
nav2_kit.bringup_nav2()

grid = np.zeros((200, 200), dtype=np.uint8)          # 0 = free
grid[:, 100] = nav2_kit.LETHAL_OBSTACLE              # a wall down the middle
grid[95:105, 100] = nav2_kit.FREE_SPACE             # ... with a doorway

costmap = nav2_kit.costmap_from_numpy(grid, resolution=0.05, origin=(0.0, 0.0))
path = nav2_kit.plan_navfn(costmap, start=(20, 100), goal=(180, 100))  # (mx,my) cells
if path is not None:
    print(path.shape, path[0], path[-1])             # (N,2) float32, start..goal
```
`plan_navfn` returns `None` when there is no plan. See
`scripts/nav2_kit_demos/d01_plan_grid.py`.

---

## Pattern 2 — use Nav2's own classes directly  (mirror)
*Use for:* anything the two helpers don't cover — the namespaces are the real Nav2.

```python
cns, nns = nav2_kit.bringup_nav2()
cm = cns.Costmap2D(100, 100, 0.05, 0.0, 0.0, 0)      # Nav2's own ctor, verbatim
cm.setCost(10, 20, nav2_kit.LETHAL_OBSTACLE)
print(ord(cm.getCost(10, 20)))                       # 254  (note the ord())
wx, wy = cppyy.gbl.std.ref(...)                      # or: cm.mapToWorld(mx,my,wx,wy)
nav = nns.NavFn(100, 100)                            # the planner algorithm, no node
```
`Costmap2D` and `NavFn` are the real C++ classes; every method
(`getSizeInCellsX`, `mapToWorld`, `worldToMap`, `resizeMap`, …) is available.

---

## Pattern 3 — cells ↔ world coordinates
*Use for:* turning a cell-coordinate plan into metric poses (e.g. a `nav_msgs/Path`).

```python
def cell_to_world(cx, cy, res=0.05, origin=(0.0, 0.0)):
    return origin[0] + (cx + 0.5) * res, origin[1] + (cy + 0.5) * res

path_xy = [cell_to_world(cx, cy) for cx, cy in path]  # path from plan_navfn
```
`Costmap2D.mapToWorld(mx, my, wx, wy)` does the same for integer cells (via C++
output references); the formula above also works for the subpixel path coordinates.

---

## Pattern 4 — publish map + plan to ROS 2 / rviz2  (via rclcppyy)
*Use for:* visualizing or feeding a real ROS 2 graph. Build **C++** messages.

```python
import cppyy
from rclcppyy.bringup_rclcpp import bringup_rclcpp
rclcpp = bringup_rclcpp()
if not rclcpp.ok():
    rclcpp.init()
from nav_msgs.msg import Path                          # Python type registers topic
cppyy.include("nav_msgs/msg/path.hpp")
cppyy.include("geometry_msgs/msg/pose_stamped.hpp")

msg = cppyy.gbl.nav_msgs.msg.Path()                    # the C++ message; poses is a vector
msg.header.frame_id = "map"
for wx, wy in path_xy:
    pose = cppyy.gbl.geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x, pose.pose.position.y = float(wx), float(wy)
    pose.pose.orientation.w = 1.0
    msg.poses.push_back(pose)

node = rclcpp.Node("planner")
node.create_publisher(Path, "plan", 1).publish(msg)
```
`nav_msgs/OccupancyGrid` works the same way (fill `info` + `data`); see the full
showcase `scripts/nav2_kit_demos/d02_own_nav_stack.py` (map + plan + `TwistStamped`).

---

## Pattern 5 — a follow loop (pure pursuit)  (the Python controller half)
*Use for:* driving along a plan. Nav2's RegulatedPurePursuit controller is
lifecycle-coupled (not usable standalone — see the REPORT), so a follow controller is
plain Python. Classic pure pursuit steers by curvature toward a lookahead point:

```python
import math
def pure_pursuit(pose, path_xy, idx, lookahead, max_v, max_w):
    x, y, th = pose
    while idx < len(path_xy) - 1 and math.hypot(path_xy[idx][0]-x, path_xy[idx][1]-y) < lookahead:
        idx += 1
    tx, ty = path_xy[idx]
    dist = math.hypot(tx-x, ty-y)
    alpha = math.atan2(math.sin(a := math.atan2(ty-y, tx-x)-th), math.cos(a))  # wrap
    kappa = 2.0 * math.sin(alpha) / max(dist, 1e-3)
    v = max_v
    return v, max(-max_w, min(max_w, v*kappa)), idx
```
`d02_own_nav_stack.py` wires this to a simulated diff-drive and publishes
`TwistStamped`.

---

## Gotchas (short version)
- **`getCost` returns a 1-char `str`** — use `ord(...)`. Kit constants
  (`nav2_kit.LETHAL_OBSTACLE` …) are plain ints for you.
- **`plan_navfn` needs cell coordinates** `(mx, my)`, returns `(N,2)` float32 cells
  (start→goal), or `None` if unreachable. Convert to world with Pattern 3.
- **NavFn call order matters:** the kit does it for you, but if you drive `NavFn`
  yourself — `setNavArr` **before** `setCostmap` (it resets the cost array), then
  `calcNavFnAstar` **and then** `calcPath` (`calcNavFnAstar` only builds the potential
  field; it does not populate a path).
- **`setCostmap(..., isROS=True)`** rescales ROS cost values into NavFn's internal
  band and adds an obstacle border, exactly like the Nav2 server.
- **Grid orientation:** `(H, W)`, `grid[y, x]`; matches `OccupancyGrid` row-major.
- **Smac and the RPP controller are not surfaced** — lifecycle-coupled (Smac also
  needs OMPL headers). See the REPORT for the evidence.
- **`warmup()` once** during init to move the first-use JIT off your first real call.
