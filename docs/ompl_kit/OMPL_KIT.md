# ompl_kit — cheat sheet for a coding agent

You are writing Python that drives the **Open Motion Planning Library (OMPL)** — a
C++ sampling-based motion planner — through `rclcppyy.kits.ompl_kit`. The kit
**mirrors OMPL's C++ API**: `bringup_ompl()` returns the real `ompl::base` /
`ompl::geometric` namespaces (the conventional `ob` / `og`) and you use
`ob.RealVectorStateSpace`, `og.SimpleSetup`, `og.RRTConnect`,
`setStateValidityChecker`, `setStartAndGoalStates`, `solve`, `getSolutionPath`
exactly as in the OMPL C++ tutorials. The kit only removes the cppyy friction
(bringup, the validity `std::function` signature, the `as` keyword, RNG seeding,
path extraction). You do **not** need to know cppyy.

(For *why* this exists and the C++-vs-Python comparison, see [WHY.md](WHY.md); for
the cross-inheritance mechanics and benchmarks, see [REPORT.md](REPORT.md).)

**Requires** the `ompl` pixi env: `pixi run -e ompl python your_script.py`.

**Golden rules**
- Call `ob, og = ompl_kit.bringup_ompl()` once; it returns the base and geometric
  namespaces. Pass `with_geometric=False` if you only need state spaces (skips the
  ~60 ms planner JIT). Bringup is idempotent (~0.5 s, once).
- Wrap a raw space/planner/checker in the library `Ptr` to hand it to OMPL:
  `ob.StateSpacePtr(space)`, `ob.PlannerPtr(planner)`,
  `ob.StateValidityCheckerPtr(checker)`. The wrap **transfers ownership** — safe, no
  double-free.
- The validity checker is the planner's inner-loop callback. Two ways: a Python
  function via `ompl_kit.validity_checker(fn)`, or a Python subclass of
  `ob.StateValidityChecker`. Both are shown below.
- Inside a validity/cost callback, cppyy **auto-downcasts** the state, so
  `state[0]` / `state[1]` read a `RealVectorStateSpace`'s coordinates directly.
- **Pin** any Python subclass instance you hand to C++ (`owner=` /
  `cppyy_kit.keep_alive`), or it is collected and the next call raises "callable was
  deleted".
- Seed with `ompl_kit.set_seed(n)` **before** solving; the global RNG can't be
  re-seeded mid-process (use a fresh process per seed).

---

## Pattern 1 — a 2D plan, validity checker as a Python function  (the minimal path)
*Use for:* planning where the validity/obstacle logic is a plain function.

```python
from rclcppyy.kits import ompl_kit
ob, og = ompl_kit.bringup_ompl()

def is_state_valid(state):                       # planner's inner-loop callback
    return (state[0]-0.5)**2 + (state[1]-0.5)**2 > 0.25**2   # outside a circle

space = ob.RealVectorStateSpace(2)               # OMPL's own API, verbatim
bounds = ob.RealVectorBounds(2)
bounds.setLow(0.0); bounds.setHigh(1.0)
space.setBounds(bounds)

ss = og.SimpleSetup(ob.StateSpacePtr(space))
ss.setStateValidityChecker(ompl_kit.validity_checker(is_state_valid, owner=ss))

start = ob.ScopedState[ob.RealVectorStateSpace](ss.getStateSpace())
start[0], start[1] = 0.1, 0.1
goal = ob.ScopedState[ob.RealVectorStateSpace](ss.getStateSpace())
goal[0], goal[1] = 0.9, 0.9
ss.setStartAndGoalStates(start, goal)
ss.setPlanner(ob.PlannerPtr(og.RRTConnect(ss.getSpaceInformation())))

if ss.solve(1.0):
    ss.simplifySolution()
    print(ompl_kit.path_to_list(ss.getSolutionPath(), dim=2))   # [(x,y), ...]
```
`validity_checker(fn, owner=ss)` wraps `fn` as OMPL's
`std::function<bool(const State*)>` and pins it on `ss`. See
`scripts/ompl_kit_demos/d01_first_plan.py`.

---

## Pattern 2 — validity checker as a Python subclass  (cross-inheritance)
*Use for:* an OMPL-idiomatic checker, or one that holds state / queries a map.

```python
from rclcppyy.kits import ompl_kit, cppyy_kit
ob, og = ompl_kit.bringup_ompl()

class CircleChecker(ob.StateValidityChecker):
    def __init__(self, si):
        super().__init__(si)                     # REQUIRED: chain the C++ base ctor
        self.calls = 0
    def isValid(self, state):                    # override the C++ virtual by name
        self.calls += 1
        return (state[0]-0.5)**2 + (state[1]-0.5)**2 > 0.25**2

# ... build space + ss as in Pattern 1 ...
checker = CircleChecker(ss.getSpaceInformation())
cppyy_kit.keep_alive(ss, checker)                # PIN it (footgun otherwise)
ss.setStateValidityChecker(ob.StateValidityCheckerPtr(checker))
```
The C++ planner calls the Python `isValid` through the vtable. Works because
`isValid` is a *plain* virtual. Cost: ~345 ns/call (~190x a native checker) — fine
until validity dominates, then lower it (Pattern 5).

---

## Pattern 3 — optimal planning with a Python OptimizationObjective  (RRT\*)
*Use for:* minimizing a custom cost (path length, clearance, energy) with RRT\* /
an optimizing planner. Same cross-inheritance shape, two virtuals.

```python
class PathLength(ob.OptimizationObjective):
    def __init__(self, si):
        super().__init__(si); self.si = si
    def stateCost(self, s):    return ob.Cost(1.0)
    def motionCost(self, s1, s2): return ob.Cost(self.si.distance(s1, s2))

obj = PathLength(si)
cppyy_kit.keep_alive(ss, obj)
ss.setOptimizationObjective(ob.OptimizationObjectivePtr(obj))
ss.setPlanner(ob.PlannerPtr(og.RRTstar(si)))
ss.solve(1.0)                                    # motionCost called ~1M times/s
```

---

## Pattern 4 — a compound state space (SE2/SE3)  (explicit downcast)
*Use for:* poses with orientation. The auto-downcast covers `RealVectorStateSpace`;
for a compound space, cast explicitly with `as_state` (Python can't write `.as`).

```python
def is_valid(state):
    se2 = ompl_kit.as_state(state, ob.SE2StateSpace.StateType)
    return in_free_space(se2.getX(), se2.getY(), se2.getYaw())
```
`bringup_ompl` pre-includes SE2/SE3 headers, so `ob.SE2StateSpace` is ready.

---

## Pattern 5 — lower the hot checker to native C++  (when validity dominates)
*Use for:* a planner that calls the checker millions of times (RRT\*, long solves).
Prototype in Python (Patterns 1–3), then lower that one function to C++ — same OMPL
calls, ~150x faster inner loop.

```python
import cppyy
from rclcppyy.kits import ompl_kit, cppyy_kit
ob, og = ompl_kit.bringup_ompl()

cppyy.cppdef(r"""
class CppCircleChecker : public ompl::base::StateValidityChecker {
public:
  CppCircleChecker(const ompl::base::SpaceInformationPtr& si)
    : ompl::base::StateValidityChecker(si) {}
  bool isValid(const ompl::base::State* s) const override {
    const auto* rv = s->as<ompl::base::RealVectorStateSpace::StateType>();
    double x = (*rv)[0], y = (*rv)[1];
    return (x-0.5)*(x-0.5) + (y-0.5)*(y-0.5) > 0.25*0.25;   // never leaves C++
  }
};
""")
checker = cppyy.gbl.CppCircleChecker(ss.getSpaceInformation())
cppyy_kit.keep_alive(ss, checker)
ss.setStateValidityChecker(ob.StateValidityCheckerPtr(checker))
```
`bench_ompl_validity.py` (`pixi run -e ompl bench-ompl`) measures Python vs this.

---

## Pattern 6 — publish a plan as a ROS 2 nav_msgs/Path
*Use for:* feeding a plan to RViz / a navigation stack. Build the **C++** message.

```python
import os; os.environ.setdefault("ROS_DOMAIN_ID", "45")
import cppyy
from rclcppyy.bringup_rclcpp import bringup_rclcpp, add_ros2_include_paths
from rclcppyy.kits import ompl_kit

waypoints = [...]                                # from ompl_kit.path_to_list(...)
rclcpp = bringup_rclcpp()
if not rclcpp.ok(): rclcpp.init()
from nav_msgs.msg import Path                     # Python type registers the topic

add_ros2_include_paths()
cppyy.include("nav_msgs/msg/path.hpp"); cppyy.include("geometry_msgs/msg/pose_stamped.hpp")
msg = cppyy.gbl.nav_msgs.msg.Path()              # the C++ message; poses is a vector
msg.header.frame_id = "map"
for x, y in waypoints:
    pose = cppyy.gbl.geometry_msgs.msg.PoseStamped()
    pose.pose.position.x, pose.pose.position.y = float(x), float(y)
    pose.pose.orientation.w = 1.0
    msg.poses.push_back(pose)

node = rclcpp.Node("planner")
node.create_publisher(Path, "plan", 10).publish(msg)
```
See `scripts/ompl_kit_demos/d02_publish_path.py`.

---

## Gotchas (short version)
- **Pin** any Python subclass instance (checker/objective) you hand to C++, or the
  next dispatch raises `TypeError: callable was deleted`. `validity_checker(owner=)`
  does it for you; a raw subclass is yours to `keep_alive`.
- **`validity_checker` needs no hint** — the kit fixes the `bool(const State*)`
  signature. Don't rely on `cppyy_kit.callback`'s inference here: it would infer a
  `State&` reference, which won't bind to `setStateValidityChecker`.
- Inside a callback, `state[0]`/`state[1]` work for `RealVectorStateSpace`
  (auto-downcast); a compound space needs `ompl_kit.as_state(state, ...StateType)`.
- **Seed before solving** with `ompl_kit.set_seed(n)`; a seed set after the first
  sample is ignored (OMPL warns). Re-seeding in one process is unreliable — use a
  fresh process per reproducible run.
- Wrap raws in the library `Ptr` (`ob.StateSpacePtr`, `ob.PlannerPtr`) to pass them
  to OMPL; the wrap transfers ownership (no double-free).
- `path_to_list(path, dim)` needs the space dimension (the path doesn't carry it),
  and reads real-vector coordinates; read compound waypoints via `as_state`.
- Only RRTConnect/RRTstar are pre-included; any other planner is one
  `cppyy.include("ompl/geometric/planners/.../X.h")` away, then `og.X`.
