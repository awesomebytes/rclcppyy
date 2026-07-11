# Why ompl_kit — the Open Motion Planning Library from Python via cppyy

`ompl_kit` lets you drive [OMPL](https://ompl.kavrakilab.org) — the standard C++
sampling-based motion-planning library — from Python: the real C++ code owns the
state spaces and runs the planners (RRTConnect, RRT\*, …), while your Python
supplies the problem and, crucially, the **state-validity checker** the planner
calls in its inner loop. It does this against the OMPL that is already installed,
with **no code generation and no build step**.

That last point is the whole story. OMPL *has* official Python bindings — and they
are the cautionary tale this repo exists to answer. This doc shows what cppyy gives
you over both the C++ workflow and OMPL's own bindings. For the API, see
[OMPL_KIT.md](OMPL_KIT.md); for the feasibility evidence, cross-inheritance
mechanics, benchmarks, and gaps, see [REPORT.md](REPORT.md).

---

## The thing OMPL's own bindings make painful

OMPL's Python bindings are generated with **Py++ / pygccxml** (an older
`castxml`-based codegen). In practice that means:

- a **multi-hour build** that parses OMPL's (boost-heavy) headers with `castxml`
  and emits a mountain of C++ wrapper code, then compiles it;
- **~6 GB of RAM** to get through that compile — enough that the docs warn about it
  and it OOMs on modest machines;
- bindings that **lag the C++ releases**, because every API addition needs the
  codegen re-run and the wrappers regenerated (the project is now migrating to
  nanobind precisely because this is unsustainable);
- and, at the end, a **fixed surface**: only what the generator was told to wrap,
  only the specializations it emitted.

So the honest "before" isn't "write C++ and compile it once" — it's "spend an
afternoon and 6 GB building a binding that's already behind." Contrast that with the
cppyy "after": `pixi install -e ompl` and `python your_plan.py`, JIT-including the
installed headers in **~0.5 s** at startup.

---

## Side by side: OMPL's first geometric-planning tutorial, C++ vs Python

On the left, the shape of OMPL's official
[geometric planning tutorial](https://ompl.kavrakilab.org/geometricPlanningSE3.html)
(`RigidBodyPlanning`), verbatim in spirit — **and its build system**. On the right,
the complete runnable file this repo ships,
`scripts/ompl_kit_demos/d01_first_plan.py` (2D, with a circular obstacle so the plan
visibly routes around it). The planning code — `SimpleSetup`,
`setStateValidityChecker`, `setStartAndGoalStates`, `solve`, `getSolutionPath` — is
identical; the Python drops the entire build system and writes the validity checker
as a plain Python function.

### C++ — `plan.cpp` + `CMakeLists.txt` (official tutorial shape)

```cpp
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state) {
  const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
  double x = (*s)[0], y = (*s)[1];
  return (x-0.5)*(x-0.5) + (y-0.5)*(y-0.5) > 0.25*0.25;   // outside the obstacle
}

int main() {
  auto space(std::make_shared<ob::RealVectorStateSpace>(2));
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0.0); bounds.setHigh(1.0);
  space->setBounds(bounds);

  og::SimpleSetup ss(space);
  ss.setStateValidityChecker(isStateValid);

  ob::ScopedState<> start(space), goal(space);
  start[0] = 0.1; start[1] = 0.1;
  goal[0]  = 0.9; goal[1]  = 0.9;
  ss.setStartAndGoalStates(start, goal);
  ss.setPlanner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));

  if (ss.solve(1.0)) { ss.simplifySolution(); ss.getSolutionPath().print(std::cout); }
  return 0;
}
```

```cmake
cmake_minimum_required(VERSION 3.5)
project(plan)
find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})
add_executable(plan plan.cpp)
target_link_libraries(plan ${OMPL_LIBRARIES})
```

…and this does not run yet. You need the `CMakeLists.txt`, a `cmake . && make` to
configure/compile/link against `libompl` (and its boost dependencies), and a rebuild
on every edit — before you can execute the binary. (Or: build the Py++ bindings, per
the section above.)

### Python — `d01_first_plan.py` (ompl_kit, shipped in this repo)

```python
#!/usr/bin/env python
"""2D plan around a circular obstacle; validity checker in Python."""
from rclcppyy.kits import ompl_kit

ob, og = ompl_kit.bringup_ompl()

OBSTACLE = (0.5, 0.5, 0.25)

def is_state_valid(state):                       # the planner's inner-loop callback
    cx, cy, r = OBSTACLE                          # state auto-downcast -> state[i]
    return (state[0]-cx)**2 + (state[1]-cy)**2 > r**2

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
    print(ompl_kit.path_to_list(ss.getSolutionPath(), dim=2))
```

Run it directly: `pixi run -e ompl demo-ompl-plan`. Prints a path whose every
waypoint is >0.25 from the obstacle centre — the plan routes around the circle.

### What we gain (right here, from the comparison above)

- **No compile step, no CMake, no codegen.** The C++ program needs a build; OMPL's
  own Python bindings need a multi-hour, 6 GB codegen build. The ompl_kit file runs
  the instant you invoke it — the only startup cost is a one-time **~0.5 s** cppyy
  bringup (JIT-including the headers + loading `libompl.so`), and only what you touch
  is compiled.
- **The validity checker is just Python.** `is_state_valid` reads like the C++
  `isStateValid`, but it is a Python function the C++ planner calls directly — and
  cppyy **auto-downcasts** the `const State*` so `state[0]`/`state[1]` work without
  the `state->as<...>()` cast. You can put a breakpoint in it, print from it, call
  NumPy from it. (See [REPORT.md](REPORT.md) §3 for what that costs.)
- **Header-following, so it tracks the installed version.** cppyy reads OMPL's own
  headers at runtime, so ompl_kit matches whatever OMPL is installed (1.7.0 here) —
  no hand-maintained binding to fall behind the C++ releases.
- **Same library, full ecosystem.** It is the same `libompl.so`, so every planner
  (`PRM`, `RRT*`, `BIT*`, `KPIECE`, …) and every state space is reachable the moment
  you `cppyy.include` its header — no per-feature binding work.

**What the C++ version buys that this one doesn't.** A compiled binary pays no JIT
at startup and gets full static type-checking; a native validity checker is ~150x
faster per call in a hot loop (see the lowering discussion below).

---

## The headline: subclass an OMPL C++ base *in Python*

The capability that makes OMPL a real test of the playbook — not just another
"library from Python" — is **cross-language inheritance**. OMPL's idiomatic way to
supply a validity checker or an optimization objective is to *subclass* a C++ base
(`ob::StateValidityChecker`, `ob::OptimizationObjective`) and override a virtual.
With cppyy you do that **in Python**:

```python
class CircleChecker(ob.StateValidityChecker):
    def __init__(self, si):
        super().__init__(si)                  # chain to the C++ base constructor
    def isValid(self, state):                 # override the C++ virtual, in Python
        return (state[0]-0.5)**2 + (state[1]-0.5)**2 > 0.25**2

ss.setStateValidityChecker(ob.StateValidityCheckerPtr(CircleChecker(si)))
```

The C++ planner calls this Python `isValid` through the C++ vtable — **170 times in
a trivial solve, and over a million times** in a 1 s RRT\* optimization run (see
[REPORT.md](REPORT.md) §2–3). This is the first kit in the stack to use
cross-inheritance: BT.CPP blocked it (its `tick()` is `final`), but OMPL's virtuals
are plain, so it works. OMPL's own bindings expose this only through the
generated-trampoline build; here it is a plain Python class.

---

## The honest part: Python in a real hot loop, and lowering

A sampling planner calls the validity checker thousands to millions of times per
solve, so this is the first kit where "Python in the loop" is a *hot* loop we can
actually measure. The number is honest and stated plainly in
[REPORT.md](REPORT.md) §3:

| validity checker | ns per call | vs native |
|---|--:|--:|
| Python fn via `callback()` | ~282 ns | ~159x slower |
| Python subclass (cross-inherit) | ~345 ns | ~190x slower |
| native C++ (JIT'd) | ~1.8 ns | 1x |

**~150–190x slower per call.** For a trivial plan (a few hundred checks) that is
~40 µs — lost entirely in the planner's own ~9 ms of overhead, so all three variants
solve in the same time. It only bites when validity *dominates* (the million-call
RRT\* case: ~0.35 s of pure boundary overhead).

The point isn't to hide this — it's that cppyy gives you the **lowering path** in
the same script: prototype the checker in Python (edit-run in seconds), and when a
planner hammers it, lower that one function to a native C++ `StateValidityChecker`
(a ~6-line `cppyy.cppdef`) — same OMPL calls, same structure, 150x faster inner
loop. Prototype in Python, lower the hot leaf to C++. `bench_ompl_validity.py`
measures all three side by side (`pixi run -e ompl bench-ompl`).

---

## Two ways to use it

### Mode A — plan from Python, checker in Python
Prototype motion planning with the validity/cost logic in Python: put obstacles,
clearance, and heuristics in a plain function or subclass, iterate in seconds
against the real OMPL planners. This is `d01_first_plan.py`. Good for experimenting
with problem setups and planners where edit-run cycles matter more than raw solve
throughput.

### Mode B — plan from Python, publish to ROS 2
`scripts/ompl_kit_demos/d02_publish_path.py` plans a 2D path and publishes it as a
**C++ `nav_msgs/Path`** on a real topic via rclcppyy — directly consumable by RViz
or a navigation stack. The path message is a C++ message end to end; Python only
fills the handful of waypoint poses. A tiny showcase that OMPL planning and the ROS
2 middleware share one process with no glue between them.

---

## Advantages of the cppyy approach

Grounded in the spike's measured numbers (see [REPORT.md](REPORT.md)):

- **No codegen, no 6 GB build.** `python x.py` is the whole workflow; contrast
  OMPL's Py++ bindings (multi-hour, ~6 GB RAM, lagging) and the C++ tutorial's
  `find_package(ompl)` + `cmake && make`. Bringup is a one-time ~0.5 s JIT.
- **Header-following, tracks the installed version.** No hand-maintained binding to
  drift behind the C++ releases.
- **Cross-language inheritance for free.** Subclass an OMPL C++ base in Python and
  the planner calls your override — the capability OMPL's own bindings make you
  rebuild trampolines for.
- **A prototype-to-native lowering path.** As with bt_kit / pcl_kit, this is the L0
  rung: prototype with cppyy JIT today; lower hot checkers to compiled C++ when the
  solve demands it — same OMPL calls, same structure. (Freeze/L2 are the research
  direction — see PLAN.md.)

---

## Limits

ompl_kit is a v0 spike. Validity/cost checkers cross the boundary one state at a
time (no batching yet), only `RealVectorStateSpace` auto-downcasts to `state[i]`
(compound spaces need `as_state`), only RRTConnect/RRTstar headers are pre-included
(other planners are one `cppyy.include` away), only geometric planning is surfaced
(not `ompl::control`), and the global RNG can't be re-seeded mid-process (reproducible
runs use a fresh process). The full, honest list is in [REPORT.md](REPORT.md) §5.
