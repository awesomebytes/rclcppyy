# Why bt_kit — BehaviorTree.CPP from Python via cppyy

`bt_kit` lets you build and run [BehaviorTree.CPP](https://www.behaviortree.dev)
v4 trees from Python: the real C++ engine parses the XML, owns the tree, and
ticks it, while the leaf nodes are ordinary Python functions. There is no
official Python binding for BehaviorTree.CPP (py_trees is a separate, incompatible
library), so this capability does not otherwise exist — and because it is the
*same* C++ library reading the *same* XML, everything in the BT.CPP ecosystem
(Groot2, the Nav2 behavior trees, plugins) stays compatible.

This doc explains what that gives you over the C++ workflow, and the two distinct
ways to use it. For the API, see [BT.CPP_KIT.md](BT.CPP_KIT.md); for the
feasibility evidence, gaps, and benchmarks, see [REPORT.md](REPORT.md).

---

## Side by side: the complete tutorial-1 program, C++ vs Python

These are the **complete programs**, not fragments. On the left, the official
["first tree" tutorial](https://www.behaviortree.dev/docs/tutorial-basics/tutorial_01_first_tree)
from behaviortree.dev. On the right, the complete runnable file this repo ships,
`scripts/bt_kit_demos/t01_first_tree.py`. Read them together: with the thin
C++-mirror API, the Python is almost a line-for-line transliteration of the C++
`main()` — minus the build system and the class boilerplate.

### C++ — `first_tree.cpp` (official tutorial)

```cpp
#include "behaviortree_cpp/bt_factory.h"
using namespace BT;

// A custom SyncActionNode, created by inheritance (the recommended way).
class ApproachObject : public BT::SyncActionNode
{
public:
  ApproachObject(const std::string& name) : BT::SyncActionNode(name, {}) {}

  // You must override the virtual function tick().
  BT::NodeStatus tick() override
  {
    std::cout << "ApproachObject: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

// A plain function used as a condition.
BT::NodeStatus CheckBattery()
{
  std::cout << "[ Battery: OK ]" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

// Methods of an existing class, wrapped as actions.
class GripperInterface
{
public:
  GripperInterface() : _open(true) {}

  NodeStatus open()
  {
    _open = true;
    std::cout << "GripperInterface::open" << std::endl;
    return NodeStatus::SUCCESS;
  }
  NodeStatus close()
  {
    std::cout << "GripperInterface::close" << std::endl;
    _open = false;
    return NodeStatus::SUCCESS;
  }
private:
  bool _open;
};

static const char* xml_text = R"(
 <root BTCPP_format="4">
   <BehaviorTree ID="MainTree">
     <Sequence name="root_sequence">
       <CheckBattery   name="check_battery"/>
       <OpenGripper    name="open_gripper"/>
       <ApproachObject name="approach_object"/>
       <CloseGripper   name="close_gripper"/>
     </Sequence>
   </BehaviorTree>
 </root>
 )";

int main()
{
  BehaviorTreeFactory factory;

  factory.registerNodeType<ApproachObject>("ApproachObject");
  factory.registerSimpleCondition("CheckBattery", [&](TreeNode&) { return CheckBattery(); });

  GripperInterface gripper;
  factory.registerSimpleAction("OpenGripper",  [&](TreeNode&) { return gripper.open();  });
  factory.registerSimpleAction("CloseGripper", [&](TreeNode&) { return gripper.close(); });

  auto tree = factory.createTreeFromText(xml_text);
  tree.tickWhileRunning();
  return 0;
}
```

…and this does not run yet. It needs a `CMakeLists.txt`
(`find_package(behaviortree_cpp REQUIRED)`, `add_executable(first_tree ...)`,
`target_link_libraries(first_tree BT::behaviortree_cpp)`) and a build —
`colcon build` (or `cmake . && make`) — to compile and link a binary before you
can execute it.

### Python — `t01_first_tree.py` (bt_kit, shipped in this repo)

```python
#!/usr/bin/env python
"""
BehaviorTree.CPP official tutorial 1 ("Your first behavior tree"), in Python via
rclcppyy's bt_kit. This mirrors the C++ tutorial line-for-line -- same factory,
same registerSimpleAction / registerSimpleCondition, same createTreeFromText /
tickWhileRunning -- only the leaf callbacks are Python.
"""
from rclcppyy.kits import bt_kit

bt = bt_kit.bringup_bt()

XML = """
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence name="root_sequence">
      <CheckBattery   name="check_battery"/>
      <OpenGripper    name="open_gripper"/>
      <ApproachObject name="approach_object"/>
      <CloseGripper   name="close_gripper"/>
    </Sequence>
  </BehaviorTree>
</root>
"""


def check_battery(node):
    print("[ Battery: OK ]")
    return bt.NodeStatus.SUCCESS


def open_gripper(node):
    print("GripperInterface::open")
    return bt.NodeStatus.SUCCESS


def approach_object(node):
    print("ApproachObject: approach_object")
    return bt.NodeStatus.SUCCESS


def close_gripper(node):
    print("GripperInterface::close")
    return bt.NodeStatus.SUCCESS


def main():
    factory = bt.BehaviorTreeFactory()
    factory.registerSimpleCondition("CheckBattery", check_battery)
    factory.registerSimpleAction("OpenGripper", open_gripper)
    factory.registerSimpleAction("ApproachObject", approach_object)
    factory.registerSimpleAction("CloseGripper", close_gripper)

    tree = factory.createTreeFromText(XML)
    tree.tickWhileRunning()


if __name__ == "__main__":
    main()
```

Run it directly: `pixi run -e bt demo-bt-t01`. Same output as the C++ program
(`[ Battery: OK ] / GripperInterface::open / ApproachObject: approach_object /
GripperInterface::close`).

### What we gain (right here, from the comparison above)

- **No compile step.** The C++ program needs a CMakeLists and a `colcon build`
  before it can run; the Python file runs the instant you invoke it. The only
  startup cost is a one-time **~0.85 s** cppyy bringup (JIT-including the header +
  loading the `.so`), and only what you touch is JIT-compiled.
- **No wrapper, no codegen.** Nothing is generated. `factory`,
  `registerSimpleAction`, `registerSimpleCondition`, `createTreeFromText`,
  `tickWhileRunning` are the library's own names — the Python reads like the C++.
- **Same XML, same engine.** The tree text is identical and is parsed and ticked
  by the same `libbehaviortree_cpp.so`, so Groot2 / Nav2 ecosystem compatibility
  is unchanged.
- **Mixed C++/Python leaves in one tree.** You can keep some leaves in Python and
  drop others to C++ (JIT'd, or existing libraries) in the *same* tree — see
  Mode B and `t03_mixed_tree.py` below.

**What the C++ version buys that this one doesn't.**
`registerNodeType<ApproachObject>` uses a *compile-time* C++ type, which gives
static type checking and a full node *manifest* — the metadata Groot2 uses to
populate its editor palette and that BT.CPP writes into a `TreeNodesModel`. From
Python you register via functors / `register_stateful` instead (a Python class
can't be a C++ template argument), so Python-defined nodes don't yet contribute a
manifest, and custom control/decorator node *types* still have to be written in
C++. See [REPORT.md](REPORT.md) §5 for the full list.

---

## Two ways to use it

### Mode A — use BehaviorTree.CPP from Python
Prototype whole trees, with the leaves in Python, at Python speed of iteration.
This is the capability that simply doesn't exist otherwise: a maintained C++ BT
engine, driven entirely from a Python script, with the official XML. Tutorials
1 and 2 (`t01_first_tree.py`, `t02_ports.py`) are this mode. Good for bringing up
a tree, wiring behavior logic, experimenting with structure, and testing — where
edit-run cycles in seconds matter more than raw tick throughput.

### Mode B — orchestrate existing C++ from a Python-driven tree
Because cppyy can call *any* C++ in the environment, a leaf can drive real,
already-installed C++ software while Python only wires the tree together. Leaves
of different languages coexist in one tree: some Python, some JIT'd C++, some
calling into existing shared libraries. `scripts/bt_kit_demos/t03_mixed_tree.py`
demonstrates the full spectrum in a single `Sequence`:

1. `CheckSensors` — a **Python** leaf;
2. `ComputePlan` — a **JIT-compiled C++** functor (no Python on that tick);
3. `PublishStatus` — a leaf that drives **existing C++**: it publishes a ROS 2
   `std_msgs/String` through **rclcpp** (via rclcppyy) from inside the tick.

That third leaf is the repo's thesis in one line: *a C++ behavior-tree engine,
orchestrated from Python, calling into a real installed C++ stack (rclcpp), with
no wrapper code generated and no build step.* The demo runs a subscriber in the
same process and confirms all three messages actually flowed (`RESULT: OK`). This
is how you reuse existing robotics C++ — controllers, planners, drivers — but
compose and sequence it from Python, which is far faster to change than a
recompiled C++ tree.

---

## Advantages of the cppyy approach

Grounded in the spike's measured numbers (see [REPORT.md](REPORT.md) §3–4):

- **No code generation, no wrapper build.** Nothing is generated or compiled
  ahead of time. Contrast the C++ tutorial's `cmake && make`; here `python x.py`
  is the whole workflow. Bringup is a one-time ~0.85 s JIT.
- **Header-following, so it tracks the installed version.** cppyy reads the
  library's own headers at runtime, so bt_kit automatically matches whatever
  BehaviorTree.CPP is installed (4.9.0 here) — no hand-maintained binding to drift
  out of sync when the library updates.
- **On-demand template instantiation.** Templated members like
  `getInput<std::string>` are instantiated the moment you use them, from Python —
  no need to pre-declare which specializations a binding exposes.
- **Same XML + full ecosystem compatibility.** The trees are ordinary BT.CPP XML,
  so they open in Groot2 and interoperate with the Nav2 / ROS behavior-tree
  ecosystem unchanged. You are not forking the format or the engine.
- **Mixed-language leaves in one tree.** Python, JIT'd C++, and calls into
  existing `.so`s coexist in a single tree (Mode B). You pay a language boundary
  only where you actually put a Python leaf — ~0.3 µs per leaf-tick (~2× a C++
  leaf), i.e. ~630k Python-leaf ticks/s, far above any robot control rate.
- **A prototype-to-native lowering path.** The kit is the L0 rung of a
  progressive-lowering idea: **L0** — prototype with cppyy JIT (what runs today:
  Python leaves, JIT'd, ~0.85 s bringup); **L1** — *freeze*: ship a precompiled
  cppyy dictionary (`.pcm`/`.so`) so startup pays no JIT; **L2** — *lower*: emit
  native C++ for the hot leaves (or the whole tree) compiled ahead of time — same
  XML, same structure, the Python leaves become C++. You prototype in Python and
  progressively harden toward native without rewriting the tree. (L1/L2 are the
  research direction, not yet built — see PLAN.md's Experiments section.)

---

## Limits

A 2026-07-11 deep pass closed most of the v0 limits: typed ports
(int/float/bool/vectors), per-node stateful instances, loggers + Groot2 publishing
+ a tick observer, readable XML errors, and subtrees/scripting all work now (see
[REPORT.md](REPORT.md) §5, with a skip-safe test suite). What remains: Python
leaves run under the GIL on the tree thread (so `ParallelNode` gives no true
parallelism — fine for orchestration), custom node *types* (control/decorator)
still need C++, directioned and struct/JSON ports aren't modelled, and startup
pays a one-time ~0.85 s header JIT (the AOT "freeze" needs a Cling C++ module, not
just a dictionary — §5 Gap 8). Raw cppyy still has segfault edges, which is exactly
why the kit keeps it behind a curated surface.
