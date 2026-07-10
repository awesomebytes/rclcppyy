# Why bt_kit — BehaviorTree.CPP from Python via cppyy

`bt_kit` lets you build and run [BehaviorTree.CPP](https://www.behaviortree.dev)
v4 trees from Python: the real C++ engine parses the XML, owns the tree, and
ticks it, while the leaf nodes are ordinary Python functions. There is no
official Python binding for BehaviorTree.CPP (py_trees is a separate, incompatible
library), so this capability does not otherwise exist — and because it is the
*same* C++ library reading the *same* XML, everything in the BT.CPP ecosystem
(Groot2, the Nav2 behavior trees, plugins) stays compatible.

This doc explains what that gives you over the C++ workflow, and the two distinct
ways to use it. For the API, see [KIT.md](KIT.md); for the feasibility evidence,
gaps, and benchmarks, see [REPORT.md](REPORT.md).

---

## Side by side: tutorial 1 in C++ vs in Python

Below is the official ["first tree" tutorial](https://www.behaviortree.dev/docs/tutorial-basics/tutorial_01_first_tree)
(abridged from behaviortree.dev — comments trimmed), then the complete bt_kit
equivalent that this repo ships and runs (`scripts/bt_kit_demos/t01_first_tree.py`).

### C++ (the official tutorial)

```cpp
#include "behaviortree_cpp/bt_factory.h"
using namespace BT;

// A custom SyncActionNode, created by inheritance.
class ApproachObject : public BT::SyncActionNode {
public:
  ApproachObject(const std::string& name) : BT::SyncActionNode(name, {}) {}
  BT::NodeStatus tick() override {
    std::cout << "ApproachObject: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

// A plain function used as a condition.
BT::NodeStatus CheckBattery() {
  std::cout << "[ Battery: OK ]" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

// Methods of an existing class, wrapped as actions.
class GripperInterface {
public:
  NodeStatus open()  { std::cout << "GripperInterface::open"  << std::endl; return NodeStatus::SUCCESS; }
  NodeStatus close() { std::cout << "GripperInterface::close" << std::endl; return NodeStatus::SUCCESS; }
};

int main() {
  BehaviorTreeFactory factory;
  factory.registerNodeType<ApproachObject>("ApproachObject");
  factory.registerSimpleCondition("CheckBattery", [&](TreeNode&){ return CheckBattery(); });
  GripperInterface gripper;
  factory.registerSimpleAction("OpenGripper",  [&](TreeNode&){ return gripper.open();  });
  factory.registerSimpleAction("CloseGripper", [&](TreeNode&){ return gripper.close(); });
  auto tree = factory.createTreeFromFile("./my_tree.xml");
  tree.tickWhileRunning();
  return 0;
}
```
…plus a `CMakeLists.txt` (`find_package(behaviortree_cpp)`, `add_executable`,
`target_link_libraries`) and a build step — `cmake . && make` (or `colcon build`)
— that produces a binary before anything runs.

### Python (bt_kit, shipped as t01_first_tree.py — 24 lines, no build)

```python
from rclcppyy.kits import bt_kit
bt = bt_kit.bringup_bt()

def check_battery(node):   print("[ Battery: OK ]");             return bt.NodeStatus.SUCCESS
def open_gripper(node):    print("GripperInterface::open");      return bt.NodeStatus.SUCCESS
def approach_object(node): print("ApproachObject: approach_object"); return bt.NodeStatus.SUCCESS
def close_gripper(node):   print("GripperInterface::close");     return bt.NodeStatus.SUCCESS

factory = bt.BehaviorTreeFactory()
factory.registerSimpleCondition("CheckBattery", check_battery)
factory.registerSimpleAction("OpenGripper", open_gripper)
factory.registerSimpleAction("ApproachObject", approach_object)
factory.registerSimpleAction("CloseGripper", close_gripper)
tree = factory.createTreeFromText(XML)   # same XML the C++ version loads
tree.tickWhileRunning()
```

Same factory, same method names, same XML, same output — but it runs the instant
you type `python t01_first_tree.py`: no CMakeLists, no compile, no link step. The
kit brings the C++ library up under cppyy in ~0.85 s (once) and JIT-compiles only
what you touch.

**What the C++ version buys that this one doesn't.** `registerNodeType<ApproachObject>`
uses a *compile-time* C++ type, which gives static type checking and a full node
*manifest* — the metadata Groot2 uses to populate its editor palette and that
BT.CPP writes into a `TreeNodesModel`. From Python you register via functors /
`register_stateful` instead (a Python class can't be a C++ template argument), so
Python-defined nodes don't yet contribute a manifest, and custom control/decorator
node *types* still have to be written in C++. See [REPORT.md](REPORT.md) §5 for the
full list.

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

bt_kit is a v0 spike. String-only ports, one Python instance per stateful node
ID, Python leaves hold the GIL (so no true parallelism for `ParallelNode`),
custom node *types* and Groot manifests need C++, and a few raw cppyy operations
segfault (which is exactly why the kit keeps cppyy behind a curated surface). The
full, honest list — with the reasons and what an agent hits next — is in
[REPORT.md](REPORT.md) §5.
