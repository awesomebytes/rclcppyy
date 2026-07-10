# bt_kit — cheat sheet for a coding agent

You are writing Python that drives **BehaviorTree.CPP v4** (a C++ behavior-tree
engine) through `rclcppyy.kits.bt_kit`. You do **not** need to know cppyy — the
kit hides it. Write the leaf logic in Python; the C++ engine parses the XML,
owns the tree, and ticks it.

**Requires** the `bt` pixi env: `pixi run -e bt python your_script.py`
(the env provides `ros-jazzy-behaviortree-cpp`).

**Golden rules**
- Node **status** is `bt_kit.SUCCESS` / `bt_kit.FAILURE` / `bt_kit.RUNNING`
  (also `IDLE`, `SKIPPED`). Return one of these from a leaf. Returning `None`
  means SUCCESS; returning a `bool` maps True→SUCCESS, False→FAILURE.
- The **XML is the official BT.CPP XML, verbatim.** Node names in the XML must
  match the `name` you register.
- Ports are **string-typed** in v0. Read/write them through the `bb` argument.
- Hold onto the `tree` object while you tick it (it pins the Python callbacks).

---

## Pattern 1 — define an action leaf, build a tree, tick it
*Use for:* the common case — synchronous leaves that do something and return a
status. This is tutorial 1.

```python
from rclcppyy.kits import bt_kit

@bt_kit.action_node("ApproachObject")      # name must match the XML tag
def approach(bb):
    print("approaching")
    return bt_kit.SUCCESS

@bt_kit.condition_node("CheckBattery")      # a condition is just a leaf too
def check_battery(bb):
    return bt_kit.SUCCESS                    # or a bool

XML = """
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <CheckBattery/>
      <ApproachObject/>
    </Sequence>
  </BehaviorTree>
</root>
"""

tree = bt_kit.tree_from_xml(XML)            # inline text OR a path to an .xml file
status = bt_kit.tick_while_running(tree)    # ticks until root stops RUNNING
assert status == bt_kit.SUCCESS
```

---

## Pattern 2 — ports and the blackboard
*Use for:* passing values into a leaf from the XML, or between leaves via a
`{blackboard}` entry. This is tutorial 2.

```python
@bt_kit.action_node("SaySomething", ports=["message"])   # declare port names
def say(bb):
    print(bb.get("message"))                 # read an input port (str)
    return bt_kit.SUCCESS

@bt_kit.action_node("ThinkWhatToSay", ports=["text"])
def think(bb):
    bb.set("text", "The answer is 42")        # write an output port
    return bt_kit.SUCCESS
```
```xml
<Sequence>
  <SaySomething   message="hello world"/>     <!-- literal in -->
  <ThinkWhatToSay text="{the_answer}"/>        <!-- writes blackboard key -->
  <SaySomething   message="{the_answer}"/>     <!-- reads it back -->
</Sequence>
```
`bb.get(key)` returns a `str` (or `None` if unset); `bb.set(key, value)` writes
`str(value)`. `bb["key"]` / `bb["key"] = v` are sugar. Ports are bidirectional,
so the same declared name works for both reading and writing.

---

## Pattern 3 — asynchronous / stateful leaf (returns RUNNING over many ticks)
*Use for:* a long-running action that isn't done in one tick (navigation,
waiting, counting). A plain `@action_node` may **not** return RUNNING (it's a
synchronous node); use `@stateful_action` on a *class*.

```python
@bt_kit.stateful_action("CountTo", ports=["count"])
class CountTo:
    def on_start(self, bb):                   # called once when the node starts
        self.target = int(bb.get("count") or 3)
        self.n = 0
        return bt_kit.RUNNING
    def on_running(self, bb):                 # called on every subsequent tick
        self.n += 1
        return bt_kit.SUCCESS if self.n >= self.target else bt_kit.RUNNING
    def on_halted(self, bb):                  # optional: called if interrupted
        pass
```
Caveat: v0 uses **one instance per registered ID** — two `<CountTo>` nodes in one
tree share state. Give them distinct IDs if you need independent state.

---

## Pattern 4 — a fast leaf at C++ speed (no Python per tick)
*Use for:* a hot leaf that must not pay the Python boundary cost (~0.3 µs/tick),
e.g. a tight inner check ticked millions of times. Write the leaf in C++ and JIT
it. (Python leaves are plenty fast — ~600k ticks/s — so reach for this only when
profiling says so.)

```python
import cppyy
BT = bt_kit.bringup_bt()                      # ensures headers/lib are loaded

cppyy.cppdef(r"""
namespace mykit {
  inline void registerFast(BT::BehaviorTreeFactory& f) {
    f.registerSimpleAction("FastCheck",
      [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; });
  }
}
""")
factory = BT.BehaviorTreeFactory()
cppyy.gbl.mykit.registerFast(factory)
tree = factory.createTreeFromText(XML)        # bypasses the kit registry
tree.tickWhileRunning()                       # returns a BT::NodeStatus
```
Rule of thumb: keep any **STL container construction** (maps, port lists) and any
**node subclassing** inside `cppyy.cppdef` C++ — doing those from Python can
**segfault** the process with no traceback.

---

## Gotchas (short version)
- **Don't** subclass BT C++ node classes in Python (`class X(BT.StatefulActionNode)`)
  — it fails to compile (`final` virtuals). Use `@stateful_action` (Pattern 3),
  which uses a vetted C++ shim under the hood.
- **Don't** build a `PortsList`/`std::map` in Python — segfault. Use `ports=[...]`.
- Keep the `tree` referenced while ticking; dropping it can free the callbacks.
- One tick of `tick_while_running` / `tick_once` returns a status int; compare it
  to `bt_kit.SUCCESS` etc.
