# bt_kit — cheat sheet for a coding agent

You are writing Python that drives **BehaviorTree.CPP v4** (a C++ behavior-tree
engine) through `rclcppyy.kits.bt_kit`. The kit **mirrors the C++ API** — the same
`BehaviorTreeFactory`, `registerSimpleAction`, `createTreeFromText`,
`tickWhileRunning` you know from the official BT.CPP tutorials — so write it the way
you'd write the C++ tutorial, with the leaf callbacks in Python. You do **not** need
to know cppyy; the kit removes that friction.

**Requires** the `bt` pixi env: `pixi run -e bt python your_script.py`.

**Golden rules**
- Call `bt = bt_kit.bringup_bt()` once; it returns the `BT` namespace.
- A leaf is `def fn(node): ...; return status`. Status is `bt.NodeStatus.SUCCESS`
  (exactly like C++) or the shortcut `bt_kit.SUCCESS` — also `FAILURE`, `RUNNING`.
  Returning `None` means SUCCESS; a `bool` maps True→SUCCESS, False→FAILURE.
- The **XML is the official BT.CPP XML, verbatim.** XML node tags must match the
  names you register.
- Ports are **string-typed** in v0; read/write them via the `node` argument.
- Keep the `tree` (and the `factory`) referenced while you tick.

---

## Pattern 1 — actions/conditions, build a tree, tick it  (tutorial 1)
*Use for:* the common case — synchronous leaves that act and return a status.

```python
from rclcppyy.kits import bt_kit
bt = bt_kit.bringup_bt()

def approach(node):
    print("approaching")
    return bt.NodeStatus.SUCCESS

def check_battery(node):
    return bt.NodeStatus.SUCCESS              # a condition is just a leaf too

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

factory = bt.BehaviorTreeFactory()
factory.registerSimpleCondition("CheckBattery", check_battery)
factory.registerSimpleAction("ApproachObject", approach)
tree = factory.createTreeFromText(XML)        # inline text OR a path to an .xml file
status = tree.tickWhileRunning()              # ticks until root stops RUNNING
assert status == bt_kit.SUCCESS
```
(`registerSimpleAction` / `register_simple_action` and `createTreeFromText` /
`create_tree_from_text` both exist — camelCase mirrors C++, snake_case is there if
you prefer it.)

---

## Pattern 2 — ports and the blackboard  (tutorial 2)
*Use for:* passing values into a leaf from the XML, or between leaves via a
`{blackboard}` entry.

```python
def say(node):
    print(node.get_input("message"))          # read an input port -> str
    return bt_kit.SUCCESS                      # C++: node.getInput<std::string>("message")

def think(node):
    node.set_output("text", "The answer is 42")  # write an output port
    return bt_kit.SUCCESS

factory.registerSimpleAction("SaySomething", say, ports=["message"])
factory.registerSimpleAction("ThinkWhatToSay", think, ports=["text"])
```
```xml
<Sequence>
  <SaySomething   message="hello world"/>       <!-- literal in -->
  <ThinkWhatToSay text="{the_answer}"/>          <!-- writes blackboard key -->
  <SaySomething   message="{the_answer}"/>        <!-- reads it back -->
</Sequence>
```
`node.get_input(key)` returns a `str` (or `None` if unset); `node.set_output(key,
value)` writes `str(value)`. `node["key"]` / `node["key"] = v` and the camelCase
`getInput`/`setOutput` work too. Ports are bidirectional — the same declared name
serves both reading and writing.

---

## Pattern 3 — asynchronous / stateful leaf (returns RUNNING over many ticks)
*Use for:* a long-running action not done in one tick (navigation, waiting,
counting). A plain action may **not** return RUNNING — use `register_stateful` with
a class exposing the C++ StatefulActionNode hooks.

```python
class CountTo:
    def onStart(self, node):                   # called once when the node starts
        self.target = int(node.get_input("count") or 3)
        self.n = 0
        return bt.NodeStatus.RUNNING
    def onRunning(self, node):                 # called every subsequent tick
        self.n += 1
        return bt.NodeStatus.SUCCESS if self.n >= self.target else bt.NodeStatus.RUNNING
    def onHalted(self, node):                  # optional: called if interrupted
        pass

factory.register_stateful("CountTo", CountTo, ports=["count"])
```
(`onStart`/`onRunning`/`onHalted` mirror C++; snake_case `on_start`/... also work.)
Caveat: v0 uses **one instance per registered ID** — two `<CountTo>` nodes in one
tree share state. Give them distinct IDs if you need independent state.

---

## Pattern 4 — a fast leaf at C++ speed (no Python per tick)
*Use for:* a hot leaf that must not pay the Python boundary cost (~0.3 µs/tick),
e.g. a tight inner check ticked millions of times. Write it in C++ and JIT it.
(Python leaves are plenty fast — ~630k ticks/s — so reach for this only if
profiling says so.)

```python
import cppyy
bt = bt_kit.bringup_bt()

cppyy.cppdef(r"""
namespace mykit {
  inline void registerFast(BT::BehaviorTreeFactory& f) {
    f.registerSimpleAction("FastCheck",
      [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; });
  }
}
""")
factory = bt.BehaviorTreeFactory()
cppyy.gbl.mykit.registerFast(factory)          # C++ registration, bypasses the kit
tree = factory.createTreeFromText(XML)
tree.tickWhileRunning()                         # returns a BT::NodeStatus
```

---

## Gotchas (short version)
- **Don't** subclass BT C++ node classes in Python (`class X(BT.StatefulActionNode)`)
  — fails to compile (`final` virtuals). Use `register_stateful` (Pattern 3).
- **Don't** build a `PortsList`/`std::map` in Python — it **segfaults** the
  process. Use `ports=[...]`, or do container work inside `cppyy.cppdef` C++.
- Keep the `tree` (and `factory`) referenced while ticking; dropping them can free
  the callbacks.
- `tree.tickWhileRunning()` returns a `BT::NodeStatus`; it compares equal to the
  `bt_kit.SUCCESS` / `FAILURE` / `RUNNING` ints and to `bt.NodeStatus.*`.
