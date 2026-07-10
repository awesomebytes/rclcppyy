#!/usr/bin/env python
"""
Micro-benchmark for bt_kit: how fast can a small tree be ticked, and what does
crossing into Python for each leaf cost?

Fixed tree: a Sequence of 3 leaves that each return SUCCESS immediately (no
sleeping / waiting). One "tick" = one full traversal of the tree.

Variants:
  (a) C++ JIT leaves   -- leaves are C++ functors; the engine + leaves run at
                          C++ speed (only the per-tick tickOnce call is Python).
  (b) Python leaves    -- leaves are Python callables through bt_kit; measures
                          the Python<->C++ boundary cost per leaf.
  (c) pure-Python BT   -- a hand-rolled Python sequence executor, i.e. no C++
                          engine at all. (py_trees, the natural contrast, is not
                          packaged for robostack-jazzy/conda-forge, so this
                          stands in for "what you'd write without the kit".)

JIT/bringup time is reported separately and excluded from the measured window.

Run: pixi run -e bt bench-bt
"""
import time

from rclcppyy.kits import bt_kit

DURATION = 2.0  # seconds per measured variant
LEAVES = 3

XML = """
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <Leaf/>
      <Leaf/>
      <Leaf/>
    </Sequence>
  </BehaviorTree>
</root>
"""


def run_for(tick_once, seconds):
    """Call tick_once() in a tight loop for `seconds`; return (ticks, elapsed)."""
    ticks = 0
    start = time.perf_counter()
    deadline = start + seconds
    while time.perf_counter() < deadline:
        tick_once()
        ticks += 1
    return ticks, time.perf_counter() - start


def build_cpp_tree(cppyy, BT):
    """Variant (a): a factory whose single leaf is a C++ functor."""
    cppyy.cppdef(r"""
    namespace btbench {
      inline void registerFast(BT::BehaviorTreeFactory& f) {
        f.registerSimpleAction("Leaf",
          [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; });
      }
    }
    """)
    factory = BT.BehaviorTreeFactory()
    cppyy.gbl.btbench.registerFast(factory)
    return factory.createTreeFromText(XML), factory


def build_kit_tree():
    """Variant (b): a leaf implemented in Python through the kit."""
    @bt_kit.action_node("Leaf")
    def leaf(bb):
        return bt_kit.SUCCESS
    return bt_kit.tree_from_xml(XML)


def build_pure_python():
    """Variant (c): no C++ engine -- a Python list of leaves ticked in order."""
    def leaf():
        return bt_kit.SUCCESS

    leaves = [leaf] * LEAVES

    def tick_once():
        for node in leaves:
            if node() != bt_kit.SUCCESS:
                return bt_kit.FAILURE
        return bt_kit.SUCCESS

    return tick_once


def report(label, ticks, elapsed):
    tps = ticks / elapsed
    us_per_tick = 1e6 * elapsed / ticks
    print(f"  {label:<34} {tps:>14,.0f} ticks/s   {us_per_tick:>10.3f} us/tick")
    return tps


def main():
    import cppyy  # noqa: F401  (imported lazily; bringup pulls it in)

    t0 = time.perf_counter()
    BT = bt_kit.bringup_bt()
    bringup_s = time.perf_counter() - t0
    print(f"bringup_bt (JIT include + load + cppdef): {bringup_s:.3f} s\n")

    cpp_tree, _factory = build_cpp_tree(cppyy, BT)
    kit_tree = build_kit_tree()
    py_tick = build_pure_python()

    # Warm each once (first tick may lazily resolve things).
    cpp_tree.tickOnce()
    kit_tree.tickOnce()
    py_tick()

    print(f"Ticking a Sequence of {LEAVES} immediate-SUCCESS leaves, "
          f"{DURATION:.0f} s per variant:\n")
    a = report("(a) C++ JIT leaves", *run_for(cpp_tree.tickOnce, DURATION))
    b = report("(b) Python leaves via bt_kit", *run_for(kit_tree.tickOnce, DURATION))
    c = report("(c) pure-Python sequence", *run_for(py_tick, DURATION))

    print(f"\n  C++ leaves are {a / b:.1f}x faster than Python leaves through the kit.")
    print(f"  The C++ engine with Python leaves is {b / c:.2f}x the pure-Python loop.")


if __name__ == "__main__":
    main()
