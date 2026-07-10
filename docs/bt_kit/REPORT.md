# bt_kit spike — driving BehaviorTree.CPP v4 from Python via cppyy

**Date:** 2026-07-10 · **Env:** pixi `bt` (robostack-jazzy + conda-forge),
`ros-jazzy-behaviortree-cpp 4.9.0`, `cppyy 3.5.0`, Python 3.12.13, linux-64.
**Question:** can the official BehaviorTree.CPP tutorials be written in *short
Python* and executed by the *C++ tree engine*, with minimal glue and no official
binding (none exists — py_trees is a separate, incompatible library)?

**Verdict: YES, with bounded caveats. GO** for continuing to invest in the kit
strategy, provided the kit hides cppyy from the user (several raw cppyy
operations segfault the process). Details below.

---

## 1. Possible at all? — capability probe matrix

Each capability was probed in isolation from the `bt` env against the installed
4.9.0 headers/library.

| # | Capability | Possible? | How / why |
|---|---|:--:|---|
| 1 | Basic tree: built-in nodes, `createTreeFromText`, `tickWhileRunning` | **YES** | `BT::BehaviorTreeFactory` constructs, parses XML, ticks. Clean. |
| 2 | Python action leaf via `registerSimpleAction` | **YES** | Python callable wrapped in `std::function<NodeStatus(TreeNode&)>`, pinned alive. Ticks and returns status correctly. |
| 3 | Ports + blackboard: `InputPort`/`OutputPort`, `getInput<T>`/`setOutput<T>`, `{bb}` remap | **YES** | Template member calls `node.getInput['std::string'](key)` / `setOutput` work. **But** the `PortsList` (`unordered_map<string,PortInfo>`) must be built in a **C++ helper** — constructing/inserting it from Python **segfaults** cppyy's `MapFromPairs`. |
| 4a | Cross-inheritance: Python class deriving `BT::StatefulActionNode` | **NO** | cppyy's Python-override dispatcher regenerates *all* virtuals, but `StatefulActionNode::tick()`/`halt()` are `final` → `TypeError: no python-side overrides supported (failed to compile the dispatcher code)`. |
| 4b | Stateful/async via a JIT'd C++ shim holding `std::function` hooks | **YES** | `cppyy.cppdef` a `StatefulActionNode` subclass with `std::function` slots for `onStart/onRunning/onHalted`; the builder lambda lives entirely in C++ so the `unique_ptr` never crosses into Python; only the hooks cross. Multi-tick RUNNING→SUCCESS works. |

**One hard failure (4a), one workaround-required (3), everything else clean.**
The 4a failure is the important structural one — see Gaps.

### Fragility notes (things that worked but felt sharp)
- Building the `PortsList` map in Python **crashes the interpreter** (SIGSEGV,
  no Python traceback). The fix — build it in a one-line C++ helper — is
  reliable, but an agent using raw cppyy would hit an un-debuggable crash.
- Returning `std::unique_ptr<TreeNode>` *from a Python* `std::function` builder
  fails (`C++ type cannot be converted to memory`). Keeping the builder lambda
  in C++ (Python only supplies the `std::function` hooks) sidesteps it.
- Keep-alive is mandatory: unpinned functors → `callable was deleted` at tick
  time. The kit pins them on the returned tree.

---

## 2. Glue cost + bringup / JIT time

| Metric | Value |
|---|---|
| Kit module `rclcppyy/kits/bt_kit.py` | 288 lines total (215 code), of which a **~40-line embedded C++ helper** (`cppdef`) — so ≈ 175 lines of Python glue |
| JIT `cppyy.include("behaviortree_cpp/bt_factory.h")` | **~0.86 s** (one-time) |
| Full `bringup_bt()` (include + `load_library` + `cppdef` glue) | **~0.88 s** (one-time, idempotent) |
| Per-tree registration (decorated nodes → factory) | negligible (µs) |

Bringup is ~3x faster than the rclcpp bringup (~2.5 s) because BT.CPP's headers
are far smaller than `rclcpp/rclcpp.hpp`.

### User-facing demo size (the "short Python" evidence)
Official tutorials, XML verbatim, leaves in Python. LOC excludes the XML string,
comments, docstrings, blank lines.

| Demo | User Python LOC | What it exercises |
|---|:--:|---|
| `t01_first_tree.py` | **22** | 4 leaves (1 condition + 3 actions), Sequence, tick |
| `t02_ports.py` | **14** | input port read, output port write, `{blackboard}` roundtrip |

(Both counts include ~5 lines of `main()`/entry-point boilerplate; the leaf
definitions themselves are the bulk.)

Verified output:
```
# t01
[ Battery: OK ]
GripperInterface::open
ApproachObject: approach_object
GripperInterface::close
# t02
Robot says: hello world
Robot says: The answer is 42
```

---

## 3. Runtime metrics

Fixed tree: `Sequence` of 3 leaves each returning SUCCESS immediately. One tick =
one full traversal. 2 s warm window per variant, JIT/bringup excluded. One run on
this machine (indicative, not statistically rigorous):

| Variant | ticks/s | µs/tick |
|---|--:|--:|
| (a) C++ JIT leaves (engine + leaves at C++ speed) | ~1,250,000 | ~0.80 |
| (b) Python leaves through bt_kit | ~615,000 | ~1.63 |
| (c) pure-Python sequence loop (no C++ engine) | ~7,600,000 | ~0.13 |

**Reading these numbers honestly:**
- Crossing into Python per leaf costs **~2x** vs C++ leaves (0.8 → 1.6 µs/tick
  for 3 leaves; ~0.3 µs of boundary cost per leaf). Cheap for orchestration.
- The C++ engine is **~10x slower than a trivial 3-item Python loop** for this
  degenerate tree. That is expected and is the key insight: **the C++ engine is
  not a speed play** for tiny trees — its per-tick cost (node traversal, status
  propagation, blackboard) dwarfs a bare loop. Its value is the *engine*
  (reactive/parallel control nodes, decorators, XML authoring, logging, Groot),
  not raw tick throughput.
- (c) is a **floor**, not a fair py_trees stand-in: py_trees (a real pure-Python
  BT with tree/blackboard semantics) carries its own traversal overhead and
  would land far below this trivial loop — plausibly near or below (b). py_trees
  is **not packaged** for robostack-jazzy/conda-forge (`pixi search` finds
  nothing), so the apples-to-apples contrast was dropped; (c) stands in as
  "what you'd hand-write without the kit."

At ~615k ticks/s, Python-leaf trees tick far faster than any real robot control
rate (typically 10–1000 Hz), so the boundary cost is a non-issue for the intended
use.

---

## 4. GAPS — what an LLM-agent user hits next

1. **`registerNodeType<T>()` is impossible for a Python `T`.** BT.CPP's primary
   extension point is a compile-time template over your C++ node class. A Python
   class can't be that `T`. **Consequence:** every custom node must go through
   `registerSimpleAction`/`registerSimpleCondition` (functors) or a JIT'd C++
   shim. Custom **control nodes and decorators** authored in Python are not
   reachable this way — you'd JIT them in C++.
2. **Python cross-inheritance from any class with `final` virtuals fails**
   (probe 4a). `StatefulActionNode`, and likely others, mark `tick()`/`halt()`
   `final`; cppyy's override dispatcher can't regenerate them. The **only** route
   to stateful/async nodes from Python is the C++ shim + `std::function` hooks
   (which the kit provides as `@stateful_action`).
3. **cppyy container construction segfaults.** Building `PortsList`
   (`unordered_map<string,PortInfo>`) in Python crashed the process. The kit
   keeps all map/pair work in a C++ helper. An agent must **never** be handed raw
   cppyy for this — segfaults give no Python traceback.
4. **Typed ports beyond string.** v0 models only string, bidirectional ports.
   Int/double/enum/struct/JSON ports need per-type `getInput<T>`/`setOutput<T>`
   and BT's type registration (`RegisterJsonDefinition`), none wired yet.
5. **GIL vs parallel/reactive nodes.** Python leaves hold the GIL. A
   `ParallelNode` ticking Python leaves gets no real concurrency; a
   `ThreadedAction` implemented in Python would contend/deadlock on the GIL.
   Fine for sequential/reactive orchestration; not for CPU-parallel leaves.
6. **Multi-instance stateful nodes share one Python object.** The kit binds a
   single Python instance per registered stateful *ID*; two `<CountTo>` nodes in
   one tree share state. Per-node instances need the builder to construct a fresh
   Python object per node (doable, deferred).
7. **Groot2 / `TreeNodesModel` / loggers not exposed.** Python-registered nodes
   do carry manifests (so `writeTreeNodesModelXML` *should* work), but no Groot2
   publisher, file logger, or `TreeNodesModel` export is wired. An agent wanting
   live monitoring hits a wall.
8. **XML validation errors are C++-flavored.** Malformed XML / unknown node IDs
   surface as `BT::RuntimeError` translated by cppyy into a Python exception —
   usable, but the messages read as C++ and line info is coarse.
9. **Keep-alive discipline.** Unpinned Python functors are collected →
   `callable was deleted` at tick. The kit hides this (pins on the tree), but any
   manual cppyy use reintroduces it.

---

## 5. Recommendation — GO (curated kit, not raw cppyy)

The core hypothesis is **proven**: official BT.CPP tutorials run **verbatim XML**
on the **C++ engine** with leaves in **14–22 lines of Python**, ~0.9 s bringup,
correct output — and there is no competing official Python binding, so this is a
genuine "impossible → possible" result. Stateful/async, the riskiest probe, also
works via the C++-shim escape hatch.

The gaps are real but bounded and mostly about *breadth* (typed ports, control
nodes, Groot) rather than *feasibility*. The single most important finding for
strategy: **cppyy must stay behind the kit.** The segfault-prone container
handling and the `registerNodeType<T>`/`final`-virtual limitations mean an LLM
agent pointed at raw cppyy would produce process crashes with no traceback. A
curated `bt_kit` surface (decorators + `tree_from_xml` + `tick_*`, C++ kept in
vetted `cppdef` helpers) removes every sharp edge encountered here and keeps the
user-facing code short. That is exactly the "kit" thesis, and it holds.

**Next investments, in priority order:** (a) typed ports; (b) per-node stateful
instances; (c) Python-authored control/decorator nodes via generated C++ shims;
(d) Groot2/logger passthrough; (e) precompiled dictionary to drop the ~0.9 s JIT.
