# cppyy_kit — common patterns for driving a C++ library from Python

This is the shared playbook behind rclcppyy's "kits" (`bt_kit` for
BehaviorTree.CPP, `pcl_kit` for the Point Cloud Library). A kit wraps a C++
library so it can be driven from short Python that **mirrors the library's own
API**, hiding only the cppyy friction. That friction is the same from one library
to the next, so it is factored into `rclcppyy/kits/cppyy_kit.py`; this document is
the narrative and the evidence behind it, for the next kit author (human or LLM).

Two independent kits confirm every pattern below: BehaviorTree.CPP (a callback /
tree engine) and PCL (templated bulk-data algorithms) stress different edges, and
the union is what a general `cppyy_kit` must cover.

## The three-ingredient recipe

Every kit is the same three moves:

1. **Bringup** — locate the install, add include paths, JIT-include the headers,
   and `load_library` the `.so` set.
2. **Hide that library's cppyy sharp edges** — the traps below (containers,
   ownership, lifetime, template attributes).
3. **Mirror the library's own API** — expose the real class/method names so a
   user's (or an LLM's training on the) library transfers 1:1; no DSL, no hidden
   state.

`bringup_bt()` and `bringup_pcl()` are the same shape; the leaf/algorithm calls
after bringup are the library's own (`factory.registerSimpleAction`,
`pcl.VoxelGrid[pcl.PointXYZ]`).

---

## Pattern catalog

### 1. Bringup: `load_library` is mandatory; `add_library_path` is not enough
cppyy resolves a symbol by finding its **owning `.so` at call time** by scanning
its own library search path. Adding a path is not enough — every library you call
into must be `load_library`'d by soname. `cppyy_kit.load_libraries(sonames,
search_paths)` centralizes this.
- **bt:** one lib — `libbehaviortree_cpp.so`.
- **pcl:** a set — `libpcl_common/octree/kdtree/search/sample_consensus/filters`
  (filters pulls the rest transitively at runtime).
- Do not rely on `LD_LIBRARY_PATH`; an installed package has no activation hook,
  and cppyy uses its own search path for call-time resolution.

### 2. Bringup cost & staging: gate the expensive includes
Bringup time is dominated by the **header JIT-parse**. Split it into cheap and
expensive stages and let the caller skip what they don't need.
- **bt:** ~0.85 s, ~89% of it the single `cppyy.include("bt_factory.h")`; logger
  headers (which pull zmq/flatbuffers) are included lazily only when an
  observability helper is first called.
- **pcl:** core ~1.3 s; adding the ROS message headers (`pcl_conversions`) costs
  another ~1.9 s, gated behind `bringup_pcl(with_ros=False)`.

### 3. Lifetime: pin Python objects against C++ (`keep_alive`)
cppyy does **not** keep a Python callable (nor its `std::function` wrapper, nor a
buffer backing a view) alive just because C++ holds it. A collected callback
raises `TypeError: callable was deleted` when fired. `cppyy_kit.keep_alive(owner,
*objs)` pins them on a long-lived owner.
- **bt:** leaf functors + their `std::function`s are pinned on the factory and
  carried onto the tree (which owns the nodes that hold them).
- **pcl:** the source cloud is pinned on the ctypes buffer backing a zero-copy
  NumPy view, so the view can't outlive its storage silently.
- This bit us in our own test: a throwaway `lambda` handed to `std_function` was
  collected before the call — bind the callable to a name / pin it.

### 4. Crossing functions **into** C++ (`std_function`)
Wrap a Python callable as `std::function<sig>` (`cppyy_kit.std_function`). The
callback runs in **whatever C++ thread invokes it**; cppyy takes the GIL. For a
single-threaded driver (a tick loop, a `spin_some`) there is no contention.
- **bt:** leaf ticks, condition checks, and the stateful hooks are all Python
  callables wrapped this way.
- Return values convert (int/enum/void/value types). See the ownership trap next.

### 5. Crossing objects **out** without crossing ownership (`HandleRegistry`)
Returning a `std::unique_ptr<T>` **from** a Python `std::function` fails
(`C++ type cannot be converted to memory`). To let C++ create per-instance Python
state, keep the ownership-creating lambda entirely in C++ and have it call a
Python **builder that returns an integer handle**; dispatch later callbacks by
that handle. `cppyy_kit.HandleRegistry` is the table.
- **bt:** the per-tree-node stateful builder — C++ builds each node, calls the
  Python builder (→ handle), and the shim's onStart/onRunning/onHalted dispatch by
  handle. This is what makes two nodes of the same registered ID keep independent
  state.

### 6. Containers & bulk data: build in C++, pass raw addresses
Constructing/inserting STL containers **from Python** can **SIGSEGV with no
traceback** (cppyy's `MapFromPairs` on a map, aligned-storage construction). Keep
all container/buffer work inside a `cppyy.cppdef` helper.
- **bt:** the `PortsList` (`unordered_map<string,PortInfo>`) is built in C++ from
  two **parallel `vector<string>`** (names, types) — passing a `vector<pair>` or
  the map itself from Python is what crashes.
- **pcl:** NumPy↔cloud copies are a `cppdef` helper taking `reinterpret_cast`-able
  integer addresses (`arr.ctypes.data` as `uintptr_t`) and doing the
  `memcpy`/strided copy in C++ — a per-element Python loop is ~90x slower.

### 7. On-demand templates: include the `impl` headers
A precompiled `.so` only carries the specializations its authors compiled. To let
Cling instantiate `Template<UserType>` at JIT time, include the library's
`impl/*.hpp` (or `*.hxx`). This is the difference between a fixed-surface binding
and cppyy — **any** type works on demand.
- **pcl:** including `pcl/impl/pcl_base.hpp` + `filters/impl/voxel_grid.hpp` lets
  `PointCloud<T>` / `VoxelGrid<T>` instantiate for point types no binding shipped.
- **bt:** template **member** calls work directly — `node.getInput[T](key)` from
  Python; unwrap the returned `Expected<T>` with `has_value()`/`value()`
  (`cppyy_kit.unwrap_expected`).

### 8. Call C++ function templates directly; let cppyy deduce
When a template argument can be deduced from a runtime argument, call the function
straight from Python — no explicit `[T]`, no wrapper.
- **pcl:** `pcl.toROSMsg(cloud, msg)` / `pcl.fromROSMsg(msg, cloud)` deduce
  `PointT` from the cloud. Reach for a `cppdef` helper only when a template arg
  can't be deduced or ownership must not cross (Pattern 5).

### 9. Cling is an older clang: attributes & failed-`cppdef` crashes
Cling's parser trips on some modern constructs, and a **failed `cppdef` can crash
during transaction revert** (no Python traceback).
- **pcl:** a trailing type-attribute (`struct { ... } EIGEN_ALIGN16;`) parse-errors
  — use a prefix form (`struct alignas(16) X`). Custom point types must be declared
  that way.
- **Mitigation:** probe risky glue out-of-process first —
  `cppyy_kit.probe_cppdef(code, include_paths=, headers=, libraries=)` compiles it
  in a throwaway subprocess and returns `(ok, message)` without risking the main
  interpreter.

### 10. Error ergonomics: strip the signature wall
cppyy prefixes a C++ exception with the mangled call signature and ` => `. Split
on ` => ` and collapse whitespace for a readable one-line message, re-raised as a
kit exception (`cppyy_kit.pretty_cpp_error`, `CppyyKitError`).
- **bt:** `BtXmlError` turns the `createTreeFromText(...) =>` wall into
  `RuntimeError: Error at line 4: -> Node not recognized: Nope`.

### 11. Enums compare equal to ints
C++ enums behave like their int values across the boundary
(`BT.NodeStatus.SUCCESS == 2`). Expose plain-int constants for convenience while
keeping the real enum available (`bt_kit.SUCCESS` and `bt.NodeStatus.SUCCESS`).

### 12. Mirror, don't sugar
Patch/return the library's real classes so methods keep their C++ names (add
snake_case aliases). A bespoke DSL was prototyped for bt_kit and **rejected**: it
needed a module-global registry (a footgun across trees/re-imports/tests) and
forced knowledge that doesn't transfer. Both kits ship the mirror.

### 13. GIL / concurrency (what "parallel" means)
Kit callbacks run in the **calling C++ thread**. A single-threaded engine (a tick
loop, a spin) never contends.
- **bt:** `ParallelNode` is cooperative bookkeeping, not OS threads — Python leaves
  under it run sequentially (no true parallelism, no contention). A leaf that
  sleeps / does I/O releases the GIL, so spinning the tree from a background thread
  does not deadlock; a busy-blocking leaf would. Don't expose `ThreadedAction`
  (real C++ worker thread) without explicit GIL handling.

---

## Today vs L1 ("freeze")

**Today (L0, JIT):** everything above runs by JIT-compiling the library's headers
at bringup — a one-time, idempotent per-process cost (bt ~0.85 s, pcl ~1.3 s).
Correct and fast at steady state; the only downside is startup latency.

**L1 (freeze) — not yet done, and the ROOT-dictionary route does NOT get you
there.** The bringup cost is ~89% header JIT-parse. A `rootcling`/`genreflex`
dictionary (`+ cppyy.load_reflection_info`) builds and loads in ~0.02 s, but it
supplies **reflection/autoload metadata, not a precompiled AST** — Cling still
lazily parses the header on first class use (measured: the first
`BehaviorTreeFactory()` after loading the dict still cost ~0.8 s). Skipping the
parse needs Cling to consume a **precompiled header / C++ module** for the header
set (a `module.modulemap` + `-fmodules` build, or a Cling PCH, version-matched to
cppyy-cling), which is the mechanism cppyy already uses for its own std PCH. That
is a dedicated sub-project — the real L1 task — not a quick win.

**What a kit should do now:** make bringup idempotent and staged; treat the JIT as
a one-time startup cost; keep the AOT path in mind (don't bake in anything that
would block a future module build) but don't hand-roll a ROOT dictionary expecting
a speedup.

---

*Evidence lives in the per-kit reports: `docs/bt_kit/REPORT.md` (capability matrix,
deep-pass verdicts, AOT probe) and `docs/pcl_kit/REPORT.md` (copy accounting,
showcase benchmark). This document is the merged, library-independent layer.*
