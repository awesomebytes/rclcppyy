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

### 3. Crossing a Python function **into** C++ (`callback`)
Hand a Python callable to C++ in one line — `cppyy_kit.callback(fn)` — with the
signature inferred and the lifetime pinned for you:
```python
def on_value(x: int, y: float) -> bool:      # hints -> "bool(int, double)"
    return x > y
fn = cppyy_kit.callback(on_value)            # ready to pass to any C++ std::function slot
```
- **Inference** maps `int`->int, `float`->double, `bool`->bool, `str`->std::string,
  `None`->void (return), and any cppyy C++ class (via `__cpp_name__`) as a
  reference. Parameters with Python defaults / `*args` are ignored (not C++ args).
- **Explicit wins** for reference/const/pointer forms or types inference can't
  name: `cppyy_kit.callback(tick, signature="BT::NodeStatus(BT::TreeNode&)",
  owner=factory)`. This is exactly how bt_kit registers leaf/stateful hooks.
- **Threading:** the callback runs in whatever C++ thread invokes it (cppyy takes
  the GIL); a single-threaded driver (a tick loop, a `spin_some`) never contends.
- `cppyy_kit.std_function(sig, fn)` is the low-level escape hatch (raw wrapper, you
  handle lifetime yourself); prefer `callback`.

### 4. Lifetime: the "callable was deleted" footgun (now handled)
cppyy does **not** keep a Python callable (nor its `std::function` wrapper, nor a
buffer backing a view) alive just because C++ holds it — a collected callback
raises `TypeError: callable was deleted` when fired. This bit us for real: a
throwaway `lambda` handed to the raw `std_function` was collected before the call.
- **`callback()` makes it impossible to hit silently:** it always pins. With
  `owner=` the wrapper + fn live as long as that object; without `owner=` they are
  pinned in a module-level registry for the process lifetime
  (`cppyy_kit.release_callbacks()` drops those when you're sure C++ is done).
- For **non-callback** objects (a buffer backing a zero-copy view, a logger),
  `cppyy_kit.keep_alive(owner, *objs)` is the primitive. **pcl:** the source cloud
  is pinned on the ctypes buffer backing a NumPy view so it can't outlive its
  storage. **bt:** leaf callbacks are pinned on the factory (via `callback(owner=)`)
  and carried onto the tree.

### C++ → Python direction (no helper needed)
The reverse crossing is already one line, so it is documented, not wrapped
(mirror-don't-sugar):
```python
cppyy.gbl.mylib.some_fn(21)          # a cppdef'd/loaded C++ function IS a Python callable
holder.store(cppyy.gbl.mylib.some_fn)  # and can be handed straight back into a C++ API
```
Round-trip works too: a Python `callback()` stored in a C++ `std::function`,
invoked from C++, can call back into further Python — verified in
`test/test_cppyy_kit.py`.

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

### 14. Teardown: release global-state C++ objects before Python finalizes
A cppyy process ends by running two teardown mechanisms with **no ordering
contract** between them: Python finalization (which drops the last references to
cppyy-proxied C++ objects, running their destructors) and cppyy's own atexit hook
(which tears down Cling / the JIT). A C++ object that owns **process-global or
static state** — an `rclcpp` Context and the DDS participant / background threads
it owns, a ZMQ-backed BT logger — is the hazard: if its destructor runs after
Cling is gone (or a DDS thread touches freed state), the process can **SIGSEGV
with no Python traceback**, *after* all useful work is done.
- **History / evidence:** rclcppyy scripts long papered over this with
  `os._exit(0)` right after printing their results — a hard exit skips every
  destructor, so the return code is deterministic but nothing is actually cleaned
  up. A root-cause pass on the current stack (cppyy 3.5, ROS Jazzy, cyclonedds
  *and* fastrtps) could **not reproduce a crash** in ~8 scenarios — plain
  pub/sub, the bt+rclcpp mixed tree, the pcl+rclcpp pipeline, module-global entity
  lifetimes, and single/double explicit `rclcpp::shutdown()` — so the dodges were
  vestigial (present since each file's first commit). The hazard is nonetheless
  real in principle, so the fix makes teardown **ordered and explicit** instead of
  relying on the accident that end-of-`main` locals get RAII-released while the
  interpreter is still healthy.
- **Fix:** `cppyy_kit.register_teardown(cb)` + `cppyy_kit.shutdown()` — a LIFO,
  idempotent, best-effort registry, wired to `atexit`. atexit runs after `main`
  returns but **before** module globals are cleared and before cppyy's
  (earlier-registered, therefore later-running) Cling teardown — the correct
  window, with both Python and cppyy still healthy. rclcppyy registers
  `shutdown_rclcpp()`, a guarded once-only `rclcpp::shutdown()`; the guard also
  closes the historical **double-`rcl_shutdown`** race (`rclcpp` installs its own
  SIGINT/SIGTERM handler that can call shutdown a second time). Kits with no
  process-global C++ state (bt, pcl) register nothing: their objects are
  per-instance and RAII-released on scope exit, and the JIT'd namespaces are
  Cling's to tear down. `test/test_clean_exit.py` is the regression tripwire.

### 15. First-use JIT: make it visible, move it with `warmup()`
The first time a given C++ signature is crossed, cppyy JIT-compiles a call wrapper
for it. It is a one-time, **per-signature** cost — and a big one at a kit's entry
points (bt_kit's first `registerSimpleAction` ~0.4 s to codegen the
`std::function<NodeStatus(TreeNode&)>` thunk + the register call wrapper; the first
pcl NumPy→VoxelGrid→NumPy frame ~0.45 s). A **freeze/PCH does not remove it** (the
PCH is an AST, this is call-wrapper codegen triggered by the Python call), and
`-O0`/`-O1` make no difference (it is Clang front-end instantiation, not LLVM
optimisation). So a script's *first live call* stalls unexpectedly.

Two moves, both in `cppyy_kit`:
- **Make it visible.** Wrap a known-expensive kit entry point in
  `with cppyy_kit.first_use(label, warmup_hint):`. On the first call that exceeds a
  threshold it prints a one-time, LLM-actionable line to stderr — e.g. *"bt_kit.
  register_simple_action JIT-compiled a call wrapper on first use (408 ms). Call
  bt_kit.warmup() once during init… Silence: RCLCPPYY_JIT_NOTICE=0."* Thereafter
  (and when disabled or warming) it is a bare passthrough — zero overhead.
- **Move it.** A kit's `warmup()` exercises its expensive signatures on throwaway
  objects (under `cppyy_kit.suppress_first_use_notice()`, via the
  `cppyy_kit.warmup(*thunks)` building block) so the wrappers are JIT'd and cached
  process-globally during init. Measured: bt_kit `time-to-first-tick` 678 → 98 ms
  (the spike moves into a ~0.9 s init); pcl showcase frame-0 630 → 4 ms.

*Scope choice:* instrument the **kit-owned entry points** (registration, bringup),
not every cppyy call (too broad, adds overhead) nor a generic opt-in context (can't
name the API/warmup in the notice). This is reliable where the kit owns the entry
point (bt registration); for kits that mirror a raw algorithm API whose first-use
cost is *inside* un-wrapped library calls (pcl's VoxelGrid), the notice is
best-effort and `warmup()` is the primary tool. `warmup()` stays per-kit (only the
kit knows what to exercise); the notice/suppress/runner are shared.

---

## Today vs L1 ("freeze") — L1 now WORKS

**Today (L0, JIT):** everything above runs by JIT-compiling the library's headers
at bringup — a one-time, idempotent per-process cost (bt ~0.9 s, pcl ~1.3 s).
Correct and fast at steady state; the only downside is startup latency.

**L1 (freeze) — DONE for bt_kit; the mechanism is a Cling PCH, not a dictionary.**
The full recipe, artifact lifecycle, numbers and limitations live in
[FREEZE.md](FREEZE.md); the short version:

- The bringup cost is ~89 % header JIT-parse. A `rootcling`/`genreflex` dictionary
  does **not** help — it supplies reflection/autoload metadata, not a parsed AST,
  so Cling still lazily re-parses on first class use (measured ~0.8 s). *(This was
  the prior probe's dead end.)*
- What works is the mechanism **cppyy already uses for its own std headers:** a
  **Cling precompiled header**. Build a PCH that bakes the kit's headers on top of
  cppyy's std set (`rootcling -generate-pch`, reusing `etc/dictpch/makepch.py`'s
  command), and point `CLING_STANDARD_PCH` at it. Cling materialises the header AST
  from the PCH at interpreter start, so `cppyy.include(...)` becomes a ~6 ms lookup
  instead of a ~0.9 s parse. **Measured: `include(bt_factory.h)` ~890 ms → ~6 ms
  (~140×); bringup total ~950 ms → ~90 ms (~10.7×).** Same 16-test suite green on
  the frozen path (`pixi run -e bt test-bt-frozen`).
- **Two rules make it real.** (1) `CLING_STANDARD_PCH` must be set *before the
  first `import cppyy`* (Cling binds its PCH at interpreter init; `import rclcppyy`
  imports cppyy transitively), so activation is via a launcher that sets the env
  and `exec`s the target (`scripts/freeze/run_frozen.py`). (2) The AST-only PCH
  doesn't emit the header's *internal-linkage statics* (bt: `BT::UndefinedAnyType`)
  and the library's copy is a non-exported local symbol, so on the frozen path the
  kit emits one strong definition under the exact mangled name; applied only when
  frozen (in L0 the live parse already defines it).
- **What freezing does NOT remove:** the first-use JIT of cppyy's per-signature
  call wrappers (`registerSimpleAction`'s `std::function` thunk etc. — ~0.7 s for
  t01, unchanged L0↔L1). A header PCH only kills the *parse*. Cutting the first-use
  JIT is a separate step (L2 native lowering, or caching the instantiations).
- **Generalises:** the same recipe takes `rclcpp/rclcpp.hpp` from ~1.71 s to ~6 ms
  — the PCH-load floor is header-size-independent, so this is not a BT special case.

**What a kit should do now:** make bringup idempotent and staged; treat the JIT as
a one-time startup cost. When startup latency matters, freeze it (FREEZE.md) — a
per-kit PCH build plus, if the freeze surfaces unresolved internal-linkage symbols,
a one-line force-symbol entry.

---

*Evidence lives in the per-kit reports: `docs/bt_kit/REPORT.md` (capability matrix,
deep-pass verdicts, AOT probe) and `docs/pcl_kit/REPORT.md` (copy accounting,
showcase benchmark). This document is the merged, library-independent layer.*
