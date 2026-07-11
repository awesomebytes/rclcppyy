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
- **ompl:** ~538 ms — *lower than bt or pcl despite pulling in boost*. The cost
  tracks the **transitively-included header stack**, not the library's "size" or
  reputation: ompl+boost (538 ms) < bt.CPP (0.9 s) < pcl (1.3 s). Measure the
  actual `include(...)` before assuming a big library means a slow bringup.

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
- **A class hint can only infer `T&`** — but cppyy will bind a
  `std::function<...(T&)>` even where the API wants `const T*` (ompl's
  `setStateValidityChecker`), and the mismatch then fails *later* at the call site.
  callback **warns once** naming the fix. For the exact form, annotate the
  parameter with the **C++ type string**, used verbatim:
  `def check(s: "const ompl::base::State*") -> bool: ...` →
  `bool(const ompl::base::State*)`. (flake8/pyflakes flags such a string annotation
  as `F722`, a forward-ref false positive — add `# noqa: F722`, or use
  `signature=` instead.)
- **Explicit `signature=` wins** for anything, e.g.
  `cppyy_kit.callback(tick, signature="BT::NodeStatus(BT::TreeNode&)",
  owner=factory)` — exactly how bt_kit registers leaf/stateful hooks and ompl_kit
  fixes the validity-checker pointer form.
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
- **nav2 (3rd instance):** the `unsigned char*` costmap buffer, same recipe
  (address as `uintptr_t`, `memcpy` in `cppdef`), ~600–3600× a Python loop.
  Output-by-pointer-array (NavFn's `getPathX()/getPathY()` `float*` + length) is
  the same "keep it in C++": one helper that `memcpy`s the outputs, not marshalling
  C arrays across the boundary.
- **Copy-in vs alias-in (vision).** The above all *own* storage, so one copy in is
  unavoidable. When the C++ type can **alias** an external buffer it is genuinely
  **zero-copy**: `cv::Mat(rows, cols, type, void* data, step)` wraps a ROS
  `Image` buffer pointer-identically. Still a `cppdef` helper (cppyy rejects a Python
  int as `void*`; pass `uintptr_t`), and you **must keep the source buffer alive**
  for the Mat's lifetime (a lifetime guard, not a copy). Distinguish "buffer you can
  alias" (zero-copy) from "storage you must own" (one copy).
- **`std::make_shared<T>()` can be flaky from Python** (overload-cache sensitivity,
  seen in control_kit) — build the object in a small C++ factory helper instead.

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
- **vision:** a **dependent-type** template member (a templated member accessed on a
  value whose type depends on a template parameter, inside a patched header) needs
  the explicit `.template` disambiguator — `obj.member.template ptr<T>()` — the
  clang two-phase-lookup requirement; not needed when the same call is on a concrete
  type.

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
- **A failed `cppyy.include` contaminates the interpreter too, not just `cppdef`
  (nav2).** When one header failed mid-parse (a missing transitive dep), the *next,
  unrelated* `cppyy.include` in the same process failed spuriously — though it
  includes cleanly in a fresh process. So probe a **risky include** (heavy/uncertain
  transitive deps) out-of-process, exactly as for `cppdef`.
- **`generate_parameter_library` headers SIGSEGV the Cling parser (moveit).** Any
  `*_parameters.hpp` (fmt + rsl + validators) crashes on include — and modern ROS 2
  packages all generate one. Never `cppyy.include` it; find the *clean* base header
  and load the class/plugin directly. (ros2_control's headers happen to be
  Cling-clean — this wall is per-package, so probe.)
- **Mitigation:** probe risky glue out-of-process first —
  `cppyy_kit.probe_cppdef(code, include_paths=, headers=, libraries=)` compiles it
  in a throwaway subprocess and returns `(ok, message)` without risking the main
  interpreter. Pass it the **full ament include-path set** (every package's include
  dir, via `get_packages_with_prefixes`), not just the target library's — else a
  header that transitively pulls the ROS message tree fails on a missing transitive
  header (a false negative).

### 10. Error ergonomics: strip the signature wall
cppyy prefixes a C++ exception with the mangled call signature and ` => `. Split
on ` => ` and collapse whitespace for a readable one-line message, re-raised as a
kit exception (`cppyy_kit.pretty_cpp_error`, `CppyyKitError`).
- **bt:** `BtXmlError` turns the `createTreeFromText(...) =>` wall into
  `RuntimeError: Error at line 4: -> Node not recognized: Nope`.

### 11. Values that don't cross as ints: enums, `unsigned char`, macros
C++ enums behave like their int values across the boundary
(`BT.NodeStatus.SUCCESS == 2`). Expose plain-int constants for convenience while
keeping the real enum available (`bt_kit.SUCCESS` and `bt.NodeStatus.SUCCESS`).
But three neighbours are silent traps:
- **`unsigned char` (and `uint8_t`-backed `enum class`) crosses as a length-1
  Python `str`, not an int** (nav2, control). `Costmap2D::getCost()` and its
  `static constexpr unsigned char` cost constants come back as `'\xfe'`, and
  `'\xfe' == 254` is `False`. Read with `ord(...)`, and expose **plain-int**
  constants from the kit. (The enum *member* is still an int-able proxy; it's a
  *returned value / struct-member read* of the uint8 type that becomes a `str`.)
- **A `using`-alias of an enum resolves to plain Python `int`** (control) — losing
  the enum-ness. Reference the **real nested enum type** (`Outer::Inner::Enum`), not
  the alias.
- **Type-constant `#define` macros are invisible to cppyy** (vision: `CV_8UC1`,
  `CV_8U`). Re-expose the few you need as real `const int` in a `cppdef` block.

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
- **cppyy does NOT release the GIL on a blocking C++ call (control, measured).** So
  a blocking C++ call cannot be overlapped with Python work by putting it on a
  *Python* thread — it holds the GIL the whole time. Run it on a **C++** thread
  instead: a plain-function `std::thread` in a `cppdef` helper (note `std::async`
  does **not** JIT in Cling — use `std::thread`). control_kit's blocking
  controller-switch does exactly this.

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

### 16. Cross-language inheritance (Python derives a C++ virtual base)
The heaviest crossing, first proven in ompl_kit: a Python class *derives* a C++
class and C++ calls its overrides in a hot loop (RRT\* calls a Python
`StateValidityChecker` millions of times/solve). It works — with rules:
- Derive the cppyy class directly; **`super().__init__(base_args)` is mandatory**
  (the C++ base must be constructed, e.g. with its `SpaceInformation`).
- Override the virtual by its **exact C++ name** (`isValid`, `stateCost`) — cppyy
  matches on the name.
- **Only plain virtuals** can be overridden across the boundary. A `final` (or
  non-virtual) member cannot — this is exactly why bt_kit's `final` `tick()` needed
  a C++ shim instead (Pattern 5 / bt REPORT). Check the base before promising it.
- **Pin the subclass instance** with `keep_alive` (or an `owner`): the "callable
  was deleted" footgun (Pattern 4) applies to override *instances* too — C++ holds
  the object, cppyy won't keep it alive for you.
- Pointer arguments arrive **auto-downcast** (Pattern 17b) so member access on the
  concrete type works with no explicit cast.
Cost here: ~350 ns/override call, 1–3 M dispatches/s — invisible for small problems,
material when the override dominates (then lower it to C++, the L2 rung).

**Deriving a *framework* base and injecting it (control_kit sharpens this):**
- **Derive the *compiled* base, never a `cppdef`'d intermediate.** cppyy's override
  dispatcher fails to resolve return types (`<unknown>`) when the base was itself
  JIT-defined; subclass the real library class directly.
- **Inject the instance where C++ stores it by `shared_ptr` via a C++-built no-op-
  deleter `shared_ptr`.** Assigning a `shared_ptr` that aliases a cross-inherited
  Python object *from Python* fails (`C++ type cannot be converted to memory`);
  build the aliasing `shared_ptr` (no-op deleter, so Python keeps ownership — pin
  the instance) in a `cppdef` helper. This is how control_kit hands a Python
  `ControllerInterface` subclass to the real `controller_manager`.
- **Reach protected base members through a same-layout accessor** — a
  `struct : Base` that `reinterpret_cast`s and reads them, exposed as free
  functions — since cppyy can't touch `protected` across the boundary.

### 17. `shared_ptr` ownership + RTTI downcast (two cppyy conveniences)
- **(a) Wrapping a raw pointer in the library's `shared_ptr` transfers ownership.**
  Constructing a `SomethingPtr(raw)` from a cppyy-owned raw object flips the raw's
  `__python_owns__` to `False` — cppyy yields ownership to the `shared_ptr`, so
  there is **no double-free**. This makes the pervasive "wrap the raw in the
  library's Ptr and hand it on" idiom (`ob.StateSpacePtr(space)`) safe, and mirrors
  `make_shared`.
- **(b) Pointer arguments are auto-downcast by RTTI.** cppyy presents a base-typed
  pointer argument (a callback's `const State*`) as its **concrete runtime type**,
  so `state[0]` / `state[1]` work without an explicit downcast. You rarely need a
  cast helper; when you do (a stored base pointer), use `getattr(obj, "as")[T]()`
  (Pattern 18).
- **(c) cppyy dereferences a `shared_ptr` to bind a `const T&` parameter** (moveit:
  `srdf::Model::initString` took a `ModelInterfaceSharedPtr` directly) — smart-
  pointer forwarding is reliable for reference params, not just member access.
- **(d) Eigen block/coeff assignment does NOT cross** (moveit: `iso.translation()[i]
  = v` → "object does not support item assignment"). Build Eigen objects in a
  `cppdef` helper (assemble the whole vector/matrix in C++), not element-by-element
  from Python. Eigen is everywhere in robotics C++, so this recurs.

### 18. Reserved-word method names: `getattr(obj, "as")[T]()` for C++ `as<T>()`
The pervasive C++ idiom `obj->as<T>()` is a Python `SyntaxError` (`as` is a
keyword — even `obj.as` won't parse). The spelling is `getattr(obj, "as")[T]()`
(fetch the attribute by string, then subscript the template arg). A one-liner, but
a guaranteed stumble for any library with a method named `as`, `from`, `import`,
`class`, `global`, … — reach for `getattr` when a C++ name collides with a keyword.

### 19. In-process pluginlib + a parameterized node (the ROS 2 plugin/param bootstrap)
Modern ROS 2 stacks load their algorithms as pluginlib plugins configured by node
parameters. Both work in-process (moveit_kit proved it; control_kit reuses it):
- **pluginlib:** `load_library("libclass_loader.so")` + **the plugin base-class
  library** (for its typeinfo), construct `pluginlib::ClassLoader<Base>(pkg,
  "Base::type")` in a `cppdef`, then `createUniqueInstance(lookup_name)` — pluginlib
  `dlopen`s the plugin `.so` itself via the ament index (do **not** cppyy-load the
  plugin). The "add the library named in the JIT link error" loop resolves the rest.
- **parameterized node:** `NodeOptions().automatically_declare_parameters_from_overrides(true)
  .parameter_overrides(vec)` + `make_shared<rclcpp::Node>(name, options)`, fed by a
  **YAML → dotted-`rclcpp::Parameter` flattener** (nested dict → dotted names,
  homogeneous lists → typed arrays) — the reusable primitive.
- **Teardown (Pattern 14, sharpened):** a pluginlib instance/loader must be
  `reset()` before Cling teardown or the process cores at exit — `register_teardown`
  it. The `class_loader` "will NOT be unloaded" warning is benign/expected.

### 20. Kit-authoring triage: is the C++ core drivable, and when to fall back
Before investing in a kit, a couple of one-line greps tell you what's separable:
- **Lifecycle coupling (nav2).** Grep the class's ctor / `configure` signatures: if
  it takes plain data it's drivable; if it takes a `LifecycleNode` / `*ROS` wrapper /
  a pluginlib base, it needs the server (out of scope for a "drive the core" road, or
  use the pluginlib bootstrap §19). `nm -DC` / a header grep up front beats
  discovering it after the JIT investment.
- **Missing transitive headers (vision/gtsam).** A header-heavy library can be
  **un-JIT-able** if a transitive include is absent from the env (gtsam →
  `boost/optional.hpp`). Grep the target's transitive includes for env-absent deps
  first. When blocked *and* the work is **batch** (not a hot loop), the library's own
  **Python binding is a legitimate fallback** for that step — cppyy is not the only
  tool, and a kit can mix (drive the hot C++ path via cppyy, use the binding for a
  one-shot batch step).

### 21. Vendored-source direct-compile (when there's no package)
For a small, well-understood subset of a library that ships no conda package
(DBoW2), clone it + apply a **documented, marker-guarded in-place patch** + compile
with a direct `$CXX` invocation into a `.so` — this beats fighting the library's
CMake/ExternalProject. It generalizes the L2 lowering recipe (`build_l2_node` →
`build_dbow2`): a reproducible build script, artifact gitignored, env-version tagged.

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
