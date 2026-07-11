# Freezing a kit — L0 → L1 (and one leaf to L2)

**Status: WORKING.** The bt_kit header parse — ~89 % of bringup, the one real cost
of the JIT approach — is *eliminated* by loading a prebuilt Cling precompiled
header (PCH), and the same 16-test suite passes on the frozen path.

This is the "freeze" rung of the lowering cycle:

| Rung | What it is | bt_kit today |
|---|---|---|
| **L0** | JIT prototype — headers parsed by Cling at bringup | the default kit |
| **L1** | **frozen** — header AST loaded from a prebuilt PCH, no per-run parse | **this doc** |
| **L2** | native C++ emitted for a hot path | one leaf, below (§5) |

The contract is the same tests at every rung: `pixi run -e bt test-bt` (16 tests)
is green on L0 *and* L1, and the L2 leaf is differential-tested against its L0
Python original.

---

## 1. The mechanism (why a PCH, not a dictionary)

Bringup cost is dominated by one call: `cppyy.include("behaviortree_cpp/bt_factory.h")`
JIT-parses the header stack (~0.83–0.91 s, measured). A prior probe
(`docs/bt_kit/REPORT.md` §5 Gap 8) showed a **ROOT/genreflex dictionary does not
help** — it supplies reflection/autoload metadata, not a parsed AST, so Cling
still lazily re-parses the header on first class use.

What *does* work is the mechanism **cppyy already uses for its own std headers**: a
**Cling precompiled header**. Cling loads the PCH named by the `CLING_STANDARD_PCH`
environment variable when the interpreter initialises. We build a PCH that bakes
the kit's headers *on top of* cppyy's standard-header set and point
`CLING_STANDARD_PCH` at it. On the frozen path the header AST is materialised from
the PCH at interpreter start; `cppyy.include(...)` becomes a lookup (~6 ms) instead
of a parse.

The build reuses cppyy's shipped machinery: `rootcling -generate-pch` over
`etc/dictpch/allHeaders.h` + `allLinkDefs.h` with the env's `allCppflags.txt`
(exactly what `etc/dictpch/makepch.py` does), plus the kit header and its include
path inserted. The artifact is a normal Cling PCH, ~48 MB.

### The one snag: internal-linkage statics

The AST-only PCH carries a *declaration* for the header's internal-linkage statics
but the JIT never emits their *definition*, and the library's own copy is a
non-exported local symbol. So JIT-compiled glue that ODR-uses one fails to link.
For bt_kit there is exactly one: `BT::UndefinedAnyType`
(`static std::type_index = typeid(nullptr)` in `safe_any.hpp`, used by every
`Any`/`PortInfo`). The fix, applied **only on the frozen path**, emits one strong,
externally-visible definition under its exact mangled name so every JIT module
resolves to it (`rclcppyy/kits/freeze.py::_FORCE_SYMBOLS`). In L0 the live-parsed
header already defines it, so this glue is *not* applied there (a second definition
would clash). If a freeze of another header surfaces more such symbols, add them to
that per-kit table.

---

## 2. Recipe — freeze bt_kit

```bash
pixi install -e bt
pixi run build                 # install the package (bt_kit + freeze module)
pixi run -e bt freeze-bt-build # build the PCH into build/freeze/  (~30–60 s)

# run anything with the frozen PCH active:
pixi run -e bt demo-bt-t01-frozen
pixi run -e bt test-bt-frozen  # the SAME 16 tests, frozen
pixi run -e bt freeze-bench    # L0 vs L1 numbers (§4)
```

To freeze an arbitrary script, wrap it with the launcher:

```bash
RCLCPPYY_FROZEN=1 python scripts/freeze/run_frozen.py your_script.py [args...]
```

### Why a launcher (the import-order rule)

`CLING_STANDARD_PCH` must be set **before the first `import cppyy`** — Cling binds
its PCH at interpreter init, and setting the variable afterwards is ignored
(measured: 911 ms, i.e. still parsing). Because `import rclcppyy` imports cppyy
transitively, you cannot set it from inside a kit. `scripts/freeze/run_frozen.py`
resolves the artifact *without* importing rclcppyy/cppyy, sets the environment, and
`exec`s the target in the same process image, so the target's first cppyy import
already sees the frozen PCH. `bringup_bt()` warns if `RCLCPPYY_FROZEN` is set but no
frozen PCH is active (i.e. the launcher was bypassed) and falls back to JIT.

---

## 3. Artifact lifecycle

* **Location:** `build/freeze/bt_kit.pch.native.<cppstd>.<cppyy-cling-version>`
  (e.g. `…native.17.6.32.8`). `build/` is gitignored — **never commit the PCH or
  the L2 `.so`** (they are large and environment-specific).
* **Env-version-matched:** the filename carries the C++ standard and the
  cppyy-cling version. A PCH is only valid for the Cling it was built with; a
  version bump changes the tag, so a stale artifact is obvious and
  `freeze.artifact_path()` simply won't find one → the launcher runs JIT and prints
  how to rebuild.
* **Rebuild when:** cppyy-cling or behaviortree_cpp changes version, or the kit's
  header set changes. Just rerun `freeze-bt-build`.
* **Not built?** Everything still works unfrozen (JIT) — the freeze is purely a
  startup-latency optimisation, never a correctness dependency.

---

## 4. Numbers (measured, this machine, shared — medians of cold runs)

`pixi run -e bt freeze-bench`. Bringup is a once-per-process cost, so each sample
is a fresh subprocess.

| Bringup stage | L0 JIT | L1 frozen | speedup |
|---|--:|--:|--:|
| `include(bt_factory.h)` — **the parse** | ~890 ms | **~6 ms** | **~140×** |
| `load_library` | ~6 ms | ~5 ms | 1.2× |
| `cppdef(glue)` | ~50 ms | ~78 ms | 0.6× |
| **bringup total** (through cppdef) | **~950 ms** | **~90 ms** | **~10.7×** |
| first factory + register + tick (first-use JIT) | ~690 ms | ~690 ms | 1.0× |
| **end-to-end `t01_first_tree.py`** (process start→exit) | **~1.9 s** | **~1.1 s** | **1.7× (−0.8 s)** |

**What the freeze removes, plainly:** the ~0.83–0.91 s header **parse**, and only
that. What **remains** after freezing:

* `load_library` (~5 ms) and the `cppdef` C++ glue (~78 ms — slightly *higher*
  frozen, because the glue's template instantiations are JIT-emitted fresh rather
  than reused from the live parse);
* **first-use JIT (~0.69 s, unchanged L0↔L1):** the first `registerSimpleAction`
  costs ~0.41 s (cppyy generating the `std::function<NodeStatus(TreeNode&)>`
  wrapper + call thunk), the second ~0.10 s, `create_tree_from_text` ~37 ms, first
  tick ~9 ms. A **header PCH does not touch this** — it is cppyy's per-signature
  call-wrapper codegen, not a parse. Cutting it is a separate effort (cache the
  JIT'd instantiations, or lower the hot path to L2).

### The mechanism generalises (second data point)

Same recipe applied to `rclcpp/rclcpp.hpp` (the rclcpp bringup's dominant cost):

| | L0 JIT | L1 frozen |
|---|--:|--:|
| `include("rclcpp/rclcpp.hpp")` | **~1.71 s** | **~6 ms** (~290×) |

Both libraries collapse to the same ~6 ms PCH-load floor regardless of header
size — evidence the freeze is library-independent, not a BT.CPP special case. (The
rclcpp measurement is parse-elimination only; a full frozen rclcpp bringup would
need its own force-symbol pass and is out of scope here.)

---

## 5. L2 — one leaf lowered to native C++

t01's `ApproachObject` Python leaf, emitted as a native `BT::SyncActionNode` in a
compiled plugin `.so` and registered **JIT-free** via
`factory.registerFromPlugin(...)` (the engine `dlopen`s it; no cppyy, no Python in
the tick path).

```bash
pixi run -e bt freeze-l2-build   # compile scripts/freeze/l2_approach_object.cpp -> .so
pixi run -e bt freeze-l2-diff    # differential test vs the L0 Python leaf
```

Differential result (same tree XML, same node ID):

* **Correctness:** identical stdout (`ApproachObject: approach_object`) and status
  (SUCCESS) — the test is the contract across the rung.
* **Tick rate** (single-leaf tree, SUCCESS/tick, no I/O): L0 Python leaf ~0.55
  µs/tick vs **L2 native ~0.20 µs/tick — ~2.7× faster**, i.e. the Python↔C++
  boundary cost per leaf is removed.

L2 here is hand-written; the point proven is the *rung* — a leaf authored/prototyped
in Python (L0) has a mechanical native equivalent (L2) that passes the same test
and runs at engine speed. Registration still crosses cppyy once
(`registerFromPlugin`), but the leaf executes as native code every tick.

---

## 6. Files

| File | Role |
|---|---|
| `rclcppyy/kits/freeze.py` | artifact path/version tag, frozen-path detection, force-symbol glue |
| `scripts/freeze/build_bt_pch.py` | build the frozen PCH (rootcling `-generate-pch`) |
| `scripts/freeze/run_frozen.py` | launcher: set `CLING_STANDARD_PCH` before cppyy, exec target |
| `scripts/freeze/bench_freeze.py` | L0-vs-L1 numbers |
| `scripts/freeze/l2_approach_object.cpp` / `build_l2_node.py` / `l2_diff.py` | L2 leaf + build + differential test |
| `test/test_bt_freeze.py`, `test/_freeze_helper.py` | frozen-path tests (parse eliminated + correct) |
| `rclcppyy/kits/bt_kit.py` | `bringup_bt()` applies force-symbols when frozen; `bt_kit.frozen()` |

## 7. Limitations

* The PCH is a startup-latency optimisation for the **parse only**; the first-use
  JIT of cppyy call wrappers (~0.7 s for t01) is untouched (§4).
* Artifacts are Cling-version-specific and must be rebuilt (never committed) on any
  cppyy-cling / library version change.
* Freezing a new header may surface further internal-linkage symbols to force
  (§1); the failure mode is a clear "unresolved while linking" error naming the
  symbol.
* The launcher must run before any cppyy import (import-order rule, §2).
