# Proposal: cppyy_kit as the base package — re-architecting the kit suite

**Status: PROPOSAL (2026-07-11)** — evaluation of the external ideation doc
(`cppyy_kit_reference.md`, a conversation Sam had with another AI) against what
this repo has actually built and measured, followed by a phased re-architecture
plan. Nothing here is committed work until approved.

---

## 1. Ideas from the reference doc — evaluated against our evidence

### 1.1 Good and missing (adopt)

| Idea (doc §) | Verdict | Why, grounded in our evidence |
|---|---|---|
| **Content-hash compile cache** (§5.7) | **Adopt — highest value** | The single best idea in the doc. Our freeze work proved the Cling PCH kills the header parse (890→6 ms) but NOT the per-signature wrapper JIT (~0.69 s, measured identical L0/L1, not reducible by flags). The doc's approach — hash the C++ source, compile ONCE to a real `.so`, `dlopen` thereafter — is our L2 direct-compile recipe applied automatically to every `cppdef`. It would make the "slow first call" happen once per machine *ever*, superseding warmup's cost-relocation with cost-elimination. We already own every ingredient (direct-compile recipe §21, artifact tagging from FREEZE.md). |
| **`require()` header-only fetcher** (§5.5) | **Adopt, conda-first** | Generalizes our §21 vendored-source pattern (DBoW2 took a bespoke script). Policy matters: prefer conda/pixi packages when they exist (they usually do — eigen, fmt, nlohmann); `require()` is the fallback for header-only libs that aren't packaged or need exact versions. Must cache + checksum like our dataset tooling. |
| **`@cpp` decorator** (§5.6) | **Adopt** | Natural unification of what we already half-built: `callback()`'s type-hint signature inference + the marshaling patterns (numpy→pointer+size is our §6). Body-never-executed with annotation-driven marshaling is clean, and it composes with the compile cache (each `@cpp` block = one cached `.so`). |
| **`.pyi` stub generation** (§5.1) | **Adopt — revive** | Was on rclcppyy's ORIGINAL roadmap (`scripts/create_stubs.py`, stalled WIP). Belongs at the cppyy_kit level; directly serves the LLM-ergonomics principle (typed corridors help agents too). |
| **async/nogil helpers** (§2.2) | **Adopt, corrected** | The doc's premise ("cppyy calls release the GIL automatically") is **factually wrong** — control_kit *measured* that cppyy holds the GIL on blocking C++ calls (COMMON_PATTERNS §13). But the goal is right: a `nogil()` wrapper (C++-side GIL release shim) + an asyncio `run_in_executor` integration would make blocking C++ calls event-loop-safe. Build it on our corrected foundation. |
| **Layered packaging: base + kit packages** (§5.1/§5.8) | **Adopt — the re-architecture** | Matches Sam's instinct and our reality: cppyy_kit is already ROS-free by content; six kits already follow "depends on the base". See §3. |
| **Fallback contract + `status()` introspection** (§5.2) | **Adopt the pattern, not the target** | We already do capability-detection-with-fallback ad hoc (CUDA auto-detect, frozen-path fallback, gtsam binding fallback). Codify it once in cppyy_kit (capability probe + fallback + introspectable status) so kits stop reinventing it. |

### 1.2 Doesn't make sense for us (skip, with reasons)

| Idea | Verdict | Why |
|---|---|---|
| **`kits.fast` — monkey-patching builtins/stdlib** (§5.2) | **Skip** | A different product (general Python acceleration, competing with orjson/polars/numpy). Our data is never 10M-element Python lists — it's messages/clouds/images already living in C++. The doc itself lists the costs (CPython-version compatibility surface, float-reduction correctness, patch conflicts), and our own monkeypatching experience says global patching is the most fragile thing we ship. Not our mission. |
| **Lambda transpilation** (§5.3) | **Skip** | AST→C++ transpiling is admitted-fragile with an ill-defined boundary. Our L0→L2 lowering with tests-as-the-contract is the principled version of the same desire. The `op.*` functor algebra is a DSL — violates mirror-don't-sugar. If a kit ever needs composed C++ functors, add the minimal helper then, evidence-first. |
| **"GIL bypass is automatic"** (§2.1) | **Reject the claim** | Measured false for cppyy (control_kit). Keep the corrected record; build explicit nogil instead. |
| **`kits.parallel` as a headline** (§5.3) | **Defer** | Parallel STL via cppyy is real capability, but it's generic-Python-audience material, not robotics. Community-kit territory after the base exists. Exception: **`concurrent`** primitives (lock-free SPSC queues via moodycamel) DO matter for robotics pipelines — a small module once `require()` exists. |
| **Freeze CLI → "wheel with no Cling dependency"** (§5.7) | **Reframe** | Overpromised: full AOT of arbitrary reflection surfaces is beyond current cppyy; our measured ladder (PCH keeps Cling; wrappers still JIT; L2 removes cppyy per-path) is the honest version. The compile cache gets most of the practical benefit without the false promise. |

---

## 2. What the doc missed that we learned the hard way

Anyone building the doc's design would rediscover our 21 patterns: the GIL truth
(§13), silent SIGSEGVs (containers, failed cppdef/include — §9), keep-alive
lifetime discipline (§3), ownership rules (§5, §16), the `final`-virtual and
`generate_parameter_library` walls, value-crossing traps (§11), and the honest
performance framing (boundary cost per call; engine value ≠ tick speed). Our
COMMON_PATTERNS.md + cppyy_kit primitives are the moat; the doc's contribution is
packaging, cache, and ergonomics ideas on top.

---

## 3. Re-architecture plan

### Target layout (monorepo, separate pixi/conda packages)

```
repo root
├── cppyy_kit/              # Package 1: ROS-FREE base (the doc's "base")
│   ├── (current rclcppyy/kits/cppyy_kit.py content)
│   ├── cache.py            # NEW: content-hash cppdef→.so compile cache
│   ├── require.py          # NEW: header-only fetcher (conda-first policy)
│   ├── cpp.py              # NEW: @cpp decorator (annotation marshaling)
│   ├── nogil.py            # NEW: GIL-release shim + asyncio integration
│   ├── stubs.py            # REVIVED: .pyi generation
│   ├── freeze/             # generalized PCH + direct-compile + vendored-source tooling
│   └── capability.py       # NEW: probe/fallback/status pattern, codified
├── rclcppyy/               # Package 2: ROS core (depends on cppyy_kit)
│   └── (bringup, monkeypatching, messages, serialization, rosbag, TF)
├── kits/                   # Packages 3..n (each depends on cppyy_kit; ROS kits also on rclcppyy)
│   ├── bt_kit/  pcl_kit/  ompl_kit/  nav2_kit/  moveit_kit/  control_kit/
│   └── cv_kit/  dbow_kit/           # vision pair
├── recipe/                 # one rattler-build recipe per package
└── docs/, scripts/, test/  # as today
```

- **Same repo** (monorepo), **separate conda packages** on the existing
  prefix.dev channel: `cppyy-kit` (no ROS deps — conda-forge candidate later),
  `ros-jazzy-rclcppyy` (as today), `ros-jazzy-<kit>-kit` each declaring its own
  C++ deps (behaviortree-cpp, pcl, ompl, nav2-*, moveit, ros2-control…).
- Kits are **pure-Python packages** (no ament needed — only rclcppyy keeps
  ament_cmake for the ROS index); their recipes are trivial compared to the one
  we already ship.
- **Import compatibility**: `rclcppyy.kits.X` shims re-exporting the new
  top-level modules for one release cycle, with a deprecation note.
- **Versioning**: lockstep initially (one tag releases the whole suite via a
  release.yml matrix over `recipe/*/recipe.yaml`); split later only if needed.

### Phases

- **Phase A — Extract & re-plumb (mechanical, no new features).**
  Move cppyy_kit to top-level; kits import it; compat shims; multi-recipe
  release matrix; CI path-filtered test jobs. Gate: every existing suite green,
  every artifact builds, fresh-env install of each package proven (the Phase-4
  playbook per package).
- **Phase B — Enrich the base (the adopted ideas, value order).**
  1. Compile cache (kills first-use JIT persistently — measure vs warmup).
  2. `require()` + port the DBoW2 build onto it.
  3. `@cpp` decorator (unify with callback() inference).
  4. `nogil()` + asyncio helper (with a measured GIL-release proof).
  5. Stubs revival. 6. capability/status codification.
  Each lands with the usual gates + COMMON_PATTERNS updates.
- **Phase C — Publish the suite.** Tag → matrix build → prefix.dev; README
  install matrix; per-kit WHY docs become per-package READMEs.
- **Phase D — Outward (optional).** `concurrent` module; conda-forge submission
  of `cppyy-kit`; community-kit contribution guide (COMMON_PATTERNS as the
  authoring manual).

### Risks

- **Churn vs. momentum**: Phase A touches every import path — do it in one
  focused pass with the full gate matrix, not incrementally alongside feature work.
- **cppyy-kit without ROS** needs its own CI leg (plain conda-forge env) to keep
  the ROS-free claim honest.
- **Naming**: `cppyy_kit` on conda-forge may collide with the reference doc's
  hypothetical if it ever ships; claim the name early (publish to our channel
  first, conda-forge when stable).
