# rclcppyy Maturation Plan

Goal: take `rclcppyy` from "ROSCon demo repo" to a reliably installable package:
a one-command pixi workspace, examples and benchmarks runnable in a couple of
commands, CI proving it stays that way, and a conda package published on a
prefix.dev channel.

## Current-state audit (evidence)

| Finding | Evidence |
|---|---|
| Workspace setup requires manual `pixi.toml` copying + a workaround script | `README.md` "Run demos": copy `fix_cppyy_api_path.sh`, comment/uncomment activation line between first and second `pixi install` |
| Workaround exists only because the **pip wheel** of cppyy ships a lowercase `cpycppyy/` header dir | `fix_cppyy_api_path.sh` symlinks `cpycppyy` → `CPyCppyy`; `EXTRA_CLING_ARGS=-Wno-nonportable-include-path` silences the resulting warning |
| conda-forge ships `cppyy` 3.5.0 with **correctly capitalized** headers (`include/python3.12/CPyCppyy/API.h`), builds for py310–py313 | verified by inspecting `cpycppyy-1.13.0-py312h0a2e395_0.conda` from conda-forge (2026-07) |
| README's recommended `pixi.toml` cannot actually build or run the demos | missing `colcon-common-extensions`, `cmake`, `compilers`/`ninja`, and `ros-jazzy-rmw-cyclonedds-cpp` (the Run section exports `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) |
| No `pixi.toml`, no `pixi.lock`, no `.gitignore`, no CI in the repo | repo root listing |
| Broken test | `test/test_node.py:6` does `from rclcppyy import Node` — `rclcppyy/__init__.py` exports `RclcppyyNode`, not `Node` |
| Unused/stale packaging file | `setup.py` is dead code for an `ament_cmake` package (install goes through `ament_python_install_package`, `CMakeLists.txt:15`) and globs a nonexistent `launch/` dir |
| Everything under `scripts/` gets installed, including non-scripts | `CMakeLists.txt:18-22` — `PATTERN "*.py"` without `FILES_MATCHING` filters nothing; stray extensionless `scripts/target` file included |
| Committed junk | `roscon_uk_2025/presentation/.hypothesis/` (hypothesis example DB) |
| Incomplete `package.xml` deps | runtime imports include `ament_index_python`, `rosidl_runtime_py`, `rosbag2_py`, `std_msgs`, `sensor_msgs`, `numpy` — only `rclpy`/`rclcpp` declared |
| Target channels all exist | `robostack-jazzy` (ros-base 0.11), `robostack-humble` (0.10), `robostack-kilted` (0.12) verified via `pixi search` |
| Slow bringup acknowledged | `README.md` TODO: ~2.5 s `cppyy.include("rclcpp/rclcpp.hpp")` JIT on every start (`bringup_rclcpp.py:354`) |

## Phase 1 — Reliable workspace: `git clone` + `pixi install`, zero workarounds

The core fix: ship the workspace manifest **in this repo** and use conda-forge
cppyy instead of the pip wheel.

- [x] Add `pixi.toml` at repo root (workspace = repo; `colcon` discovers the
      package in cwd, `build/ install/ log/` land in repo root):
  - channels: `["robostack-jazzy", "conda-forge"]`
  - deps: `ros-jazzy-ros-base`, `ros-jazzy-rmw-cyclonedds-cpp`, `cppyy >=3.5,<4`
    (conda-forge, **not** pypi), `colcon-common-extensions`, `cmake`, `ninja`,
    `make`, `compilers`, `pkg-config`, `python 3.12.*`, `pytest`, `numpy`,
    `setuptools <81`
  - activation env: `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
    `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (README documents fastrtps
    latency/big-message issues — make the good default automatic)
  - guarded activation script that sources `install/setup.bash` only if present
    (no first-run failure)
- [x] Validate conda-forge cppyy end-to-end in that env: `bringup_rclcpp()`,
      pub/sub monkeypatched benchmarks, tutorial example. This is the only
      unproven assumption in this plan — do it first. Keep
      `EXTRA_CLING_ARGS` as a documented escape hatch if cling needs flags.
- [x] Delete `fix_cppyy_api_path.sh` once validated.
- [x] Commit `pixi.lock` for reproducibility.
- [x] Add `.gitignore` (`.pixi/`, `build/`, `install/`, `log/`, `__pycache__/`,
      `.hypothesis/`); remove committed `.hypothesis/` data.
- [x] Pixi tasks: `build` (`colcon build --packages-select rclcppyy`), `clean`,
      `test`.
- [x] Rewrite README setup section: `git clone && cd rclcppyy && pixi run build`
      — remove the workaround narrative and the stale "wait a few days" banner.

**Acceptance:** on a clean machine with only pixi installed:
`git clone … && cd rclcppyy && pixi run build && pixi run test` succeeds with
no manual steps.

**Status: DONE (2026-07-10).** Validated end-to-end: cppyy JIT, `bringup_rclcpp()`
(1.75 s internal, faster than the ~2.5 s expected), clean `colcon build`, bringup
test green, monkeypatched pub/sub at 1 kHz with ~9000 msgs received / 0 dropped.
`EXTRA_CLING_ARGS` and the symlink workaround confirmed obsolete. Two
manifest-level additions were required (declarative, no shell workarounds):
1. `LD_LIBRARY_PATH=$CONDA_PREFIX/lib` in `[activation.env]` — conda/robostack
   rely on RPATH and leave `LD_LIBRARY_PATH` empty, but cppyy resolves C++
   symbols at call time by scanning it; without this every `cppyy.gbl.rclcpp`
   call fails to resolve.
2. `pytest >=7.4,<8` — pytest 9 crashes robostack-jazzy's `launch_testing`
   pytest plugins at startup.
Locked: cppyy 3.5.0, cppyy-cling 6.32.8, ros-jazzy-ros-base 0.11.0, python 3.12.13.

## Phase 2 — Examples & benchmarks in a couple of commands

- [x] **Blocker found during Phase 1 validation:** the plain-bringup rclpy-style
      API is broken — `adapt_node_pub_sub_to_python()` is commented out
      (`bringup_rclcpp.py:364`), so `create_publisher(String, …)` fails with
      "Template method resolution failed", which breaks
      `publisher_member_function.py`. Enable/fix the adapter (or route the
      tutorial through the working monkeypatch path) without regressing
      `enable_cpp_acceleration()`.
      **Resolved 2026-07-10 (cc3b52b, a021d17):** adapters reworked and
      re-enabled — descriptor-based `create_publisher`/`create_subscription`
      supporting both rclpy-style calls and the native `[MsgT]` template
      syntax (verified non-regressed), rclpy-style `create_timer`,
      `destroy_node` shim, callback keep-alive pinning. Tutorial runs
      unmodified; monkeypatch path unchanged (1 kHz smoke: 0 dropped).
- [x] Benchmark runner (`scripts/benchmarks/run_benchmarks.py`): spawns
      pub+sub pairs (rclpy vs rclcppyy), samples CPU via `psutil` for N
      seconds, prints a comparison table matching the README numbers.
      Replaces the current 4-shells-plus-`top` workflow. Note: the existing
      bench scripts crash on SIGINT teardown (pre-existing double-shutdown /
      segfault after last message) — the runner must own child-process
      lifecycle and not treat teardown exit codes as benchmark failure.
      - `pixi run bench` → full comparison at 1 kHz and 10 kHz
      - `pixi run bench -- --rate 10000` → parameterized
- [x] Demo tasks: `pixi run demo-tutorial` (publisher_member_function),
      `pixi run demo-pubsub` (pub+sub pair in one command).
      **Done (7ddc580, cd1a416).** Measured on this machine (15 s/rate,
      0 dropped everywhere): 1 kHz — rclpy 18.1/20.6 % vs rclcppyy 3.8/4.8 %
      (pub/sub CPU); 10 kHz — rclpy 71.9/76.9 % vs rclcppyy 14.6/18.7 %.
      The README's ~4–5× CPU reduction reproduced. `--json` output available
      for CI assertions; full default `pixi run bench` ≈ 81 s wall.
- [x] Optional `demos` feature/environment for heavy extras (`opencv`,
      `ros-jazzy-pcl-conversions`, `pcl`, `gstreamer`/`pygobject`, `typer`,
      `hypothesis`) so the default env stays lean. Mark `roscon_uk_2025/` as a
      presentation archive (needs external bag file / cloudini checkout) —
      exclude from install.
      **Done (3d097cb, 01c0d05, merged d44ea72):** `demos` env with
      solve-group "default" (default env's 462-package lock byte-identical);
      tasks `demo-images` (runs fully, 1280x720 bgr8 roundtrip verified),
      `demo-hypothesis` (runs fully), `demo-pointcloud-voxelgrid` (PCL JIT
      compile + node spin validated; needs external bag for data flow).
      gstreamer / cloudini / pointcloud-passthrough documented as
      external-dependency demos. PCL include path made version-robust
      (glob `pcl-*`). Notable versions: opencv 5.0.0 headless, pcl 1.15.1.
- [x] Repo/packaging hygiene (done 2026-07-10, cb2114d + a021d17):
  `Node` alias exported and `test_node.py` green (the alias exposed a real
  `RclcppyyNode.destroy_node` bug — cppyy publishers stored in rclpy's
  `_publishers` broke rclpy teardown — fixed); `setup.py` and `scripts/target`
  removed; CMake install now `FILES_MATCHING PATTERN "*.py"` (verified: only
  .py files install); `package.xml` deps completed from actual imports,
  version bumped to 0.1.0.
- [x] Add smoke tests: rclpy-API pub/sub roundtrip through C++ backend,
  message monkeypatch, serialization parity (today only `test_bringup.py`
  and `test_node.py` exist).
  **Done (f98db15):** 3 new test modules + subprocess helpers (monkeypatch is
  process-global/irreversible, so contaminating scenarios run in throwaway
  subprocesses asserting on marker output; helpers `os._exit(0)` past the
  known cppyy shutdown wart). Deadline-bounded spins, discovery-gated
  publishing. 4 consecutive full-suite runs green, ~15.5 s each.

**Acceptance:** `pixi run bench` produces the rclpy-vs-rclcppyy CPU table in
one command; every demo listed in the README runs via a single `pixi run` task.

## Phase 3 — CI

- [x] GitHub Actions using `prefix-dev/setup-pixi` with pixi-env caching:
      `pixi run build` + `pixi run test` + a short headless `pixi run bench`
      smoke (few seconds, assert rclcppyy actually publishes) on linux-64.
      Suggested smoke: `pixi run bench --variants rclcppyy --rate 1000
      --duration 3 --warmup-timeout 120 --json` (~10–15 s + cold JIT; bump
      the warmup timeout on slow runners; assert `msgs_received > 0`).
- [x] Lint job (existing `ament_lint_auto` via `colcon test`).
      Initially deferred: local `colcon test` showed 2712/2758 lint failures
      (mostly style codes; 64 real F401 + 22 F841), spanning the whole tree.
      **Done (6ef5dc3):** plain `flake8` with a committed `.flake8` (120 cols;
      style/pydocstyle codes relaxed by config; A003 exempted because
      `rosbag2_py_compat` deliberately mirrors the rosbag2_py API), scoped to
      `rclcppyy/` + `test/`. ~45 real findings fixed (F401/F841/F811, bare
      except, E402 with justified noqa in test helpers). `pixi run lint` exits
      0 and is wired into ci.yml. `colcon test`'s ament_lint_auto remains red
      and intentionally unused; scripts/ + roscon_uk_2025/ out of lint scope.
- [x] Badge in README.

**Status: CI authored (f8bf71a)** — ci.yml: checkout → setup-pixi@v0.10.0
(pixi 0.70.0, default env only, cache keyed on pixi.lock, cache-write on
main pushes) → build → test → bench smoke (`--warmup-timeout 300` hedge for
2-core cold JIT), concurrency-cancel for superseded runs. All three commands
re-validated green locally before encoding. Estimated ~8–12 min cold /
~4–6 min cached. NOT yet exercised on GitHub — nothing has been pushed;
first push to main seeds the cache.

**Acceptance:** PRs get a green/red signal; the Phase 1 guarantee is enforced
mechanically, not by discipline.

## Phase 4 — Conda package on prefix.dev

- [x] Write a `rattler-build` `recipe/recipe.yaml`, robostack-style:
      package `ros-jazzy-rclcppyy`, standard ament_cmake build script,
      `host`/`run` deps on `ros-jazzy-rclcpp`, `ros-jazzy-rclpy`,
      `ros-jazzy-ament-index-python`, `cppyy >=3.5,<4`. (Hand-written recipe;
      vinca not needed for a single package.)
      **Done (06ceef1):** recipe/ (recipe.yaml schema v1 + build.sh +
      vendored LICENSE); optional pixi `pkg` env with rattler-build and a
      `pkg-build` task (no-default-feature, default lock untouched). Builds
      `ros-jazzy-rclcppyy-0.1.0-hb0f4dca_0.conda` (~135 KiB), recipe import
      test green.
- [x] Local proof: `rattler-build build` then install the artifact into a
      fresh pixi env and run the smoke test — the package must work **without
      colcon build** (that's the point of releasing it).
      **Done:** fresh workspace outside the repo, channels = [local file://
      output, robostack-jazzy, conda-forge], single dep — `pixi install`,
      import + `enable_cpp_acceleration()`, and a 5/5 pub/sub roundtrip
      (builtin_interfaces/Time) with LD_LIBRARY_PATH empty and no repo on
      PYTHONPATH. Prerequisite fixed first (063f1a9): explicit
      `cppyy.load_library` of librclcpp / per-message typesupport /
      librosbag2_storage at bringup — `add_library_path` alone is NOT
      scanned for call-time resolution; LD-stripped full suite passes.
- [x] Create prefix.dev channel (`awesomebytes`); upload via
      `rattler-build upload prefix`.
      **RELEASED 2026-07-10:** `ros-jazzy-rclcppyy-0.1.0-hb0f4dca_0.conda`
      live on the channel. Auth via OIDC trusted publishing (prefix.dev
      Repository Access authorizes awesomebytes/rclcppyy release.yml;
      `id-token: write` in the workflow; no stored tokens — the initial
      PREFIX_API_KEY attempt was replaced, 9474f1d). Verified end-to-end from
      a fresh workspace: install from `https://repo.prefix.dev/awesomebytes`
      → `enable_cpp_acceleration()` works. NOTE the conda channel URL is
      `https://repo.prefix.dev/awesomebytes` — the `prefix.dev/channels/...`
      form is the web UI only and 404s for conda clients.
- [x] Release workflow: on git tag → build → test → upload (with
      `PREFIX_API_KEY` secret).
      **Done (1a54dbc):** release.yml on `v*` tags — pkg-build → artifact
      proof in a throwaway workspace (mirrors the local proof) → upload.
      Unexercised on GitHub until first push + tag.
- [x] README "Install" section:
      ```toml
      channels = ["https://repo.prefix.dev/awesomebytes", "robostack-jazzy", "conda-forge"]
      [dependencies]
      ros-jazzy-rclcppyy = "*"
      ```

**Acceptance:** a user with no clone of this repo gets a working
`import rclcppyy; rclcppyy.enable_cpp_acceleration()` from
`pixi add ros-jazzy-rclcppyy` alone.

## Phase 5 — Post-release maturity (stretch, prioritized)

1. **Bringup time** (README TODO): precompile a cppyy dictionary
   (`.pcm` + `.so`) for `rclcpp.hpp` at build/install time, or cache the first
   JIT per machine — target well under the current ~2.5 s.
2. **Multi-distro**: add `humble` / `kilted` pixi features + envs (channels
   verified to exist); CI matrix; per-distro packages (`ros-humble-…`,
   `ros-kilted-…`).
3. **linux-aarch64** platform in workspace, CI, and recipes.
4. **IDE stubs** (`scripts/create_stubs.py`, currently WIP) shipped with the
   package.
5. **Split demos from core** (roadmap item): `rclcppyy` (core) +
   `rclcppyy_demos` once the core API stabilizes.

## Experiments (exploratory, beyond the release plan)

- **bt_kit spike (2026-07-10, GO):** BehaviorTree.CPP 4.9 driven from Python
  via cppyy — official tutorials run verbatim (XML unchanged) with Python leaf
  nodes in 16–24 user LOC; bringup ~0.85 s; ~630k ticks/s with Python leaves
  (~1.28M with C++-JIT leaves) — boundary cost irrelevant at robot rates.
  API decision: **thin C++-mirror** (the kit patches cppyy's real
  `BehaviorTreeFactory`; `registerSimpleAction`/`createTreeFromText`/
  `tickWhileRunning` keep their C++ names, snake_case aliases accepted) —
  chosen over a decorator DSL because an LLM's existing BT.CPP knowledge
  transfers 1:1 and there's no hidden registry state; only stateful nodes
  deviate (`register_stateful`, since `registerNodeType<T>` can't take a
  Python class). Full probe matrix, benchmark, API comparison, gaps:
  `docs/bt_kit/REPORT.md`; explainer with C++-vs-Python side-by-side:
  `docs/bt_kit/WHY.md`; LLM cheat sheet: `docs/bt_kit/BT.CPP_KIT.md`.
  Kit: `rclcppyy/kits/bt_kit.py` (optional `bt` pixi env; default env
  untouched). Key findings: Python can author leaves only (BT marks tick()
  final → no cross-inheritance); cppyy has sharp edges (container
  construction can SIGSEGV; unique_ptr ownership across the boundary fails) —
  strongest argument FOR the curated-kit strategy of hiding cppyy.

- **bt_kit deep pass (2026-07-11, f77f422/87c67b9/4eba875):** 7/8 gaps closed —
  typed ports, per-tree-node stateful instances, observability (cout/file
  loggers, TreeObserver, Groot2Publisher — robostack's .so links ZMQ), GIL
  characterized (leaves tick on the tree thread; Parallel is cooperative;
  sleeping leaves release the GIL, no deadlock), readable `BtXmlError`,
  subtrees + v4 scripting free, kit test suite (auto-skips outside the bt env;
  `test-bt` = 7 green). AOT probe (honest partial): a ROOT dictionary builds
  and loads in 0.02 s but does NOT skip the 0.83 s header parse — Cling
  lazily parses on first class use; real L1 freeze needs a Cling C++
  module/PCH for the header (documented as a dedicated next step).
- **pcl_kit spike (2026-07-11, GO; 3febabf/0595306/2c4f7bf, merged 1ae684d):**
  PCL from Python via cppyy — on-demand template instantiation incl. point
  types no wrapper ever shipped and a custom `POINT_CLOUD_REGISTER`'d struct;
  NumPy→cloud floor is one C++ memcpy (0.49 ms @100k pts; a Python loop is
  ~90× worse); zero-copy view out. Showcase, clean-machine re-measure
  (subscribe → VoxelGrid → publish, 100k pts @10 Hz): pcl_kit 3.85 ms avg /
  6.5 % CPU vs rclpy+NumPy 57.0 ms / 61.3 % CPU — **14.8× lower latency,
  9.4× less CPU, at 76 vs 77 user LOC**. Docs mirror bt_kit
  (`docs/pcl_kit/`).
- **cppyy_kit reconciliation (2026-07-11, 06f541c/b4ea0a2/4bc1adb):** common
  layer `rclcppyy/kits/cppyy_kit.py` (119 code lines: load_libraries,
  keep_alive, std_function, HandleRegistry, unwrap_expected,
  pretty_cpp_error, probe_cppdef, package_prefix); both kits refactored onto
  it behavior-preserving (bench smokes unchanged); cut criterion: recurring
  cppyy *friction* generalizes, library semantics stay kit-local. Merged
  13-pattern catalog with evidence from both kits + the Today-vs-L1 AOT
  finding: `docs/kits/COMMON_PATTERNS.md` (the doc future kit authors and LLM
  agents read first). 9 new tests; `test-bt` now runs 16.
  Open next steps: L1 freeze sub-project (Cling C++ module/PCH),
  third kit (Nav2 bridge-plugin / OMPL / Ceres), README section for kits.

- **Teardown fix (2026-07-11, 7d3faf6/b5d5f82/2cf2688):** the exit segfault is
  NOT reproducible on the current stack (~8 scenarios × both RMWs, all clean) —
  the `os._exit` dodges were vestigial since the destroy_node/keep-alive fixes.
  All 6 removed. The in-principle hazard (no ordering contract between Python
  finalization and Cling teardown for C++ objects owning process-global state)
  is now guarded: `cppyy_kit.register_teardown/shutdown` (LIFO, idempotent,
  atexit-wired in the correct window) + once-only `rclcppyy.shutdown_rclcpp`
  (also closes the double-rcl_shutdown race). Regression tripwire
  `test/test_clean_exit.py` in the default suite (now 7 tests).
- **L0→L1→L2 ladder demonstrated (2026-07-11, 112a56c, merged e6cb773):**
  the freeze is real — a **Cling PCH** (`rootcling -generate-pch` on top of
  cppyy's std set; `CLING_STANDARD_PCH` must be set before the first cppyy
  import → `scripts/freeze/run_frozen.py` launcher). Measured: header parse
  890 ms → 6 ms (~140×), bringup ~950 → ~90 ms (10.7×), t01 end-to-end
  1.9 → 1.1 s; same recipe on rclcpp.hpp: 1.71 s → 6 ms (~290×). The same 16+
  kit tests pass frozen (tests-as-contract). Remaining cost identified
  precisely: cppyy's first-use JIT of per-signature call wrappers (~0.69 s,
  identical L0/L1) — next target needs L2 or cached instantiations.
  **L2 also works**: ApproachObject leaf lowered to a native plugin `.so`
  (registerFromPlugin, JIT-free), identical output on the same XML
  (differential-tested), 0.57 → 0.20 µs/tick (~2.8×). Recipe:
  `docs/kits/FREEZE.md`; artifacts gitignored + env-version-tagged.

- **Callback ergonomics v2 (2026-07-11, cd57ef4/0c15487/4acc163):**
  `cppyy_kit.callback(fn, signature=None, owner=None)` — one line,
  C++ signature inferred from Python type hints (int/float/bool/str/void +
  cppyy proxy classes as `T&`; Python-default params skipped; explicit
  `signature=` wins; unmappable params fail EARLY with a named
  `CppyyKitError`), lifetime always auto-pinned (owner object or module
  registry + `release_callbacks()`), so "callable was deleted" is no longer
  silently reachable. C++→Python needs no helper (a cppdef'd fn IS a Python
  callable) — documented + round-trip tested. bt_kit refactored onto it.
  test-bt 31 / test-bt-frozen 29, all green; frozen path unregressed.

- **Warmup + first-use JIT (2026-07-11, fcee3ec/1cdd216/2459642):** Sam's
  FREEZE.md feedback implemented. `cppyy_kit.first_use()` instruments kit-owned
  entry points — one-time, suppressible (RCLCPPYY_JIT_NOTICE=0), LLM-actionable
  notice naming the exact warmup() to call; `bt_kit.warmup()` /
  `pcl_kit.warmup()` + generic runner. Measured: first live tick 678→98 ms,
  pcl showcase frame-0 630→4 ms; **best-case cold start = freeze + warmup
  (~85 ms bringup + ~0.9 s one-time init, then everything fast)**. The cost
  itself is NOT reducible in cppyy 3.5 (Clang front-end template
  instantiation; -O0 identical; PCH is AST-only, carries no thunk codegen) —
  relocate (warmup) or eliminate (L2 native plugin) are the answers, both
  shipped. FREEZE.md §4 now answers the "what remains" question directly.
- **README + backlog batch (2026-07-11, 0d34cbd/5130cf7/1784625, merged
  1d01170):** README "Kits" section with measured-numbers table + quick-run;
  primitive-sequence conversion fixed (root cause deeper than expected: rclpy
  uses list/array.array representations per element type — octet/boolean/
  string sequences were also broken; bulk std::vector[T](field) construction,
  Float64MultiArray regression test; default suite now 8); checkout@v5 in
  both workflows (Node-20 deprecation gone).

- **ompl_kit spike (2026-07-11, GO; ef4a222/329590a/ddaeeee, merged f20c60f):**
  third kit, thinnest yet (72 LOC, zero embedded C++ — cppyy absorbs
  inheritance/ownership/downcast). **Cross-inheritance headline confirmed**
  (the pattern BT's `final` blocked): Python subclass of StateValidityChecker
  called 170× by C++ RRTConnect; Python `motionCost` called 1,034,069× in a
  1 s RRTstar solve. Mechanics: super().__init__ chaining, exact C++ virtual
  names, RTTI auto-downcast of pointer args, keep_alive the instance.
  Honest hot-loop number: Python validity ~159× slower per call (282 ns vs
  1.8 ns) — invisible at 136 calls, ~0.35 s at 1M; the lowering story (6-line
  cppdef checker) is the in-script answer. Bringup ~538 ms (cheapest kit).
  callback() dogfood found a real gap: class-typed hints infer `T&` but OMPL
  needs `const T*` — succeeds silently, fails at the setter. **Fixed
  (88c6165):** verbatim C++ type strings as annotations
  (`s: "const ompl::base::State*"`) + a one-time warning on bare class-hint
  reference inference; regression-tested. COMMON_PATTERNS gained §16–18
  (cross-inheritance mechanics, shared_ptr/RTTI, reserved-word methods);
  README Kits section now lists all three kits (63b0535). Demos incl.
  nav_msgs/Path publishing via rclcppyy. Docs mirror the structure
  (`docs/ompl_kit/`).

- **nav2_kit spike (2026-07-11, GO; d2b8362, merged d69bd09):** "your own nav
  stack" from Nav2's separable algorithm cores — `Costmap2D` + `NavFn` driven
  from Python (no lifecycle servers/pluginlib/tf); showcase = complete
  miniature nav stack in one file (world → costmap → C++ NavFn plan →
  Python pure-pursuit follow → live OccupancyGrid/Path/TwistStamped topics,
  GOAL REACHED verified). numpy→costmap = one memcpy (~600–3600× a Python
  loop, third confirmation of the bulk-buffer lesson). Bench: NavFn ~80×
  a pure-Python A* at 512²+. Honest boundary: Smac + RPP controller are
  lifecycle-coupled (BLOCKED standalone — key finding: grep ctors for
  LifecycleNode/*ROS coupling BEFORE committing to a core). New gotcha:
  `unsigned char` crosses as 1-char str, not int (`ord()` needed). Agent
  couldn't write REPORT.md (harness quirk) — supervisor committed it from
  delivered content. Docs: `docs/nav2_kit/`.

## Risks & mitigations

- **conda-forge cppyy behaves differently from the pip wheel** (cling resource
  dir, compiler header lookup). Mitigation: Phase 1 validates before anything
  is deleted; committed `pixi.lock` pins known-good versions; `EXTRA_CLING_ARGS`
  documented as escape hatch.
- ~~**cppyy needs `LD_LIBRARY_PATH=$CONDA_PREFIX/lib` at runtime**~~
  **RESOLVED (063f1a9):** `bringup_rclcpp`/`serialization` now explicitly
  `cppyy.load_library` the needed libs (librclcpp eagerly; per-message
  typesupport on demand; librosbag2_storage for serialization), so nothing
  depends on `LD_LIBRARY_PATH` anymore. The activation line stays in
  pixi.toml as belt-and-braces only.
- ~~**Known limitation (found in Phase 4 proof):** nested Python messages
  don't auto-convert on the plain-bringup publish path.~~
  **RESOLVED (bca982e):** one shared recursive converter
  (`convert_python_msg_to_cpp` in `bringup_rclcpp.py`) now serves both the
  plain-bringup publish wrapper and `RclcppyyNode`; nested-message regression
  test added (suite = 6 tests, LD-stripped sanity re-verified).
- **Known limitation (remaining, tracked in backlog):** the shared converter
  resolves any non-empty sequence element as a *message*, so non-empty
  primitive arrays (e.g. `float64[]`) raise during Python→C++ conversion on
  the plain-bringup publish path — preserved as-is for parity with the
  original monkeypatch-path behavior. Fix: detect primitive element types and
  fill `std::vector<primitive>` directly.
- **cppyy has no rosdep key**, so `package.xml` can't declare it portably.
  Acceptable: distribution targets conda/pixi where the recipe declares it
  directly; document apt/rosdep as unsupported for now.
- **robostack version drift** (e.g. `ros-jazzy-ros-base` 0.11 pin in README).
  Use `>=0.11` in the manifest and let `pixi.lock` do the pinning; CI catches
  breakage on re-lock.
- **Monkeypatching fragility across rclpy versions**: smoke tests in CI are
  the tripwire; multi-distro matrix (Phase 5) widens coverage.
