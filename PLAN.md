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
- [ ] Benchmark runner (`scripts/benchmarks/run_benchmarks.py`): spawns
      pub+sub pairs (rclpy vs rclcppyy), samples CPU via `psutil` for N
      seconds, prints a comparison table matching the README numbers.
      Replaces the current 4-shells-plus-`top` workflow. Note: the existing
      bench scripts crash on SIGINT teardown (pre-existing double-shutdown /
      segfault after last message) — the runner must own child-process
      lifecycle and not treat teardown exit codes as benchmark failure.
      - `pixi run bench` → full comparison at 1 kHz and 10 kHz
      - `pixi run bench -- --rate 10000` → parameterized
- [ ] Demo tasks: `pixi run demo-tutorial` (publisher_member_function),
      `pixi run demo-pubsub` (pub+sub pair in one command).
- [ ] Optional `demos` feature/environment for heavy extras (`opencv`,
      `ros-jazzy-pcl-conversions`, `pcl`, `gstreamer`/`pygobject`, `typer`,
      `hypothesis`) so the default env stays lean. Mark `roscon_uk_2025/` as a
      presentation archive (needs external bag file / cloudini checkout) —
      exclude from install.
- [x] Repo/packaging hygiene (done 2026-07-10, cb2114d + a021d17):
  `Node` alias exported and `test_node.py` green (the alias exposed a real
  `RclcppyyNode.destroy_node` bug — cppyy publishers stored in rclpy's
  `_publishers` broke rclpy teardown — fixed); `setup.py` and `scripts/target`
  removed; CMake install now `FILES_MATCHING PATTERN "*.py"` (verified: only
  .py files install); `package.xml` deps completed from actual imports,
  version bumped to 0.1.0.
- [ ] Add smoke tests: rclpy-API pub/sub roundtrip through C++ backend,
  message monkeypatch, serialization parity (today only `test_bringup.py`
  and `test_node.py` exist)

**Acceptance:** `pixi run bench` produces the rclpy-vs-rclcppyy CPU table in
one command; every demo listed in the README runs via a single `pixi run` task.

## Phase 3 — CI

- [ ] GitHub Actions using `prefix-dev/setup-pixi` with pixi-env caching:
      `pixi run build` + `pixi run test` + a short headless `pixi run bench`
      smoke (few seconds, assert rclcppyy actually publishes) on linux-64.
- [ ] Lint job (existing `ament_lint_auto` via `colcon test`).
- [ ] Badge in README.

**Acceptance:** PRs get a green/red signal; the Phase 1 guarantee is enforced
mechanically, not by discipline.

## Phase 4 — Conda package on prefix.dev

- [ ] Write a `rattler-build` `recipe/recipe.yaml`, robostack-style:
      package `ros-jazzy-rclcppyy`, standard ament_cmake build script,
      `host`/`run` deps on `ros-jazzy-rclcpp`, `ros-jazzy-rclpy`,
      `ros-jazzy-ament-index-python`, `cppyy >=3.5,<4`. (Hand-written recipe;
      vinca not needed for a single package.)
- [ ] Local proof: `rattler-build build` then install the artifact into a
      fresh pixi env and run the smoke test — the package must work **without
      colcon build** (that's the point of releasing it).
- [ ] Create prefix.dev channel (e.g. `https://prefix.dev/channels/rclcppyy`);
      upload via `rattler-build upload prefix`.
- [ ] Release workflow: on git tag → build → test → upload (with
      `PREFIX_API_KEY` secret).
- [ ] README "Install" section:
      ```toml
      channels = ["https://prefix.dev/channels/rclcppyy", "robostack-jazzy", "conda-forge"]
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

## Risks & mitigations

- **conda-forge cppyy behaves differently from the pip wheel** (cling resource
  dir, compiler header lookup). Mitigation: Phase 1 validates before anything
  is deleted; committed `pixi.lock` pins known-good versions; `EXTRA_CLING_ARGS`
  documented as escape hatch.
- **cppyy needs `LD_LIBRARY_PATH=$CONDA_PREFIX/lib` at runtime** for call-time
  symbol resolution; the pixi workspace provides it via activation. The Phase 4
  conda package has no workspace activation — the recipe needs an `activate.d`
  script, or `bringup_rclcpp` should load the needed libraries explicitly
  (`cppyy.load_library`), which would also fix it for all install methods.
- **cppyy has no rosdep key**, so `package.xml` can't declare it portably.
  Acceptable: distribution targets conda/pixi where the recipe declares it
  directly; document apt/rosdep as unsupported for now.
- **robostack version drift** (e.g. `ros-jazzy-ros-base` 0.11 pin in README).
  Use `>=0.11` in the manifest and let `pixi.lock` do the pinning; CI catches
  breakage on re-lock.
- **Monkeypatching fragility across rclpy versions**: smoke tests in CI are
  the tripwire; multi-distro matrix (Phase 5) widens coverage.
