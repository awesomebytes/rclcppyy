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

- [ ] Add `pixi.toml` at repo root (workspace = repo; `colcon` discovers the
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
- [ ] Validate conda-forge cppyy end-to-end in that env: `bringup_rclcpp()`,
      pub/sub monkeypatched benchmarks, tutorial example. This is the only
      unproven assumption in this plan — do it first. Keep
      `EXTRA_CLING_ARGS` as a documented escape hatch if cling needs flags.
- [ ] Delete `fix_cppyy_api_path.sh` once validated.
- [ ] Commit `pixi.lock` for reproducibility.
- [ ] Add `.gitignore` (`.pixi/`, `build/`, `install/`, `log/`, `__pycache__/`,
      `.hypothesis/`); remove committed `.hypothesis/` data.
- [ ] Pixi tasks: `build` (`colcon build --packages-select rclcppyy`), `clean`,
      `test`.
- [ ] Rewrite README setup section: `git clone && cd rclcppyy && pixi run build`
      — remove the workaround narrative and the stale "wait a few days" banner.

**Acceptance:** on a clean machine with only pixi installed:
`git clone … && cd rclcppyy && pixi run build && pixi run test` succeeds with
no manual steps.

## Phase 2 — Examples & benchmarks in a couple of commands

- [ ] Benchmark runner (`scripts/benchmarks/run_benchmarks.py`): spawns
      pub+sub pairs (rclpy vs rclcppyy), samples CPU via `psutil` for N
      seconds, prints a comparison table matching the README numbers.
      Replaces the current 4-shells-plus-`top` workflow.
      - `pixi run bench` → full comparison at 1 kHz and 10 kHz
      - `pixi run bench -- --rate 10000` → parameterized
- [ ] Demo tasks: `pixi run demo-tutorial` (publisher_member_function),
      `pixi run demo-pubsub` (pub+sub pair in one command).
- [ ] Optional `demos` feature/environment for heavy extras (`opencv`,
      `ros-jazzy-pcl-conversions`, `pcl`, `gstreamer`/`pygobject`, `typer`,
      `hypothesis`) so the default env stays lean. Mark `roscon_uk_2025/` as a
      presentation archive (needs external bag file / cloudini checkout) —
      exclude from install.
- [ ] Repo/packaging hygiene:
  - fix `test/test_node.py` (export `Node = RclcppyyNode` alias or fix import)
  - add smoke tests: bringup, rclpy-API pub/sub roundtrip through C++ backend,
    message monkeypatch, serialization parity
  - delete unused `setup.py`; fix `CMakeLists.txt` install to
    `FILES_MATCHING PATTERN "*.py"`; remove `scripts/target`
  - complete `package.xml` deps (`ament_index_python`, `rosidl_runtime_py`,
    `rosbag2_py`, `std_msgs`, `sensor_msgs`, `python3-numpy`); bump to 0.1.0

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
- **cppyy has no rosdep key**, so `package.xml` can't declare it portably.
  Acceptable: distribution targets conda/pixi where the recipe declares it
  directly; document apt/rosdep as unsupported for now.
- **robostack version drift** (e.g. `ros-jazzy-ros-base` 0.11 pin in README).
  Use `>=0.11` in the manifest and let `pixi.lock` do the pinning; CI catches
  breakage on re-lock.
- **Monkeypatching fragility across rclpy versions**: smoke tests in CI are
  the tripwire; multi-distro matrix (Phase 5) widens coverage.
