# Releasing rclcppyy (0.2.0+)

As of 0.2.0, rclcppyy is the **drop-in rclpy accelerator product built on top of the
cppyy_kit suite**. Its rclcpp core was carved into the standalone `rclcpp_kit`
package, and the cppyy kits into their own packages (the
[cppyy_kit](https://github.com/awesomebytes/cppyy_kit) repo). rclcppyy re-exports the
moved pieces through deprecation shims, so it now **depends on the suite at runtime**:

- `rclcppyy.bringup_rclcpp` / `serialization` / `rosbag2_cpp` / `rosbag2_py_compat` /
  `tf` → re-export `rclcpp_kit.*` (conda: `ros-jazzy-rclcpp-kit`).
- `rclcppyy.kits.cppyy_kit` / `.freeze` → re-export `cppyy_kit` (conda: `cppyy-kit`).
- `rclcppyy.kits.{bt,pcl,ompl,nav2,moveit,control,cv,dbow}_kit` → re-export the
  standalone kit packages, which the user installs separately if wanted.

This creates a **publish order**: the suite must be on the prefix.dev `awesomebytes`
channel before an rclcppyy release can build, prove, and upload.

## Pre-release dev/test wiring (current state, suite unpublished)

The suite is not on `awesomebytes` yet (that needs Sam's prefix.dev authorization).
Until then:

- **Dev + CI import** rclcpp_kit / cppyy_kit from a **sibling `cppyy_kit` source
  checkout** via a PYTHONPATH bridge — `workspace_activation.sh` puts exactly those
  two packages on `PYTHONPATH` (default `../cppyy_kit`; override with `CPPYY_KIT_SRC`).
  CI (`.github/workflows/ci.yml`) checks out `awesomebytes/cppyy_kit` and sets
  `CPPYY_KIT_SRC`. No file:// channel is committed to `pixi.lock` (it would not
  resolve on other machines / CI).
- **Local recipe validation** uses a **local rattler-build channel**: `pixi run -e
  pkg pkg-build` passes `-c file://.../cppyy_kit/output` so the recipe's import-smoke
  test env can resolve the suite run-deps. (Build the suite's local channel first in
  the cppyy_kit repo: `pixi run -e pkg <suite pkg-build>`.)

## Release choreography (do these in order)

**Do not tag rclcppyy until step 1 is done.**

1. **Publish the suite** from the cppyy_kit repo to `awesomebytes` (needs Sam's
   prefix.dev auth): `cppyy-kit`, `ros-jazzy-rclcpp-kit`, and the domain kits, all
   artifact-proven from a fresh env first (the suite's own release discipline).

2. **Swap rclcppyy from the pre-release bridge to the published channel:**
   - `pixi.toml`: add channel `https://repo.prefix.dev/awesomebytes`; add deps
     `ros-jazzy-rclcpp-kit` and `cppyy-kit` (pin to the published version); re-lock
     (`pixi lock`).
   - Remove the pre-release bridge: the `# M3 PYTHONPATH bridge` block in
     `workspace_activation.sh`, and the `_cppyy_kit` checkout + `CPPYY_KIT_SRC` env in
     `.github/workflows/ci.yml`. (Real conda deps replace them.)
   - Optional: point the `pkg-build` task's `-c` at `awesomebytes` instead of the
     local path (or leave it for local dev; the release workflow already uses
     `awesomebytes` directly).

3. **Verify** on a clean checkout: `pixi run build`, `pixi run lint`, `pixi run test`
   (core suite + kit-shim smokes), `pixi run bench --variants rclcppyy --rate 1000
   --duration 3 --json` (≈3000 msgs, 0 dropped). Push; CI must be green.

4. **Tag `v0.2.0`.** `.github/workflows/release.yml` then: builds the
   `ros-jazzy-rclcppyy` conda package (recipe run-deps resolve from `awesomebytes`),
   proves it installs + runs a pub/sub roundtrip in a throwaway workspace whose
   channels include `awesomebytes`, and uploads via OIDC. `recipe/recipe.yaml` and
   `release.yml` are already prepared for this (they reference `awesomebytes`); step 2
   only touches the dev/CI wiring.

## Deprecation timeline

The `rclcppyy.*` re-export shims and `rclcppyy.kits.*` shims emit `DeprecationWarning`
(except `rclcppyy.bringup_rclcpp`, which the product itself uses internally). They
keep existing imports working across 0.2.x; plan removal for a later major bump once
downstreams have moved to `rclcpp_kit.*` / the standalone kit packages.
