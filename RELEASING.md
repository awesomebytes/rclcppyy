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

## Channel swap: done (suite published, bridge removed)

The suite is live on `awesomebytes` (all 11 packages, artifact-proven). The
pre-release PYTHONPATH bridge described below has been removed:

- `pixi.toml` now carries `https://repo.prefix.dev/awesomebytes` as a channel and
  `ros-jazzy-rclcpp-kit ==0.1.0` / `cppyy-kit ==0.1.0` as real conda deps (re-locked).
- `workspace_activation.sh` no longer sets up the `.suite_bridge` PYTHONPATH shim.
- `.github/workflows/ci.yml` no longer checks out `awesomebytes/cppyy_kit` or sets
  `CPPYY_KIT_SRC` — rclcpp_kit/cppyy_kit resolve as ordinary conda deps in the
  `default` pixi env.
- `pkg-build`'s `-c` now points at `https://repo.prefix.dev/awesomebytes` instead of
  a local rattler-build channel.

The old wiring (kept here for history / for anyone bringing up a similar bridge
before their own suite is published):

- **Dev + CI import** rclcpp_kit / cppyy_kit from a **sibling `cppyy_kit` source
  checkout** via a PYTHONPATH bridge — `workspace_activation.sh` put exactly those
  two packages on `PYTHONPATH` (default `../cppyy_kit`; override with `CPPYY_KIT_SRC`).
  CI checked out `awesomebytes/cppyy_kit` and set `CPPYY_KIT_SRC`. No file:// channel
  was committed to `pixi.lock` (it would not resolve on other machines / CI).
- **Local recipe validation** used a **local rattler-build channel**: `pixi run -e
  pkg pkg-build` passed `-c file://.../cppyy_kit/output` so the recipe's import-smoke
  test env could resolve the suite run-deps.

## Release choreography (do these in order)

**Do not tag rclcppyy until step 1 is done.**

1. ~~**Publish the suite** from the cppyy_kit repo to `awesomebytes`~~ — **done.**
   `cppyy-kit`, `ros-jazzy-rclcpp-kit`, and the domain kits are live, all
   artifact-proven from a fresh env first (the suite's own release discipline).

2. ~~**Swap rclcppyy from the pre-release bridge to the published channel**~~ — **done**
   (see "Channel swap: done" above).

3. **Verify** on a clean checkout: `pixi run build`, `pixi run lint`, `pixi run test`
   (core suite + kit-shim smokes), `pixi run bench --variants rclcppyy --rate 1000
   --duration 3 --json` (≈3000 msgs, 0 dropped). Push; CI must be green.

4. **Tag `v0.2.0`.** `.github/workflows/release.yml` then: builds the
   `ros-jazzy-rclcppyy` conda package (recipe run-deps resolve from `awesomebytes`),
   proves it installs + runs a pub/sub roundtrip in a throwaway workspace whose
   channels include `awesomebytes`, and uploads via OIDC. `recipe/recipe.yaml` and
   `release.yml` are already prepared for this (they reference `awesomebytes`); step 2
   only touched the dev/CI wiring.

## Deprecation timeline

The `rclcppyy.*` re-export shims and `rclcppyy.kits.*` shims emit `DeprecationWarning`
(except `rclcppyy.bringup_rclcpp`, which the product itself uses internally). They
keep existing imports working across 0.2.x; plan removal for a later major bump once
downstreams have moved to `rclcpp_kit.*` / the standalone kit packages.
