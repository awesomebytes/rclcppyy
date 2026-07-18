# Releasing rclcppyy 0.3.0

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

## Dependency identities

Suite 0.1.0 is published on `awesomebytes`, but this branch requires the unreleased
suite 0.2.0 API. Development, CI, and release therefore use two explicit lanes:

- **Source lane:** `suite-source.lock.json` names one full suite commit and exact
  recipe version. Workspace activation overlays only that revision. The
  `suite-contract` task verifies its Git identity, clean state, all recipe versions,
  and active Python roots. CI checks out the same full commit.
- **Installed lane:** the package job builds suite 0.2.0 and rclcppyy 0.3.0 into one
  isolated local channel, then proves imports and real ROS behavior without source
  paths on x86_64 and ARM64. The ARM64 stack includes the suite's source-locked
  `cppyy` 3.5.0 bridge over conda-forge's compiled components.

The default lock still contains published suite 0.1.0 only as bootstrap dependency
metadata. It is not accepted as source-test evidence, cannot satisfy
`suite-contract`, and is not the release dependency set.

## Release choreography (do these in order)

**Do not tag rclcppyy until step 1 is done.**

1. **Tag and publish suite `v0.2.0`** from its locked commit after its release job
   builds, freshly installs, checksums, and attests the 11 suite artifacts plus
   the native ARM64 `cppyy` bridge. Prefix.dev OIDC authorization for that
   repository must already be enabled.

2. **Confirm the published dependency set.** Build rclcppyy from a clean checkout
   against `cppyy-kit ==0.2.0` and `ros-jazzy-rclcpp-kit ==0.2.0` on
   `awesomebytes`; do not use the source overlay for this proof.

3. **Verify** source and installed lanes: `pixi run suite-contract`, `pixi run build`,
   `pixi run lint`, `pixi run test`, the backend-required benchmark smoke, and the
   local package-stack proof. Push; all required x86-64 and ARM64 source and
   installed-package jobs must be green. The ARM proof must contain the clean
   suite commit, exact upstream source and patch hashes, local bridge artifact
   hash, and native import/`cppdef` runtime-log hash.

4. **Tag `v0.3.0`.** The release workflow first rejects any tag that differs from
   `pixi.toml`, `package.xml`, or `recipe/recipe.yaml`. It then builds the
   x86_64 and ARM64 `ros-jazzy-rclcppyy` conda packages from immutable local stack
   snapshots, proves each installs and runs same-handle pub/sub plus a native
   service in a throwaway workspace, records architecture-specific checksums,
   provenance, SBOM, and raw gate evidence, and uploads both via OIDC.

## Deprecation timeline

The `rclcppyy.*` re-export shims and `rclcppyy.kits.*` shims emit `DeprecationWarning`
(except `rclcppyy.bringup_rclcpp`, which the product itself uses internally). They
keep existing imports working across 0.3.x; plan removal for a later major bump once
downstreams have moved to `rclcpp_kit.*` / the standalone kit packages.
