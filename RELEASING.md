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
- **CI installed lane:** the package job builds suite 0.2.0 and rclcppyy 0.3.0 into
  one isolated local channel, then proves imports and real ROS behavior without
  source paths on x86_64 and ARM64.
- **Release installed lane:** each native runner first downloads the exact published
  suite artifacts, verifies their channel digests and GitHub provenance against
  `suite-source.lock.json`, and exposes only those retained bytes through a local
  conda channel. The product build and throwaway install proof both consume that
  channel. ARM64 also retains and verifies the published native `cppyy` 3.5.0
  bridge.

The default lock still contains published suite 0.1.0 only as bootstrap dependency
metadata. It is not accepted as source-test evidence, cannot satisfy
`suite-contract`, and is not the release dependency set.

## Release choreography (do these in order)

**Do not tag rclcppyy until step 1 is done.**

1. **Tag and publish suite `v0.2.0`** from its locked commit after its release job
   builds, freshly installs, checksums, and attests the 11 suite artifacts plus
   the native ARM64 `cppyy` bridge. Prefix.dev OIDC authorization for that
   repository must already be enabled.

2. **Confirm the published dependency set.** The exact `cppyy-kit ==0.2.0` and
   `ros-jazzy-rclcpp-kit ==0.2.0` build identities must exist on `awesomebytes`
   with provenance from the locked suite commit. The same applies to the exact
   native `cppyy ==3.5.0` bridge identity on ARM64. A matching version or build
   string without matching retained channel bytes is not release evidence.

3. **Verify** source and installed lanes: `pixi run suite-contract`, `pixi run build`,
   `pixi run lint`, `pixi run test`, the backend-required benchmark smoke, and the
   local package-stack proof. Push; all required x86-64 and ARM64 source and
   installed-package jobs must be green. The ARM proof must contain the clean
   suite commit, exact upstream source and patch hashes, local bridge artifact
   hash, and native import/`cppdef` runtime-log hash.

4. **Tag `v0.3.0`.** The release workflow first rejects any tag that differs from
   `pixi.toml`, `package.xml`, or `recipe/recipe.yaml`. It then retains the exact
   published dependency bytes, builds the x86_64 and ARM64 product packages against
   them, and proves each throwaway install selected those same dependency hashes
   while running same-handle pub/sub plus a native service. Each runner records a
   validated conda inventory, file-level SPDX SBOM, provenance, and portable
   attestation bundles. A single publication job re-verifies both complete bundles
   before channel access. It rejects conflicting existing identities, uploads only
   missing product artifacts, and polls until both published architecture bytes
   match the verified local artifacts.

## Deprecation timeline

The `rclcppyy.*` re-export shims and `rclcppyy.kits.*` shims emit `DeprecationWarning`
(except `rclcppyy.bringup_rclcpp`, which the product itself uses internally). They
keep existing imports working across 0.3.x; plan removal for a later major bump once
downstreams have moved to `rclcpp_kit.*` / the standalone kit packages.
