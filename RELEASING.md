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

Suite 0.3.0 is published and is the locked suite dependency for the product. The
linux-64 Pixi package dependencies pin both suite packages to 0.3.0; source tests
on both architectures also check out the exact suite source commit from
`suite-source.lock.json`.

- **Source lane:** `suite-source.lock.json` names one full suite commit and exact
  recipe version. Workspace activation overlays only that revision. The
  `suite-contract` task verifies its Git identity, clean state, all recipe versions,
  and active Python roots. CI checks out the same full commit.
- **Routine CI lane:** on pushes and pull requests, native x86_64 and ARM64 jobs
  verify the pinned source, build, lint, validate the compatibility manifest, and
  run the nine selected `test-ci-fast` integration checks. This smoke lane is kept
  targeted to finish under 10 minutes per architecture; it is not the full release
  test suite.
- **Release preflight lane:** the version-tag workflow runs the full product test
  suite plus launch/custom-interface and reviewed upstream-contract checks on both
  native architectures before package jobs begin.
- **Release installed lane:** each native runner first downloads the exact published
  suite artifacts, verifies their channel digests and GitHub provenance against
  `suite-source.lock.json`, and exposes only those retained bytes through a local
  conda channel. The product build and throwaway install proof both consume that
  channel. ARM64 also retains and verifies the published native `cppyy` 3.5.0
  bridge.

The default Pixi manifest and lock use suite 0.3.0 on linux-64. ARM64 source tests
use the exact source pin and native cppyy bridge; release package jobs separately
verify the published ARM64 suite and bridge artifacts.

## Release choreography (do these in order)

**Do not tag rclcppyy until step 1 is done.**

1. **Confirm the published suite `v0.3.0` release.** Verify that its tag matches
   the locked commit and that its release job built, freshly installed, checksummed,
   and attested the 11 suite artifacts plus the native ARM64 `cppyy` bridge.
   Prefix.dev OIDC authorization for the product repository must be enabled before
   its package workflow runs.

2. **Confirm the published dependency set.** The exact `cppyy-kit ==0.3.0` and
   `ros-jazzy-rclcpp-kit ==0.3.0` build identities must exist on `awesomebytes`
   with provenance from the locked suite commit. The same applies to the exact
   native `cppyy ==3.5.0` bridge identity on ARM64. A matching version or build
   string without matching retained channel bytes is not release evidence.

3. **Push the product candidate and pass routine CI.** Both native x86_64 and ARM64
   jobs must pass the pinned-source check, build, lint, compatibility contract, and
   nine-test `test-ci-fast` smoke. The full suite, reviewed upstream contracts, and
   installed package proofs run in the version-tag release workflow, not this
   routine push/PR lane.

4. **Tag `v0.3.0`.** The release workflow first rejects any tag that differs from
   `pixi.toml`, `package.xml`, or `recipe/recipe.yaml`. Its x86_64 and ARM64
   preflight jobs run the full `test-ci` suite, reviewed upstream contracts, and
   launch/custom-interface proofs. After both preflights pass, the package jobs
   retain the exact published dependency bytes, build product packages against them,
   and prove each throwaway install selected those same dependency hashes while
   running same-handle pub/sub plus a native service. Each runner records a
   validated conda inventory, file-level SPDX SBOM, provenance, and portable
   attestation bundles. The inventory checks and records the product artifact's
   exact direct dependency metadata. The SPDX document covers the retained release
   artifacts and their files; external ROS and conda runtime packages are not
   expanded into SPDX package records or dependency relationships. A single
   publication job re-verifies both complete bundles before channel access. It
   rejects conflicting existing identities, uploads only missing product artifacts,
   and polls until both published architecture bytes match the verified local
   artifacts.

## Deprecation timeline

The `rclcppyy.*` re-export shims and `rclcppyy.kits.*` shims emit `DeprecationWarning`
(except `rclcppyy.bringup_rclcpp`, which the product itself uses internally). They
keep existing imports working across 0.3.x; plan removal for a later major bump once
downstreams have moved to `rclcpp_kit.*` / the standalone kit packages.
