# Reviewed upstream rclpy contract gate

The repository runs a bounded selection of the upstream rclpy tests with
rclcppyy's compatible backend enabled. This is independent compatibility evidence,
not a vendored copy of upstream tests and not a claim that unselected tests pass.

The machine-readable policy is
`compatibility/upstream-rclpy-contract.json`. It fixes all of the following:

- the upstream repository, full Git commit, release, and installed package version;
- a digest of every `rclpy/test/test_*.py` path and its contents;
- explicit hashes for non-test helper/resource files needed by selected tests;
- the exact selected test files and why each is in the gate;
- every unselected test file, grouped under an explicit reviewed exclusion reason.

The selected and excluded paths must exactly partition the pinned inventory. A
branch name, tag, dirty checkout, different remote, version mismatch, added test,
renamed test, changed test, or unreviewed path makes validation fail. Updating the
gate therefore requires reviewing and committing a new manifest, not silently
following upstream.

## Acquire once, run offline

Source acquisition is intentionally separate from validation and execution:

```bash
git clone https://github.com/ros2/rclpy.git _deps/rclpy-contract
git -C _deps/rclpy-contract checkout --detach \
  baf9d72cfa127e391a89b4ab51ba9e55c37041fd

pixi run -e upstream-contract upstream-contract-validate
pixi run -e upstream-contract upstream-contract
```

After the checkout and Pixi environment have been acquired, both commands run
without network access. Set `RCLPY_CONTRACT_SRC` to use another location. The
optional `upstream-contract` Pixi environment carries `test_msgs`; the default
runtime and development environment does not gain that test-only dependency.

Each selected upstream file runs in a fresh process. Pinned helper files and
resources are copied beside it in a neutral temporary package, which supports
relative imports and file-relative resource lookup without placing the upstream
`rclpy` source package on `PYTHONPATH`. That preserves the upstream
suite's process-global context assumptions and prevents one file's signal,
executor, or shutdown state from contaminating another. The runner stages only the
selected file, so it uses the installed rclpy package rather than importing Python
modules from the source checkout.

The bootstrap must be active in every child process. The publisher selection also
requires a completed C++ publish record and rejects any Python publish fallback.
JUnit files and `summary.json` are written under
`build/test-results/upstream-rclpy/`.

## Scope and updates

The reviewed runtime slice covers actions and action graph queries, callback
groups, clients, concurrent entity creation, active entity destruction,
single-threaded, multi-threaded, and event-driven executors, guard conditions,
context lifecycle, lifecycle nodes, logging and rosout, node graph and control-plane
behavior, local and remote parameters, publishers, QoS and QoS events, rates,
serialization, services and service introspection, subscriptions, simulated time,
timers, message waiting, and custom waitables.

Exclusions are visible follow-up work and must not be reported as passing
coverage. Two pinned upstream files are excluded separately because they fail
unchanged with stock `rclpy==7.1.11`: the installed pybind Node rejects the
destruction-order test's ad hoc `node.handle.context_handle` assignment, and the
stock client rejects a graph-derived `TypeHash` in the type-description request.
These are stock baseline failures, not architecture or RMW skips and not backend
evidence.

To update the pin or selection:

1. Check out the proposed full commit and confirm its `rclpy/package.xml` version
   matches the installed package intended for CI.
2. Review every upstream `test_*.py` addition, removal, rename, and content
   change.
3. Recompute the inventory with the same
   `sha256-path-content-v1` algorithm implemented by the runner.
4. Assign every file to the selection or one substantive exclusion group.
5. Run the focused validator tests and the live contract on x86_64 and ARM64.

The normal CI workflow checks out the exact revision directly. The runner never
fetches, checks out, or modifies upstream source.
