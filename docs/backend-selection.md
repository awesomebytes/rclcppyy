# Backend selection and promotion policy

`rclcppyy` has a small product surface, but it exposes several execution lanes.
Choose the lane from the compatibility requirement first, then use measured backend
evidence to decide whether a more specialized lane is worthwhile.

## Decision table

| Requirement | Entry point | Contract | Failure behavior |
| --- | --- | --- | --- |
| Run existing `rclpy` software unchanged | `enable_cpp_acceleration()` | Exact stock Python object identities and behavior, including `Publisher.publish` | Operations remain stock Python and report that authority |
| Try same-handle C++ publishing without changing application code | `enable_cpp_acceleration(profile="publisher_cpp")` | Exact stock publisher object and graph endpoint; explicit publish implementation change | Falls back to stock publishing visibly if preparation or a publish fails |
| Prove that a selected operation cannot fall back | `enable_cpp_acceleration(profile="required_cpp")` | Exact stock contract for supported operations | Rejects an unsupported operation before its side effects |
| Use a C++-only ROS facility from Python | `rclcppyy.native()` | Explicit native API; not a drop-in `rclpy` replacement | Capability queries and normal exceptions make unsupported facilities visible |
| Remove Python from a measured hot path | `rclcpp_kit` native callbacks, services, clients, actions, components, lifecycle nodes, or fused pipelines | Explicit opt-in contract for ownership, scheduling, and delivery | Compilation and construction are explicit; generated code and counters are inspectable |
| Consider contract-changing automatic optimization | `profile="optimized"` | Only changes documented by an individually reviewed optimization | Stock executor waits are bounded to recover from a missed signal wake; no native lowering is silently enabled |

## Compatible profile

Use the default profile for existing applications:

```python
import rclcppyy

rclcppyy.enable_cpp_acceleration(warn_fallback=True)

# Existing imports and application code continue unchanged.
import rclpy
```

The original `Node`, `Context`, executors, generated messages, entities, and
`Publisher.publish` remain authoritative. `rclcppyy` does not create a companion
node or split ownership. This gives unedited software a conservative baseline with
backend reporting and no implicit publisher implementation change.

The compatibility manifest at `compatibility/jazzy.json` is the source of truth
for certified, stock-authoritative, experimental, unsupported, and unassessed
surfaces. Its closed upstream mapping accounts for every selected test and reviewed
exclusion in the pinned Jazzy contract. Stock-authoritative means compatible stock
Python behavior, not a C++ route; activation alone is not evidence that a particular
operation used C++.

## Publisher-C++ profile

Use the explicit permissive profile when the unchanged Python application should
try the certified same-handle publisher route:

```python
import rclcppyy

rclcppyy.enable_cpp_acceleration(profile="publisher_cpp")
```

The stock publisher object, node, graph endpoint, QoS, and destruction remain
authoritative. Only `Publisher.publish` converts and serializes through C++ against
the borrowed native handle. Preparation or runtime failure permanently taints that
publisher, falls back to the original stock operation, and is visible in status.

This is not the default because the historical controlled Jazzy/CycloneDDS relay
artifacts `build/relay-boundary-cyclone-117cc2d.json` and
`build/relay-boundary-cyclone-6396bd3.json` did not meet the promotion bar. Their
raw paired relay-CPU ratios were above stock in five of five and four of five
repetitions, respectively. Both artifacts explicitly prohibit interpretation and
performance claims; the observation is used only to choose the conservative
product default. They remain ignored local evidence, while the current six-lane
benchmark independently characterizes compatible and explicit publisher-C++ modes.

## Required-C++ profile

Use the strict profile in tests, benchmarks, and deployments where fallback would
invalidate the result:

```python
import rclcppyy

rclcppyy.enable_cpp_acceleration(profile="required_cpp")
```

An operation without a certified route raises `BackendUnavailableError` before
creating an entity or starting executor work. Stock infrastructure that is part of
preserving the compatible object model can still exist; the runtime status explains
its authority. Future completion, for example, remains stock-authoritative because
replacing it would destabilize executor semantics.

Strict mode is an assertion mechanism, not a promise that the whole application is
C++. Tests must check the operation records they depend on.

## Optimized profile

The optimized profile currently enables one explicit scheduling change:

```python
import rclcppyy

rclcppyy.enable_cpp_acceleration(profile="optimized")
```

`rclpy.spin()` and direct `SingleThreadedExecutor.spin()` and
`MultiThreadedExecutor.spin()` calls retain the stock node, Context, executor,
callback, exception, and ownership implementation, but wait for work in intervals
of at most 100 ms. This bounds recovery when signal shutdown invalidates the
Context after its first guard-condition wake was consumed by a previous wait set.
Runtime status records the `optimized_bounded_wait` policy, interval, outcome, and
executor type.

The bound can rebuild an idle wait set up to ten times per second. It is therefore
an opt-in reliability tradeoff, not a claimed CPU or latency optimization. The
compatible, publisher-C++, and required-C++ profiles retain their existing
indefinite stock waits; direct custom executor overrides remain application-owned.

## Native lane

Use the managed native lane when the application intentionally wants real `rclcpp`
objects and accepts a different API:

```python
import rclcppyy

with rclcppyy.native(["my_program"]) as session:
    node = session.create_node("native_node")
    executor = session.create_executor("single_threaded")
    executor.add_node(node)
    # session.rclcpp exposes the underlying cppyy rclcpp namespace.
```

The session owns its C++ context, nodes, executors, callback groups, and registered
resources. Prefer its managed factories over retaining temporary C++ objects by
hand. Query `session.capabilities` and publisher capabilities instead of assuming
that intra-process communication or loaned messages are available under every RMW.

Native callbacks, fused pipelines, editable native services, and domain type
adapters are lower-level `rclcpp_kit` facilities. They are appropriate only after
profiling identifies Python boundary crossings, serialization, scheduling, or data
conversion as the relevant cost.

Typed managed clients and action clients keep futures and asynchronous goal state
in C++ while exposing tokens and raw typed handles. Managed lifecycle nodes and
component containers expose the real `rclcpp_lifecycle` and `rclcpp_components`
objects, retaining only construction, executor membership, and teardown in the
adapter. Component loading remains the standard composition service protocol and
accepts registered AOT C++ plugins, not Python classes.

Cold native service and client glue is compiled to a content-addressed DSO before
its declarations are loaded into Cling. This permits both facilities to coexist in
one interpreter without conflicting C++ standard-library thread-local `call_once`
state. If no runtime compiler is available, an individual adapter retains the
original Cling fallback, but multiple cold glue facilities are not a guaranteed
combination. Query `session.capabilities.native_service_client_coexistence`; a
runtime compiler or prebuilt warm artifact is required for guaranteed coexistence.

## Backend evidence

`rclcppyy.status()` returns a bounded, JSON-serializable process snapshot:

```python
report = rclcppyy.status()
for operation in report["operations"]:
    print(operation["backend"], operation["metadata"], operation["reason"])
```

Accept a performance or compatibility claim only when all of these are present:

1. The publisher and subscriber, or the complete operation under test, emit the
   expected backend records.
2. The stock and candidate runs validate the same decoded wire-value contract and
   use the same QoS, RMW, rate, duration, warmup, machine controls, and process
   topology.
3. Raw benchmark JSON validates against `schemas/benchmark-v3.schema.json`.
4. Correctness, teardown, and drop counts pass before CPU or latency is compared.
5. Repeated runs on a dedicated machine show a material result; hosted CI smoke
   timings are validation evidence and never a performance claim.

See `docs/benchmarks.md` for the executable matrix, compatibility evidence gate,
and statistical conventions.

## Optimization promotion gate

Contract-changing behavior stays opt-in. Moving one optimization into the default
compatible profile requires an explicit review that records:

1. The exact API and behavioral contract affected.
2. Differential, integration, concurrency, teardown, and installed-package tests.
3. Backend evidence proving the intended route and absence of accidental fallback.
4. Repeated x86_64 and ARM64 measurements on controlled runners, including negative
   workloads and raw artifacts.
5. Ownership, lifetime, RMW, and fallback analysis, plus a documented disable flag.
6. A compatibility-manifest update approved in the same change.

An optimization is not promoted solely because it is faster in one benchmark.
Unsupported middleware capabilities, uncertain ownership, semantic changes, or an
unmeasured workload keep it explicit even when the native implementation is sound.

The repository assigns the router, policy, manifest, and this gate to CODEOWNERS.
The protected default branch must require a CODEOWNERS approval for that assignment
to become an enforced merge gate; the files alone do not replace branch protection.
