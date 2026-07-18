# Backend selection and promotion policy

`rclcppyy` has a small product surface, but it exposes several execution lanes.
Choose the lane from the compatibility requirement first, then use measured backend
evidence to decide whether a more specialized lane is worthwhile.

## Decision table

| Requirement | Entry point | Contract | Failure behavior |
| --- | --- | --- | --- |
| Run existing `rclpy` software unchanged | `enable_cpp_acceleration()` | Exact stock Python object identities and behavior | Certified operations use C++; other operations delegate to stock Python and report that decision |
| Prove that a selected operation cannot fall back | `enable_cpp_acceleration(profile="required_cpp")` | Exact stock contract for supported operations | Rejects an unsupported operation before its side effects |
| Use a C++-only ROS facility from Python | `rclcppyy.native()` | Explicit native API; not a drop-in `rclpy` replacement | Capability queries and normal exceptions make unsupported facilities visible |
| Remove Python from a measured hot path | `rclcpp_kit` native callbacks, services, or fused pipelines | Explicit opt-in contract for ownership, scheduling, and delivery | Compilation and construction are explicit; generated code and counters are inspectable |
| Consider contract-changing automatic optimization | `profile="optimized"` | Only changes documented by an individually reviewed optimization | The profile currently reserves permission; it does not silently enable a native lowering |

## Compatible profile

Use the default profile for existing applications:

```python
import rclcppyy

rclcppyy.enable_cpp_acceleration(warn_fallback=True)

# Existing imports and application code continue unchanged.
import rclpy
```

The original `Node`, `Context`, executors, generated messages, and entities remain
authoritative. A C++ route may operate on an existing native handle only when the
route is certified for that operation. The complete operation otherwise remains
stock Python; `rclcppyy` does not create a companion node or split ownership.

The compatibility manifest at `compatibility/jazzy.json` is the source of truth
for certified, experimental, and unsupported surfaces. Activation alone is not
evidence that a particular operation used C++.

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
2. The stock and candidate runs use the same message values, QoS, RMW, rate,
   duration, warmup, machine controls, and process topology.
3. Raw benchmark JSON validates against `schemas/benchmark-v2.schema.json`.
4. Correctness, teardown, and drop counts pass before CPU or latency is compared.
5. Repeated runs on a dedicated machine show a material result; hosted CI smoke
   timings are validation evidence and never a performance claim.

See `docs/benchmarks.md` for the executable matrix and statistical conventions.

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
