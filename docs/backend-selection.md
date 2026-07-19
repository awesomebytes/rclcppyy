# Backend selection and promotion policy

`rclcppyy` has a small product surface, but it exposes several execution lanes.
Choose the lane from the compatibility requirement first, then use measured backend
evidence to decide whether a more specialized lane is worthwhile.

## Decision table

| Requirement | Entry point | Contract | Failure behavior |
| --- | --- | --- | --- |
| Run existing `rclpy` software unchanged | `enable_cpp_acceleration()` | Exact stock Python object identities and behavior, including `Publisher.publish` | Operations remain stock Python and report that authority |
| Try same-handle C++ publishing without changing application code | `enable_cpp_acceleration(profile="publisher_cpp")` | Exact stock publisher object and graph endpoint; explicit publish implementation change | Falls back to stock publishing visibly if preparation or a publish fails |
| Keep supported messages in C++ storage through publish and take | `enable_cpp_acceleration(profile="message_facade")` before message imports | Exact stock Node, Context, Publisher, Subscription, callback-group, graph, and destruction objects; generated-style `String` and `UInt64` classes own C++ storage | Unsupported layouts/options stay visibly stock; a failed take raises without an unsafe retry |
| Exercise the bounded source-compatible direct-C++ slice | `enable_cpp_acceleration(profile="direct_cpp", interfaces=("package/msg/Message", "package/srv/Service"))` before node/interface imports | One native `rclcpp` node authority; requested installed interfaces and nested message dependencies are actual generated C++ classes | Missing interfaces, stale imports, Python message objects, unreviewed options, and unsupported operations fail before entity creation |
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

Compatible import and activation are intentionally lightweight: they do not import
cppyy, initialize Cling, load native factories, or import the legacy companion node.
Those dependencies load on demand through the existing public exports. Future
completion methods also retain exact stock identity because every executor callback
Task completes through that hot path.

Stock authority includes Jazzy's custom action endpoint QoS, explicit goal UUIDs,
`ActionClient.send_goal()`, and `ClientGoalHandle.cancel_goal()` and `get_result()`
methods. Lifecycle `ERROR`/`on_error` processing, invalid-transition exceptions,
and shutdown from inactive or active states likewise remain exact stock behavior.

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
product default. They remain ignored local evidence, while the current seven-lane
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
its authority. Future completion remains the exact stock implementation under every
profile because executor callback Tasks use these methods on the hot path.

Strict mode is an assertion mechanism, not a promise that the whole application is
C++. Tests must check the operation records they depend on.

## C++ message-facade profile

Use this explicit profile only on the reviewed ROS 2 Jazzy, `rclpy` 7.1.11, and
Cyclone DDS stack, and enable it before importing message classes:

```python
import rclcppyy

rclcppyy.enable_cpp_acceleration(profile="message_facade")

from std_msgs.msg import String, UInt64
```

The imported classes retain generated-message names, constructors, field access,
repr, equality, copy/deepcopy, pickle, and reviewed `check_fields` behavior for
valid field values. Each instance owns a distinct C++ message. Entity creation uses
the preserved original generated class for type support, then exposes the facade
class on the exact stock Publisher or Subscription object. Publishing serializes
that existing C++ message directly through the stock publisher handle. The
version-gated stock executor hook takes serialized CDR through the exact stock
subscription handle and deserializes it into a new owning C++ message before the
Python callback.

The certified whitelist is intentionally limited to `std_msgs/msg/String` and
`std_msgs/msg/UInt64`. Unsupported layouts and classes imported before activation
stay stock. Raw subscriptions, subscriptions with event callbacks or content
filters, and custom publisher classes also stay stock because their complete
contracts have not been certified on the direct route. Publish-route failure is
reported before using the stock operation. Subscription-take failure is reported
and propagated without retry because a serialized take may already have consumed
the sample.

Because storage remains a valid C++ message at all times, invalid field types or
ranges can still be rejected by C++ when `check_fields=False`; stock generated
messages may defer that failure until serialization. This explicit divergence is
one reason the profile is opt-in.

Activation resolves Cling/template setup before installing the facade classes.
Correctness and future performance measurements must exclude activation, JIT, DDS
discovery, and endpoint warmup from the steady-state window. No performance benefit
is claimed until the dedicated repeated benchmark clears the normal promotion
gate on both x86_64 and ARM64.

## Direct-C++ first slice

`direct_cpp` is a deliberately narrow correctness profile for the future
source-compatible native backend:

```python
import rclcppyy

rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp",
    interfaces=("std_msgs/msg/Header", "std_srvs/srv/Trigger"),
)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, String
```

On the reviewed Jazzy/Cyclone stack, `Node` can still be subclassed and the common
positional `create_publisher(Message, topic, depth)` and
`create_subscription(Message, topic, callback, depth)` calls create typed raw
`rclcpp` entities. `String(data=...)`, `UInt64(data=...)`, and explicitly requested
installed interfaces construct the actual cppyy C++ classes. The registry resolves
and validates the complete nested-message dependency closure before changing any
Python import alias. For example, requesting `std_msgs/msg/Header` also aliases its
`builtin_interfaces/msg/Time` field to the generated C++ class. Publish uses that
object directly, with no Python-message conversion or serialization path.
`rclpy.spin_once(node, timeout_sec=...)` drives the session-owned native
single-threaded executor.

The same profile supports `std_srvs/srv/SetBool` by default and explicitly
registered installed services such as `std_srvs/srv/Trigger` through the common
`create_service`, `create_client`, request construction, `call_async`, and top-level
`rclpy.spin_until_future_complete` surface. Request and response objects are the
actual generated C++ classes. Each call uses one C++ copy
to transfer the constructed request value into shared native ownership; the
returned `rclpy.task.Future` is per-operation Python control state whose result is
the shared C++ response. A service callback receives owning C++ request/response
values and returns a C++ response, with one request copy and one response
assignment. These copies and Python crossings are explicit status evidence; no
Python message conversion is involved.

`std_msgs/msg/String` and `std_msgs/msg/UInt64` remain the default message registry.
Additional interfaces must use canonical `package/msg/Message` or
`package/srv/Service` spelling and must have installed rosidl resource metadata, a
generated C++ header, a canonical cppyy alias, and a loadable C++ typesupport
library. A registered service's nested message dependencies are resolved before
any alias changes. Generated Python message classes are never accepted by direct
entity factories. Topic QoS is still a positive integer depth; services use only
the default service QoS. Activation must precede
`rclpy.node`, `rclpy.executors`, and every generated message import. Callback groups,
events, QoS overrides, raw/content-filter subscriptions, custom publisher classes,
public executors, coroutine service callbacks, synchronous client `call`, service
introspection, parameters, and the rest of the uncovered `rclpy` surface are
rejected rather than falling back onto a second authority.

cppyy's callback argument is borrowed for the duration of the shared-pointer
call. To preserve the `rclpy` expectation that a callback may retain its message,
the baseline slice hands the callback one owning native C++ copy. This is still a
C++ message with no Python representation conversion or serialization, but it is
a measured compatibility cost. The opt-in
`optimizations=("subscription_shared_lease",)` route instead transfers rclcpp's
received `unique_ptr<MessageT>` into shared ownership and exposes the same allocation
to Python. It records zero `MessageT` deep copies while preserving retained-message
lifetime. The controlled relay benchmark keeps the copy and lease routes separate;
neither result is inferred from borrowed-handle or native-only measurements.

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
