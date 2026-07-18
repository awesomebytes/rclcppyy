# ADR 0001: Stock node authority in compatibility mode

- Status: accepted
- Date: 2026-07-18
- Decision owners: rclcppyy maintainers

## Context

The prototype compatibility node constructs both a stock Python node and a
separate `rclcpp::Node`. That enabled an early C++ pub/sub path, but the two
nodes have different graph identities, contexts, parameter services, executor
ownership, and destruction lifetimes. A sidecar cannot transparently implement
the `rclpy` contract.

The primary product requirement is unchanged Python software. Compatibility
therefore takes precedence over access to features that require owning a full
`rclcpp::Node`.

## Decision

In the compatible and required-C++ profiles, the original `rclpy.node.Node`, its
`rclpy.context.Context`, and its executor are the sole authorities. Enabling
acceleration must not create a companion node, initialize another context, or
replace stock executor semantics.

Compatibility hooks patch methods on the original Node class so direct
construction, aliases, and subclasses keep their identity. Each entity creation
is routed independently:

1. Use a certified C++ implementation only when every requested argument and
   lifetime rule is supported.
2. In the compatible profile, delegate the complete operation to stock `rclpy`
   when the C++ route is not certified.
3. In the required-C++ profile, raise a structured error instead of falling back.
4. Never accept and silently ignore an option.

The default compatible policy deliberately keeps stock `Publisher.publish`
authoritative even though a same-handle route is certified. That route is selected
only by `publisher_cpp` or `required_cpp`; current controlled relay evidence did not
justify making its implementation change automatic. The optimized profile also
keeps stock publishing unless a future reviewed promotion changes that contract.

Backend decisions are observable through `rclcppyy.status()`. Required-C++ tests
and benchmarks must assert the observed backend, not merely that activation was
requested.

The optimization profile may alter delivery, memory, or scheduling behavior only
behind explicit flags. Facilities that need an owning `rclcpp::Context`, native
executor, intra-process manager, composition, or loaned-message ownership remain
in a separate native lane and are never presented as transparent compatibility.

## Ownership rules

- `rclpy` owns and finalizes every compatibility node and context handle.
- A C++ adapter may borrow a native handle but must never finalize it.
- Python ownership pins the authoritative object while a borrower exists.
- Borrowed C++ entities are destroyed before the stock entity, node, and context.
- Destruction is idempotent and is tested both explicitly and during interpreter
  shutdown.
- Distro and ABI probes fail closed before a private native pointer is used.

The compatibility layer and routing policy stay in `rclcppyy`. Reusable typed
factories, native-handle adapters, and C++ ownership helpers belong in
`rclcpp_kit`; generic compilation, lifetime, and concurrency primitives belong in
`cppyy_kit`.

## Evidence

Commit `474e86e` adds an isolated Phase 0 probe. On ROS 2 Jazzy it:

- keeps an exact stock Node and custom Context authoritative;
- constructs an `rclcpp::Publisher<std_msgs::msg::String>` against the borrowed
  `rcl_node_t` without creating an `rclcpp::Node`;
- completes a C++ publisher to stock Python subscriber round trip;
- reports exactly one requested node and endpoint identity in the ROS graph; and
- destroys the publisher before the node/context and exits normally.

The probe is architecture evidence, not a supported API. It uses the private
`node.handle.pointer` ABI, a non-owning `NodeBaseInterface`, and has no compatible
`rclcpp::Context`. It proves typed publishing only; subscriptions, timers,
services, clients, events, callback groups, and executor integration require
separate prototypes and contract tests.

## Rejected alternatives

### Companion `rclcpp::Node`

Rejected for compatibility. It creates duplicate graph and service identities,
can select the wrong context/domain, and cannot preserve stock executor or
teardown semantics. Giving the companion the same name hides none of those
problems.

### C++-authoritative compatibility facade

Rejected for the transparent lane. Reimplementing the broad Python Node,
executor, entity, waitable, parameter, lifecycle, and introspection contracts
would expand the product surface substantially and still break existing aliases
and subclasses during incremental rollout.

### Process-wide backend selection

Rejected as the default fallback boundary. With a stock authoritative node, a
complete entity operation can safely remain Python when its C++ implementation
is unsupported. Process-wide required-C++ remains available as a policy, not as
the compatibility architecture.

## Consequences

This decision preserves graph and Python object identity and allows incremental,
visible acceleration with a small product surface. It also limits what can be
called transparent: some `rclcpp` features are available only through the native
lane, and private-handle acceleration cannot ship until its ABI, ownership,
fallback, and teardown probes pass on every supported ROS distribution and
architecture.

The existing sidecar implementation remains transitional code. New compatibility
work must move toward this decision and must not add new behavior that depends on
the sidecar identity.
