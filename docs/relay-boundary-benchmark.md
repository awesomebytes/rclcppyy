# Controlled relay-boundary benchmark

This benchmark characterizes one fixed ROS 2 relay across eight execution
boundaries. It is a raw evidence generator, not a release gate or a source of
performance claims. Jazzy with CycloneDDS is the current validated gate; the
protocol retains explicit RMW evidence so later backends cannot be conflated
with that baseline.

## Variants

1. `stock-rclpy`: the unmodified Python relay.
2. `compatible-rclcppyy`: the same Python relay function, with compatible
   activation as the only setup difference. Stock `rclpy.Publisher.publish`
   remains authoritative.
3. `publisher-cpp-rclcppyy`: the same Python relay function, explicitly
   activated with `profile="publisher_cpp"` for same-handle C++ publishing.
4. `direct-cpp-rclcppyy`: the same Python transform and publish body, activated
   with `profile="direct_cpp"` so messages and entities are actual C++ objects;
   each callback receives one owning C++ value copy.
5. `direct-lease-rclcppyy`: the same direct C++ source shape with
   `optimizations=("subscription_shared_lease",)`. The rclcpp unique message
   allocation transfers into shared ownership without copying `MessageT`.
6. `native-python-callback`: native rclcpp entities with a Python transform
   callback.
7. `native-fused`: a prebuilt, content-addressed C++ fused pipeline.
8. `aot-staged`: a conventional Release-mode C++ relay.

Every variant is driven by the same Release-mode AOT executable, transform,
reliable/volatile `KeepLast(1)` QoS, closed-loop message sequence, warmup, and
two-process topology. There is one outstanding `UInt64` message at a time and
the transform is `output = input * 2 + 1`.

## Evidence

The driver verifies exact endpoint cardinality, topics, QoS, node ownership,
wire values, checksums, and teardown. CycloneDDS can report remote node owner
metadata as unknown; in that case the artifact preserves both observed and
expected identities and accepts the unknown value only with the exact
two-process-group and unique-topic proof. Other owner mismatches fail.

Generated routes must hit artifacts built during isolated cold/warm prewarm
phases. The AOT relay records its compiler, Release command, executable hash,
and source hashes. Versioned sentinel records separate machine evidence from
captured initialization diagnostics.

Primary observations are relay and driver process CPU time, every raw
round-trip latency, p50/p95/p99/max latency, and closed-loop throughput. Relay
emits an `armed` record after its CPU clock starts, and only then may the driver
enter its measured loop. Both clocks stop immediately at measured completion.
Driver teardown is held until the relay CPU window has stopped. Post-warmup
peak-RSS growth is only a bounded 64 MiB runaway guard; it is never summarized,
ranked, or compared.

The explicit `publisher_cpp` publish-operation evidence is captured after
warmup, before the CPU window starts. The measured window then requires the
publisher's permanent fallback taint to remain clear and its final backend to
remain C++. Compatible mode instead proves a Python publisher decision and the
absence of a C++ publish-operation marker. Teardown phases are versioned stderr
diagnostics. If a Python relay report times out, the parent asks for an
all-thread stack dump before terminating its process group and retains that
stderr in the failure artifact.

The direct-C++ lane proves that `UInt64`, the native node, publisher,
subscription, and executor are cppyy C++ objects owned by one `NativeSession`.
The kit counter at the copy-construction point must equal the callback count,
proving exactly one owning native C++ copy per callback. Converter and
serialization entry points are replaced with fail-closed guards; the report must
contain zero guard calls, zero Python-message conversions, and zero serialization
operations. The direct lane uses integer-depth QoS and a `rclpy.spin_once` thread,
which is the bounded direct profile's supported control plane and is recorded as
a distinct execution model.

The direct-lease lane has the same C++ authority and Python callback body. Its
native and Python-observed message addresses must match, and the callback count
must equal its lease, shared-control-block, shared-owner, and Python-crossing
counters. It requires zero `MessageT` deep copies and zero lease exceptions. The
copy and lease routes are separate production options in the same rotating run so
CPU effects are paired without conflating representation or application work.

## Running

```bash
pixi run relay-boundary-smoke --output build/relay-boundary-smoke.json
pixi run relay-boundary-bench --output build/relay-boundary-measurement.json
```

Smoke mode uses one short repetition to validate compilation, backend markers,
parity, protocol, and teardown. Measurement mode defaults to five rotating
repetitions of 2,000 measured messages after 100 warmup messages. Run
measurement mode only on a quiet, controlled host and retain the JSON artifact;
the schema and in-process validator deliberately keep performance claims and
automatic winner selection disabled.
