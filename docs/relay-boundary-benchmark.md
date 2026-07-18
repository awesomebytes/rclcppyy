# Controlled relay-boundary benchmark

This benchmark characterizes one fixed ROS 2 relay across five execution
boundaries. It is a raw evidence generator, not a release gate or a source of
performance claims.

## Variants

1. `stock-rclpy`: the unmodified Python relay.
2. `compatible-rclcppyy`: the same Python relay function, with compatible
   activation as the only setup difference.
3. `native-python-callback`: native rclcpp entities with a Python transform
   callback.
4. `native-fused`: a prebuilt, content-addressed C++ fused pipeline.
5. `aot-staged`: a conventional Release-mode C++ relay.

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
and driver clocks stop immediately at measured completion. Driver teardown is
held until the relay CPU window has stopped. Post-warmup peak-RSS growth is only
a bounded 64 MiB runaway guard; it is never summarized, ranked, or compared.

## Running

```bash
pixi run relay-boundary-bench --smoke --output build/relay-boundary-smoke.json
pixi run relay-boundary-bench --output build/relay-boundary-measurement.json
```

Smoke mode uses one short repetition to validate compilation, backend markers,
parity, protocol, and teardown. Measurement mode defaults to five rotating
repetitions of 2,000 measured messages after 100 warmup messages. Run
measurement mode only on a quiet, controlled host and retain the JSON artifact;
the schema and in-process validator deliberately keep performance claims and
automatic winner selection disabled.
