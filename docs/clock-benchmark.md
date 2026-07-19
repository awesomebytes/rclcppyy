# Controlled clock/timestamp benchmark

This benchmark characterizes process CPU cost for reading the current time on
ROS 2 Jazzy with Cyclone DDS. It produces raw evidence with performance claims
disabled.

## Variants

1. `stock-rclpy`: a stock `Node.get_clock()` `Clock`.
2. `compatible-rclcppyy`: activation-only control with the same stock
   authority (`profile="compatible"`; `Clock`/`Time` are never patched by any
   profile, so this lane's code path is identical to stock -- it only proves
   activation itself adds no overhead).
3. `direct-cpp-rclcppyy`: `DirectNode.get_clock()`, backed by the native
   foundation's retained `NativeNodeClock` (the node's exact `rclcpp::Clock`).
4. `native-orchestrated`: a bare `NativeNodeClock` created directly through a
   `NativeSession`, bypassing `DirectNode` entirely -- the floor this backend
   could reach with no facade overhead at all.

## Workloads

1. `now`: the clock's own `now()` (a Python `rclpy.time.Time` for
   stock/compatible/direct-cpp; a raw `cppyy.gbl.rclcpp.Time` for
   native-orchestrated).
2. `now-nanoseconds`: the fastest available route to a raw integer
   nanosecond count. `rclpy.clock.Clock` has no `now_nanoseconds()`, so
   stock/compatible go through `now().nanoseconds`; direct-cpp and
   native-orchestrated call the retained `NativeNodeClock.now_nanoseconds()`
   int hot path directly (for direct-cpp this reaches into the private
   native clock the facade retains, not a new public method -- `DirectClock`
   deliberately does not gain a `now_nanoseconds()` of its own, since stock
   `Clock` doesn't have one either and adding one would be an unreviewed
   public-surface superset).

The direct-cpp and native-orchestrated lanes are read-only characterizations
of an already-open node clock; no publish/subscribe or executor spin is
involved, so this is a pure CPU micro-benchmark, not a graph benchmark.

## Fixed contract

Every fresh worker performs 1,000 warmup operations followed by 10,000
measured operations, by default; five repetitions rotate the existing lane
order (each workload's variant order shifts by repetition and workload index,
mirroring the local-parameter benchmark's rotation policy). The primary
metric is worker `CLOCK_PROCESS_CPUTIME_ID` nanoseconds per measured
operation. Secondary evidence includes wall nanoseconds per operation and
operations per second. Each sample proves its final observed nanosecond
value is a positive integer and did not rewind more than a small tolerance
relative to a post-measurement reading, guarding against a clock that
silently stopped advancing without requiring exact equality across two
separate calls.

Conversion and serialization entry points are replaced with fail-closed
guards before the measured window opens; every sample's evidence must
contain zero application-message conversions, zero serialization calls, and
zero CDR calls. `performance_claims_allowed` stays `false` in every record,
in both measurement and smoke mode. General benchmark claims and
cross-lane interpretation remain disabled.

## Running

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  pixi run clock-bench \
  --output build/clock-measurement.json
```

Run measurement mode only on a quiet, controlled host and retain the JSON
artifact. Use `--smoke` for a short, five-warmup/twenty-five-operation,
single-repetition run that validates wiring, backend markers, and protocol
without characterizing anything:

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  pixi run python scripts/benchmarks/run_clock_benchmark.py --smoke \
  --output build/clock-smoke.json
```
