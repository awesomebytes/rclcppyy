# Controlled clock-timer / rate-sleep benchmark

This benchmark characterizes process CPU cost for the ROS-clock timer hot
path (`Node.create_timer`'s default, now a `GenericTimer` on the node's own
ROS clock rather than a steady `WallTimer`) and for `Node.create_rate().sleep()`
on ROS 2 Jazzy with Cyclone DDS. It produces raw evidence with performance
claims disabled.

## Variants

1. `stock-rclpy`: stock `Node.create_timer`/`create_rate` and executor.
2. `compatible-rclcppyy`: activation-only control with the same stock
   authority (`profile="compatible"` never patches `Node`, `Timer`, or
   `Rate`; this lane proves activation itself adds no overhead).
3. `direct-cpp-rclcppyy`: `DirectNode.create_timer` (a session-owned
   `rclcpp::GenericTimer` on the node's own ROS clock) and
   `DirectNode.create_rate().sleep()` (the node's `NativeClockSleeper`, no
   spinning executor required).
4. `native-orchestrated`: the facade floor -- `create_clock_timer`/
   `NativeClockSleeper` driven directly through a bare `NativeSession`,
   bypassing `DirectNode`/`DirectRate` entirely.

## Workloads

1. `clock-timer`: one timer firing per operation, driven by spinning the
   applicable executor until the callback's counter advances.
2. `rate-sleep`: one `Rate.sleep()` (or, for `native-orchestrated`, one raw
   `NativeClockSleeper.sleep_until()`) per operation. Stock's `Rate` needs a
   background-thread-spun executor (its `sleep()` blocks on a
   `threading.Event` a `Timer` callback sets); the direct and native lanes
   need none, since they sleep directly on the node's clock sleeper.

Both workloads are fixed at a 1 ms period, matching the timer-executor
benchmark's own period. The `native-orchestrated` rate-sleep lane applies the
same fixed-rate catch-up guard `DirectRate` does (resync to `now + period`
after a stall of more than one period) for a fair comparison: without it, a
one-time process-startup delay before the first sleep would permanently
offset a naive schedule into the past.

## Fixed contract

Every fresh worker performs 500 warmup operations followed by 2,000 measured
operations, by default; five repetitions rotate the existing lane order (each
workload's variant order shifts by repetition and workload index, mirroring
the clock/timestamp benchmark's rotation policy). The primary metric is
worker `CLOCK_PROCESS_CPUTIME_ID` nanoseconds per measured operation.
Secondary evidence includes wall nanoseconds per operation and operations per
second.

Conversion and serialization entry points are replaced with fail-closed
guards before setup completes; every sample's evidence must contain zero
application-message conversions, zero serialization calls, and zero CDR
calls. Direct and native lanes additionally prove `exact_cpp_entity: true`,
and the `clock-timer` workload proves the native entity is a `GenericTimer`
(not a `WallTimer`) via its `native_type`. `performance_claims_allowed`
stays `false` in every record, in both measurement and smoke mode. General
benchmark claims and cross-lane interpretation remain disabled.

## Running

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  pixi run clock-timer-bench \
  --output build/clock-timer-measurement.json
```

Run measurement mode only on a quiet, controlled host and retain the JSON
artifact. Use `--smoke` for a short, five-warmup/twenty-five-operation,
single-repetition run that validates wiring, backend markers, and protocol
without characterizing anything:

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  pixi run python scripts/benchmarks/run_clock_timer_benchmark.py --smoke \
  --output build/clock-timer-smoke.json
```
