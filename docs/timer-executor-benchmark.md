# Controlled timer-executor benchmark

This benchmark characterizes process CPU and scheduling behavior for a fixed
1 ms timer on ROS 2 Jazzy with Cyclone DDS: a steady clock for the
stock/compatible/native/AOT lanes, and the node's own ROS clock (sim-time-aware,
matching stock's `create_timer(clock=None)` default) for the direct-cpp lanes.
It produces raw evidence with performance claims disabled.

## Variants

1. `stock-rclpy`: stock timer, executor, and Python callback.
2. `compatible-rclcppyy`: activation-only control with the same stock authority.
3. `direct-cpp-rclcppyy`: the source-compatible `Node.create_timer` and
   `rclpy.spin_once` surface backed by a session-owned `rclcpp::GenericTimer` on
   the node's own ROS clock and a session-owned executor, entering Python once
   per firing.
4. `direct-public-ste`: the same direct timer and callback driven by an explicit
   patched `rclpy.executors.SingleThreadedExecutor` using
   `add_node`/`spin_once`/`remove_node`.
5. `direct-raw-ste-control`: the same direct timer and callback driven directly by
   a `NativeSession` raw `rclcpp::executors::SingleThreadedExecutor`, matching the
   former hidden-executor hot path.
6. `direct-public-mte`: the same direct timer and callback driven by the public
   `rclpy.executors.MultiThreadedExecutor` (`num_threads=2`, un-fail-closed in
   Slice 3 of the MultiThreadedExecutor-unlock plan) using
   `add_node`/`spin_once`/`remove_node`. One worker drains the single ready
   callback per `spin_once()` call, so the workload shape matches every other
   lane exactly; this measures MultiThreadedExecutor's per-firing overhead, not
   concurrent dispatch.
7. `direct-raw-mte-control`: the same direct timer and callback driven directly by
   a `NativeSession` raw `rclcpp::executors::MultiThreadedExecutor` (2 threads),
   the MultiThreadedExecutor analogue of `direct-raw-ste-control`.
8. `native-python-callback`: managed `rclcpp` timer and executor entering Python
   once per firing.
9. `native-cpp-callback`: content-addressed C++ timer callback with no per-firing
   Python crossing.
10. `aot-staged`: conventional Release-mode `rclcpp` executable.

The compatible lane is not an acceleration claim. The direct lane keeps the
unchanged Python callback workload while changing timer and executor authority.
The two C++ callback lanes keep
all recurrence and timing state in C++ and report zero per-firing Python crossings.
The MultiThreadedExecutor lanes join the general rotating order rather than the
paired STE gate below: no CPU-ratio gate is defined for them and none is implied
by their presence -- claims stay disabled for this comparison exactly as for
every other lane.

## Fixed contract

Every fresh worker performs 500 warmup firings followed by 5,000 measured firings.
Ten repetitions rotate the existing lane order. The public and raw direct lanes
remain adjacent in every repetition and alternate which one runs first. A wrapping
`uint64_t` recurrence provides an
exact final state and checksum across implementations. Each sample proves its node
appears after readiness, disappears after exit, stops after timer cancellation, and
leaves no pending post-cancel callback.

Python-callback lanes emit their measurement-ready record while the timer remains
canceled, then establish the deadline epoch and reset immediately after the record
is flushed. Their evidence therefore reports `timer_reset: false` and
`measurement_starts_after_emit: true`. Native C++ and AOT lanes retain their
arm-before-record order and report the inverse markers. This keeps protocol output
or GIL hand-off time out of the Python lanes' scheduled-deadline offset.

The primary metric is worker `CLOCK_PROCESS_CPUTIME_ID` nanoseconds per measured
firing. Secondary evidence includes effective frequency, signed and absolute
scheduled phase error p50/p95/p99/max, the first post-reset callback error, and
signed and absolute consecutive-interval error p50/p95/p99/max. Consecutive error
is `phase_error[i] - phase_error[i - 1]`, so it exposes steady interval jitter
without hiding a persistent rearm offset in the retained phase-error metric.
`max_phase_slip_periods` is the maximum positive phase error divided by the 1 ms
period; it describes schedule phase slip and does not claim that callbacks were
dropped. The v1 `missed_periods` field remains byte-contract compatible as a
deprecated alias with an enforced identical value; only `max_phase_slip_periods`
is semantically authoritative. The exact interval observation count must be one
fewer than the measured callback count. Exceptions and teardown evidence remain
mandatory. Memory is not ranked.

The artifact also contains a narrowly scoped public-STE regression result. It
compares the median CPU nanoseconds per firing across ten public/raw pairs and
passes when `direct-public-ste <= 1.03 * direct-raw-ste-control`. The paired
absolute scheduled-latency p99 values and their median ratio are reported as
secondary evidence but do not control the gate. General benchmark claims and
cross-lane interpretation remain disabled. Missing pairs produce characterization
instead of a gate result.

The runner builds the AOT executable with `-O3` and `-DNDEBUG`, uses a temporary
content-addressed native cache, proves a cold miss followed by a warm hit, and
excludes all compilation and prewarming from samples.

## Running

Run the fixed 100-sample matrix only on a quiet host:

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  pixi run timer-executor-bench \
  --output build/timer-executor-measurement.json
```

The runner rejects non-Jazzy environments, forces Cyclone DDS, terminates complete
process groups on failure, and writes a schema-validated artifact. When host
conditions are unsuitable for enforcing a CPU threshold, retain all paired evidence
without failing the regression gate:

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  pixi run timer-executor-bench \
  --characterization-only \
  --output build/timer-executor-characterization.json
```

## Results (2026-07-22, MultiThreadedExecutor lanes added)

The first full 100-sample run since `direct-public-mte`/`direct-raw-mte-control`
were added, run in `--characterization-only` mode: run-to-run CPU variance on
this host has been documented at up to 2.2x on an identical script, and the
standing project policy is that CPU-threshold enforcement and any speedup
claim wait for dedicated quiet runners regardless of a given host's state at
run time. All ten lanes emitted their full 10/10 repetitions with zero
failures. No claims are made from these numbers; they are raw evidence only.

Median `worker_cpu_ns_per_firing` per lane (nanoseconds, n=10 each):

| Lane | Median | Min | Max |
|---|---|---|---|
| `stock-rclpy` | 142043.5 | 117160.5 | 151455.5 |
| `compatible-rclcppyy` | 133034.8 | 111893.5 | 155429.7 |
| `direct-cpp-rclcppyy` | 39703.7 | 33297.5 | 47929.7 |
| `direct-public-ste` | 30746.4 | 25867.6 | 38691.0 |
| `direct-raw-ste-control` | 26407.0 | 24133.6 | 32032.4 |
| `direct-public-mte` | 35431.4 | 29883.1 | 41794.3 |
| `direct-raw-mte-control` | 29157.7 | 24580.0 | 35359.9 |
| `native-python-callback` | 30580.3 | 25333.0 | 38827.1 |
| `native-cpp-callback` | 13715.2 | 11240.1 | 15765.2 |
| `aot-staged` | 12487.8 | 8985.5 | 14921.9 |

Public/raw median CPU ratios (characterization only, no gate enforced for
either pair):

- STE: `direct-public-ste` / `direct-raw-ste-control` = **1.1643** (absolute
  latency p99 median ratio 0.9719). Prior evidence on record for this pair:
  wave-0 recorded ~1.279x; an uncommitted 5-repetition exploratory pass earlier
  this engagement measured 1.4247x. This run's 1.1643x sits inside the same
  rough band as both priors, above the enforced gate's 1.03 limit in all three
  measurements -- consistent with run-to-run variance on this host rather than
  a step change in either direction.
- MTE: `direct-public-mte` / `direct-raw-mte-control` = **1.2152**. The same
  uncommitted exploratory pass measured 1.1232x over 5 repetitions. Both
  measurements land in the same 1.1-1.3x range as the STE pair; nothing in
  this evidence indicates the now-public `MultiThreadedExecutor` construction
  path carries meaningfully different per-firing overhead than the
  `SingleThreadedExecutor` path already characterized here. This is
  characterization evidence, not a parity or regression claim -- no CPU
  gate is defined for the MTE pair, and none is implied by reporting these
  numbers.

Both lanes use `spin_once()` against a single ready callback per call (as do
all other lanes in this matrix), so this measures MultiThreadedExecutor's
per-firing dispatch overhead under that workload shape, not genuine
concurrent multi-callback throughput -- see Slice 3's own true-parallelism
proofs (`test_direct_executor.py`) for that separate question.

Full raw artifact: `build/timer-executor-characterization.json` (100 samples,
0 failures), sha256 `8172857537f855198b1a035bc3cca6dbea0418ba5e5f071297395b8ff82ccd2a`.
Since `build/` evidence has been lost once in this project's history, a
byte-identical copy (same sha256) plus its own `.sha256` file was also
retained outside `build/` for this run.
