# Benchmark Matrix

`scripts/benchmarks/run_benchmarks.py` runs a bounded, declarative cross-product
of backends, workloads, target frequencies, and application payload padding.
Each publisher/subscriber pair runs in fresh child processes and the parent owns
their startup, measurement, and teardown.

## Modes

Fast CI validation:

```bash
pixi run bench --smoke --json --output build/bench-smoke.json
```

Smoke output always contains:

```json
{
  "benchmark": {
    "mode": "smoke",
    "performance_claims_allowed": false
  }
}
```

Smoke runs prove route selection, message delivery, structured output, and clean
process isolation. Their short CPU and latency samples are not performance
evidence and must not be used for comparisons or published claims.

An intentional matrix run can select multiple axes:

```bash
pixi run bench \
  --backends rclpy,rclcppyy,rclcppyy-templated \
  --workloads small-string,nested-header \
  --rate 1000,10000 \
  --payload-bytes 0,4096 \
  --duration 15 \
  --output build/benchmark.json
```

Use `--list-matrix` to inspect the exact cases without starting ROS processes.
The legacy `--variants` spelling remains an alias for `--backends`.

Render any v2 result as a deterministic review report:

```bash
pixi run bench-report build/benchmark.json --output build/benchmark.md
```

The renderer preserves the artifact's claim policy. Smoke reports state that
performance claims are forbidden, and measurement reports do not select a winner;
comparative conclusions still require controlled repeated runs and review.

## Evidence And Statistics

Every successful row includes separate publisher/subscriber backend markers.
The parent rejects missing, malformed, or unexpected markers, so activation
alone cannot be reported as C++ execution.

The subscriber accepts explicit start/stop control messages. Counts, sequence
gaps, and latency samples are reset at start and summarized at stop. This avoids
inferring counts from log-line frequency or combining per-interval percentiles.
Latency percentiles use the nearest-rank definition over every message received
inside the acknowledged window. CPU output retains each `psutil.cpu_percent`
sample and reports its mean, median, sample standard deviation, minimum, and
maximum.

The runtime envelope is `rclcppyy.benchmark/v2`; its portable JSON Schema is
[`schemas/benchmark-v2.schema.json`](../schemas/benchmark-v2.schema.json). The
artifact records source revision/dirty state, architecture, CPU, Python and
package versions, ROS/RMW settings, cache environment, selected matrix, raw CPU
samples, failures, and child backend evidence.

## Current Limits

- Target frequency is an input, not a promise; use `effective_rate_hz` as the
  observed result.
- `payload_bytes` is application padding, not serialized wire size.
- Publisher and subscriber share the host monotonic clock, so latency is only
  valid for the current same-host process-isolated setup.
- One run is not a distribution. Performance conclusions require repeated runs
  on controlled, architecture-specific machines and analysis outside smoke mode.
- The direct native backend still executes the benchmark workload's timer and
  callback bodies in Python; its entities and message transport are C++.
