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

The raw runner always writes `performance_claims_allowed: false`, including in
measurement mode. Smoke reports identify validation-only evidence; measurement
reports identify characterization inputs and do not select a winner. A separate
reviewed analysis must combine repeated controlled runs before making a claim.

## Native Python Boundary Characterization

`run_boundary_benchmark.py` isolates one narrower question from ROS transport:
how much of a deterministic native loop remains when every iteration crosses
from C++ into a Python transform and back. It executes the same uint64 transform
in three forms:

- `cppyy-python-boundary`: a compiled C++ loop invokes a Python callable and
  consumes its C++ return value on every iteration;
- `cppyy-fused`: the transform and loop remain in C++ loaded through cppyy, with
  no per-iteration Python callback;
- `aot-cpp`: an independently compiled `-O3` ELF executable runs the shared C++
  kernel without cppyy.

Run the backend/parity smoke gate directly through the project environment:

```bash
pixi run python scripts/benchmarks/run_boundary_benchmark.py \
  --smoke \
  --output build/boundary-smoke.json
```

An intentional characterization uses fresh processes for every repetition:

```bash
pixi run python scripts/benchmarks/run_boundary_benchmark.py \
  --iterations 50000 \
  --repetitions 5 \
  --output build/boundary-characterization.json
```

The parent compiles the AOT worker in a private temporary directory, places each
sample in a new process group, assigns a unique run token, and rejects a result
unless the PID, execution model, observed Python callback count, and checksum
match the selected variant. JIT/AOT compilation, import, process startup, and
warmup are recorded or excluded from the timed loop. The portable result schema
is [`boundary-benchmark-v1.schema.json`](../schemas/boundary-benchmark-v1.schema.json).

Raw medians and ratios quantify this particular run, but the schema fixes both
`performance_claims_allowed` and comparison `interpretation_allowed` to false.
The Python-boundary delta includes the small Python transform body as well as the
language crossing; it is not a pure ABI-call measurement. The standalone kernel
does not initialize ROS, so these results do not measure executors, DDS, message
conversion, transport, or end-to-end latency. cppyy JIT code generation also is
not assumed to match the AOT compiler's `-O3` optimization level.

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

Each matrix acquires an advisory lease for a unique `ROS_DOMAIN_ID` and uses a
random run token in every topic. Concurrent matrices on one host therefore cannot
share endpoints accidentally; the leased domain and token are recorded in JSON.

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
