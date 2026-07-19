# Benchmark Matrix

`scripts/benchmarks/run_benchmarks.py` runs a bounded, declarative cross-product
of backends, workloads, target frequencies, and application payload padding.
Each publisher/subscriber pair runs in fresh child processes and the parent owns
their startup, measurement, and teardown.

The CPU-first ROS entity protocols are documented separately. Each keeps stock,
compatible, native C++ representation, and conventional AOT lanes distinct:

- [relay pub/sub](relay-boundary-benchmark.md)
- [C++-owning message facade](message-facade-benchmark.md)
- [service callback/server](service-callback-benchmark.md)
- [service client](service-client-benchmark.md)
- [timer/executor](timer-executor-benchmark.md)
- [action client](action-client-benchmark.md)
- [C++ topology fusion](fusion-pipeline-benchmark.md)

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

Smoke runs prove route selection, decoded wire-value validity, message delivery,
structured output, and clean process isolation. Their short CPU and latency samples
are not performance evidence and must not be used for comparisons or published
claims.

An intentional matrix run can select multiple axes:

```bash
pixi run bench \
  --backends rclpy,rclcppyy-direct-copy,rclcppyy-direct-lease,rclcppyy-templated \
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

The two source-compatible direct lanes run the same Python codec and callback body.
`rclcppyy-direct-copy` retains one generated-C++ callback copy;
`rclcppyy-direct-lease` transfers the received C++ allocation into shared ownership.
Both poison the Python-message converter and serializer helpers. Their backend
markers require actual C++ entities, `no_conversion` policy evidence, and the exact
generated C++ `String` or nested `Header` type. The lease marker additionally
requires actual-C++ representation and zero declared message deep copies. These
lanes remain separate from `rclcppyy-templated`, which uses the explicit native API
rather than the source-compatible `rclpy` call shape.

## Compatibility Evidence Gate

The local compatibility gate answers a narrower question before any regression
budget or performance claim is considered: did every current workload exercise the
intended stock and compatible routes, preserve its decoded value contract, and
produce repeated observations under one stable environment?

Create five complete measurement documents from a clean source tree:

```bash
mkdir -p build/compatibility-evidence
for repetition in 1 2 3 4 5; do
  pixi run bench \
    --backends rclpy,rclcppyy \
    --workloads small-string,nested-header \
    --output "build/compatibility-evidence/run-${repetition}.json"
done

pixi run bench-compatibility-evidence \
  --output build/compatibility-evidence/evidence.json \
  build/compatibility-evidence/run-{1,2,3,4,5}.json
```

The gate rejects dirty product or source-checkout dependencies, fewer than five
repetitions, changing source, module origin, machine, runtime, ROS/RMW, cache, or
matrix metadata, duplicate run tokens, missing workloads or backend pairs,
contradictory backend markers, empty deliveries, and any decoded wire-value
violation. Every flat and nested message is checked against the workload's
sequence, monotonic timestamp, and exact padding contract.

The output conforms to
[`compatibility-performance-evidence-v1.schema.json`](../schemas/compatibility-performance-evidence-v1.schema.json).
It maps the default transparent stock-publish path to its verified Python publisher
and subscriber backends and workload coverage, preserves the stable
source/environment and normalized matrix,
then records each repetition's candidate and stock values with direction counts.
`stock_better_in_all_repetitions` is an explicit negative local observation;
`candidate_better_in_all_repetitions`, `equal_in_all_repetitions`, and
`mixed_observation` are equally literal descriptions of the inputs. They do not
apply a noise threshold or establish a portable benefit. The artifact therefore
fixes both `performance_claims_allowed` and `interpretation_allowed` to `false` and
keeps the route's `performance_conclusion` at `not_established`.

## Regression Gate

The opt-in dedicated workflow runs the complete measurement matrix five times on
each self-hosted architecture and then invokes `bench-regression`. The comparator
accepts only clean-source measurement artifacts with no failures. It rejects
changes in source commit, machine, runtime, ROS/RMW environment, matrix dimensions,
case coverage, or observed publisher/subscriber backend evidence between repeats.

The comparator records each case's raw values and median. Reviewed comparisons are
relative: candidate and reference cases are paired within each repetition and the
gate evaluates the median of those paired ratios. This limits sensitivity to one
outlying run while preserving the same-machine comparison. Effective message rate
uses a minimum ratio; latency percentiles and publisher/subscriber CPU use maximum
ratios.

Architecture budgets live in `benchmarks/regression-budgets/` and conform to
[`benchmark-regression-budget-v1.schema.json`](../schemas/benchmark-regression-budget-v1.schema.json).
The deterministic output conforms to
[`benchmark-regression-v1.schema.json`](../schemas/benchmark-regression-v1.schema.json).
It always sets `performance_claims_allowed` to false: passing a reviewed regression
budget means no reviewed budget was exceeded, not that a broader performance claim
has been established.

Both committed architecture budgets initially use `status: calibration_required`
and contain no thresholds. This is intentional: hosted CI or development-machine
measurements must not define dedicated-hardware policy. In this state the comparator
writes `regression.json` and exits with status 2, so the opt-in job cannot appear to
pass without a reviewed budget.

To calibrate one architecture:

1. Run several dedicated workflow executions under fixed machine controls.
2. Review the uploaded raw runs and `regression.json` for stability and coverage.
3. Pin the exact CPU model and logical CPU count in that architecture's budget.
4. Add reviewer identity, review time, case selectors, and evidence-derived relative
   ratio limits; change the status to `reviewed`.
5. Re-run the dedicated workflow. Exit status 0 means every reviewed limit passed;
   status 1 means invalid evidence or a regression violation.

Run the same gate locally only with repeated results from one controlled machine:

```bash
pixi run bench-regression \
  --budget benchmarks/regression-budgets/x86_64.json \
  --output build/dedicated-benchmark/regression.json \
  build/dedicated-benchmark/run-{1,2,3,4,5}.json
```

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

Every successful row includes separate publisher/subscriber backend markers and a
verified wire-value contract.
The parent rejects missing, malformed, or unexpected markers, so activation
alone cannot be reported as C++ execution.

The subscriber accepts explicit start/stop control messages. Counts, sequence
gaps, and latency samples are reset at start and summarized at stop. This avoids
inferring counts from log-line frequency or combining per-interval percentiles.
Latency percentiles use the nearest-rank definition over every message received
inside the acknowledged window. CPU output retains each `psutil.cpu_percent`
sample and reports its mean, median, sample standard deviation, minimum, and
maximum.

For each callback, the subscriber decodes and validates sequence, timestamp, and
payload padding before recording latency. A violation makes the case fail rather
than becoming a timing sample.

The runtime envelope is `rclcppyy.benchmark/v3`; its portable JSON Schema is
[`schemas/benchmark-v3.schema.json`](../schemas/benchmark-v3.schema.json). The
artifact records source revision/dirty state, imported module origins, dependency
checkout revisions, architecture, CPU, Python and package versions, ROS/RMW
settings, cache environment, selected matrix, raw CPU samples, failures, and child
backend evidence.

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
- The source-compatible direct lanes likewise retain the measured Python timer and
  callback bodies. They differ only in callback ownership: one generated-C++ value
  copy versus a shared lease over the received allocation.
