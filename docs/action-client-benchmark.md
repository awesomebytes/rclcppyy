# Controlled action-client benchmark

`scripts/benchmarks/run_action_client_benchmark.py` characterizes client-side CPU
cost for a sequential ROS 2 action workload. It is a raw-evidence benchmark, not a
performance claim. The generated document keeps both `claims.enabled` and
`interpretation.enabled` false.

## Fixed contract

- ROS 2 Jazzy with explicit `rmw_cyclonedds_cpp`.
- Installed `tf2_msgs/action/LookupTransform` interface.
- One fresh Release AOT C++ server and one fresh client process per sample.
- One leased ROS domain per run, unique action and node names per sample, and fresh
  process groups.
- One active goal at a time, 20 warmup goals, 500 measured goals, five repetitions,
  and rotating lane order.
- Goal target and source frame strings encode `warmup|measured` plus the exact
  one-based sequence. The server accepts only the next exact goal.
- Exactly three empty `LookupTransform.Feedback` messages per accepted goal.
- Every terminal result is `SUCCEEDED`, returns the decimal sequence in
  `transform.child_frame_id`, and uses `TF2Error.NO_ERROR`.
- The action graph must contain exactly the send-goal, get-result, cancel-goal,
  feedback, and status endpoints with the recorded default action QoS policies.
- All clients and the server use single-threaded executors.

The common server waits 20 ms after each acceptance for the client to install the
goal handle, spaces feedback publications by 10 ms, and waits 20 ms before the
terminal response. Feedback and results travel on independent action channels;
these fixed barriers keep the exact-count proof deterministic. They are identical
for all lanes and are included in latency characterization.

## Lanes

| Lane | Authority and data path |
| --- | --- |
| `stock-rclpy` | Ordinary `rclpy.action.ActionClient` with Python messages. |
| `compatible-rclcppyy` | Compatible activation only; the same Python action authority remains in control. |
| `native-python-orchestrated` | `NativeSession.create_native_action_client`, direct C++ goal objects, C++-owned futures and queues, and Python orchestration. |
| `native-cpp-state-machine` | Benchmark-private content-addressed C++ loop; goal, feedback, result, latency, and checksum state remain in C++. |
| `aot-staged` | Conventional Release AOT `rclcpp_action` client. |

The compatible lane is a contract/control lane, not an acceleration claim. Only
the native lanes are conversion-free: application goals, feedback, results, and
associated state remain in their generated C++ representations throughout.

The crossing counters cover typed goal, feedback, and result transfers across the
Python/C++ boundary. The managed Python lane therefore reports one goal, three
feedback, and one result crossing per goal. State-query polling is not a typed data
transfer and is reported separately as `orchestration_poll_count`. The native C++
state-machine and AOT lanes perform no per-goal Python work and report zero.

## Metrics

The primary metric is client `CLOCK_PROCESS_CPUTIME_ID` nanoseconds per completed
measured goal. Secondary evidence is:

- completed goals per second;
- send-to-accept latency p50/p95/p99/max;
- send-to-first-feedback latency p50/p95/p99/max;
- send-to-result latency p50/p95/p99/max;
- post-warmup peak RSS growth guard.

The server records process CPU per measured goal only as a drift diagnostic. It is
not a benchmark outcome and is not used to normalize or rank client samples.

## Reproducibility

The runner builds the common server and AOT client in a temporary private CMake
directory and rejects compile commands that do not contain `-O3` and `-DNDEBUG`.
It also prewarms the current native action-client glue and the private C++ state
machine in two fresh processes, proving a cold miss followed by a warm hit. Build,
cache, and worker setup time are excluded from samples.

Run the fixed matrix with:

```bash
pixi run action-client-bench \
  --output build/action-client-evidence.json
```

The runner exits nonzero if any sample fails. It still records the failed case so
the document must cover the exact 25-case matrix.
