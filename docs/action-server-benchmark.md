# Controlled action-server benchmark

`scripts/benchmarks/run_action_server_benchmark.py` characterizes server-side CPU
for a sequential ROS 2 action workload. It emits raw evidence with claims and
interpretation disabled.

## Fixed contract

- ROS 2 Jazzy with `rmw_cyclonedds_cpp`.
- Installed `tf2_msgs/action/LookupTransform` interface.
- One fresh server and one common Release AOT `rclcpp_action` client per sample.
- Single-threaded executors, one active goal, public action QoS, and isolated ROS
  domains/process groups.
- 20 warmup goals, 500 measured goals, three feedback messages per goal, five
  repetitions, and rotating lane order.
- Exact goal sequence, feedback count, result payload, checksum, graph teardown,
  crossing count, and bounded post-warmup RSS growth.

The primary metric is server `CLOCK_PROCESS_CPUTIME_ID` nanoseconds per completed
goal. Client CPU is drift-only diagnostics. Secondary metrics are completed goals
per second and accept, first-feedback, and result latency distributions.

## Lanes

| Lane | Authority and data path |
| --- | --- |
| `stock-rclpy` | Ordinary Python ActionServer and generated Python messages. |
| `direct-source-compatible` | Public callback shape over `rclcpp_action` with generated C++ Goal, UUID, Feedback, Result, and envelopes. |
| `native-python-orchestrated` | Managed native server with C++ values and Python orchestration. |
| `native-cpp-state-machine` | Cached C++ server state machine with zero Python callbacks per goal. |
| `aot-staged` | Conventional Release AOT `rclcpp_action` server. |

Every dynamic exact-C++ lane arms conversion, serialization, and CDR tripwires.
The source-compatible and managed lanes also report exact-C++ shared handoffs,
goal-ID materializations, feedback/result submissions, and adapter deep copies.

Run the fixed matrix with:

```bash
pixi run action-server-bench \
  --output build/action-server-evidence.json
```

## Current characterization

Clean artifact `build/action-server-cyclone-b2f0f2d-corrected.json` uses product
`b2f0f2d` and suite `948fd3d`; all 25 samples pass. Median paired direct/stock
ratios are 0.6475 server CPU, 0.7386/0.7364 accept p50/p99,
0.9917/0.9522 first-feedback p50/p99, 0.9893/0.9758 result p50/p99, and 1.0107
throughput. Direct samples arm all 16 boundary tripwires and record zero boundary
calls.

The direct lane performs 2,080 counted exact-C++ deep copies per 520-goal sample
and consumes 2.588x the server CPU of the AOT lane. This is an optimization target,
not a representation failure. Claims remain disabled pending dedicated-host and
native ARM64 repetition.
