# Technique Selection

| Tier | Use when | Mechanism | Required proof |
|---|---|---|---|
| 0 | Existing software must remain unchanged | Compatible activation, startup hook, status report | Stock differential parity and expected mixed backend report |
| 1 | Setup/JIT/cache work dominates and semantics need not change | Warm PCH, content-addressed typed factories, stable ownership | Cold/warm startup samples and identical behavior |
| 2 | The application explicitly wants C++ ROS facilities | Managed Context/NodeOptions/executors/callback groups, intra-process, runtime loan query | Capability result, ownership/teardown tests, isolated option benchmark |
| 3 | A callback or relay is measurably hot | Editable native callback or fused pipeline; every/latest/batch policies | Wire/value parity, zero hot Python crossings, queue/exception/shutdown tests |
| 4 | Conversion into a native library dominates | Domain-kit type adapter or library-native storage | Lifetime/mutability proof, copy accounting, end-to-end benchmark |

## Decision Notes

- Small Python messages may lose to stock behavior after Python-to-C++ conversion.
- Prefer native messages or fused processing when the data can remain in C++.
- `latest` changes delivery and coalesces pending input. `batch` adds a bounded,
  drop-newest queue. Both require explicit opt-in and workload-specific counters.
- Intra-process, loaning, reuse, allocators, and multi-threaded execution change
  ownership or scheduling. Enable and measure one choice at a time.
- Query loan support on the concrete publisher and active RMW; never infer it from
  ROS distribution or message type alone.
