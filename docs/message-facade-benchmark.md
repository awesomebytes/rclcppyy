# Message-facade characterization

This focused benchmark compares identical stock `rclpy` and
`profile="message_facade"` self-publish/subscription loops for
`std_msgs/msg/UInt64` and a fixed 64-byte `std_msgs/msg/String` payload. It is a
characterization protocol, not a performance claim.

```bash
pixi run message-facade-bench \
  --repetitions 5 \
  --warmup-messages 500 \
  --messages 5000 \
  --output build/message-facade-characterization.json
```

Each sample runs in a fresh process with a stock
`SingleThreadedExecutor`. Activation, Cling/template setup, DDS discovery, and a
fixed warmup are complete before either process CPU or elapsed time starts. Work
is sequential ping-pong, so every measured publish has one acknowledged callback
and one latency observation. Repetitions rotate both message-type order and
stock/facade order to limit systematic first-run bias.

The artifact reports CPU nanoseconds per message first, then nearest-rank p50 and
p99 latency and throughput. RSS is only a 64 MiB growth guard; it is not a ranking
metric. Every successful facade sample must prove:

1. exact stock `Node`, `Publisher`, `Subscription`, and executor identities;
2. direct C++ publisher and serialized-take status records;
3. the existing `std_msgs::msg::UInt64` or `std_msgs::msg::String`
   representation;
4. a converter-forbidden guard and zero Python-to-C++ whole-message conversions;
5. direct-route counters equal to warmup plus measured work;
6. complete ordered values, checksum, no drops/exceptions, and clean teardown.

The raw JSON always sets `performance_claims_allowed` to `false`. Promotion stays
blocked until native ARM64 correctness passes and repeated controlled-host runs
show reproducible CPU wins without unacceptable p50, p99, throughput, or RSS-guard
regressions. Hosted CI smoke timing is correctness evidence only.
