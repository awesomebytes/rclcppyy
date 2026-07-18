# Controlled C++ topology-fusion benchmark

This benchmark measures an explicit C++ optimization which removes ROS graph
boundaries between four fixed `std_msgs::msg::UInt64` transforms. It targets
ROS 2 Jazzy with CycloneDDS first. CPU time owned by the relay process is the
primary metric; end-to-end latency and closed-loop throughput are secondary.
Peak RSS growth is only a 64 MiB runaway guard.

## Compared implementations

`aot-staged` is the conventional baseline. All four stages are compiled
together in Release mode with `-O3 -DNDEBUG`. They run in one relay process,
one `rclcpp::Node`, and one single-threaded executor. Intra-process
communication is enabled through `rclcpp::NodeOptions`, so the staged baseline
can use rclcpp's composition optimization. It retains five observable ROS
topics and four subscription/publisher pairs.

`cppyy-fused` loads one content-addressed, prewarmed C++ callback and kernel
through cppyy. It uses one relay process, one `rclcpp::Node`, one
single-threaded executor, one subscription, and one publisher. Intra-process
communication is enabled consistently, although there are no intermediate
entities on which it can act.

`aot-fused` implements the same single-callback topology as a conventional
Release AOT executable. It is the compiled C++ ceiling needed to distinguish a
fusion benefit from cppyy-specific overhead.

All relay lanes receive, transform, and publish the generated C++
`std_msgs::msg::UInt64` representation. Python starts and controls the cppyy
lane, but no message callback, Python message, serialization bridge, or
Python/C++ message conversion occurs in a measured relay path.

## Exact contract and tradeoff

Every lane has the same external input topic, final output topic, QoS, input
sequence, four logical transforms, final values, and checksum. A common
Release AOT closed-loop driver supplies one outstanding request and records
every round-trip latency.

Fusion is deliberately opt-in because it changes graph observability. The
three intermediate ROS topics and their six entities disappear. Existing
subscribers, recorders, QoS boundaries, remaps, or introspection that depend on
those topics cannot observe them in fused mode. Fusion is valid only after the
application confirms those intermediate boundaries are not part of its
contract.

Each sample uses a fresh relay/driver process pair, process groups, unique
topics, and a leased domain. The driver completes discovery, endpoint-owner
and actual-QoS inspection, and warmup before the relay is armed. The runner
rotates variant order by repetition. It compiles AOT code and performs isolated
cold/warm cppyy cache proof before samples; compilation is excluded from timed
regions. Driver teardown proves relay endpoints disappear.

## Running

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DISTRO=jazzy \
  pixi run python scripts/benchmarks/run_fusion_pipeline_benchmark.py \
  --smoke --output build/fusion-pipeline-smoke.json

RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DISTRO=jazzy \
  pixi run python scripts/benchmarks/run_fusion_pipeline_benchmark.py \
  --output build/fusion-pipeline-measurement.json
```

Smoke mode uses one short repetition. Measurement mode uses five rotating
repetitions of 2,000 measured messages after 100 warmup messages. The versioned
protocol, JSON Schema, and in-process validator keep performance claims and
automatic winner selection disabled. Interpret results only from a quiet-host
measurement artifact, with `cppyy-fused` compared against both AOT lanes.
