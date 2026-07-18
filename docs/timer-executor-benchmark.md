# Controlled timer-executor benchmark

This benchmark characterizes process CPU and scheduling behavior for a fixed
1 ms steady timer on ROS 2 Jazzy with Cyclone DDS. It produces raw evidence with
performance claims disabled.

## Variants

1. `stock-rclpy`: stock timer, executor, and Python callback.
2. `compatible-rclcppyy`: activation-only control with the same stock authority.
3. `direct-cpp-rclcppyy`: the source-compatible `Node.create_timer` and
   `rclpy.spin_once` surface backed by the same session-owned `rclcpp` timer and
   executor, entering Python once per firing.
4. `native-python-callback`: managed `rclcpp` timer and executor entering Python
   once per firing.
5. `native-cpp-callback`: content-addressed C++ timer callback with no per-firing
   Python crossing.
6. `aot-staged`: conventional Release-mode `rclcpp` executable.

The compatible lane is not an acceleration claim. The direct lane keeps the
unchanged Python callback workload while changing timer and executor authority.
The two C++ callback lanes keep
all recurrence and timing state in C++ and report zero per-firing Python crossings.

## Fixed contract

Every fresh worker performs 500 warmup firings followed by 5,000 measured firings.
Five repetitions rotate lane order. A wrapping `uint64_t` recurrence provides an
exact final state and checksum across implementations. Each sample proves its node
appears after readiness, disappears after exit, stops after timer cancellation, and
leaves no pending post-cancel callback.

The primary metric is worker `CLOCK_PROCESS_CPUTIME_ID` nanoseconds per measured
firing. Secondary evidence includes effective frequency, signed and absolute
scheduled-deadline error p50/p95/p99/max, missed periods, exceptions, and a bounded
post-warmup RSS growth guard. Memory is not ranked.

The runner builds the AOT executable with `-O3` and `-DNDEBUG`, uses a temporary
content-addressed native cache, proves a cold miss followed by a warm hit, and
excludes all compilation and prewarming from samples.

## Running

Run the fixed 30-sample matrix only on a quiet host:

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  pixi run timer-executor-bench \
  --output build/timer-executor-measurement.json
```

The runner rejects non-Jazzy environments, forces Cyclone DDS, terminates complete
process groups on failure, and writes a schema-validated artifact without selecting
a winner.
