# Controlled service-callback benchmark

This benchmark characterizes the server-side CPU cost of one fixed
`std_srvs/srv/SetBool` service across six callback boundaries. It is a raw
evidence generator, not a release gate or a source of performance claims. ROS 2
Jazzy with CycloneDDS is the only accepted environment for this v1 protocol.

## Variants

1. `stock-rclpy`: an unmodified Python service.
2. `compatible-rclcppyy`: the same Python server function and callback, with
   compatible activation as the only setup difference. The service remains a
   Python-authoritative `rclpy.service.Service`.
3. `native-python-callback`: a benchmark-private, content-addressed
   `rclcpp::Service` bridge that enters Python exactly once per request. Python
   returns the success decision; C++ materializes the exact response string.
4. `direct-cpp-rclcppyy`: the source-compatible `Node.create_service` shape in
   the opt-in `direct_cpp` profile. The entity and request/response values are
   generated C++ objects. Each call has one Python callback crossing, one owning
   C++ request copy, and one C++ response assignment; message conversion and
   serialization paths remain at zero.
5. `native-cpp-callback`: `NativeSession.create_native_service` with no Python
   callback crossing.
6. `aot-staged`: a conventional Release-mode C++ server.

Every server implements the same contract: `success` equals request `data`, and
`message` is exactly `enabled` or `disabled`. The same Release-mode AOT client
drives every route with reliable/volatile `KeepLast(10)` service QoS, alternating
boolean input, and one outstanding request at a time.

## Measurement boundary

Each sample leases one ROS domain and creates a fresh server/client process
pair, fresh process groups, unique node names, and a unique service name. The
client proves that exactly one target `SetBool` service belongs to the expected
server node before issuing warmup requests. The measured barrier is:

1. graph discovery and topology verification;
2. client warmup completion;
3. server receives `START`, confirms the exact warmup count, resets or baselines
   counters, and starts its process CPU clock;
4. server emits the exact `armed` record;
5. client receives `START` and enters the measured loop.

The server process CPU nanoseconds per measured request is the primary
observation. Secondary observations are client process CPU, every raw RTT,
p50/p95/p99/max RTT, and closed-loop requests per second. Request counts,
alternating-input parity, response checksum, Python callback crossings, and
pending requests are exact acceptance conditions.

After the client finishes, the server stops its CPU clock and tears down. Only
then may the client continue; it must observe the service endpoint disappear
before exiting. Timeouts terminate whole process groups. Python-hosted server
timeouts first request an all-thread stack dump, which is retained in failure
evidence. Post-warmup peak-RSS growth is only a 64 MiB runaway guard and is
never ranked or interpreted as a performance result.

## Build and cache evidence

The AOT executable is built in a temporary private directory. Its Release
compile command must contain `-O3` and `-DNDEBUG`; source, compile database, and
executable hashes are stored in the result. Dynamic helper compilation uses a
separate temporary cache. Fresh cold and warm prewarm processes must prove a
miss followed by a hit for identical content-addressed artifacts before any
sample runs. Compilation is excluded from measured samples, and both temporary
trees are removed afterward.

## Running

```bash
pixi run service-callback-bench --smoke \
  --output build/service-callback-smoke.json
pixi run service-callback-bench \
  --output build/service-callback-measurement.json
```

Smoke mode uses a short workload to validate compilation, authority markers,
protocol barriers, exact parity, and teardown. Measurement mode defaults to
five rotating repetitions of 2,000 measured requests after 100 warmup requests.
Run measurement mode only on a quiet, controlled host and retain the JSON
artifact. The schema and in-process validator keep performance claims, winner
selection, maximum-rate claims, and cross-machine claims disabled.
