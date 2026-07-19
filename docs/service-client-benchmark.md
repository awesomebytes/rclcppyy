# Controlled service-client benchmark

This benchmark characterizes client-side CPU cost for one fixed
`std_srvs/srv/SetBool` service across six execution boundaries. It is a raw
evidence generator, not a release gate or a source of performance claims. ROS 2
Jazzy with CycloneDDS is the only accepted environment for this v1 protocol.

## Variants

1. `stock-rclpy`: an unmodified Python client.
2. `compatible-rclcppyy`: the same Python client function, with compatible
   activation as the only setup difference. The client remains a
   Python-authoritative `rclpy.client.Client`.
3. `direct-cpp-rclcppyy`: the production `direct_cpp` client behind the usual
   `Node.create_client()`, `SetBool.Request(...)`, `call_async()`, and
   `rclpy.spin_until_future_complete()` call shape. The node and client are C++
   authoritative, Request and Response are the actual generated C++ types, and
   each operation returns an actual `rclpy.task.Future`. There is one Python
   request crossing, one Python response crossing, exactly one native C++ value
   copy of the request, and no Python message conversion.
4. `native-python-orchestrated`: the existing managed native client with Python
   orchestrating each request. It allocates and submits the direct shared C++
   `SetBool::Request`; no Python message conversion occurs. Exact managed-client
   counters record one Python request and response crossing per call.
5. `native-cpp-state-machine`: a benchmark-private, content-addressed C++ client
   that owns graph verification, requests, waits, validation, and timing for the
   complete loop. There is no per-request Python crossing or Python message
   conversion.
6. `aot-staged`: a conventional Release-mode C++ client.

Every client talks to the same staged Release AOT C++ server. The service
contract is exact: response `success` equals request `data`, and `message` is
`enabled` or `disabled`. All lanes use reliable/volatile `KeepLast(10)` service
QoS, alternating boolean input, and one outstanding request at a time.

## Measurement boundary

Each sample leases one ROS domain and creates a fresh server/client process
pair, fresh process groups, unique node names, and a unique service name. The
client proves that exactly one `std_srvs/srv/SetBool` endpoint belongs to the
expected server node before completing warmup. The measured barrier is:

1. common server readiness;
2. exact client discovery, type/owner/cardinality proof, and client warmup;
3. server receives `START`, confirms the exact warmup count, and starts its
   process CPU clock;
4. server emits the exact `armed` record;
5. client receives `START` and enters the measured loop.

Client process CPU nanoseconds per completed response is the primary
observation. Secondary client observations are every raw RTT, p50/p95/p99/max
RTT, and closed-loop requests per second. The common server process CPU is
retained only as a drift diagnostic and is never ranked. Counts, input parity,
response checksum, Python orchestration, request/response crossings, message
conversions, exceptions, and pending requests are exact acceptance conditions.
The direct C++ lane additionally records its status decision, C++ request and
response representations, Future control model, handoff semantics, and measured
C++ request-copy count. Request, Future, and response identities are checked
during warmup, outside the timed loop. Fail-fast guards make any Python-message
conversion or serialization call fail the sample. Its teardown proof requires
endpoint disappearance, a closed client, a destroyed node, a stopped native
executor, and a closed native session.

After the client reports, the server stops its clock and tears down. Only then
may the client continue; it must observe the service endpoint disappear before
exiting. Timeouts terminate whole process groups. Dynamic client timeouts first
request an all-thread stack dump and retain that diagnostic in failure evidence.
Post-warmup peak-RSS growth is only a 64 MiB runaway guard and is never ranked or
interpreted as a performance result.

## Build and cache evidence

The AOT executable is built in a temporary private directory. Its Release
compile command must contain `-O3` and `-DNDEBUG`; source, compile database, and
executable hashes are stored in the result. Dynamic helper compilation uses a
separate temporary cache. Fresh cold and warm prewarm processes must prove a
miss followed by a hit for identical managed-client and C++ state-machine
artifacts before any sample runs. Compilation is excluded from measured samples,
and both temporary trees are removed afterward.

The direct C++ and managed Python-orchestrated lanes consume the same prewarmed
managed-client artifact. This keeps compilation outside the sample and makes the
comparison about the production Python-facing control path rather than cache
state.

## Running

Invoke the isolated runner through the project environment:

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  pixi run service-client-bench \
  --smoke --output build/service-client-smoke.json

RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  pixi run service-client-bench \
  --output build/service-client-measurement.json
```

Smoke mode uses a short workload to validate compilation, authority markers,
representation/crossing evidence, protocol barriers, exact parity, and teardown.
Measurement mode defaults to five rotating repetitions of 2,000 measured
requests after 100 warmup requests. Run measurement mode only on a quiet,
controlled host and retain the JSON artifact. The schema and in-process
validator keep performance claims, winner selection, maximum-rate claims, and
cross-machine claims disabled.
