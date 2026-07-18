# Hazards

- **Handle ownership:** Never create two owners for one `rcl_*` handle. A borrowed
  adapter must not retain the stock entity beyond the operation.
- **Message ABI:** Generated Python/rclpy typesupport may use C layouts while
  `rclcpp` uses C++ message layouts. Cross the same stock publisher with serialized
  CDR, not a direct C++ object passed to `rcl_publish`.
- **Callbacks:** Pin Python callables for the full C++ lifetime. Native callbacks
  must catch exceptions and expose counters without re-entering Python.
- **Threads and GIL:** Do not assume cppyy releases the GIL. Use a proven no-GIL
  wrapper for CPU kernels; keep ROS executor callbacks entirely C++ when claimed.
- **Queues:** Bound every queue. Define overflow as drop-newest, drop-oldest, block,
  or coalesce and report counters. Never change this in compatible mode.
- **Views:** A zero-copy view must retain its owner and become invalid when ownership
  changes. Document mutability and reject use-after-close.
- **Loaned messages:** Capability varies by RMW and entity. Use RAII, query at
  runtime, and keep an explicit allocation fallback only when requested.
- **Caches:** Include architecture, ABI, compiler, C++ standard, ROS/RMW, source,
  flags, headers, and libraries in keys. A corrupt artifact must rebuild safely.
- **Teardown:** Cancel executors, close worker-owned resources, release nodes, then
  shut down Context while Python and Cling are alive. Test signal and exception exits.
- **Measurements:** Separate first-use compilation, warm cache, conversions, DDS,
  callback time, and observer overhead. A smoke run is not a release claim.
