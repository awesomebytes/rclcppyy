# Managed Native Worker

The stock worker uses a reentrant callback group and a two-thread Python executor.
The Tier 2 rewrite opts into a custom managed `rclcpp` Context, real C++ nodes and
entities, real callback group and executor objects, and intra-process communication
through `NodeOptions`. The transform remains Python so the evidence reports one
transform-boundary crossing per input rather than implying callback lowering.

This changes ROS object ownership and scheduling and is therefore explicit opt-in.
The oracle covers ordered values and normal teardown; it does not claim complete
scheduling equivalence under concurrency.
