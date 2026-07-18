# Rewrite Patterns

## Compatible activation

```python
import rclcppyy
rclcppyy.enable_cpp_acceleration(warn_fallback=True)
# Existing imports and application code remain unchanged.
```

Assert the route with `rclcppyy.status()`. Use `profile="required_cpp"` only in a
test or deployment that should fail before every unsupported entity operation.

## Managed native lane

```python
with rclcppyy.native(["program"]) as ros:
    options = ros.rclcpp.NodeOptions()
    node = ros.create_node("worker", options=options, use_intra_process=True)
    executor = ros.create_executor("multi_threaded", threads=2)
    executor.add_node(node)
```

All returned entities are real C++ objects. Use `ros.rclcpp` for unwrapped APIs.

## Native callback and fused pipeline

Exercise the route before asserting its counters. This example republishes while
waiting so DDS discovery cannot turn a lost first sample into a false failure.

```python
import time

import rclcppyy
from std_msgs.msg import String


def deliver_until(executor, publish, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        publish()
        for _ in range(10):
            executor.spin_some()
            if predicate():
                return
            time.sleep(0.005)
    raise AssertionError("native route did not process a message")


observed = []
with rclcppyy.native(["native-route-proof"]) as ros:
    node = ros.create_node("worker")
    peer = ros.create_node("peer")
    executor = ros.create_executor()
    executor.add_node(node)
    executor.add_node(peer)

    callback = ros.create_native_callback(
        node, String, "callback_input",
        "set_value(static_cast<int64_t>(message.data.size()));",
    )
    callback_source = peer.create_publisher(String, "callback_input", 10)
    deliver_until(
        executor,
        lambda: callback_source.publish(String(data="1234567")),
        lambda: callback.stats().processed > 0,
    )
    callback_stats = callback.stats()
    assert callback.value() == 7
    assert callback_stats.received > 0
    assert callback_stats.processed > 0
    assert callback_stats.exceptions == 0
    assert callback_stats.python_boundary_crossings == 0

    relay = ros.create_fused_pipeline(
        node, String, String, "pipeline_input", "pipeline_output",
        'output.data = input.data + ":native";',
        delivery="every",
    )
    pipeline_source = peer.create_publisher(String, "pipeline_input", 10)
    sink = peer.create_subscription(
        String, "pipeline_output", lambda message: observed.append(str(message.data)), 10)
    assert sink is not None
    deliver_until(
        executor,
        lambda: pipeline_source.publish(String(data="payload")),
        lambda: "payload:native" in observed,
    )
    pipeline_stats = relay.stats()
    assert pipeline_stats.received > 0
    assert pipeline_stats.processed > 0
    assert pipeline_stats.published > 0
    assert pipeline_stats.exceptions == 0
    assert pipeline_stats.python_boundary_crossings == 0

assert callback.closed and relay.closed
```

The transform sees `input`, `output`, and `publish`. Set `publish = false` to
filter. Use `latest` or `batch` only when coalescing/drop semantics are acceptable.
Counters are evidence only after processed work; pair them with application-visible
parity and the independent backend assertions required by `evidence.md`.
