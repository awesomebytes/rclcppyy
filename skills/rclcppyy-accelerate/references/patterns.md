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

## Native callback

```python
counter = ros.create_native_callback(
    node, String, "input",
    "set_value(static_cast<int64_t>(message.data.size()));",
)
assert counter.stats().python_boundary_crossings == 0
```

## Fused pipeline

```python
relay = ros.create_fused_pipeline(
    node, String, String, "input", "output",
    'output.data = input.data + ":native";',
    delivery="every",
)
```

The transform sees `input`, `output`, and `publish`. Set `publish = false` to
filter. Use `latest` or `batch` only when coalescing/drop semantics are acceptable.
