# Custom-interface AOT fixture

This source-only fixture generates one nested message, service, and action package,
then compiles a normal `rclcpp`/`rclcpp_action` peer ahead of time. The Python
runner exercises the same peer twice:

- stock `rclpy` publishes, subscribes, calls the service, and drives the action;
- the compatible backend repeats the application and requires its custom-message
  publisher to report `cpp`, while unsupported subscription/control-plane routes
  remain stock Python.

Both modes write JSON evidence under `build/test-results`. Run the complete,
bounded proof with:

```bash
pixi run custom-interfaces
```

The fixture belongs to native source CI on x86-64 and ARM64. It is deliberately
not part of the released package or ARM64 package-install evidence.
