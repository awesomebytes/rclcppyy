#!/usr/bin/env python3
"""White-box lifecycle check for the compiled Python subscription bridge.

Destroying the public subscription must close its native bridge handle, and
repeated close/destroy calls must stay idempotent. The MTE overlap probes
cover callback lifetime while a native dispatch is in flight.
"""
import gc
import os
import weakref

import rclcppyy

rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import UInt64  # noqa: E402


def main():
    rclpy.init(args=[])
    node = Node("destroy_no_sever_%d" % os.getpid())
    class Callback:
        def __call__(self, _message):
            pass

    callback = Callback()
    callback_ref = weakref.ref(callback)
    sub = node.create_subscription(
        UInt64, "/direct_cpp/destroy_no_sever/topic", callback, 10)

    native = getattr(sub, "_native", None)
    assert native is not None, "DirectSubscription._native missing"
    assert native.callback_handoff == "compiled_python_callback"
    assert native.source_id
    assert not native.closed
    # Drop both public callback references. The compiled C++ dispatch state
    # must retain the callable for as long as the subscription is live.
    sub._callback = None
    del callback
    gc.collect()
    assert callback_ref() is not None

    result = node.destroy_subscription(sub)
    assert result is True
    assert sub.closed
    assert native.closed
    assert native.close() is False
    # Idempotent: destroying twice is a documented no-op, not a double-free.
    assert node.destroy_subscription(sub) is False
    del sub
    gc.collect()
    assert callback_ref() is None, "closed bridge retained the Python callback"

    node.destroy_node()
    rclpy.shutdown()
    assert not rclpy.ok()
    print("COMPILED_CALLBACK_BRIDGE_CLOSE_OK", flush=True)


if __name__ == "__main__":
    main()
