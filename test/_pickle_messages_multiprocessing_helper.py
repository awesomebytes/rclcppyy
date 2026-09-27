#!/usr/bin/env python3
"""A C++-backed message survives a ``multiprocessing`` ``Queue`` round trip.

The worker process is spawned fresh (``spawn`` start method, not ``fork``) and
never imports ``rclcppyy`` or activates C++ acceleration, so unpickling the
message there exercises the same "no activation in this process" path a real
worker pool would hit.
"""

import multiprocessing


def _worker(receive_queue, send_queue):
    message = receive_queue.get()
    send_queue.put({
        "module": type(message).__module__,
        "linear_x": message.linear.x,
        "linear_y": message.linear.y,
        "angular_z": message.angular.z,
    })


def main():
    import rclcppyy

    rclcppyy.enable_cpp_acceleration(
        profile="direct_cpp", interfaces=("geometry_msgs/msg/Twist",))
    from geometry_msgs.msg import Twist

    message = Twist()
    message.linear.x = 4.0
    message.linear.y = -2.0
    message.angular.z = -1.5
    assert type(message).__module__.startswith("cppyy.")

    context = multiprocessing.get_context("spawn")
    receive_queue = context.Queue()
    send_queue = context.Queue()
    process = context.Process(target=_worker, args=(receive_queue, send_queue))
    process.start()
    try:
        receive_queue.put(message)
        result = send_queue.get(timeout=60)
        process.join(timeout=60)
        assert process.exitcode == 0, "worker process failed: exit %r" % (process.exitcode,)
    finally:
        process.join(timeout=5)
        if process.is_alive():
            process.terminate()

    assert not result["module"].startswith("cppyy."), (
        "worker never activated C++ acceleration but still produced a "
        "C++-backed message: %r" % (result["module"],))
    assert result["linear_x"] == 4.0
    assert result["linear_y"] == -2.0
    assert result["angular_z"] == -1.5
    print("PICKLE_MESSAGES_MULTIPROCESSING_OK", flush=True)


if __name__ == "__main__":
    main()
