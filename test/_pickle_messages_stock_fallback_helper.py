#!/usr/bin/env python3
"""Cross-process proof: unpickling never requires ``direct_cpp`` activation.

Run as ``produce <path>`` (activates direct_cpp, pickles a C++-backed
``PoseStamped`` to ``<path>``) or ``consume <path>`` (a fresh interpreter that
never activates anything, unpickling the same bytes into a stock message).
"""

import pickle
import sys


def produce(path):
    import rclcppyy

    rclcppyy.enable_cpp_acceleration(
        profile="direct_cpp", interfaces=("geometry_msgs/msg/PoseStamped",))
    from geometry_msgs.msg import PoseStamped

    message = PoseStamped()
    assert type(message).__module__.startswith("cppyy.")
    message.header.frame_id = "map"
    message.pose.position.x = 1.0
    message.pose.position.y = 2.0
    message.pose.orientation.w = 1.0
    with open(path, "wb") as handle:
        pickle.dump(message, handle)
    print("PICKLE_MESSAGES_PRODUCE_OK", flush=True)


def consume(path):
    from geometry_msgs.msg import PoseStamped

    assert not type(PoseStamped).__module__.startswith("cppyy."), (
        "this process must never have activated C++ acceleration")
    with open(path, "rb") as handle:
        restored = pickle.load(handle)
    assert not type(restored).__module__.startswith("cppyy."), (
        "unpickling without activation produced a C++-backed message: %r" %
        (type(restored),))
    assert type(restored) is PoseStamped
    assert restored.header.frame_id == "map"
    assert restored.pose.position.x == 1.0
    assert restored.pose.position.y == 2.0
    assert restored.pose.orientation.w == 1.0
    print("PICKLE_MESSAGES_STOCK_FALLBACK_OK", flush=True)


def main():
    mode, path = sys.argv[1], sys.argv[2]
    if mode == "produce":
        produce(path)
    elif mode == "consume":
        consume(path)
    else:
        raise ValueError("unknown mode: %r" % (mode,))


if __name__ == "__main__":
    main()
