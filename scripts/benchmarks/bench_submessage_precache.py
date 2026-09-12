#!/usr/bin/env python3
"""Microbenchmark: cppyy sub-message field access, with vs without the
subscriber-side ``__dict__`` pre-cache.

See ``rclcpp_kit.direct_entities._submessage_precache`` (the subscriber
dispatch trampoline that now runs this automatically) and the field-access
investigation it implements: cppyy resolves a sub-message field (e.g.
``twist.linear``) through a ~87ns proxy-creation path on every access.
Pre-populating ``msg.__dict__`` with that proxy once drops repeat access
to ~44ns, because cppyy's ``__getattribute__`` checks ``__dict__`` before
re-resolving the C++ member.

This isolates the field-access cost itself via ``timeit``, independent of
the ROS transport pipeline: the transport savings from keeping messages
C++ end-to-end are three to four orders of magnitude larger than this
effect (the investigation's "Key Insight"), so measuring inside a live
pub/sub loop would drown the signal in transport/scheduling noise.
"""
import argparse
import timeit

from rclcpp_kit.bringup_rclcpp import bringup_rclcpp
from rclcpp_kit.direct_entities import _submessage_precache
from rclcpp_kit.direct_message_types import load_message_type


def _ns_per_access(message, field, accesses, repeat):
    best = min(
        timeit.repeat(
            lambda: getattr(message, field), number=accesses, repeat=repeat))
    return best / accesses * 1e9


def bench_field(cpp_type, field, accesses, repeat):
    uncached = cpp_type()
    uncached_ns = _ns_per_access(uncached, field, accesses, repeat)

    cached = cpp_type()
    precache = _submessage_precache(cpp_type)
    if precache is None:
        raise SystemExit(
            "%s has no composite fields to pre-cache -- pick a message "
            "type with a sub-message field (e.g. geometry_msgs/Twist, "
            "std_msgs/Header)" % cpp_type)
    precache(cached)
    cached_ns = _ns_per_access(cached, field, accesses, repeat)
    return uncached_ns, cached_ns


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package", default="geometry_msgs")
    parser.add_argument("--name", default="Twist")
    parser.add_argument(
        "--fields", nargs="+", default=["linear", "angular"],
        help="composite (sub-message) field names to benchmark")
    parser.add_argument("--accesses", type=int, default=200_000)
    parser.add_argument("--repeat", type=int, default=5)
    args = parser.parse_args()

    bringup_rclcpp()
    descriptor = load_message_type(args.package, args.name)
    cpp_type = descriptor.cpp_type

    print("%s: %d accesses x %d repeats, best-of-repeat reported" % (
        descriptor.cpp_type_name, args.accesses, args.repeat))
    for field in args.fields:
        uncached_ns, cached_ns = bench_field(
            cpp_type, field, args.accesses, args.repeat)
        print(
            "  .%s  uncached=%6.1fns  cached=%6.1fns  speedup=%.2fx" % (
                field, uncached_ns, cached_ns, uncached_ns / cached_ns))


if __name__ == "__main__":
    main()
