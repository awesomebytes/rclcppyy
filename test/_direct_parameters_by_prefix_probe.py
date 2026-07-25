#!/usr/bin/env python3
"""Fresh-process stock/direct differential for ``get_parameters_by_prefix``
(wave 7 slice 3, PLAN-wave7.md §4.3).

Stock's own body reads its naive prefix rule literally against its private
``self._parameters`` cache -- the double-separator edge case
(``"foo."`` internally becomes ``"foo.."``) is a real, documented-as-naive
behavior to replicate, not a bug to fix. This declares the exact same
parameter set under both backends and asserts the two backends produce
byte-for-byte identical suffix->value mappings for every prefix case,
including that edge case, plus confirms the values are ``Parameter``
instances (matching stock's actual runtime shape, not its own inconsistent
declared type hint -- see PLAN-wave7.md §4.3).
"""

import argparse
import json
import os


REPORT_PREFIX = "DIRECT_PARAMETERS_BY_PREFIX_REPORT="
CASES = ("foo.", "foo", "", "nope")


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("stock", "direct"), required=True)
    return parser.parse_args()


def main():
    args = _arguments()
    if args.backend == "direct":
        import rclcppyy

        rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    rclpy.init()
    node = Node("parameters_by_prefix_probe_%s_%d" % (args.backend, os.getpid()))
    try:
        node.declare_parameter("foo.ping", 1)
        node.declare_parameter("foo..oddname", 2)
        node.declare_parameter("bar.baz", 3)
        node.declare_parameter("standalone", 4)

        results = {}
        all_parameter_type = True
        for prefix in CASES:
            result = node.get_parameters_by_prefix(prefix)
            all_parameter_type = all_parameter_type and all(
                isinstance(value, Parameter) for value in result.values())
            results[prefix] = {
                suffix: parameter.value for suffix, parameter in result.items()
            }
        # The node's own default-declared parameters (use_sim_time etc.)
        # differ in count/name between stock and direct_cpp defaults; the
        # empty-prefix case is compared with those stripped out below by the
        # test, not here, to keep this probe backend-agnostic.
    finally:
        node.destroy_node()
        rclpy.shutdown()

    report = {
        "backend": args.backend,
        "results": results,
        "all_values_are_parameter_instances": all_parameter_type,
    }
    print(REPORT_PREFIX + json.dumps(report, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
