#!/usr/bin/env python3
"""Fresh-process stock/direct differential for the 11 payload-tainted
parameter-method signatures (wave 7 slice 3, PLAN-wave7.md §4).

Run twice (once per ``--backend``), each in its own interpreter, exactly
like ``_direct_parameter_get_probe.py`` -- this is the only way to observe a
genuinely pristine stock ``rclpy.node.Node`` signature (importing
``rclpy.node`` in the *same* process that later activates ``direct_cpp``
trips ``_check_early_activation()``'s stale-import guard) alongside a
genuinely activated ``DirectNode`` one. A pristine, never-activated stock
process (``--backend stock``) never taints -- ``rcl_interfaces.msg`` is
never rebound there at all, matching the ledger's own "stock" observation
mode. The taint ``rclcppyy._payload_signature`` sidesteps only shows up
*inside* an activated process's own view of the ORIGINAL stock ``Node``
class (``direct_cpp._PATCHES``'s captured original, PLAN-wave7.md §4.1) --
``--backend direct`` additionally reports that in-process view, under
``tainted_stock_signatures``, to prove the taint being sidestepped is real,
not hypothetical.
"""

import argparse
import inspect
import json


METHOD_NAMES = (
    "declare_parameter",
    "declare_parameters",
    "describe_parameter",
    "describe_parameters",
    "list_parameters",
    "add_on_set_parameters_callback",
    "remove_on_set_parameters_callback",
    "set_parameters",
    "set_parameters_atomically",
    "set_descriptor",
    "get_parameters_by_prefix",
)
REPORT_PREFIX = "DIRECT_PARAMETER_SIGNATURE_REPORT="


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("stock", "direct"), required=True)
    return parser.parse_args()


def main():
    args = _arguments()
    tainted_stock_signatures = None
    if args.backend == "direct":
        import rclcppyy

        rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

        from rclcppyy.direct_cpp import _PATCHES, DirectNode

        stock_node = next(
            original for _module, name, original, replacement in _PATCHES
            if name == "Node" and replacement is DirectNode)
        tainted_stock_signatures = {
            name: str(inspect.signature(getattr(stock_node, name)))
            for name in METHOD_NAMES
        }

    from rclpy.node import Node

    report = {
        "backend": args.backend,
        "signatures": {
            name: str(inspect.signature(getattr(Node, name)))
            for name in METHOD_NAMES
        },
        "tainted_stock_signatures": tainted_stock_signatures,
    }
    print(REPORT_PREFIX + json.dumps(report, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
