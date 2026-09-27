#!/usr/bin/env python3
"""Authority coverage tripwire for the batch-(a) local-parameter promotion set.

Enumerates the exact ledger rows the prove-authority lane promotes to
``exact_direct_cpp_authority`` for the local-parameter slice (PLAN-prove-authority
§2a/§3, uncommitted): the 11 parameter-owning ``rclpy.node.Node`` methods, plus
the 7-member ``Parameter`` facade class as it appears at its 5 public module
aliases (35 rows, 46 total). Every name below is exercised concretely under
the direct profile with converters/serializers poisoned; the stock/direct
differential proves behavior parity. A name this probe does not exercise is
not part of the promoted set (the coverage tripwire is the arbiter).

``declare_parameter``, ``set_parameters``, ``describe_parameter(s)``,
``list_parameters``, and siblings are deliberately *not* claimed here: those
ledger rows are structurally ``missing_mismatch`` (signature drift) and stay
unassessed regardless of behavioral evidence. This probe uses a couple of
them only as scaffolding (to populate/mutate parameters), never as a claim.
"""

import argparse
import importlib
import json
import os
import warnings


PROBE_PREFIX = "RCLCPPYY_PROVE_PARAMETER_AUTHORITY_PROBE "

# The 11 rclpy.node.Node members this probe proves (area=node, direct_backend,
# unassessed today -- promoted here).
NODE_PARAMETER_METHODS = (
    "get_parameter", "get_parameter_or", "get_parameters", "get_parameter_type",
    "get_parameter_types", "has_parameter", "undeclare_parameter",
    "add_pre_set_parameters_callback", "add_post_set_parameters_callback",
    "remove_pre_set_parameters_callback", "remove_post_set_parameters_callback",
)

# The 5 public module paths at which the single Parameter class is importable
# (rclpy re-exports it, rclpy.node/parameter_client/parameter_event_handler
# import it for typing/construction) -- 7 members each, 35 rows.
PARAMETER_FACADE_ALIASES = (
    "rclpy", "rclpy.parameter", "rclpy.node",
    "rclpy.parameter_client", "rclpy.parameter_event_handler",
)
PARAMETER_FACADE_MEMBERS = (
    "__init__", "from_parameter_msg", "get_parameter_value",
    "name", "to_parameter_msg", "type_", "value",
)


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("stock", "direct"), required=True)
    return parser.parse_args()


def main():
    args = _arguments()
    boundary_calls = []

    if args.backend == "direct":
        import rclcppyy

        rclcppyy.enable_cpp_acceleration(profile="direct_cpp")

        def forbidden_boundary(*boundary_args, **boundary_kwargs):
            boundary_calls.append((boundary_args, boundary_kwargs))
            raise AssertionError(
                "a promoted parameter operation used a conversion or serialization path")

        bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
        kit = importlib.import_module("rclcpp_kit")
        serialization = importlib.import_module("rclcpp_kit.serialization")
        rclpy_serialization = importlib.import_module("rclpy.serialization")
        bringup.convert_python_msg_to_cpp = forbidden_boundary
        kit.convert_python_msg_to_cpp = forbidden_boundary
        serialization.serialize_message = forbidden_boundary
        serialization.deserialize_message = forbidden_boundary
        rclpy_serialization.serialize_message = forbidden_boundary
        rclpy_serialization.deserialize_message = forbidden_boundary

    import rclpy  # noqa: E402
    import rclpy.node as node_module  # noqa: E402
    import rclpy.parameter as parameter_module  # noqa: E402
    import rclpy.parameter_client as parameter_client_module  # noqa: E402
    import rclpy.parameter_event_handler as parameter_event_handler_module  # noqa: E402
    from rcl_interfaces.msg import ParameterDescriptor  # noqa: E402
    from rclpy.exceptions import ParameterNotDeclaredException  # noqa: E402
    from rclpy.node import Node  # noqa: E402

    proven = {}

    # -- Parameter facade: alias identity + the 7 members, at all 5 aliases --
    alias_modules = {
        "rclpy": rclpy,
        "rclpy.parameter": parameter_module,
        "rclpy.node": node_module,
        "rclpy.parameter_client": parameter_client_module,
        "rclpy.parameter_event_handler": parameter_event_handler_module,
    }
    canonical = parameter_module.Parameter
    alias_identity = {}
    for alias in PARAMETER_FACADE_ALIASES:
        is_same = alias_modules[alias].Parameter is canonical
        alias_identity[alias] = is_same
        for member in PARAMETER_FACADE_MEMBERS:
            proven["%s.Parameter.%s" % (alias, member)] = bool(is_same)

    instance = canonical("probe_case", value=41)
    exercised_members = {
        "name": instance.name == "probe_case",
        "type_": instance.type_ is not None,
        "value": instance.value == 41,
    }
    parameter_value = instance.get_parameter_value()
    exercised_members["get_parameter_value"] = parameter_value is not None
    message = instance.to_parameter_msg()
    exercised_members["to_parameter_msg"] = message is not None
    restored = canonical.from_parameter_msg(message)
    exercised_members["from_parameter_msg"] = (
        restored.name == instance.name and restored.value == instance.value
    )

    parameter_facade_cpp = {}
    if args.backend == "direct":
        import cppyy  # noqa: E402

        parameter_facade_cpp["get_parameter_value_is_cpp"] = isinstance(
            parameter_value, cppyy.gbl.rcl_interfaces.msg.ParameterValue)
        parameter_facade_cpp["to_parameter_msg_is_cpp"] = isinstance(
            message, cppyy.gbl.rcl_interfaces.msg.Parameter)

    # -- DirectNode parameter methods (11 rows) --
    rclpy.init()
    node = Node("prove_parameter_authority_%s_%d" % (args.backend, os.getpid()))
    behavior = {}
    try:
        node.declare_parameter("existing", 42)
        node.declare_parameter(
            "removable", "value", ParameterDescriptor(dynamic_typing=True))
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            node.declare_parameter("dynamic_unset")

        behavior["has_parameter_existing"] = node.has_parameter("existing")
        behavior["has_parameter_missing"] = node.has_parameter("never_declared")
        proven["rclpy.node.Node.has_parameter"] = True

        got = node.get_parameter("existing")
        behavior["get_parameter_existing"] = {"name": got.name, "value": got.value}
        proven["rclpy.node.Node.get_parameter"] = True

        or_declared = node.get_parameter_or("existing", 999)
        or_missing_default = node.get_parameter_or("never_declared")
        or_missing_alt = node.get_parameter_or("never_declared", 7)
        behavior["get_parameter_or"] = {
            "declared_value": or_declared.value,
            "missing_default_value": or_missing_default.value,
            "missing_alt_value": (
                or_missing_alt.value if hasattr(or_missing_alt, "value")
                else or_missing_alt
            ),
        }
        proven["rclpy.node.Node.get_parameter_or"] = True

        many = node.get_parameters(["existing", "removable"])
        behavior["get_parameters"] = [item.value for item in many]
        proven["rclpy.node.Node.get_parameters"] = True

        one_type = node.get_parameter_type("existing")
        behavior["get_parameter_type"] = int(one_type)
        proven["rclpy.node.Node.get_parameter_type"] = True

        many_types = node.get_parameter_types(["existing", "removable"])
        behavior["get_parameter_types"] = [int(item) for item in many_types]
        proven["rclpy.node.Node.get_parameter_types"] = True

        pre_events = []
        post_events = []

        def pre_callback(parameters):
            pre_events.append([parameter.name for parameter in parameters])
            return list(parameters)

        def post_callback(parameters):
            post_events.append([parameter.name for parameter in parameters])

        node.add_pre_set_parameters_callback(pre_callback)
        proven["rclpy.node.Node.add_pre_set_parameters_callback"] = True
        node.add_post_set_parameters_callback(post_callback)
        proven["rclpy.node.Node.add_post_set_parameters_callback"] = True

        # set_parameters is scaffolding only (its own row is missing_mismatch,
        # not claimed here); it exists to drive the pre/post callbacks above.
        node.set_parameters([canonical("existing", value=43)])
        behavior["callback_events"] = {"pre": pre_events, "post": post_events}
        behavior["get_parameter_after_set"] = node.get_parameter("existing").value

        node.remove_pre_set_parameters_callback(pre_callback)
        proven["rclpy.node.Node.remove_pre_set_parameters_callback"] = True
        node.remove_post_set_parameters_callback(post_callback)
        proven["rclpy.node.Node.remove_post_set_parameters_callback"] = True

        node.undeclare_parameter("removable")
        proven["rclpy.node.Node.undeclare_parameter"] = True
        behavior["has_parameter_after_undeclare"] = node.has_parameter("removable")

        try:
            node.get_parameter("removable")
        except ParameterNotDeclaredException:
            behavior["get_parameter_after_undeclare_raises"] = True
        else:
            behavior["get_parameter_after_undeclare_raises"] = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    assert boundary_calls == []
    for method in NODE_PARAMETER_METHODS:
        assert proven.get("rclpy.node.Node.%s" % method) is True, method

    report = {
        "backend": args.backend,
        "proven": proven,
        "behavior": behavior,
        "exercised_members": exercised_members,
        "alias_identity": alias_identity,
        "parameter_facade_cpp": parameter_facade_cpp,
        "forbidden_boundary_calls": len(boundary_calls),
    }
    print(PROBE_PREFIX + json.dumps(report, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
