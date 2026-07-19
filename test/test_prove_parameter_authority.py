"""Authority coverage tripwire: direct-C++ local parameters (live, domain 78).

PLAN-prove-authority (uncommitted, docs/plans/) batch (a) local-parameter
slice. This is the honest arbiter of the promoted-row count for that slice:
only the paths this test proves reachable, C++-represented, and stock-parity
under the direct profile are promoted in the ledger annotations; everything
else -- including the structurally-mismatched declare/set/describe/list
mutation surface -- stays ``unassessed``.
"""

from __future__ import annotations

import json

from _run_helper import format_output, run_helper


PROBE_PREFIX = "RCLCPPYY_PROVE_PARAMETER_AUTHORITY_PROBE "

NODE_PARAMETER_METHODS = (
    "get_parameter", "get_parameter_or", "get_parameters", "get_parameter_type",
    "get_parameter_types", "has_parameter", "undeclare_parameter",
    "add_pre_set_parameters_callback", "add_post_set_parameters_callback",
    "remove_pre_set_parameters_callback", "remove_post_set_parameters_callback",
)
PARAMETER_FACADE_ALIASES = (
    "rclpy", "rclpy.parameter", "rclpy.node",
    "rclpy.parameter_client", "rclpy.parameter_event_handler",
)
PARAMETER_FACADE_MEMBERS = (
    "__init__", "from_parameter_msg", "get_parameter_value",
    "name", "to_parameter_msg", "type_", "value",
)

# The exact promoted-row set (46): 11 rclpy.node.Node parameter methods + 35
# Parameter-facade members (7 members x 5 module aliases). This literal set is
# what R1 may annotate for the local-parameter slice -- nothing more.
PROMOTED_PATHS = frozenset(
    ["rclpy.node.Node.%s" % method for method in NODE_PARAMETER_METHODS]
) | frozenset(
    "%s.Parameter.%s" % (alias, member)
    for alias in PARAMETER_FACADE_ALIASES
    for member in PARAMETER_FACADE_MEMBERS
)


def _probe(backend):
    process = run_helper("_prove_parameter_authority_probe.py", "--backend", backend, timeout=180)
    assert process.returncode == 0, format_output(process)
    lines = [
        line for line in process.stdout.splitlines()
        if line.startswith(PROBE_PREFIX)
    ]
    assert len(lines) == 1, format_output(process)
    return json.loads(lines[0][len(PROBE_PREFIX):])


def test_promoted_path_set_is_exactly_the_coverage_tripwire_manifest():
    direct = _probe("direct")
    assert set(direct["proven"]) == PROMOTED_PATHS
    assert all(direct["proven"].values())
    assert len(PROMOTED_PATHS) == 46


def test_parameter_facade_is_the_identical_class_at_every_public_alias():
    direct = _probe("direct")
    assert direct["alias_identity"] == dict.fromkeys(PARAMETER_FACADE_ALIASES, True)
    assert all(direct["exercised_members"].values())


def test_parameter_facade_payloads_are_generated_cpp_under_direct_only():
    direct = _probe("direct")
    assert direct["parameter_facade_cpp"] == {
        "get_parameter_value_is_cpp": True,
        "to_parameter_msg_is_cpp": True,
    }


def test_direct_node_parameter_methods_match_stock_behavior_with_no_boundary_crossings():
    stock = _probe("stock")
    direct = _probe("direct")

    assert stock["forbidden_boundary_calls"] == 0
    assert direct["forbidden_boundary_calls"] == 0
    assert direct["behavior"] == stock["behavior"]

    behavior = direct["behavior"]
    assert behavior["has_parameter_existing"] is True
    assert behavior["has_parameter_missing"] is False
    assert behavior["get_parameter_existing"] == {"name": "existing", "value": 42}
    assert behavior["get_parameter_or"]["declared_value"] == 42
    assert set(behavior["get_parameters"]) == {42, "value"}
    assert behavior["callback_events"]["pre"] == [["existing"]]
    assert behavior["callback_events"]["post"] == [["existing"]]
    assert behavior["get_parameter_after_set"] == 43
    assert behavior["has_parameter_after_undeclare"] is False
    assert behavior["get_parameter_after_undeclare_raises"] is True
