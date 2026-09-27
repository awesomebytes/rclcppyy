"""Owner-level evidence for the twelve residual annotation decisions."""

from __future__ import annotations

import json

from _run_helper import format_output, run_helper


PROBE_PREFIX = "RCLCPPYY_STOCK_RESIDUAL_OWNER_PROBE "

DIRECT_NODE_PATHS = frozenset({
    "rclpy.action.graph.Node.PARAM_REL_TOL",
    "rclpy.lifecycle.LifecycleNode.PARAM_REL_TOL",
    "rclpy.lifecycle.Node.PARAM_REL_TOL",
    "rclpy.lifecycle.node.LifecycleNode.PARAM_REL_TOL",
    "rclpy.lifecycle.node.Node.PARAM_REL_TOL",
    "rclpy.node.Node.PARAM_REL_TOL",
    "rclpy.parameter_client.Node.PARAM_REL_TOL",
    "rclpy.parameter_event_handler.Node.PARAM_REL_TOL",
    "rclpy.wait_for_message.Node.PARAM_REL_TOL",
})
GOAL_HANDLE_PATHS = frozenset({
    "rclpy.action.client.ClientGoalHandle.__hash__",
    "rclpy.action.server.ServerGoalHandle.__hash__",
})
VALIDATOR_PATH = "rclpy.type_support.check_is_valid_msg_type"
ALL_PATHS = DIRECT_NODE_PATHS | GOAL_HANDLE_PATHS | {VALIDATOR_PATH}


def _probe(backend):
    process = run_helper(
        "_stock_residual_owner_probe.py", "--backend", backend, timeout=180)
    assert process.returncode == 0, format_output(process)
    lines = [
        line for line in process.stdout.splitlines()
        if line.startswith(PROBE_PREFIX)
    ]
    assert len(lines) == 1, format_output(process)
    return json.loads(lines[0][len(PROBE_PREFIX):])


def _reported_paths(report):
    return (
        set(report["node_members"])
        | set(report["goal_handle_members"])
        | {report["validator"]["path"]}
    )


def test_probe_path_set_is_exactly_the_residual_decision_tripwire():
    assert _reported_paths(_probe("stock")) == ALL_PATHS
    assert _reported_paths(_probe("direct")) == ALL_PATHS
    assert len(ALL_PATHS) == 12


def test_goal_handle_hash_members_are_the_exact_stock_none_singleton():
    stock = _probe("stock")
    direct = _probe("direct")

    assert set(direct["goal_handle_members"]) == GOAL_HANDLE_PATHS
    for path in GOAL_HANDLE_PATHS:
        assert stock["goal_handle_members"][path]["is_none"] is True
        assert direct["goal_handle_members"][path]["is_none"] is True
        assert direct["goal_handle_members"][path]["same_object_as_stock"] is True
        assert direct["goal_handle_members"][path]["owner"].startswith(
            "rclcppyy.direct_actions.Direct")


def test_direct_node_tolerance_is_equal_but_not_the_stock_object():
    stock = _probe("stock")
    direct = _probe("direct")

    assert set(direct["node_members"]) == DIRECT_NODE_PATHS
    for path in DIRECT_NODE_PATHS:
        assert stock["node_members"][path]["value"] == 1e-6
        assert direct["node_members"][path]["value"] == 1e-6
        assert direct["node_members"][path]["same_object_as_stock"] is False
        assert direct["node_members"][path]["owner"] == (
            "rclcppyy.direct_cpp.DirectNode")


def test_direct_validator_matches_stock_contract_and_behavior():
    stock = _probe("stock")["validator"]
    direct = _probe("direct")["validator"]

    assert stock["signature"] == direct["signature"] == "(msg_type)"
    assert stock["valid_result_is_none"] is True
    assert direct["valid_result_is_none"] is True
    assert stock["invalid"] == direct["invalid"]
    assert stock["valid_message_has_cpp_name"] is False
    assert direct["valid_message_has_cpp_name"] is True
    assert direct["same_object_as_stock"] is False
    assert direct["module"] == "rclcppyy.direct_cpp"
    assert direct["qualname"] == (
        "_prepare_check_is_valid_msg_type.<locals>.check_is_valid_msg_type")
