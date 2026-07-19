"""Authority coverage tripwire: direct-C++ actions (live, domain 78).

PLAN-prove-authority (uncommitted, docs/plans/) batch (a) action slice. This
is the honest arbiter of the promoted-row count for that slice: only the
paths this test proves reachable, stock-peer-interoperable, and C++-valued
under the direct profile are promoted; the synchronous
``send_goal``/``get_result``/``cancel_goal`` variants are proven to
fail-closed and are excluded, not merely assumed unsupported.
"""

from __future__ import annotations

import json

from _run_helper import format_output, run_helper


PROBE_PREFIX = "RCLCPPYY_PROVE_ACTION_AUTHORITY_PROBE "

ACTION_CLIENT_PROMOTED = (
    "destroy", "send_goal_async", "server_is_ready", "wait_for_server")
CLIENT_GOAL_HANDLE_PROMOTED = (
    "__eq__", "__ne__", "__repr__", "accepted", "cancel_goal_async",
    "get_result_async", "goal_id", "stamp", "status")
ACTION_SERVER_PROMOTED = (
    "action_type", "destroy", "notify_execute", "notify_goal_done",
    "register_cancel_callback", "register_execute_callback",
    "register_goal_callback", "register_handle_accepted_callback")
SERVER_GOAL_HANDLE_PROMOTED = (
    "__eq__", "__ne__", "abort", "canceled", "destroy", "execute", "executing",
    "goal_id", "is_active", "is_cancel_requested", "publish_feedback",
    "request", "status", "succeed")

# The exact promoted-row set (47): ActionClient (4 members x 2 module
# aliases) + ClientGoalHandle (9) + ActionServer (8 members x 2 module
# aliases) + ServerGoalHandle (14).
PROMOTED_PATHS = frozenset(
    "%s.%s" % (module, name)
    for module in ("rclpy.action.ActionClient", "rclpy.action.client.ActionClient")
    for name in ACTION_CLIENT_PROMOTED
) | frozenset(
    "rclpy.action.client.ClientGoalHandle.%s" % name
    for name in CLIENT_GOAL_HANDLE_PROMOTED
) | frozenset(
    "%s.%s" % (module, name)
    for module in ("rclpy.action.ActionServer", "rclpy.action.server.ActionServer")
    for name in ACTION_SERVER_PROMOTED
) | frozenset(
    "rclpy.action.server.ServerGoalHandle.%s" % name
    for name in SERVER_GOAL_HANDLE_PROMOTED
)

# Proven to fail-closed unconditionally -- excluded from the promoted set.
DECLINED_FAIL_CLOSED = frozenset({"send_goal", "get_result", "cancel_goal"})


def _probe():
    process = run_helper("_prove_action_authority_probe.py", timeout=360)
    assert process.returncode == 0, format_output(process)
    lines = [
        line for line in process.stdout.splitlines()
        if line.startswith(PROBE_PREFIX)
    ]
    assert len(lines) == 1, format_output(process)
    return json.loads(lines[0][len(PROBE_PREFIX):])


def test_promoted_path_set_is_exactly_the_coverage_tripwire_manifest():
    report = _probe()
    assert set(report["proven"]) == PROMOTED_PATHS
    assert all(report["proven"].values())
    assert len(PROMOTED_PATHS) == 47


def test_declined_synchronous_variants_are_proven_fail_closed_not_assumed():
    report = _probe()
    assert set(report["fail_closed"]) == DECLINED_FAIL_CLOSED
    assert all(report["fail_closed"].values())
