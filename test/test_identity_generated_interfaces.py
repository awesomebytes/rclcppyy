"""Generated-interface C++ representation and public-alias proofs (Finding C).

The direct profile rebinds the 19 generated message interfaces to their C++
representation while leaving the one service and one action facade as the
stock Python classes. This is ``generated_interface_aliases`` -- a separate
ledger section, not ``entries`` -- so it is proven by test only and never
annotated (identity-proof plan §1/§3).
"""

from __future__ import annotations

import json
from pathlib import Path

from _run_helper import format_output, run_helper


PROBE_PREFIX = "RCLCPPYY_IDENTITY_GENERATED_INTERFACES_PROBE "
REPO_ROOT = Path(__file__).resolve().parent.parent
LEDGER_PATH = REPO_ROOT / "compatibility" / "rclpy-api-ledger-jazzy.json"

STOCK_FACADES = {
    "std_srvs/srv/SetBool",
    "tf2_msgs/action/LookupTransform",
}


def _probe(backend, interfaces=None):
    arguments = ["--backend", backend]
    if interfaces is not None:
        arguments += ["--interfaces", json.dumps(interfaces)]
    process = run_helper("_identity_generated_interfaces_probe.py", *arguments)
    assert process.returncode == 0, format_output(process)
    lines = [
        line for line in process.stdout.splitlines()
        if line.startswith(PROBE_PREFIX)
    ]
    assert len(lines) == 1, format_output(process)
    return json.loads(lines[0][len(PROBE_PREFIX):])


def _ledger_generated_interface_aliases():
    document = json.loads(LEDGER_PATH.read_text(encoding="utf-8"))
    return document["generated_interface_aliases"]


def test_direct_discovered_interfaces_match_the_committed_ledger_exactly():
    direct = _probe("direct")
    discovered = sorted(
        (row["interface"], row["kind"]) for row in direct["interfaces"]
    )
    ledger_rows = _ledger_generated_interface_aliases()
    pinned = sorted((row["interface"], row["kind"]) for row in ledger_rows)

    assert discovered == pinned
    assert len(pinned) == 21
    assert sum(1 for _, kind in pinned if kind == "message") == 19
    assert sum(1 for _, kind in pinned if kind == "service") == 1
    assert sum(1 for _, kind in pinned if kind == "action") == 1


def test_message_interfaces_resolve_to_cpp_representation_under_direct():
    direct = _probe("direct")
    ledger_public_type = {
        row["interface"]: row["public_type"]
        for row in _ledger_generated_interface_aliases()
    }

    for row in direct["interfaces"]:
        if row["kind"] != "message":
            continue
        assert row["has_cpp_name"] is True, row["interface"]
        assert row["module"].startswith("cppyy.gbl."), row["interface"]
        assert row["qualname"].endswith("_<std::allocator<void>>"), row["interface"]
        assert row["resolved_repr"] == ledger_public_type[row["interface"]]


def test_service_and_action_facades_stay_stock_python_under_direct():
    direct = _probe("direct")
    ledger_public_type = {
        row["interface"]: row["public_type"]
        for row in _ledger_generated_interface_aliases()
    }

    by_interface = {row["interface"]: row for row in direct["interfaces"]}
    for interface in STOCK_FACADES:
        row = by_interface[interface]
        assert row["has_cpp_name"] is False, interface
        assert not row["module"].startswith("cppyy."), interface
        assert row["resolved_repr"] == ledger_public_type[interface]


def test_all_21_interfaces_resolve_to_stock_python_under_stock_backend():
    direct = _probe("direct")
    requested = [
        {"interface": row["interface"], "kind": row["kind"]}
        for row in direct["interfaces"]
    ]
    stock = _probe("stock", interfaces=requested)

    assert len(stock["interfaces"]) == 21
    for row in stock["interfaces"]:
        assert row["has_cpp_name"] is False, row["interface"]
        assert not row["module"].startswith("cppyy."), row["interface"]
