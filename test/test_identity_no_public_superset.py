"""Direct public surface is not a superset of stock (live, ROS_DOMAIN_ID 68).

Finding D (identity-proof plan §0/§2 file 5): six leaked public names made
the direct public surface a superset of stock. Scope is exactly node +
publisher, per plan -- subscription (and action/executor/callback-group)
classes carry a backend superset already tracked by the ledger's own
summary.superset_violations counter, and are deliberately out of scope
here; touching them is not this test's job.

The live check covers a plain Node/Publisher (what can actually be
constructed under the direct profile today -- see the Phase A report for
why LifecycleNode cannot be); the static check covers the two classes
Finding D originally named, at the class level, without instantiation.
"""

from __future__ import annotations

import json

from _run_helper import format_output, run_helper


PROBE_PREFIX = "RCLCPPYY_IDENTITY_NO_PUBLIC_SUPERSET_PROBE "


def _probe(backend):
    process = run_helper("_identity_no_public_superset_probe.py", "--backend", backend)
    assert process.returncode == 0, format_output(process)
    lines = [
        line for line in process.stdout.splitlines()
        if line.startswith(PROBE_PREFIX)
    ]
    assert len(lines) == 1, format_output(process)
    return json.loads(lines[0][len(PROBE_PREFIX):])


def _assert_no_superset(stock_names, direct_names, label):
    leaked = sorted(set(direct_names) - set(stock_names))
    assert not leaked, f"{label} leaks public names direct-only: {leaked}"


def test_live_node_and_publisher_are_not_a_superset_of_stock():
    stock = _probe("stock")
    direct = _probe("direct")

    for name in ("Node", "Publisher"):
        _assert_no_superset(
            stock["live_types"][name], direct["live_types"][name], f"live {name}")


def test_lifecycle_node_and_publisher_classes_are_not_a_superset_of_stock():
    stock = _probe("stock")
    direct = _probe("direct")

    for name in ("LifecycleNode", "LifecyclePublisher"):
        _assert_no_superset(
            stock["static_types"][name], direct["static_types"][name], name)


def test_no_leaked_name_appears_anywhere_in_the_probed_surface():
    direct = _probe("direct")
    leaked_names = (
        "direct_cpp_parameter_cache_stats", "native_entity", "closed",
        "action_clients", "action_servers", "callback_groups",
    )
    all_direct_names = set()
    for names in direct["live_types"].values():
        all_direct_names.update(names)
    for names in direct["static_types"].values():
        all_direct_names.update(names)

    stock = _probe("stock")
    all_stock_names = set()
    for names in stock["live_types"].values():
        all_stock_names.update(names)
    for names in stock["static_types"].values():
        all_stock_names.update(names)

    leaked_and_present = sorted(
        name for name in leaked_names
        if name in all_direct_names and name not in all_stock_names
    )
    assert not leaked_and_present
