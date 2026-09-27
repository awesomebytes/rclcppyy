"""Direct entity type identity + isinstance proofs (live, ROS_DOMAIN_ID 68).

Proves "entity identity" behaviorally for the Identity gate: whatever
concrete implementation class the direct profile hands back from
``create_publisher``/``create_subscription``, ``isinstance`` against the
currently-exposed public ``rclpy.node.Node`` / ``rclpy.publisher.Publisher``
/ ``rclpy.subscription.Subscription`` holds, and payload objects carry the
generated C++ representation (Finding C). No annotation attaches here --
entity/node rows belong to the node/lifecycle lanes and the allocation plan
§1.2 stock stand-in (identity-proof plan §2 file 4).
"""

from __future__ import annotations

import json

from _run_helper import format_output, run_helper


PROBE_PREFIX = "RCLCPPYY_IDENTITY_ENTITIES_PROBE "


def _probe(backend):
    process = run_helper("_identity_entities_probe.py", "--backend", backend)
    assert process.returncode == 0, format_output(process)
    lines = [
        line for line in process.stdout.splitlines()
        if line.startswith(PROBE_PREFIX)
    ]
    assert len(lines) == 1, format_output(process)
    return json.loads(lines[0][len(PROBE_PREFIX):])


def test_isinstance_holds_against_public_aliases_under_both_backends():
    for backend in ("stock", "direct"):
        payload = _probe(backend)
        assert payload["isinstance"] == {
            "node_is_Node": True,
            "publisher_is_Publisher": True,
            "subscription_is_Subscription": True,
        }, backend


TYPE_TO_ALIAS_NAME = {
    "node": "Node", "publisher": "Publisher", "subscription": "Subscription",
}


def test_direct_entity_types_are_pure_python_control_plane():
    direct = _probe("direct")

    for descriptor in direct["types"].values():
        assert descriptor["module"] == "rclcppyy.direct_cpp"
        assert descriptor["has_cpp_name"] is False
    # The public aliases (rclpy.node.Node etc.) are exactly what create_*
    # returns under direct -- the module attribute itself is replaced, not a
    # separate subclass sitting alongside the stock class.
    for type_key, alias_name in TYPE_TO_ALIAS_NAME.items():
        assert direct["types"][type_key] == direct["public_aliases"][alias_name]


def test_stock_entity_types_are_the_stock_rclpy_classes():
    stock = _probe("stock")

    assert stock["types"]["node"] == {
        "module": "rclpy.node", "qualname": "Node", "has_cpp_name": False,
    }
    assert stock["types"]["publisher"] == {
        "module": "rclpy.publisher", "qualname": "Publisher", "has_cpp_name": False,
    }
    assert stock["types"]["subscription"] == {
        "module": "rclpy.subscription", "qualname": "Subscription",
        "has_cpp_name": False,
    }
    for type_key, alias_name in TYPE_TO_ALIAS_NAME.items():
        assert stock["types"][type_key] == stock["public_aliases"][alias_name]


def test_payload_objects_carry_cpp_representation_only_under_direct():
    stock = _probe("stock")
    direct = _probe("direct")

    assert stock["constructed_payload_type"]["has_cpp_name"] is False
    assert stock["received_payload_type"]["has_cpp_name"] is False
    assert direct["constructed_payload_type"]["has_cpp_name"] is True
    assert direct["received_payload_type"]["has_cpp_name"] is True
    assert direct["constructed_payload_type"]["module"] == "cppyy.gbl.std_msgs.msg"
    assert direct["received_payload_type"] == direct["constructed_payload_type"]
    assert stock["received_value"] == direct["received_value"] == 97
