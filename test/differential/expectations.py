"""Explicit parity assertions and current compatibility exclusions."""

from dataclasses import dataclass


@dataclass(frozen=True)
class ParityExpectation:
    identifier: str
    path: tuple
    reason: str


@dataclass(frozen=True)
class BackendGap:
    identifier: str
    kind: str
    backend: str
    metadata: tuple
    reason: str


CERTIFIED_PARITY = (
    ParityExpectation(
        "node.facade_type",
        ("identity", "facade", "type"),
        "Activation retains the stock public Node type.",
    ),
    ParityExpectation(
        "node.facade_name",
        ("identity", "facade", "name"),
        "The public node name matches stock behavior.",
    ),
    ParityExpectation(
        "node.facade_namespace",
        ("identity", "facade", "namespace"),
        "The public node namespace matches stock behavior.",
    ),
    ParityExpectation(
        "node.single_graph_identity",
        ("identity", "graph_nodes"),
        "Activated mode retains the same single graph identity.",
    ),
    ParityExpectation(
        "context.requested_context_authority",
        ("context", "node_uses_requested_context"),
        "The node retains the caller-provided Context object.",
    ),
    ParityExpectation(
        "context.no_default_context_side_effect",
        ("context", "default_context_ok"),
        "An explicit context does not initialize the default context.",
    ),
    ParityExpectation(
        "remapping.facade_name",
        ("remapping", "facade", "name"),
        "Node-name remapping is reflected by the public facade.",
    ),
    ParityExpectation(
        "remapping.facade_namespace",
        ("remapping", "facade", "namespace"),
        "Namespace remapping is reflected by the public facade.",
    ),
    ParityExpectation(
        "remapping.single_graph_identity",
        ("remapping", "graph_nodes"),
        "Remapping does not create an additional graph node.",
    ),
    ParityExpectation(
        "parameters.local_value",
        ("parameters", "value"),
        "Local parameter overrides remain readable from the facade.",
    ),
    ParityExpectation(
        "parameters.single_service_owner",
        ("parameters", "parameter_service_owners"),
        "Parameter services retain a single authoritative owner.",
    ),
    ParityExpectation(
        "publisher.stock_object_type",
        ("entity_routing", "publisher", "type"),
        "The accelerated publisher retains the stock public object type.",
    ),
    ParityExpectation(
        "publisher.topic_name",
        ("entity_routing", "publisher", "topic_name"),
        "Publisher topic resolution remains unchanged.",
    ),
    ParityExpectation(
        "subscription.stock_object_type",
        ("entity_routing", "subscription", "type"),
        "Fallback subscription dispatch retains the stock public object type.",
    ),
    ParityExpectation(
        "subscription.topic_name",
        ("entity_routing", "subscription", "topic_name"),
        "Subscription topic resolution remains unchanged.",
    ),
    ParityExpectation(
        "publisher.subscription_roundtrip",
        ("entity_routing", "received"),
        "C++ publishing reaches the stock subscription with the same payload.",
    ),
    ParityExpectation(
        "teardown.public_node_removed",
        ("teardown", "public_node_present_after_destroy"),
        "destroy_node removes the public Python graph node.",
    ),
    ParityExpectation(
        "teardown.no_sidecar_after_destroy",
        ("teardown", "graph_nodes_after_destroy"),
        "destroy_node leaves no hidden graph node behind.",
    ),
)


EXPECTED_BACKEND_GAPS = (
    BackendGap(
        "subscription.same_handle_cpp",
        "entities",
        "cpp",
        (("entity_type", "subscription"),),
        "Subscription take and dispatch do not yet have a certified same-handle C++ route.",
    ),
)


def value_at_path(observations, path):
    value = observations
    for component in path:
        value = value[component]
    return value
