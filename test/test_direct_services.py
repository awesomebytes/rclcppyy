"""Unit coverage for the explicit direct-C++ service registry."""

import pytest

from rclcppyy import direct_services


def test_normalize_service_interfaces_is_canonical_and_deduplicated():
    assert direct_services.normalize_interfaces("std_srvs/srv/Trigger") == (
        "std_srvs/srv/Trigger",
    )
    assert direct_services.normalize_interfaces(
        ("std_srvs/srv/Trigger", "std_srvs/srv/Trigger")
    ) == ("std_srvs/srv/Trigger",)


@pytest.mark.parametrize(
    "interfaces",
    (
        ("std_srvs/msg/Trigger",),
        ("std_srvs/srv/trigger",),
        ("std_srvs/action/Trigger",),
        ("std_srvs/Trigger",),
    ),
)
def test_normalize_service_interfaces_rejects_noncanonical_names(interfaces):
    with pytest.raises(ValueError, match="package/srv/Service"):
        direct_services.normalize_interfaces(interfaces)


@pytest.mark.parametrize("interfaces", ((None,), (1,), ("",)))
def test_normalize_service_interfaces_rejects_non_strings(interfaces):
    with pytest.raises(TypeError, match="non-empty strings"):
        direct_services.normalize_interfaces(interfaces)


def test_registered_interface_registry_accepts_only_messages_and_services():
    assert direct_services.normalize_registered_interfaces(
        (
            "std_srvs/srv/Trigger",
            "std_msgs/msg/Header",
            "std_srvs/srv/Trigger",
        )
    ) == ("std_msgs/msg/Header", "std_srvs/srv/Trigger")
    with pytest.raises(ValueError, match="package/msg/Message or package/srv/Service"):
        direct_services.normalize_registered_interfaces(
            ("tf2_msgs/action/LookupTransform",))


def test_nested_service_message_dependencies_are_canonical():
    class Request:
        @staticmethod
        def get_fields_and_field_types():
            return {
                "header": "std_msgs/Header",
                "poses": "sequence<geometry_msgs/msg/Pose, 8>",
                "value": "double",
            }

    assert direct_services._dependencies(Request) == (
        "geometry_msgs/msg/Pose",
        "std_msgs/msg/Header",
    )


def test_installation_preserves_default_setbool_binding_accessor():
    class Service:
        pass

    class Request:
        pass

    class Response:
        pass

    bindings = tuple(
        direct_services.DirectServiceBinding(
            interface=interface,
            service_type=Service,
            original_request_type=Request,
            original_response_type=Response,
            cpp_request_type=Request,
            cpp_response_type=Response,
            cpp_type_name=interface.replace("/srv/", "::srv::"),
            header="unused.hpp",
            request_fields=(),
            response_fields=(),
        )
        for interface in ("std_srvs/srv/Trigger", "std_srvs/srv/SetBool")
    )
    installation = direct_services.DirectServiceInstallation((), bindings)
    assert installation.bindings == bindings
    assert installation.binding.interface == "std_srvs/srv/SetBool"
