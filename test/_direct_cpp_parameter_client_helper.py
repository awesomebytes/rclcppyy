#!/usr/bin/env python3
"""Live unchanged AsyncParameterClient proof over direct C++ entities."""

import importlib
import inspect
import os
import threading
import time
import warnings

import rclcppyy


PARAMETER_INTERFACES = (
    "rcl_interfaces/msg/ParameterEvent",
    "rcl_interfaces/srv/DescribeParameters",
    "rcl_interfaces/srv/GetParameters",
    "rcl_interfaces/srv/GetParameterTypes",
    "rcl_interfaces/srv/ListParameters",
    "rcl_interfaces/srv/SetParameters",
    "rcl_interfaces/srv/SetParametersAtomically",
)
rclcppyy.enable_cpp_acceleration(
    profile="direct_cpp", interfaces=PARAMETER_INTERFACES)

import cppyy  # noqa: E402
import rclpy  # noqa: E402
from rcl_interfaces.msg import (  # noqa: E402
    ListParametersResult,
    Parameter as ParameterMsg,
    ParameterDescriptor,
    ParameterEvent,
    ParameterValue,
    SetParametersResult,
)
from rcl_interfaces.srv import (  # noqa: E402
    DescribeParameters,
    GetParameters,
    GetParameterTypes,
    ListParameters,
    SetParameters,
    SetParametersAtomically,
)
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.exceptions import ParameterNotDeclaredException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.parameter_client import AsyncParameterClient  # noqa: E402
from rclpy.task import Future  # noqa: E402


SERVICE_TYPES = (
    DescribeParameters,
    GetParameters,
    GetParameterTypes,
    ListParameters,
    SetParameters,
    SetParametersAtomically,
)
assert AsyncParameterClient.__module__ == "rclpy.parameter_client"
source_file = inspect.getsourcefile(AsyncParameterClient)
assert source_file is not None and source_file.endswith("rclpy/parameter_client.py")
assert "class AsyncParameterClient:" in inspect.getsource(AsyncParameterClient)
assert ParameterMsg is cppyy.gbl.rcl_interfaces.msg.Parameter
assert ParameterValue is cppyy.gbl.rcl_interfaces.msg.ParameterValue
assert ParameterEvent is cppyy.gbl.rcl_interfaces.msg.ParameterEvent
assert ParameterDescriptor is cppyy.gbl.rcl_interfaces.msg.ParameterDescriptor
assert ListParametersResult is cppyy.gbl.rcl_interfaces.msg.ListParametersResult
assert SetParametersResult is cppyy.gbl.rcl_interfaces.msg.SetParametersResult
for service_type in SERVICE_TYPES:
    cpp_service = getattr(
        cppyy.gbl.rcl_interfaces.srv, service_type.__name__)
    assert service_type.Request is cpp_service.Request
    assert service_type.Response is cpp_service.Response
print("DIRECT_CPP_PARAMETER_CLIENT_STOCK_SOURCE_CPP_ALIASES_OK")


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError(
        "conversion, serialization, or CDR entered parameter-client proof")


kit = importlib.import_module("rclcpp_kit")
bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
serialization = importlib.import_module("rclcpp_kit.serialization")
rclpy_serialization = importlib.import_module("rclpy.serialization")
kit.convert_python_msg_to_cpp = forbidden_boundary
bringup.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary
serialization.serialized_message_from_bytes = forbidden_boundary
serialization.serialized_message_to_bytes = forbidden_boundary
rclpy_serialization.serialize_message = forbidden_boundary
rclpy_serialization.deserialize_message = forbidden_boundary


def spin_until(executor, predicate, timeout=10.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    assert predicate()


def uint8_value(value):
    if isinstance(value, str):
        return ord(value)
    if isinstance(value, bytes):
        return value[0]
    return int(value)


def parameter_integer(parameter_value):
    assert type(parameter_value) is ParameterValue
    assert uint8_value(parameter_value.type) == Parameter.Type.INTEGER.value
    return int(parameter_value.integer_value)


rclpy.init(args=[])
suffix = str(os.getpid())
server = Node("direct_parameter_server_" + suffix)
client_node = Node(
    "direct_parameter_client_" + suffix,
    start_parameter_services=False,
    enable_rosout=False,
)
server.declare_parameter("stable", 1)
server.declare_parameter("atomic", 2)
with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    server.declare_parameter("dynamic")
server._set_direct_parameter_cache_hit_tracking(True)
retained_stable_one = server.get_parameter("stable")
retained_dynamic_none = server.get_parameter("dynamic")

post_observations = []


def observe_post_cache(parameters):
    for parameter in parameters:
        assert isinstance(
            parameter._rclcppyy_native_parameter.native,
            cppyy.gbl.rclcpp.Parameter,
        )
        try:
            current = server.get_parameter(parameter.name)
        except ParameterNotDeclaredException:
            current = None
        post_observations.append({
            "name": parameter.name,
            "callback_value": parameter.value,
            "cache_value": None if current is None else current.value,
            "same_facade": current is parameter,
            "missing": current is None,
        })


server.add_post_set_parameters_callback(observe_post_cache)

events = []


def on_parameter_event(event):
    assert type(event) is ParameterEvent
    if str(event.node) == server.get_fully_qualified_name():
        events.append(event)


client = AsyncParameterClient(
    client_node, server.get_fully_qualified_name())
event_subscription = client.on_parameter_event(on_parameter_event)
client_attributes = (
    "_get_parameter_client",
    "_list_parameter_client",
    "_set_parameter_client",
    "_get_parameter_types_client",
    "_describe_parameters_client",
    "_set_parameters_atomically_client",
)
assert len(client_node.clients) == 6
assert len(list(client_node.subscriptions)) == 1
assert client.wait_for_services(timeout_sec=5.0)
assert client.services_are_ready()
assert all(
    getattr(client, name).srv_type in SERVICE_TYPES
    for name in client_attributes
)
print("DIRECT_CPP_PARAMETER_CLIENT_READINESS_OK")

executor = SingleThreadedExecutor(context=server.context)
assert executor.add_node(server)
assert executor.add_node(client_node)

dynamic_message = ParameterMsg(
    name="dynamic",
    value=ParameterValue(
        type=Parameter.Type.INTEGER.value,
        integer_value=42,
    ),
)
set_done = []
set_future = client.set_parameters(
    [Parameter("stable", value=11), dynamic_message],
    callback=set_done.append,
)
assert type(set_future) is Future
executor.spin_until_future_complete(set_future, timeout_sec=10.0)
spin_until(
    executor,
    lambda: any(
        any(
            str(value.name) == "dynamic"
            for value in tuple(event.new_parameters) + tuple(event.changed_parameters)
        )
        for event in events
    ),
)
assert set_done == [set_future]
set_response = set_future.result()
assert type(set_response) is SetParameters.Response
assert len(set_response.results) == 2
assert all(
    type(result) is SetParametersResult and result.successful
    for result in set_response.results
)
assert server.get_parameter("stable").value == 11
assert server.get_parameter("dynamic").value == 42
assert retained_stable_one.value == 1
assert retained_dynamic_none.value is None
assert post_observations[-2:] == [
    {
        "name": "stable", "callback_value": 11, "cache_value": 11,
        "same_facade": True, "missing": False,
    },
    {
        "name": "dynamic", "callback_value": 42, "cache_value": 42,
        "same_facade": True, "missing": False,
    },
]

get_done = []
get_future = client.get_parameters(
    ["stable", "dynamic"], callback=get_done.append)
executor.spin_until_future_complete(get_future, timeout_sec=10.0)
get_response = get_future.result()
assert type(get_response) is GetParameters.Response
assert get_done == [get_future]
assert [parameter_integer(value) for value in get_response.values] == [11, 42]

types_future = client.get_parameter_types(["stable", "dynamic"])
executor.spin_until_future_complete(types_future, timeout_sec=10.0)
types_response = types_future.result()
assert type(types_response) is GetParameterTypes.Response
assert [uint8_value(value) for value in types_response.types] == [
    Parameter.Type.INTEGER.value,
    Parameter.Type.INTEGER.value,
]

describe_future = client.describe_parameters(["stable", "dynamic"])
executor.spin_until_future_complete(describe_future, timeout_sec=10.0)
describe_response = describe_future.result()
assert type(describe_response) is DescribeParameters.Response
assert all(
    type(descriptor) is ParameterDescriptor
    for descriptor in describe_response.descriptors
)
assert [str(value.name) for value in describe_response.descriptors] == [
    "stable", "dynamic"]

list_future = client.list_parameters(prefixes=[], depth=10)
executor.spin_until_future_complete(list_future, timeout_sec=10.0)
list_response = list_future.result()
assert type(list_response) is ListParameters.Response
assert type(list_response.result) is ListParametersResult
listed_names = {str(name) for name in list_response.result.names}
assert {"stable", "atomic", "dynamic"} <= listed_names
print("DIRECT_CPP_PARAMETER_CLIENT_SET_GET_TYPES_DESCRIBE_LIST_OK")

retained_stable_eleven = server.get_parameter("stable")
atomic_future = client.set_parameters_atomically([
    Parameter("stable", value=21),
    Parameter("atomic", value=22),
])
executor.spin_until_future_complete(atomic_future, timeout_sec=10.0)
atomic_response = atomic_future.result()
assert type(atomic_response) is SetParametersAtomically.Response
assert type(atomic_response.result) is SetParametersResult
assert atomic_response.result.successful
assert server.get_parameter("stable").value == 21
assert server.get_parameter("atomic").value == 22
assert retained_stable_eleven.value == 11
assert post_observations[-2:] == [
    {
        "name": "stable", "callback_value": 21, "cache_value": 21,
        "same_facade": True, "missing": False,
    },
    {
        "name": "atomic", "callback_value": 22, "cache_value": 22,
        "same_facade": True, "missing": False,
    },
]

retained_dynamic_42 = server.get_parameter("dynamic")
delete_future = client.delete_parameters(["dynamic"])
executor.spin_until_future_complete(delete_future, timeout_sec=10.0)
spin_until(
    executor,
    lambda: any(
        any(str(value.name) == "dynamic" for value in event.deleted_parameters)
        for event in events
    ),
)
delete_response = delete_future.result()
assert type(delete_response) is SetParameters.Response
assert len(delete_response.results) == 1
assert type(delete_response.results[0]) is SetParametersResult
assert delete_response.results[0].successful
assert not server.has_parameter("dynamic")
try:
    server.get_parameter("dynamic")
except ParameterNotDeclaredException:
    pass
else:
    raise AssertionError("remote deletion left a stale parameter cache entry")
assert retained_dynamic_42.value == 42
assert post_observations[-1] == {
    "name": "dynamic", "callback_value": None, "cache_value": None,
    "same_facade": False, "missing": True,
}
cache_stats = server.direct_cpp_parameter_cache_stats()
assert cache_stats["enabled"] is True
assert cache_stats["hits"] > 0
assert cache_stats["invalidations"] >= 1
assert cache_stats["size"] <= cache_stats["capacity"]
assert cache_stats["pending_invalidations"] == 0
print("DIRECT_CPP_PARAMETER_CLIENT_ATOMIC_DELETE_EVENTS_OK")

server.declare_parameter("race", 0)
retained_race_zero = server.get_parameter("race")
spin_stop = threading.Event()
spin_failures = []


def spin_in_thread():
    try:
        while not spin_stop.is_set():
            executor.spin_once(timeout_sec=0.01)
    except BaseException as exception:
        spin_failures.append(exception)


spin_thread = threading.Thread(
    target=spin_in_thread, name="direct-parameter-cache-spin")
spin_thread.start()
try:
    for value in range(1, 21):
        future = client.set_parameters_atomically([
            Parameter("race", value=value)])
        deadline = time.monotonic() + 5.0
        observed = []
        while not future.done() and time.monotonic() < deadline:
            observed.append(server.get_parameter("race").value)
            time.sleep(0.0005)
        assert future.done()
        assert future.result().result.successful
        assert set(observed) <= {value - 1, value}
        assert server.get_parameter("race").value == value
finally:
    spin_stop.set()
    spin_thread.join(timeout=5.0)
assert not spin_thread.is_alive()
assert spin_failures == []
assert retained_race_zero.value == 0
assert server.direct_cpp_parameter_cache_stats()["pending_invalidations"] == 0
print("DIRECT_CPP_PARAMETER_CLIENT_CONCURRENT_CACHE_OK")

retained_response = get_response
retained_event = events[-1]
executor.remove_node(client_node)
executor.remove_node(server)
assert client_node.destroy_subscription(event_subscription)
for attribute in client_attributes:
    assert client_node.destroy_client(getattr(client, attribute))
assert client_node.clients == []
assert list(client_node.subscriptions) == []
executor.shutdown()
server.remove_post_set_parameters_callback(observe_post_cache)
client_node.destroy_node()
server.destroy_node()
assert parameter_integer(retained_response.values[1]) == 42
assert type(retained_event) is ParameterEvent
rclpy.shutdown()

direct_module = importlib.import_module("rclcppyy.direct_cpp")
runtime = direct_module._runtime()
assert runtime.nodes == []
assert runtime.session is None
print("DIRECT_CPP_PARAMETER_CLIENT_NO_CONVERSION_TEARDOWN_OK")
