"""Transactional generated-C++ action bindings for ``direct_cpp``."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import inspect
import math
import re
import sys
import threading
from typing import Any

from rclcppyy._status import record_decision
from rclcppyy._surface import _DirectSurface
from rclcppyy.policy import BackendUnavailableError


DEFAULT_INTERFACES = ("tf2_msgs/action/LookupTransform",)
_INTERFACE_NAME = re.compile(
    r"^([A-Za-z][A-Za-z0-9_]*)/action/([A-Z][A-Za-z0-9_]*)$")
_FIELD_INTERFACE = re.compile(
    r"(?<![A-Za-z0-9_])([A-Za-z][A-Za-z0-9_]*)/(?:msg/)?"
    r"([A-Z][A-Za-z0-9_]*)")
_IMPLEMENTATION_MODULE = re.compile(
    r"^[A-Za-z][A-Za-z0-9_]*\.action\._[A-Za-z0-9_]+$")
_PYTHONIZED: dict[Any, Any] = {}
_ACTIVE_INSTALLATION = None
_DEFAULT_QOS = object()
_FEEDBACK_CAPACITY = 1024
_STATUS_UNKNOWN = 0
_STATUS_EXECUTING = 2
_TERMINAL_STATUS = {
    "succeed": 4,
    "canceled": 5,
    "abort": 6,
}


def _unsupported(reason: str):
    raise BackendUnavailableError(reason)


def _byte_values(values) -> bytes:
    return bytes(
        ord(value) if isinstance(value, str) else int(value)
        for value in values
    )


def _int8_value(value) -> int:
    return ord(value) if isinstance(value, str) else int(value)


def _default_goal_callback(_goal):
    from rclpy.action import GoalResponse

    return GoalResponse.ACCEPT


def _default_cancel_callback(_goal_handle):
    from rclpy.action import CancelResponse

    return CancelResponse.REJECT


def _default_handle_accepted_callback(goal_handle):
    goal_handle.execute()


def _validate_sync_callback(name, callback):
    if not callable(callback):
        raise TypeError("%s must be callable" % name)
    target = getattr(callback, "__call__", callback)
    if inspect.iscoroutinefunction(callback) or inspect.iscoroutinefunction(target):
        _unsupported("direct_cpp action servers require synchronous %s" % name)


def _sync_result(name, callback, argument):
    result = callback(argument)
    if inspect.isawaitable(result):
        close = getattr(result, "close", None)
        if close is not None:
            close()
        raise TypeError("%s must complete synchronously" % name)
    return result


@dataclass(frozen=True)
class DirectActionBinding:
    interface: str
    action_type: type
    cpp_types: Any
    original_goal_type: type
    original_feedback_type: type
    original_result_type: type
    header: str


@dataclass(frozen=True)
class DirectActionPlan:
    bindings: tuple[DirectActionBinding, ...]
    targets: tuple[tuple[Any, str, Any, Any], ...]
    constructors: tuple[tuple[Any, type], ...]
    message_dependencies: tuple[str, ...]


class DirectActionInstallation:
    def __init__(self, replacements, bindings, pythonizations=()):
        self._replacements = tuple(replacements)
        self._pythonizations = tuple(pythonizations)
        self.bindings = tuple(bindings)
        self.binding = next(
            binding for binding in self.bindings
            if binding.interface == DEFAULT_INTERFACES[0]
        )
        self._restored = False

    def restore(self):
        global _ACTIVE_INSTALLATION
        if self._restored:
            return
        for owner, name, original, replacement in reversed(self._replacements):
            if getattr(owner, name, None) is replacement:
                setattr(owner, name, original)
        for cpp_type, original_init, direct_init in reversed(self._pythonizations):
            if cpp_type.__init__ is direct_init:
                cpp_type.__init__ = original_init
                _PYTHONIZED.pop(cpp_type, None)
        if _ACTIVE_INSTALLATION is self:
            _ACTIVE_INSTALLATION = None
        self._restored = True


def normalize_interfaces(interfaces=()) -> tuple[str, ...]:
    values = (interfaces,) if isinstance(interfaces, str) else interfaces
    try:
        selected = tuple(values)
    except TypeError as exc:
        raise TypeError("interfaces must be an iterable of canonical names") from exc
    if any(not isinstance(value, str) or not value for value in selected):
        raise TypeError("interfaces must contain only non-empty strings")
    invalid = sorted(
        value for value in selected if _INTERFACE_NAME.fullmatch(value) is None)
    if invalid:
        raise ValueError(
            "direct_cpp action interfaces must use package/action/Action names: %s" %
            ", ".join(invalid)
        )
    return tuple(sorted(set(selected)))


def normalize_registered_interfaces(interfaces=()) -> tuple[str, ...]:
    """Validate the public registry across direct messages, services, and actions."""
    values = (interfaces,) if isinstance(interfaces, str) else interfaces
    try:
        selected = tuple(values)
    except TypeError as exc:
        raise TypeError("interfaces must be an iterable of canonical names") from exc
    if any(not isinstance(value, str) or not value for value in selected):
        raise TypeError("interfaces must contain only non-empty strings")

    messages = tuple(value for value in selected if "/msg/" in value)
    services = tuple(value for value in selected if "/srv/" in value)
    actions = tuple(value for value in selected if "/action/" in value)
    known = set(messages) | set(services) | set(actions)
    unknown = sorted(set(selected) - known)
    if unknown:
        raise ValueError(
            "direct_cpp interfaces must use package/msg/Message, "
            "package/srv/Service, or package/action/Action names: %s" %
            ", ".join(unknown)
        )
    from rclcppyy.direct_messages import normalize_interfaces as normalize_messages
    from rclcppyy.direct_services import normalize_interfaces as normalize_services

    normalized = (
        normalize_messages(messages)
        + normalize_services(services)
        + normalize_interfaces(actions)
    )
    return tuple(sorted(set(normalized)))


def assert_early_imports() -> None:
    stale = sorted(name for name in sys.modules if _IMPLEMENTATION_MODULE.fullmatch(name))
    if "rclpy.action" in sys.modules:
        stale.append("rclpy.action")
    if stale:
        raise RuntimeError(
            "direct_cpp must be enabled before importing generated actions: %s" %
            ", ".join(stale)
        )


def _pythonize_constructor(cpp_type: Any, original_type: type):
    if cpp_type in _PYTHONIZED:
        return None
    original_init = cpp_type.__init__
    allowed = frozenset(original_type.get_fields_and_field_types())

    def direct_init(self, *args, **kwargs):
        if args and kwargs:
            raise TypeError(
                "C++ action message constructors cannot mix positional and keyword values")
        if args:
            original_init(self, *args)
            return
        original_init(self)
        unknown = sorted(set(kwargs) - allowed)
        if unknown:
            raise TypeError(
                "unknown action message constructor fields: %s" % ", ".join(unknown))
        for name, value in kwargs.items():
            setattr(self, name, value)

    cpp_type.__init__ = direct_init
    _PYTHONIZED[cpp_type] = original_init
    return cpp_type, original_init, direct_init


def _dependencies(message_type: type, excluded=()) -> tuple[str, ...]:
    excluded_set = set(excluded)
    dependencies = set()
    for field_type in message_type.get_fields_and_field_types().values():
        for package, name in _FIELD_INTERFACE.findall(field_type):
            interface = "%s/msg/%s" % (package, name)
            if interface not in excluded_set:
                dependencies.add(interface)
    return tuple(sorted(dependencies))


def prepare(interfaces=()) -> DirectActionPlan:
    """Resolve every generated action value and envelope before alias mutation."""
    import cppyy
    from rclcpp_kit.native_action import resolve_cpp_action_type
    from rosidl_pycommon import convert_camel_case_to_lower_case_underscore
    from rosidl_runtime_py.utilities import get_action, get_message, get_service

    normalized = normalize_interfaces(interfaces)
    bindings = []
    targets = []
    constructors = []
    dependencies = set()
    target_keys = set()
    constructor_types = set()
    common_cancel = None

    def add_target(owner, attr, original, replacement):
        key = (id(owner), attr)
        if key in target_keys:
            return
        target_keys.add(key)
        targets.append((owner, attr, original, replacement))

    def add_constructor(replacement, original):
        if replacement not in constructor_types:
            constructor_types.add(replacement)
            constructors.append((replacement, original))

    seen = set()
    for interface in DEFAULT_INTERFACES + normalized:
        if interface in seen:
            continue
        seen.add(interface)
        match = _INTERFACE_NAME.fullmatch(interface)
        package, name = match.groups()
        snake_name = convert_camel_case_to_lower_case_underscore(name)
        expected_module = "%s.action._%s" % (package, snake_name)
        try:
            action_type = get_action(interface)
        except (ImportError, AttributeError, ValueError) as exc:
            raise TypeError(
                "direct_cpp action interface is not installed: %s" % interface
            ) from exc
        if action_type.__module__ != expected_module or action_type.__name__ != name:
            raise TypeError(
                "direct_cpp action did not resolve to exact canonical interface %s" %
                interface)

        generated = importlib.import_module(expected_module)
        public = importlib.import_module("%s.action" % package)
        if getattr(generated, name) is not action_type:
            raise RuntimeError("%s changed before direct alias installation" % interface)
        if getattr(public, name) is not action_type:
            raise RuntimeError("%s public alias changed before direct installation" % interface)

        impl = action_type.Impl
        prefix = "%s_" % name
        originals = {
            "goal": getattr(generated, prefix + "Goal"),
            "feedback": getattr(generated, prefix + "Feedback"),
            "result": getattr(generated, prefix + "Result"),
            "feedback_message": getattr(generated, prefix + "FeedbackMessage"),
            "goal_request": getattr(generated, prefix + "SendGoal_Request"),
            "goal_response": getattr(generated, prefix + "SendGoal_Response"),
            "result_request": getattr(generated, prefix + "GetResult_Request"),
            "result_response": getattr(generated, prefix + "GetResult_Response"),
        }
        if (
            action_type.Goal is not originals["goal"]
            or action_type.Feedback is not originals["feedback"]
            or action_type.Result is not originals["result"]
            or getattr(generated, prefix + "SendGoal") is not impl.SendGoalService
            or getattr(generated, prefix + "GetResult") is not impl.GetResultService
            or impl.FeedbackMessage is not originals["feedback_message"]
            or impl.SendGoalService.Request is not originals["goal_request"]
            or impl.SendGoalService.Response is not originals["goal_response"]
            or impl.GetResultService.Request is not originals["result_request"]
            or impl.GetResultService.Response is not originals["result_response"]
        ):
            raise RuntimeError(
                "%s payload or envelope aliases changed before installation" % interface)
        for suffix in (
            "SendGoal_Request", "SendGoal_Response",
            "GetResult_Request", "GetResult_Response",
        ):
            if getattr(public, prefix + suffix) is not getattr(generated, prefix + suffix):
                raise RuntimeError(
                    "%s public envelope aliases changed before installation" % interface)

        try:
            uuid_type = get_message("unique_identifier_msgs/msg/UUID")
            cancel_type = get_service("action_msgs/srv/CancelGoal")
        except (ImportError, AttributeError, ValueError) as exc:
            raise TypeError("direct_cpp action support types are not installed") from exc
        if impl.CancelGoalService is not cancel_type:
            raise TypeError("%s does not use canonical action_msgs/CancelGoal" % interface)
        if (
            uuid_type.__module__ != "unique_identifier_msgs.msg._uuid"
            or uuid_type.__name__ != "UUID"
            or cancel_type.__module__ != "action_msgs.srv._cancel_goal"
            or cancel_type.__name__ != "CancelGoal"
        ):
            raise TypeError("direct_cpp action support types are not canonical")

        cpp_types = resolve_cpp_action_type(action_type)
        expected_cpp_name = "%s::action::%s" % (package, name)
        cpp_impl = cpp_types.action.Impl
        cpp_values = {
            "goal": cpp_types.goal,
            "feedback": cpp_types.feedback,
            "result": cpp_types.result,
            "feedback_message": cpp_types.feedback_message,
            "goal_request": cpp_impl.SendGoalService.Request,
            "goal_response": cpp_types.goal_response,
            "result_request": cpp_impl.GetResultService.Request,
            "result_response": cpp_types.result_response,
        }
        if (
            cpp_types.cpp_name != expected_cpp_name
            or cpp_types.goal is not cpp_types.action.Goal
            or cpp_types.feedback is not cpp_types.action.Feedback
            or cpp_types.result is not cpp_types.action.Result
            or cpp_types.goal_response is not cpp_impl.SendGoalService.Response
            or cpp_types.feedback_message is not cpp_impl.FeedbackMessage
            or cpp_types.result_response is not cpp_impl.GetResultService.Response
            or cpp_types.goal_id is not
                cppyy.gbl.unique_identifier_msgs.msg.UUID
            or cpp_types.cancel_response is not cpp_impl.CancelGoalService.Response
        ):
            raise TypeError(
                "direct_cpp action C++ types do not match canonical interface %s" %
                interface)

        binding = DirectActionBinding(
            interface=interface,
            action_type=action_type,
            cpp_types=cpp_types,
            original_goal_type=originals["goal"],
            original_feedback_type=originals["feedback"],
            original_result_type=originals["result"],
            header="%s/action/%s.hpp" % (package, snake_name),
        )
        bindings.append(binding)

        for key, suffix in (
            ("goal", "Goal"),
            ("feedback", "Feedback"),
            ("result", "Result"),
            ("feedback_message", "FeedbackMessage"),
            ("goal_request", "SendGoal_Request"),
            ("goal_response", "SendGoal_Response"),
            ("result_request", "GetResult_Request"),
            ("result_response", "GetResult_Response"),
        ):
            add_target(
                generated, prefix + suffix, originals[key], cpp_values[key])
            add_constructor(cpp_values[key], originals[key])
        for key, attr in (
            ("goal", "Goal"),
            ("feedback", "Feedback"),
            ("result", "Result"),
            ("feedback_message", "FeedbackMessage"),
        ):
            owner = impl if key == "feedback_message" else action_type
            add_target(owner, attr, originals[key], cpp_values[key])
        for key, suffix, service_type in (
            ("goal_request", "SendGoal_Request", impl.SendGoalService),
            ("goal_response", "SendGoal_Response", impl.SendGoalService),
            ("result_request", "GetResult_Request", impl.GetResultService),
            ("result_response", "GetResult_Response", impl.GetResultService),
        ):
            add_target(public, prefix + suffix, originals[key], cpp_values[key])
            attr = "Request" if key.endswith("request") else "Response"
            add_target(service_type, attr, originals[key], cpp_values[key])

        local_messages = {
            "%s/msg/%s_%s" % (package, name, suffix)
            for suffix in ("Goal", "Result", "Feedback")
        }
        for original in originals.values():
            dependencies.update(_dependencies(original, local_messages))
        dependencies.add("unique_identifier_msgs/msg/UUID")

        cancel_request = cancel_type.Request
        cancel_response = cancel_type.Response
        cpp_cancel_request = cpp_impl.CancelGoalService.Request
        cpp_cancel_response = cpp_impl.CancelGoalService.Response
        if common_cancel is None:
            common_cancel = (
                cancel_type, cancel_request, cancel_response,
                cpp_cancel_request, cpp_cancel_response,
            )
        elif common_cancel[3:] != (cpp_cancel_request, cpp_cancel_response):
            raise TypeError("direct_cpp actions resolved inconsistent cancel envelopes")
        dependencies.update(_dependencies(cancel_request))
        dependencies.update(_dependencies(cancel_response))

    cancel_type, cancel_request, cancel_response, cpp_request, cpp_response = common_cancel
    cancel_generated = importlib.import_module("action_msgs.srv._cancel_goal")
    cancel_public = importlib.import_module("action_msgs.srv")
    if (
        cancel_generated.CancelGoal is not cancel_type
        or cancel_public.CancelGoal is not cancel_type
        or cancel_generated.CancelGoal_Request is not cancel_request
        or cancel_generated.CancelGoal_Response is not cancel_response
        or cancel_public.CancelGoal_Request is not cancel_request
        or cancel_public.CancelGoal_Response is not cancel_response
    ):
        raise RuntimeError("CancelGoal aliases changed before direct action installation")
    for owner, attr, original, replacement in (
        (cancel_generated, "CancelGoal_Request", cancel_request, cpp_request),
        (cancel_generated, "CancelGoal_Response", cancel_response, cpp_response),
        (cancel_public, "CancelGoal_Request", cancel_request, cpp_request),
        (cancel_public, "CancelGoal_Response", cancel_response, cpp_response),
        (cancel_type, "Request", cancel_request, cpp_request),
        (cancel_type, "Response", cancel_response, cpp_response),
    ):
        add_target(owner, attr, original, replacement)
    add_constructor(cpp_request, cancel_request)
    add_constructor(cpp_response, cancel_response)

    return DirectActionPlan(
        bindings=tuple(bindings),
        targets=tuple(targets),
        constructors=tuple(constructors),
        message_dependencies=tuple(sorted(dependencies)),
    )


def install(interfaces=(), *, plan=None) -> DirectActionInstallation:
    """Atomically install every prepared generated C++ action alias."""
    global _ACTIVE_INSTALLATION
    if plan is not None and normalize_interfaces(interfaces):
        raise ValueError("install accepts either interfaces or a prepared plan")
    selected_plan = prepare(interfaces) if plan is None else plan
    if not isinstance(selected_plan, DirectActionPlan):
        raise TypeError("plan must be a DirectActionPlan")

    pythonizations = []
    replacements = []
    try:
        for cpp_type, original in selected_plan.constructors:
            pythonization = _pythonize_constructor(cpp_type, original)
            if pythonization is not None:
                pythonizations.append(pythonization)
        for owner, name, original, replacement in selected_plan.targets:
            current = getattr(owner, name)
            if current is replacement:
                continue
            if current is not original:
                raise RuntimeError(
                    "%s.%s changed during direct action installation" %
                    (owner.__name__, name))
            setattr(owner, name, replacement)
            replacements.append((owner, name, original, replacement))
    except Exception:
        DirectActionInstallation(
            replacements, selected_plan.bindings, pythonizations).restore()
        raise
    installation = DirectActionInstallation(
        replacements, selected_plan.bindings, pythonizations)
    _ACTIVE_INSTALLATION = installation
    return installation


def resolve_supported_type(action_type: Any):
    installation = _ACTIVE_INSTALLATION
    if installation is None:
        raise RuntimeError("direct_cpp action aliases are not installed")
    binding = next(
        (candidate for candidate in installation.bindings
         if action_type is candidate.action_type),
        None,
    )
    if binding is None:
        raise TypeError(
            "direct_cpp action type is not registered; active interfaces: %s" %
            ", ".join(item.interface for item in installation.bindings))
    cpp_types = binding.cpp_types
    if (
        action_type.Goal is not cpp_types.goal
        or action_type.Feedback is not cpp_types.feedback
        or action_type.Result is not cpp_types.result
    ):
        raise TypeError(
            "direct_cpp C++ action aliases are not active for %s" % binding.interface)
    return binding


class DirectClientGoalHandle:
    """rclpy-shaped control handle backed by retained C++ action values."""

    def __init__(self, action_client, token, goal_id, goal_response):
        self._action_client = action_client
        self._token = int(token)
        self._goal_id = goal_id
        self._goal_response = goal_response
        self._status = 0

    @property
    def accepted(self):
        return bool(self._goal_response.accepted)

    @property
    def goal_id(self):
        return self._goal_id

    @property
    def stamp(self):
        return self._goal_response.stamp

    @property
    def status(self):
        return self._status

    def get_result(self):
        _unsupported("direct_cpp action handles support get_result_async(), not get_result()")

    def get_result_async(self):
        return self._action_client._get_result_async(self)

    def cancel_goal(self):
        _unsupported("direct_cpp action handles support cancel_goal_async(), not cancel_goal()")

    def cancel_goal_async(self):
        return self._action_client._cancel_goal_async(self)

    def __eq__(self, other):
        if not isinstance(other, DirectClientGoalHandle):
            return False
        return _byte_values(self.goal_id.uuid) == _byte_values(other.goal_id.uuid)

    def __ne__(self, other):
        return not self == other

    def __repr__(self):
        return "ClientGoalHandle <id=%r, accepted=%r, status=%d>" % (
            list(_byte_values(self.goal_id.uuid)), self.accepted, self.status)


class DirectActionClient(metaclass=_DirectSurface):
    """Lean source-shape facade over a C++-owned ``rclcpp_action`` client."""

    _PARITY_HIDDEN = frozenset({
        "action_name", "close", "closed", "compile_result",
        "configure_introspection", "python_feedback_callbacks", "source_id", "stats",
    })

    def __init__(
        self,
        node,
        action_type,
        action_name,
        *,
        callback_group=None,
        goal_service_qos_profile=_DEFAULT_QOS,
        result_service_qos_profile=_DEFAULT_QOS,
        cancel_service_qos_profile=_DEFAULT_QOS,
        feedback_sub_qos_profile=_DEFAULT_QOS,
        status_sub_qos_profile=_DEFAULT_QOS,
    ):
        binding = resolve_supported_type(action_type)
        if getattr(node, "_direct_cpp_node", None) is None:
            raise TypeError("direct_cpp ActionClient requires a direct_cpp Node")
        if getattr(node, "_direct_cpp_lifecycle_resource", None) is not None:
            _unsupported(
                "direct_cpp does not support ActionClient on a lifecycle node")
        self._validate_qos(
            goal_service_qos_profile,
            result_service_qos_profile,
            cancel_service_qos_profile,
            feedback_sub_qos_profile,
            status_sub_qos_profile,
        )
        name = str(action_name)
        if not name.strip():
            raise ValueError("action_name must not be empty")

        from rclcppyy.direct_cpp import _runtime

        group, native_group = node._resolve_callback_group(callback_group)
        self._node = node
        self._action_type = action_type
        self._action_name = name
        self._cpp_types = binding.cpp_types
        self._native = _runtime().require_session().create_native_action_client(
            node._require_node(),
            action_type,
            name,
            callback_group=native_group,
            feedback_capacity=_FEEDBACK_CAPACITY,
        )
        self.callback_group = group
        self._lock = threading.RLock()
        self._goal_futures = {}
        self._result_futures = {}
        self._cancel_futures = {}
        self._handles = {}
        self._feedback_callbacks = {}
        self._orphaned_goals = set()
        self._discard_results = set()
        self._discard_cancels = set()
        self._last_feedback_dropped = 0
        self._python_feedback_callbacks = 0
        self._closed = False
        node._direct_cpp_action_clients.append(self)
        group.add_entity(self)
        record_decision(
            "entities",
            "cpp",
            "direct typed rclcpp action client with generated C++ values",
            policies=(
                "direct_cpp", "direct_cpp_action", "no_conversion",
                "per_operation_future", "cpp_pending_state",
            ),
            metadata={
                "entity_type": "action_client",
                "action_name": name,
                "action_type": binding.cpp_types.cpp_name,
                "action_interface": binding.interface,
                "goal_representation": "actual_cpp",
                "goal_id_representation": "actual_cpp",
                "feedback_representation": "actual_cpp",
                "result_representation": "actual_cpp",
                "cancel_representation": "actual_cpp",
                "python_message_conversions": 0,
                "python_serialization_calls": 0,
                "future_control": "per_operation_rclpy_task_future",
                "rejected_goal_uuid_stamp": "zero_cpp_values_rclcpp_public_api_limit",
                "source_id": self._native.source_id,
            },
        )

    @staticmethod
    def _validate_qos(goal, result, cancel, feedback, status):
        from rclpy.qos import qos_profile_action_status_default
        from rclpy.qos import qos_profile_services_default
        from rclpy.qos import QoSProfile

        expected = (
            qos_profile_services_default,
            qos_profile_services_default,
            qos_profile_services_default,
            QoSProfile(depth=10),
            qos_profile_action_status_default,
        )
        supplied = (goal, result, cancel, feedback, status)
        unsupported = [
            name for name, value, default in zip(
                ("goal", "result", "cancel", "feedback", "status"),
                supplied,
                expected,
            )
            if value is not _DEFAULT_QOS and value != default
        ]
        if unsupported:
            _unsupported(
                "direct_cpp action client requires default QoS for: %s" %
                ", ".join(unsupported))

    @property
    def action_name(self):
        return self._action_name

    @property
    def closed(self):
        return self._closed

    @property
    def compile_result(self):
        return dict(self._native.compile_result)

    @property
    def source_id(self):
        return self._native.source_id

    def server_is_ready(self):
        return False if self._closed else self._native.server_is_ready()

    def wait_for_server(self, timeout_sec=None):
        if self._closed:
            return False
        if timeout_sec is None:
            while self._node.context.ok() and not self.server_is_ready():
                self._native.wait_for_server(0.25)
            return self.server_is_ready()
        timeout = float(timeout_sec)
        if not math.isfinite(timeout) or timeout < 0:
            raise ValueError("timeout_sec must be a finite non-negative number or None")
        return self._native.wait_for_server(timeout)

    def send_goal(self, goal, **kwargs):
        _unsupported("direct_cpp action clients support send_goal_async(), not send_goal()")

    def send_goal_async(self, goal, feedback_callback=None, goal_uuid=None):
        if self._closed:
            raise RuntimeError("direct_cpp action client is destroyed")
        if not isinstance(goal, self._action_type.Goal):
            raise TypeError("goal must be an actual direct_cpp C++ action Goal")
        if goal_uuid is not None:
            _unsupported("direct_cpp action clients do not support custom goal_uuid")
        if feedback_callback is not None:
            if not callable(feedback_callback):
                raise TypeError("feedback_callback must be callable")
            target = getattr(feedback_callback, "__call__", feedback_callback)
            if inspect.iscoroutinefunction(feedback_callback) or inspect.iscoroutinefunction(
                target
            ):
                _unsupported("direct_cpp action clients require synchronous feedback callbacks")

        token = int(self._native.send_cpp_value(goal))
        future = self._new_future("goal", token)
        with self._lock:
            self._goal_futures[token] = future
            if feedback_callback is not None:
                self._feedback_callbacks[token] = feedback_callback
        future.add_done_callback(self._goal_future_finished)
        return future

    def _get_result(self, goal_handle):
        _unsupported("direct_cpp action handles support get_result_async(), not get_result()")

    def _get_result_async(self, goal_handle):
        self._validate_handle(goal_handle, require_accepted=True)
        token = goal_handle._token
        with self._lock:
            if token in self._result_futures:
                _unsupported("direct_cpp supports one result Future per action goal")
            if token not in self._handles:
                raise RuntimeError("action goal is no longer active")
            future = self._new_future("result", token)
            self._result_futures[token] = future
        future.add_done_callback(self._result_future_finished)
        return future

    def _cancel_goal(self, goal_handle):
        _unsupported("direct_cpp action handles support cancel_goal_async(), not cancel_goal()")

    def _cancel_goal_async(self, goal_handle):
        self._validate_handle(goal_handle, require_accepted=True)
        token = goal_handle._token
        with self._lock:
            if token in self._cancel_futures:
                _unsupported("direct_cpp supports one cancel Future per action goal")
            if token not in self._handles:
                raise RuntimeError("action goal is no longer active")
            if not self._native.request_cancel(token):
                raise RuntimeError("action goal cannot be canceled in its current state")
            future = self._new_future("cancel", token)
            self._cancel_futures[token] = future
        future.add_done_callback(self._cancel_future_finished)
        return future

    def _validate_handle(self, handle, *, require_accepted):
        if self._closed:
            raise RuntimeError("direct_cpp action client is destroyed")
        if not isinstance(handle, DirectClientGoalHandle) or handle._action_client is not self:
            raise TypeError("goal handle does not belong to this direct_cpp ActionClient")
        if require_accepted and not handle.accepted:
            _unsupported("direct_cpp cannot operate on a rejected action goal")

    def _new_future(self, kind, token):
        from rclpy.task import Future
        from rclcppyy.direct_cpp import _runtime

        future = Future()
        future._rclcppyy_direct_runtime = _runtime()
        future._rclcppyy_direct_action_client = self
        future._rclcppyy_direct_action_kind = kind
        future._rclcppyy_direct_token = int(token)
        return future

    def _goal_future_finished(self, future):
        if future.cancelled():
            with self._lock:
                self._orphaned_goals.add(future._rclcppyy_direct_token)

    def _result_future_finished(self, future):
        if future.cancelled():
            with self._lock:
                self._discard_results.add(future._rclcppyy_direct_token)

    def _cancel_future_finished(self, future):
        if future.cancelled():
            with self._lock:
                self._discard_cancels.add(future._rclcppyy_direct_token)

    def _poll_ready(self):
        if self._closed:
            return
        with self._lock:
            tokens = tuple(
                set(self._goal_futures)
                | set(self._handles)
                | set(self._result_futures)
                | set(self._cancel_futures)
            )
        for token in tokens:
            state = self._native.poll_state(token)
            if not self._poll_goal_response(token, state):
                continue
            if not self._poll_feedback(token, int(state.feedback_ready_count)):
                return
            self._check_feedback_overflow(int(state.feedback_dropped))
            self._poll_cancel_response(token, bool(state.cancel_response_ready))
            if self._closed:
                return
            self._poll_result(token, bool(state.result_ready))

    def _check_feedback_overflow(self, dropped):
        if dropped != self._last_feedback_dropped:
            previous = self._last_feedback_dropped
            self._last_feedback_dropped = dropped
            raise RuntimeError(
                "direct_cpp action feedback queue overflowed: %d new messages" %
                (dropped - previous))

    def _poll_goal_response(self, token, state):
        with self._lock:
            future = self._goal_futures.get(token)
        if future is None or not bool(state.goal_response_ready):
            return True
        goal_id = self._native.goal_id(token)
        response = self._native.goal_response(token)
        handle = DirectClientGoalHandle(self, token, goal_id, response)
        with self._lock:
            if self._goal_futures.get(token) is not future:
                return True
            del self._goal_futures[token]
            orphaned = token in self._orphaned_goals or future.cancelled()
            self._orphaned_goals.discard(token)
            if handle.accepted and not orphaned:
                self._handles[token] = handle
            else:
                self._feedback_callbacks.pop(token, None)
        if orphaned:
            self._native.forget(token)
            return False
        if not future.done() and not future.cancelled():
            future.set_result(handle)
        if not handle.accepted:
            self._native.forget(token)
            return False
        return not self._closed

    def _poll_feedback(self, token, ready_count):
        with self._lock:
            if token not in self._handles:
                return True
        for _ in range(ready_count):
            message = self._native.take_feedback_message(token)
            callback = self._feedback_callbacks.get(token)
            if callback is None:
                continue
            result = callback(message)
            if inspect.isawaitable(result):
                close = getattr(result, "close", None)
                if close is not None:
                    close()
                _unsupported(
                    "direct_cpp feedback callbacks must complete synchronously")
            self._python_feedback_callbacks += 1
            if self._closed:
                return False
        return True

    def _poll_cancel_response(self, token, ready):
        if not ready:
            return
        with self._lock:
            future = self._cancel_futures.get(token)
        if future is None:
            return
        response = self._native.take_cancel_response(token)
        with self._lock:
            if self._cancel_futures.get(token) is not future:
                return
            del self._cancel_futures[token]
            discard = token in self._discard_cancels or future.cancelled()
            self._discard_cancels.discard(token)
        if not discard and not future.done() and not future.cancelled():
            future.set_result(response)

    def _poll_result(self, token, ready):
        if not ready:
            return
        with self._lock:
            future = self._result_futures.get(token)
        if future is None:
            return
        response = self._native.take_result_response(token)
        with self._lock:
            if self._result_futures.get(token) is not future:
                return
            del self._result_futures[token]
            handle = self._handles.pop(token, None)
            self._feedback_callbacks.pop(token, None)
            discard = token in self._discard_results or future.cancelled()
            self._discard_results.discard(token)
        if handle is not None:
            handle._status = _int8_value(response.status)
        if not discard and not future.done() and not future.cancelled():
            future.set_result(response)

    def stats(self):
        return self._native.stats()

    @property
    def python_feedback_callbacks(self):
        return self._python_feedback_callbacks

    def configure_introspection(self, *args, **kwargs):
        _unsupported("direct_cpp action clients do not support introspection")

    def destroy(self):
        self.close()

    def close(self):
        if self._closed:
            return False
        with self._lock:
            futures = tuple(self._goal_futures.values()) + tuple(
                self._result_futures.values()) + tuple(self._cancel_futures.values())
        for future in futures:
            future.cancel()
        self._native.close()
        self._closed = True
        self._goal_futures.clear()
        self._result_futures.clear()
        self._cancel_futures.clear()
        self._handles.clear()
        self._feedback_callbacks.clear()
        self.callback_group.discard_entity(self)
        self._node._discard_direct_action_client(self)
        return True


class DirectServerGoalHandle(metaclass=_DirectSurface):
    """Jazzy-shaped server goal handle over one native C++ goal record."""

    _PARITY_HIDDEN = frozenset({
        "__repr__", "abort_shared", "canceled_shared", "create_feedback_shared",
        "create_result_shared", "publish_feedback_shared", "succeed_shared",
    })

    def __init__(self, action_server, accepted_goal):
        self._action_server = action_server
        self._token = int(accepted_goal.token)
        self._goal_request = accepted_goal.goal
        self._goal_id = accepted_goal.goal_id
        self._initial_status = int(accepted_goal.status)
        self._pending_terminal = None
        self._terminal_status = None
        self._executing = self._initial_status == _STATUS_EXECUTING
        self._execute_callback_depth = 0
        self._destroyed = False
        self._lock = threading.RLock()

    @property
    def request(self):
        return self._goal_request

    @property
    def goal_id(self):
        return self._goal_id

    @property
    def is_active(self):
        with self._lock:
            if self._destroyed or self._terminal_status is not None:
                return False
            if self._pending_terminal is not None:
                return False
        return self._action_server._goal_is_active(self)

    @property
    def is_cancel_requested(self):
        if self._destroyed or self._terminal_status is not None:
            return False
        return self._action_server._goal_is_canceling(self)

    @property
    def status(self):
        with self._lock:
            if self._terminal_status is not None:
                return self._terminal_status
            if self._pending_terminal is not None:
                return _TERMINAL_STATUS[self._pending_terminal]
            if self._destroyed:
                return _STATUS_UNKNOWN
        return self._action_server._goal_status(self)

    def executing(self):
        self._action_server._begin_goal_execution(self, skip_if_canceling=False)

    def execute(self, execute_callback=None):
        self._action_server.notify_execute(self, execute_callback)

    def create_feedback_shared(self):
        """Create exact C++ feedback with explicit shared ownership.

        The value may be retained after publication. It must not be mutated
        concurrently with ``publish_feedback_shared``.
        """
        self._require_available()
        return self._action_server._native.make_feedback_shared()

    def create_result_shared(self):
        """Create an exact C++ result with explicit shared ownership.

        The value may be retained after a terminal call. It must not be mutated
        concurrently with that call.
        """
        self._require_available()
        return self._action_server._native.make_result_shared()

    def publish_feedback(self, feedback):
        self._require_available()
        if not isinstance(feedback, self._action_server.action_type.Feedback):
            raise TypeError(
                "feedback must be an actual direct_cpp C++ action Feedback")
        self._action_server._native.publish_feedback(self._token, feedback)

    def publish_feedback_shared(self, feedback):
        """Publish factory-created feedback without an adapter deep copy."""
        self._require_available()
        self._action_server._native.publish_feedback_shared(
            self._token, feedback)

    def succeed(self, response=None):
        self._request_terminal("succeed", response)

    def abort(self, response=None):
        self._request_terminal("abort", response)

    def canceled(self, response=None):
        self._request_terminal("canceled", response)

    def succeed_shared(self, response):
        self._request_terminal_shared("succeed", response)

    def abort_shared(self, response):
        self._request_terminal_shared("abort", response)

    def canceled_shared(self, response):
        self._request_terminal_shared("canceled", response)

    def _request_terminal(self, operation, response):
        self._require_available()
        if response is None and self._execute_callback_depth == 0:
            _unsupported(
                "response-less terminal state changes require an active "
                "direct_cpp execute callback")
        if response is not None and not isinstance(
            response, self._action_server.action_type.Result
        ):
            raise TypeError(
                "response must be an actual direct_cpp C++ action Result")
        with self._lock:
            if self._terminal_status is not None:
                raise RuntimeError("direct_cpp action goal is already terminal")
            if self._pending_terminal not in (None, operation):
                raise RuntimeError(
                    "direct_cpp action goal already has a pending terminal state")
            self._pending_terminal = operation
        if response is not None:
            self._action_server._commit_terminal(self, operation, response)

    def _request_terminal_shared(self, operation, response):
        self._require_available()
        if not isinstance(response, self._action_server.action_type.Result):
            raise TypeError(
                "response must be an actual direct_cpp C++ action Result")
        pointer = getattr(response, "__smartptr__", lambda: None)()
        if pointer is None or not bool(pointer):
            raise TypeError(
                "response must come from create_result_shared")
        with self._lock:
            if self._terminal_status is not None:
                raise RuntimeError("direct_cpp action goal is already terminal")
            if self._pending_terminal not in (None, operation):
                raise RuntimeError(
                    "direct_cpp action goal already has a pending terminal state")
            previous = self._pending_terminal
            self._pending_terminal = operation
        try:
            self._action_server._commit_terminal_shared(
                self, operation, response)
        except BaseException:
            with self._lock:
                self._pending_terminal = previous
            raise

    def _require_available(self):
        if self._destroyed:
            raise RuntimeError("direct_cpp server goal handle is destroyed")
        if self._action_server.closed:
            raise RuntimeError("direct_cpp action server is destroyed")

    def destroy(self):
        with self._lock:
            if self._destroyed:
                return
            self._destroyed = True
        self._action_server._forget_goal(self)

    def __eq__(self, other):
        if not isinstance(other, DirectServerGoalHandle):
            return False
        return _byte_values(self.goal_id.uuid) == _byte_values(other.goal_id.uuid)

    def __ne__(self, other):
        return not self == other

    def __repr__(self):
        return "ServerGoalHandle <id=%r, status=%d>" % (
            list(_byte_values(self.goal_id.uuid)), self.status)


class DirectActionServer(metaclass=_DirectSurface):
    """Lean synchronous ActionServer facade over ``rclcpp_action`` authority."""

    _PARITY_HIDDEN = frozenset({
        "action_name", "callback_error_ready", "close", "close_pending", "closed",
        "compile_result", "configure_introspection", "source_id", "stats",
        "take_callback_error",
    })

    def __init__(
        self,
        node,
        action_type,
        action_name,
        execute_callback=None,
        *,
        callback_group=None,
        goal_callback=_default_goal_callback,
        handle_accepted_callback=_default_handle_accepted_callback,
        cancel_callback=_default_cancel_callback,
        goal_service_qos_profile=_DEFAULT_QOS,
        result_service_qos_profile=_DEFAULT_QOS,
        cancel_service_qos_profile=_DEFAULT_QOS,
        feedback_pub_qos_profile=_DEFAULT_QOS,
        status_pub_qos_profile=_DEFAULT_QOS,
        result_timeout=900,
    ):
        binding = resolve_supported_type(action_type)
        if getattr(node, "_direct_cpp_node", None) is None:
            raise TypeError("direct_cpp ActionServer requires a direct_cpp Node")
        if getattr(node, "_direct_cpp_lifecycle_resource", None) is not None:
            _unsupported(
                "direct_cpp does not support ActionServer on a lifecycle node")
        servers = getattr(node, "_direct_cpp_action_servers", None)
        if servers is None:
            _unsupported(
                "direct_cpp ActionServer node polling integration is not active")
        self._validate_qos(
            goal_service_qos_profile,
            result_service_qos_profile,
            cancel_service_qos_profile,
            feedback_pub_qos_profile,
            status_pub_qos_profile,
        )
        name = str(action_name)
        if not name.strip():
            raise ValueError("action_name must not be empty")
        if callback_group is not None and getattr(
            callback_group, "_kind", None
        ) == "reentrant":
            _unsupported(
                "P0 direct_cpp action servers require a mutually-exclusive "
                "callback group")
        executor = getattr(node, "executor", None)
        if executor is not None and getattr(executor, "_kind", None) == "multi_threaded":
            _unsupported(
                "P0 direct_cpp action servers do not support MultiThreadedExecutor")

        self._node = node
        self._action_type = action_type
        self._action_name = name
        self._cpp_types = binding.cpp_types
        self._binding = binding
        self._lock = threading.RLock()
        self._goal_handles = {}
        self._callback_errors = []
        self._callback_depth = 0
        self._closed = False
        self._close_pending = False
        self._native = None
        self.callback_group = None
        self._goal_callback = None
        self._cancel_callback = None
        self._handle_accepted_callback = None
        self._execute_callback = None

        self.register_goal_callback(goal_callback)
        self.register_cancel_callback(cancel_callback)
        self.register_handle_accepted_callback(handle_accepted_callback)
        if execute_callback is not None:
            self.register_execute_callback(execute_callback)

        group, native_group = node._resolve_callback_group(callback_group)
        if getattr(group, "_kind", None) != "mutually_exclusive":
            _unsupported(
                "P0 direct_cpp action servers require a mutually-exclusive "
                "callback group")
        from rclcppyy.direct_cpp import _runtime

        self._native = _runtime().require_session().create_native_action_server(
            node._require_node(),
            action_type,
            name,
            goal_callback=self._decide_goal,
            cancel_callback=self._decide_cancel,
            callback_group=native_group,
            result_timeout=result_timeout,
        )
        self.callback_group = group
        servers.append(self)
        group.add_entity(self)
        record_decision(
            "entities",
            "cpp",
            "direct typed rclcpp action server with generated C++ values",
            policies=(
                "direct_cpp", "direct_cpp_action_server", "no_conversion",
                "synchronous_p0", "polled_accepted_goals",
            ),
            metadata={
                "entity_type": "action_server",
                "action_name": name,
                "action_type": binding.cpp_types.cpp_name,
                "action_interface": binding.interface,
                "goal_representation": "actual_cpp",
                "goal_id_representation": "actual_cpp",
                "feedback_representation": "actual_cpp",
                "result_representation": "actual_cpp",
                "python_message_conversions": 0,
                "python_serialization_calls": 0,
                "decision_callbacks": "synchronous_creator_thread",
                "terminal_result": "exact_cpp_deferred_commit",
                "source_id": self._native.source_id,
            },
        )

    @staticmethod
    def _validate_qos(goal, result, cancel, feedback, status):
        from rclpy.qos import qos_profile_action_status_default
        from rclpy.qos import qos_profile_services_default
        from rclpy.qos import QoSProfile

        expected = (
            qos_profile_services_default,
            qos_profile_services_default,
            qos_profile_services_default,
            QoSProfile(depth=10),
            qos_profile_action_status_default,
        )
        supplied = (goal, result, cancel, feedback, status)
        unsupported = [
            name for name, value, default in zip(
                ("goal", "result", "cancel", "feedback", "status"),
                supplied,
                expected,
            )
            if value is not _DEFAULT_QOS and value != default
        ]
        if unsupported:
            _unsupported(
                "P0 direct_cpp action servers require default QoS for: %s" %
                ", ".join(unsupported))

    @property
    def action_type(self):
        return self._action_type

    @property
    def action_name(self):
        return self._action_name

    @property
    def closed(self):
        return self._closed

    @property
    def close_pending(self):
        return self._close_pending and not self._closed

    @property
    def compile_result(self):
        return dict(self._native.compile_result)

    @property
    def source_id(self):
        return self._native.source_id

    def register_goal_callback(self, goal_callback):
        if self._closed:
            raise RuntimeError("direct_cpp action server is destroyed")
        callback = _default_goal_callback if goal_callback is None else goal_callback
        _validate_sync_callback("goal_callback", callback)
        with self._lock:
            self._goal_callback = callback

    def register_cancel_callback(self, cancel_callback):
        if self._closed:
            raise RuntimeError("direct_cpp action server is destroyed")
        callback = _default_cancel_callback if cancel_callback is None else cancel_callback
        _validate_sync_callback("cancel_callback", callback)
        with self._lock:
            self._cancel_callback = callback

    def register_handle_accepted_callback(self, handle_accepted_callback):
        if self._closed:
            raise RuntimeError("direct_cpp action server is destroyed")
        callback = (
            _default_handle_accepted_callback
            if handle_accepted_callback is None
            else handle_accepted_callback
        )
        _validate_sync_callback("handle_accepted_callback", callback)
        with self._lock:
            self._handle_accepted_callback = callback

    def register_execute_callback(self, execute_callback):
        if self._closed:
            raise RuntimeError("direct_cpp action server is destroyed")
        _validate_sync_callback("execute_callback", execute_callback)
        with self._lock:
            self._execute_callback = execute_callback

    def _record_callback_error(self, error):
        with self._lock:
            self._callback_errors.append(error)

    def _invoke_callback(self, name, callback, argument):
        # Also bump the owning node's shared in-flight counter (Slice 2.5,
        # docs/plans/PLAN-mte-unlock.md Addendum risk 4): generalizes this
        # action server's own _callback_depth gate into the same mechanism
        # destroy_node()/destroy_*() wait on, rather than duplicating a
        # second gate. _callback_depth itself is kept -- close()/
        # _service_pending_close() still use it for this entity's own
        # deferred-close semantics.
        enter_in_flight = getattr(self._node, "_enter_in_flight", None)
        exit_in_flight = getattr(self._node, "_exit_in_flight", None)
        with self._lock:
            self._callback_depth += 1
        if enter_in_flight is not None:
            enter_in_flight()
        try:
            return _sync_result(name, callback, argument)
        finally:
            if exit_in_flight is not None:
                exit_in_flight()
            with self._lock:
                self._callback_depth -= 1

    def callback_error_ready(self):
        with self._lock:
            return bool(self._callback_errors)

    def take_callback_error(self):
        with self._lock:
            if not self._callback_errors:
                raise RuntimeError("no direct action-server callback error is ready")
            return self._callback_errors.pop(0)

    def _decide_goal(self, goal):
        if self._closed:
            return False
        with self._lock:
            callback = self._goal_callback
        try:
            response = self._invoke_callback("goal_callback", callback, goal)
            from rclpy.action import GoalResponse

            if not isinstance(response, GoalResponse):
                raise TypeError("goal_callback must return GoalResponse")
            return response == GoalResponse.ACCEPT
        except BaseException as error:
            self._record_callback_error(error)
            return False

    def _decide_cancel(self, token):
        if self._closed:
            return False
        with self._lock:
            handle = self._goal_handles.get(int(token))
            callback = self._cancel_callback
        if handle is None:
            return False
        try:
            response = self._invoke_callback("cancel_callback", callback, handle)
            from rclpy.action import CancelResponse

            if not isinstance(response, CancelResponse):
                raise TypeError("cancel_callback must return CancelResponse")
            return response == CancelResponse.ACCEPT
        except BaseException as error:
            self._record_callback_error(error)
            return False

    def _poll_ready(self):
        if self._closed:
            return
        if self._close_pending:
            self._service_pending_close()
            return
        while self._native.callback_error_ready():
            self._record_callback_error(self._native.take_callback_error())
        while not self._closed and self._native.accepted_ready_count():
            accepted = self._native.take_accepted()
            handle = DirectServerGoalHandle(self, accepted)
            with self._lock:
                self._goal_handles[handle._token] = handle
                callback = self._handle_accepted_callback
            try:
                self._invoke_callback(
                    "handle_accepted_callback", callback, handle)
            except BaseException as error:
                self._record_callback_error(error)
                self._abort_after_callback_error(handle)
        if self._close_pending:
            self._service_pending_close()

    def _abort_after_callback_error(self, handle):
        if self._closed or handle._terminal_status is not None:
            return
        try:
            if not handle._executing and not handle.is_cancel_requested:
                self._begin_goal_execution(handle, skip_if_canceling=True)
            result = self.action_type.Result()
            self._commit_terminal(handle, "abort", result, replace_pending=True)
        except BaseException as error:
            self._record_callback_error(error)

    def notify_execute(self, goal_handle, execute_callback):
        self._validate_goal_handle(goal_handle)
        callback = self._execute_callback if execute_callback is None else execute_callback
        if callback is not None:
            _validate_sync_callback("execute_callback", callback)
        self._begin_goal_execution(goal_handle, skip_if_canceling=True)
        if callback is not None:
            self._execute_goal(callback, goal_handle)

    def notify_goal_done(self):
        return None

    def _begin_goal_execution(self, handle, *, skip_if_canceling):
        self._validate_goal_handle(handle)
        with handle._lock:
            if handle._executing:
                raise RuntimeError("direct_cpp action goal is already executing")
            if handle._terminal_status is not None or handle._pending_terminal is not None:
                raise RuntimeError("direct_cpp action goal is already terminal")
            if skip_if_canceling and handle.is_cancel_requested:
                return
            self._native.execute(handle._token)
            handle._executing = True

    def _execute_goal(self, callback, handle):
        with handle._lock:
            handle._execute_callback_depth += 1
        try:
            result = self._invoke_callback("execute_callback", callback, handle)
            if not isinstance(result, self.action_type.Result):
                raise TypeError(
                    "execute_callback must return an actual direct_cpp C++ action Result")
        except BaseException as error:
            self._record_callback_error(error)
            result = self.action_type.Result()
            if handle._terminal_status is None:
                self._commit_terminal(handle, "abort", result, replace_pending=True)
            return
        finally:
            with handle._lock:
                handle._execute_callback_depth -= 1
        if handle._terminal_status is not None:
            return
        operation = handle._pending_terminal or "abort"
        self._commit_terminal(handle, operation, result)

    def _commit_terminal(self, handle, operation, result, *, replace_pending=False):
        self._validate_goal_handle(handle)
        if operation not in _TERMINAL_STATUS:
            raise ValueError("unknown direct action terminal operation")
        if not isinstance(result, self.action_type.Result):
            raise TypeError("terminal result must be an actual direct_cpp C++ Result")
        with handle._lock:
            if handle._terminal_status is not None:
                raise RuntimeError("direct_cpp action goal is already terminal")
            if replace_pending:
                handle._pending_terminal = operation
            elif handle._pending_terminal not in (None, operation):
                raise RuntimeError("direct_cpp action goal has another terminal state")
            getattr(self._native, operation)(handle._token, result)
            handle._pending_terminal = None
            handle._terminal_status = _TERMINAL_STATUS[operation]

    def _commit_terminal_shared(self, handle, operation, result):
        self._validate_goal_handle(handle)
        if operation not in _TERMINAL_STATUS:
            raise ValueError("unknown direct action terminal operation")
        if not isinstance(result, self.action_type.Result):
            raise TypeError("terminal result must be an actual direct_cpp C++ Result")
        pointer = getattr(result, "__smartptr__", lambda: None)()
        if pointer is None or not bool(pointer):
            raise TypeError("terminal result must come from create_result_shared")
        with handle._lock:
            if handle._terminal_status is not None:
                raise RuntimeError("direct_cpp action goal is already terminal")
            if handle._pending_terminal != operation:
                raise RuntimeError("direct_cpp action goal has another terminal state")
            getattr(self._native, operation + "_shared")(handle._token, result)
            handle._pending_terminal = None
            handle._terminal_status = _TERMINAL_STATUS[operation]

    def _validate_goal_handle(self, handle):
        if self._closed:
            raise RuntimeError("direct_cpp action server is destroyed")
        if not isinstance(handle, DirectServerGoalHandle):
            raise TypeError("expected a direct_cpp ServerGoalHandle")
        if handle._action_server is not self:
            raise TypeError("server goal handle belongs to another ActionServer")
        if handle._destroyed:
            raise RuntimeError("direct_cpp server goal handle is destroyed")
        with self._lock:
            if self._goal_handles.get(handle._token) is not handle:
                raise RuntimeError("direct_cpp action goal is no longer tracked")

    def _goal_status(self, handle):
        self._validate_goal_handle(handle)
        return int(self._native.status(handle._token))

    def _goal_is_active(self, handle):
        self._validate_goal_handle(handle)
        return bool(self._native.is_active(handle._token))

    def _goal_is_canceling(self, handle):
        self._validate_goal_handle(handle)
        return bool(self._native.is_canceling(handle._token))

    def _forget_goal(self, handle):
        with self._lock:
            if self._goal_handles.get(handle._token) is handle:
                del self._goal_handles[handle._token]
        if not self._closed:
            self._native.forget(handle._token)

    def stats(self):
        return self._native.stats()

    def configure_introspection(self, *args, **kwargs):
        _unsupported("P0 direct_cpp action servers do not support introspection")

    def destroy(self):
        self.close()

    def close(self):
        if self._closed:
            return False
        with self._lock:
            if self._callback_depth:
                self._close_pending = True
                return False
        if not self._native.close():
            self._close_pending = True
            return False
        self._finalize_close()
        return True

    def _service_pending_close(self):
        if self._closed or not self._close_pending:
            return False
        with self._lock:
            if self._callback_depth:
                return False
        if self._native.close_pending:
            closed = self._native.service_deferred_close()
        else:
            closed = self._native.close()
        if closed:
            self._finalize_close()
        return bool(closed)

    def _finalize_close(self):
        if self._closed:
            return
        self._closed = True
        self._close_pending = False
        with self._lock:
            handles = tuple(self._goal_handles.values())
            self._goal_handles.clear()
            self._goal_callback = None
            self._cancel_callback = None
            self._handle_accepted_callback = None
            self._execute_callback = None
        for handle in handles:
            with handle._lock:
                handle._destroyed = True
        if self.callback_group is not None:
            self.callback_group.discard_entity(self)
        discard = getattr(self._node, "_discard_direct_action_server", None)
        if discard is not None:
            discard(self)
        else:
            try:
                self._node._direct_cpp_action_servers.remove(self)
            except (AttributeError, ValueError):
                pass


__all__ = [
    "DEFAULT_INTERFACES",
    "DirectActionBinding",
    "DirectActionClient",
    "DirectActionInstallation",
    "DirectActionPlan",
    "DirectActionServer",
    "DirectClientGoalHandle",
    "DirectServerGoalHandle",
    "assert_early_imports",
    "install",
    "normalize_interfaces",
    "normalize_registered_interfaces",
    "prepare",
    "resolve_supported_type",
]
