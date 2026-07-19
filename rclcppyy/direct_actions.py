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


def _unsupported(reason: str):
    raise BackendUnavailableError(reason)


def _byte_values(values) -> bytes:
    return bytes(
        ord(value) if isinstance(value, str) else int(value)
        for value in values
    )


def _int8_value(value) -> int:
    return ord(value) if isinstance(value, str) else int(value)


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


class DirectActionClient:
    """Lean source-shape facade over a C++-owned ``rclcpp_action`` client."""

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
        if callback_group is not None:
            _unsupported("direct_cpp action clients do not support callback_group")
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

        self._node = node
        self._action_type = action_type
        self._action_name = name
        self._cpp_types = binding.cpp_types
        self._native = _runtime().require_session().create_native_action_client(
            node._require_node(),
            action_type,
            name,
            feedback_capacity=_FEEDBACK_CAPACITY,
        )
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
        self._node._discard_direct_action_client(self)
        return True


class DirectActionServer:
    """Fail-closed marker for the not-yet-implemented direct action server."""

    def __init__(self, *args, **kwargs):
        _unsupported("direct_cpp action servers do not yet have C++ authority")


__all__ = [
    "DEFAULT_INTERFACES",
    "DirectActionBinding",
    "DirectActionClient",
    "DirectActionInstallation",
    "DirectActionPlan",
    "DirectActionServer",
    "DirectClientGoalHandle",
    "assert_early_imports",
    "install",
    "normalize_interfaces",
    "normalize_registered_interfaces",
    "prepare",
    "resolve_supported_type",
]
