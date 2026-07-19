"""Transactional generated-C++ action bindings for ``direct_cpp``."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import inspect
import math
import sys
import threading
from typing import Any

from rclcppyy._status import record_decision
from rclcppyy.policy import BackendUnavailableError


_IMPLEMENTATION_MODULE = "tf2_msgs.action._lookup_transform"
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
    action_type: type
    cpp_types: Any
    original_goal_type: type
    original_feedback_type: type
    original_result_type: type


class DirectActionInstallation:
    def __init__(self, replacements, binding, pythonizations=()):
        self._replacements = tuple(replacements)
        self._pythonizations = tuple(pythonizations)
        self.binding = binding
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


def assert_early_imports() -> None:
    stale = sorted(
        name for name in (_IMPLEMENTATION_MODULE, "rclpy.action")
        if name in sys.modules
    )
    if stale:
        raise RuntimeError(
            "direct_cpp must be enabled before importing supported actions: %s" %
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


def install() -> DirectActionInstallation:
    """Install the first action's actual C++ payload and response aliases."""
    global _ACTIVE_INSTALLATION
    from rclcpp_kit.native_action import resolve_cpp_action_type

    generated = importlib.import_module(_IMPLEMENTATION_MODULE)
    public = importlib.import_module("tf2_msgs.action")
    action_type = generated.LookupTransform
    if public.LookupTransform is not action_type:
        raise RuntimeError(
            "tf2_msgs.action.LookupTransform changed before direct installation")

    impl = action_type.Impl
    originals = {
        "goal": generated.LookupTransform_Goal,
        "feedback": generated.LookupTransform_Feedback,
        "result": generated.LookupTransform_Result,
        "feedback_message": generated.LookupTransform_FeedbackMessage,
        "goal_response": generated.LookupTransform_SendGoal_Response,
        "result_response": generated.LookupTransform_GetResult_Response,
    }
    if (
        action_type.Goal is not originals["goal"]
        or action_type.Feedback is not originals["feedback"]
        or action_type.Result is not originals["result"]
        or impl.FeedbackMessage is not originals["feedback_message"]
        or impl.SendGoalService.Response is not originals["goal_response"]
        or impl.GetResultService.Response is not originals["result_response"]
    ):
        raise RuntimeError("LookupTransform aliases changed before direct installation")

    cpp_types = resolve_cpp_action_type(action_type)
    binding = DirectActionBinding(
        action_type=action_type,
        cpp_types=cpp_types,
        original_goal_type=originals["goal"],
        original_feedback_type=originals["feedback"],
        original_result_type=originals["result"],
    )
    targets = (
        (generated, "LookupTransform_Goal", originals["goal"], cpp_types.goal),
        (generated, "LookupTransform_Feedback", originals["feedback"], cpp_types.feedback),
        (generated, "LookupTransform_Result", originals["result"], cpp_types.result),
        (action_type, "Goal", originals["goal"], cpp_types.goal),
        (action_type, "Feedback", originals["feedback"], cpp_types.feedback),
        (action_type, "Result", originals["result"], cpp_types.result),
        (
            generated,
            "LookupTransform_FeedbackMessage",
            originals["feedback_message"],
            cpp_types.feedback_message,
        ),
        (
            impl,
            "FeedbackMessage",
            originals["feedback_message"],
            cpp_types.feedback_message,
        ),
        (
            generated,
            "LookupTransform_SendGoal_Response",
            originals["goal_response"],
            cpp_types.goal_response,
        ),
        (
            public,
            "LookupTransform_SendGoal_Response",
            originals["goal_response"],
            cpp_types.goal_response,
        ),
        (
            impl.SendGoalService,
            "Response",
            originals["goal_response"],
            cpp_types.goal_response,
        ),
        (
            generated,
            "LookupTransform_GetResult_Response",
            originals["result_response"],
            cpp_types.result_response,
        ),
        (
            public,
            "LookupTransform_GetResult_Response",
            originals["result_response"],
            cpp_types.result_response,
        ),
        (
            impl.GetResultService,
            "Response",
            originals["result_response"],
            cpp_types.result_response,
        ),
    )
    pythonizations = []
    replacements = []
    try:
        for original_name, cpp_type in (
            ("goal", cpp_types.goal),
            ("feedback", cpp_types.feedback),
            ("result", cpp_types.result),
            ("feedback_message", cpp_types.feedback_message),
            ("goal_response", cpp_types.goal_response),
            ("result_response", cpp_types.result_response),
        ):
            pythonization = _pythonize_constructor(
                cpp_type, originals[original_name])
            if pythonization is not None:
                pythonizations.append(pythonization)
        for owner, name, original, replacement in targets:
            if getattr(owner, name) is not original:
                raise RuntimeError("LookupTransform aliases changed during installation")
            setattr(owner, name, replacement)
            replacements.append((owner, name, original, replacement))
    except Exception:
        DirectActionInstallation(
            replacements, binding, pythonizations).restore()
        raise
    installation = DirectActionInstallation(
        replacements, binding, pythonizations)
    _ACTIVE_INSTALLATION = installation
    return installation


def resolve_supported_type(action_type: Any):
    installation = _ACTIVE_INSTALLATION
    if installation is None:
        raise RuntimeError("direct_cpp action aliases are not installed")
    binding = installation.binding
    if action_type is not binding.action_type:
        raise TypeError(
            "direct_cpp currently supports only tf2_msgs.action.LookupTransform")
    cpp_types = binding.cpp_types
    if (
        action_type.Goal is not cpp_types.goal
        or action_type.Feedback is not cpp_types.feedback
        or action_type.Result is not cpp_types.result
    ):
        raise TypeError("direct_cpp LookupTransform C++ aliases are not active")
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
        self._poll_goal_responses()
        self._poll_feedback()
        self._poll_cancel_responses()
        self._poll_results()
        dropped = int(self._native.stats().feedback_dropped)
        if dropped != self._last_feedback_dropped:
            previous = self._last_feedback_dropped
            self._last_feedback_dropped = dropped
            raise RuntimeError(
                "direct_cpp action feedback queue overflowed: %d new messages" %
                (dropped - previous))

    def _poll_goal_responses(self):
        with self._lock:
            pending = tuple(self._goal_futures.items())
        for token, future in pending:
            if not self._native.goal_response_ready(token):
                continue
            goal_id = self._native.goal_id(token)
            response = self._native.goal_response(token)
            handle = DirectClientGoalHandle(self, token, goal_id, response)
            with self._lock:
                if self._goal_futures.get(token) is not future:
                    continue
                del self._goal_futures[token]
                orphaned = token in self._orphaned_goals or future.cancelled()
                self._orphaned_goals.discard(token)
                if handle.accepted and not orphaned:
                    self._handles[token] = handle
                else:
                    self._feedback_callbacks.pop(token, None)
            if orphaned:
                self._native.forget(token)
            elif not future.done() and not future.cancelled():
                future.set_result(handle)
                if not handle.accepted:
                    self._native.forget(token)

    def _poll_feedback(self):
        with self._lock:
            active = tuple(self._handles)
        for token in active:
            while self._native.feedback_ready(token):
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

    def _poll_cancel_responses(self):
        with self._lock:
            pending = tuple(self._cancel_futures.items())
        for token, future in pending:
            if not self._native.cancel_response_ready(token):
                continue
            response = self._native.take_cancel_response(token)
            with self._lock:
                if self._cancel_futures.get(token) is not future:
                    continue
                del self._cancel_futures[token]
                discard = token in self._discard_cancels or future.cancelled()
                self._discard_cancels.discard(token)
            if not discard and not future.done() and not future.cancelled():
                future.set_result(response)

    def _poll_results(self):
        with self._lock:
            pending = tuple(self._result_futures.items())
        for token, future in pending:
            if not self._native.result_ready(token):
                continue
            response = self._native.take_result_response(token)
            with self._lock:
                if self._result_futures.get(token) is not future:
                    continue
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
    "DirectActionBinding",
    "DirectActionClient",
    "DirectActionInstallation",
    "DirectActionServer",
    "DirectClientGoalHandle",
    "assert_early_imports",
    "install",
    "resolve_supported_type",
]
