"""Focused unit coverage for the direct C++ ActionServer facade."""

from types import SimpleNamespace

import pytest

from rclcppyy import direct_actions
from rclcppyy.policy import BackendUnavailableError


class Goal:
    pass


class Feedback:
    pass


class Result:
    pass


class GoalId:
    def __init__(self, value=1):
        self.uuid = [value] + [0] * 15


class Action:
    Goal = Goal
    Feedback = Feedback
    Result = Result


class FakeGroup:
    _kind = "mutually_exclusive"

    def __init__(self):
        self.entities = set()
        self.native = object()

    def add_entity(self, entity):
        self.entities.add(entity)

    def discard_entity(self, entity):
        self.entities.discard(entity)


class FakeNativeServer:
    def __init__(self):
        self.source_id = "native-action-server-source"
        self.compile_result = {"cached": True, "so": "/tmp/native-action-server.so"}
        self.accepted = []
        self.states = {}
        self.calls = []
        self.errors = []
        self.closed = False
        self.close_pending = False

    def queue_goal(self, token, goal=None, goal_id=None, status=1):
        accepted = SimpleNamespace(
            token=token,
            goal=Goal() if goal is None else goal,
            goal_id=GoalId(token) if goal_id is None else goal_id,
            status=status,
        )
        self.accepted.append(accepted)
        self.states[token] = status
        return accepted

    def accepted_ready_count(self):
        return len(self.accepted)

    def take_accepted(self):
        return self.accepted.pop(0)

    def callback_error_ready(self):
        return bool(self.errors)

    def take_callback_error(self):
        return self.errors.pop(0)

    def status(self, token):
        return self.states[token]

    def is_active(self, token):
        return self.states[token] in (1, 2, 3)

    def is_canceling(self, token):
        return self.states[token] == 3

    def execute(self, token):
        self.calls.append(("execute", token))
        self.states[token] = 2

    def publish_feedback(self, token, feedback):
        self.calls.append(("feedback", token, feedback))

    def succeed(self, token, result):
        self.calls.append(("succeed", token, result))
        self.states[token] = 4

    def canceled(self, token, result):
        self.calls.append(("canceled", token, result))
        self.states[token] = 5

    def abort(self, token, result):
        self.calls.append(("abort", token, result))
        self.states[token] = 6

    def forget(self, token):
        self.calls.append(("forget", token))
        self.states.pop(token, None)
        return True

    def stats(self):
        return SimpleNamespace(python_message_conversions=0)

    def close(self):
        if self.closed:
            return False
        self.closed = True
        self.calls.append(("close",))
        return True

    def service_deferred_close(self):
        if not self.close_pending:
            return False
        self.close_pending = False
        return self.close()


class FakeSession:
    def __init__(self, native):
        self.native = native
        self.options = None

    def create_native_action_server(self, node, action_type, action_name, **options):
        self.options = {
            "node": node,
            "action_type": action_type,
            "action_name": action_name,
            **options,
        }
        return self.native


class FakeNode:
    def __init__(self):
        self._direct_cpp_node = object()
        self._direct_cpp_action_servers = []
        self.default_group = FakeGroup()
        self.executor = None

    def _require_node(self):
        return self._direct_cpp_node

    def _resolve_callback_group(self, callback_group):
        group = self.default_group if callback_group is None else callback_group
        return group, None if callback_group is None else group.native

    def _discard_direct_action_server(self, server):
        try:
            self._direct_cpp_action_servers.remove(server)
        except ValueError:
            pass


@pytest.fixture
def server_factory(monkeypatch):
    native = FakeNativeServer()
    session = FakeSession(native)
    runtime = SimpleNamespace(require_session=lambda: session)
    binding = SimpleNamespace(
        interface="example_actions/action/Action",
        cpp_types=SimpleNamespace(cpp_name="example_actions::action::Action"),
    )
    monkeypatch.setattr(
        direct_actions, "resolve_supported_type", lambda action_type: binding)
    monkeypatch.setattr(direct_actions, "record_decision", lambda *args, **kwargs: None)
    import rclcppyy.direct_cpp as direct_cpp

    monkeypatch.setattr(direct_cpp, "_runtime", lambda: runtime)

    def create(**options):
        node = options.pop("node", FakeNode())
        server = direct_actions.DirectActionServer(
            node, Action, "/action", **options)
        return server, node, native, session

    return create


def test_constructor_maps_only_jazzy_control_enums_and_registers_callbacks(
    server_factory,
):
    from rclpy.action import CancelResponse, GoalResponse

    accepted_goals = []
    canceled_handles = []
    server, node, native, session = server_factory(
        goal_callback=lambda goal: (
            accepted_goals.append(goal) or GoalResponse.ACCEPT),
        handle_accepted_callback=lambda handle: None,
        cancel_callback=lambda handle: (
            canceled_handles.append(handle) or CancelResponse.ACCEPT),
    )
    goal = Goal()
    assert session.options["goal_callback"](goal) is True
    assert accepted_goals == [goal]

    native.queue_goal(7, goal=goal)
    server._poll_ready()
    handle = server._goal_handles[7]
    assert session.options["cancel_callback"](7) is True
    assert canceled_handles == [handle]
    assert handle.request is goal
    assert type(handle.goal_id) is GoalId

    server.register_goal_callback(lambda _goal: "accept")
    assert session.options["goal_callback"](Goal()) is False
    assert type(server.take_callback_error()) is TypeError
    server.register_cancel_callback(lambda _handle: True)
    assert session.options["cancel_callback"](7) is False
    assert type(server.take_callback_error()) is TypeError
    assert node.default_group.entities == {server}


def test_default_accepted_execution_defers_terminal_until_exact_result(
    server_factory,
):
    observed = []
    feedback = Feedback()
    result = Result()

    def execute(handle):
        observed.extend((handle.request, handle.goal_id))
        handle.publish_feedback(feedback)
        handle.succeed()
        assert not any(call[0] == "succeed" for call in native.calls)
        return result

    server, _node, native, _session = server_factory(execute_callback=execute)
    accepted = native.queue_goal(11)
    server._poll_ready()
    handle = server._goal_handles[11]

    assert observed == [accepted.goal, accepted.goal_id]
    assert native.calls == [
        ("execute", 11),
        ("feedback", 11, feedback),
        ("succeed", 11, result),
    ]
    assert handle.status == 4
    assert not handle.is_active
    assert type(handle.request) is Goal
    assert type(handle.goal_id) is GoalId


@pytest.mark.parametrize(
    "mode", ("raises", "runtime_awaitable", "wrong_result", "no_terminal"))
def test_execute_failures_and_missing_terminal_complete_with_cpp_abort(
    server_factory, mode
):
    async def result_later():
        return Result()

    def execute(handle):
        if mode == "raises":
            raise RuntimeError("execute failed")
        if mode == "runtime_awaitable":
            return result_later()
        if mode == "wrong_result":
            handle.succeed()
            return object()
        return Result()

    server, _node, native, _session = server_factory(execute_callback=execute)
    native.queue_goal(13)
    server._poll_ready()
    handle = server._goal_handles[13]

    assert native.calls[0] == ("execute", 13)
    assert native.calls[1][0:2] == ("abort", 13)
    assert type(native.calls[1][2]) is Result
    assert handle.status == 6
    if mode == "no_terminal":
        assert not server.callback_error_ready()
    else:
        assert type(server.take_callback_error()) in (RuntimeError, TypeError)


def test_accepted_callback_error_executes_then_aborts(server_factory):
    def fail(_handle):
        raise RuntimeError("accepted callback failed")

    server, _node, native, _session = server_factory(
        handle_accepted_callback=fail)
    native.queue_goal(17)
    server._poll_ready()

    assert native.calls[0] == ("execute", 17)
    assert native.calls[1][0:2] == ("abort", 17)
    assert type(native.calls[1][2]) is Result
    assert type(server.take_callback_error()) is RuntimeError


def test_cancel_before_execute_skips_execute_transition_and_commits_canceled_result(
    server_factory,
):
    result = Result()

    def execute(handle):
        assert handle.is_cancel_requested
        handle.canceled()
        return result

    server, _node, native, _session = server_factory(
        execute_callback=execute,
        handle_accepted_callback=lambda handle: None,
    )
    native.queue_goal(19)
    server._poll_ready()
    handle = server._goal_handles[19]
    native.states[19] = 3
    handle.execute()

    assert native.calls == [("canceled", 19, result)]
    assert handle.status == 5
    assert not handle.is_cancel_requested


def test_manual_goal_handle_path_requires_exact_cpp_values(server_factory):
    server, _node, native, _session = server_factory(
        handle_accepted_callback=lambda handle: None)
    accepted = native.queue_goal(23)
    server._poll_ready()
    handle = server._goal_handles[23]

    with pytest.raises(BackendUnavailableError, match="response-less"):
        handle.succeed()
    with pytest.raises(TypeError, match="Feedback"):
        handle.publish_feedback(object())
    with pytest.raises(TypeError, match="Result"):
        handle.succeed(object())
    handle.executing()
    result = Result()
    handle.succeed(result)
    assert native.calls == [("execute", 23), ("succeed", 23, result)]

    retained_request = handle.request
    retained_id = handle.goal_id
    handle.destroy()
    handle.destroy()
    assert ("forget", 23) in native.calls
    assert retained_request is accepted.goal
    assert retained_id is accepted.goal_id
    with pytest.raises(RuntimeError, match="destroyed"):
        handle.publish_feedback(Feedback())
    assert server.close() is True
    assert server.close() is False
    assert retained_request is accepted.goal
    assert retained_id is accepted.goal_id


def test_runtime_awaitables_are_closed_rejected_and_contained(server_factory):
    async def decision():
        return None

    coroutine = decision()
    server, _node, _native, session = server_factory(
        goal_callback=lambda _goal: coroutine,
        handle_accepted_callback=lambda handle: None,
    )
    assert session.options["goal_callback"](Goal()) is False
    assert type(server.take_callback_error()) is TypeError
    assert coroutine.cr_frame is None


def test_coroutine_mte_reentrant_qos_and_unintegrated_node_fail_before_native(
    server_factory,
):
    from rclpy.qos import QoSProfile

    async def callback(_value):
        return None

    with pytest.raises(BackendUnavailableError, match="synchronous"):
        server_factory(execute_callback=callback)

    node = FakeNode()
    node.executor = SimpleNamespace(_kind="multi_threaded")
    with pytest.raises(BackendUnavailableError, match="MultiThreadedExecutor"):
        server_factory(node=node)

    reentrant = FakeGroup()
    reentrant._kind = "reentrant"
    with pytest.raises(BackendUnavailableError, match="mutually-exclusive"):
        server_factory(callback_group=reentrant)

    with pytest.raises(BackendUnavailableError, match="default QoS"):
        server_factory(feedback_pub_qos_profile=QoSProfile(depth=1))

    node = FakeNode()
    del node._direct_cpp_action_servers
    with pytest.raises(BackendUnavailableError, match="polling integration"):
        server_factory(node=node)


def test_close_inside_callback_is_deferred_and_cleanup_is_idempotent(server_factory):
    box = {}

    def decide(_goal):
        from rclpy.action import GoalResponse

        assert box["server"].close() is False
        return GoalResponse.REJECT

    server, node, native, session = server_factory(
        goal_callback=decide,
        handle_accepted_callback=lambda handle: None,
    )
    box["server"] = server
    assert session.options["goal_callback"](Goal()) is False
    assert server.close_pending
    assert not native.closed
    server._poll_ready()

    assert server.closed
    assert native.closed
    assert server not in node._direct_cpp_action_servers
    assert server not in node.default_group.entities
    assert server.close() is False
    assert server.destroy() is None
    with pytest.raises(RuntimeError, match="destroyed"):
        server.register_execute_callback(lambda handle: Result())
