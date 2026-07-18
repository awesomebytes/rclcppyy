"""Broad stock-API application probe for one isolated backend mode."""

import argparse
import sys
import threading
import time
import traceback

from .protocol import encode_result, verify_backend_expectations


TIMEOUT_S = 5.0


def _qualified_type(value):
    return "%s.%s" % (type(value).__module__, type(value).__qualname__)


def _spin_until(node, predicate, executor, timeout=TIMEOUT_S):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        import rclpy
        rclpy.spin_once(node, executor=executor, timeout_sec=0.05)
    if not predicate():
        raise AssertionError("condition did not become true before timeout")


def _spin_executor_until(executor, predicate, timeout=TIMEOUT_S):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    if not predicate():
        raise AssertionError("executor condition did not become true before timeout")


def _call_service(client, request, executor):
    future = client.call_async(request)
    _spin_executor_until(executor, future.done)
    result = future.result()
    if result is None:
        raise AssertionError("service call completed without a response")
    return result


def _transition_descriptions(items):
    return [
        {
            "transition": [item.transition.id, item.transition.label],
            "start": [item.start_state.id, item.start_state.label],
            "goal": [item.goal_state.id, item.goal_state.label],
        }
        for item in items
    ]


def _run(mode):
    import rclpy
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from lifecycle_msgs.srv import GetAvailableStates
    from lifecycle_msgs.srv import GetAvailableTransitions
    from lifecycle_msgs.srv import GetState
    from rcl_interfaces.msg import SetParametersResult
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
    from rclpy.context import Context
    from rclpy.event_handler import PublisherEventCallbacks, SubscriptionEventCallbacks
    from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
    from rclpy.lifecycle import LifecycleNode, LifecyclePublisher, TransitionCallbackReturn
    from rclpy.node import Node as StockNode
    from rclpy.parameter import Parameter
    from rclpy.publisher import Publisher
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.qos_overriding_options import QoSOverridingOptions
    from rclpy.task import Future
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from std_srvs.srv import SetBool

    class CorpusNode(StockNode):
        pass

    class CorpusPublisher(Publisher):
        pass

    class CorpusLifecycleNode(LifecycleNode):
        def __init__(self, *args, **kwargs):
            self.transition_callbacks = []
            super().__init__(*args, **kwargs)

        def _record_transition(self, name, state):
            self.transition_callbacks.append({
                "callback": name,
                "previous": [state.state_id, state.label],
            })

        def on_configure(self, state):
            self._record_transition("configure", state)
            return super().on_configure(state)

        def on_activate(self, state):
            self._record_transition("activate", state)
            return super().on_activate(state)

        def on_deactivate(self, state):
            self._record_transition("deactivate", state)
            return super().on_deactivate(state)

        def on_cleanup(self, state):
            self._record_transition("cleanup", state)
            return super().on_cleanup(state)

        def on_shutdown(self, state):
            self._record_transition("shutdown", state)
            return super().on_shutdown(state)

    class FailingLifecycleNode(LifecycleNode):
        def __init__(self, *args, **kwargs):
            self.configure_previous_state = None
            super().__init__(*args, **kwargs)

        def on_configure(self, state):
            self.configure_previous_state = [state.state_id, state.label]
            return TransitionCallbackReturn.FAILURE

    backend_module = None
    if mode == "activated":
        import rclcppyy
        backend_module = rclcppyy
        rclcppyy.enable_cpp_acceleration()

    context = Context()
    context.init(args=[])
    cleanup = {
        "context_shutdown": False,
        "nodes_destroyed": False,
        "executor_shutdown": False,
        "spin_context_shutdown": False,
    }
    node = None
    factory_node = None
    executor = SingleThreadedExecutor(context=context)
    observations = {}
    try:
        node = CorpusNode(
            "diff_application_subclass",
            context=context,
            start_parameter_services=True,
        )
        factory_node = rclpy.create_node(
            "diff_application_factory",
            context=context,
            start_parameter_services=False,
        )
        observations["node_construction"] = {
            "subclass_type": _qualified_type(node),
            "subclass_exact": type(node) is CorpusNode,
            "subclass_base_preserved": CorpusNode.__bases__ == (StockNode,),
            "factory_type": _qualified_type(factory_node),
            "factory_exact_stock": type(factory_node) is StockNode,
            "requested_context_preserved": (
                node.context is context and factory_node.context is context),
            "default_context_ok": rclpy.get_default_context().ok(),
        }

        group = MutuallyExclusiveCallbackGroup()
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=7,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        pub = node.create_publisher(
            String,
            "options_topic",
            qos,
            callback_group=group,
            event_callbacks=PublisherEventCallbacks(use_default_callbacks=False),
            qos_overriding_options=QoSOverridingOptions([]),
            publisher_class=CorpusPublisher,
        )
        sub = node.create_subscription(
            String,
            "options_topic",
            lambda _message: None,
            qos,
            callback_group=group,
            event_callbacks=SubscriptionEventCallbacks(use_default_callbacks=False),
            qos_overriding_options=QoSOverridingOptions([]),
            raw=False,
            content_filter_options=None,
        )
        observations["qos_and_options"] = {
            "publisher_type": _qualified_type(pub),
            "subscription_type": _qualified_type(sub),
            "publisher_depth": pub.qos_profile.depth,
            "subscription_depth": sub.qos_profile.depth,
            "publisher_reliability": pub.qos_profile.reliability.name,
            "subscription_reliability": sub.qos_profile.reliability.name,
            "publisher_durability": pub.qos_profile.durability.name,
            "subscription_durability": sub.qos_profile.durability.name,
            "callback_group_has_publisher": group.has_entity(pub),
            "callback_group_has_subscription": group.has_entity(sub),
        }
        observations["entity_destruction"] = {
            "publisher_first": node.destroy_publisher(pub),
            "publisher_second": node.destroy_publisher(pub),
            "subscription_first": node.destroy_subscription(sub),
            "subscription_second": node.destroy_subscription(sub),
        }

        timer_calls = []

        def on_timer():
            timer_calls.append("timer")
            timer.cancel()

        timer = node.create_timer(
            0.01, on_timer, callback_group=group, autostart=False)
        timer.reset()
        _spin_until(node, lambda: bool(timer_calls), executor)
        observations["timer"] = {
            "calls": timer_calls,
            "period_ns": timer.timer_period_ns,
            "canceled_after_callback": timer.is_canceled(),
            "callback_group_has_timer": group.has_entity(timer),
            "destroyed": node.destroy_timer(timer),
        }

        guard_calls = []
        guard = node.create_guard_condition(
            lambda: guard_calls.append("guard"), callback_group=group)
        guard.trigger()
        _spin_until(node, lambda: bool(guard_calls), executor)
        observations["guard_condition"] = {
            "calls": guard_calls,
            "callback_group_has_guard": group.has_entity(guard),
            "destroyed": node.destroy_guard_condition(guard),
        }

        future_callbacks = []
        standalone_future = Future(executor=executor)
        standalone_future.add_done_callback(
            lambda completed: future_callbacks.append(completed.result()))
        standalone_future.set_result("future-result")
        executor.spin_once(timeout_sec=0.1)
        exception_future = Future()
        exception_future.set_exception(ValueError("future-error"))
        try:
            exception_future.result()
        except ValueError as exc:
            future_exception = type(exc).__name__
        else:
            raise AssertionError("Future.result did not propagate its exception")
        canceled_future = Future()
        cancel_result = canceled_future.cancel()
        observations["future"] = {
            "done": standalone_future.done(),
            "result": standalone_future.result(),
            "callbacks": future_callbacks,
            "exception_type": future_exception,
            "cancel_return": cancel_result,
            "canceled": canceled_future.cancelled(),
            "canceled_done": canceled_future.done(),
        }

        def set_bool(request, response):
            response.success = request.data
            response.message = "enabled" if request.data else "disabled"
            return response

        service = node.create_service(
            SetBool, "set_bool", set_bool, callback_group=group)
        client = node.create_client(
            SetBool, "set_bool", callback_group=group)
        if not client.wait_for_service(timeout_sec=TIMEOUT_S):
            raise AssertionError("local service was not discovered")
        request = SetBool.Request(data=True)
        response_future = client.call_async(request)
        rclpy.spin_until_future_complete(
            node, response_future, executor=executor, timeout_sec=TIMEOUT_S)
        response = response_future.result()
        if response is None:
            raise AssertionError("service future did not produce a response")
        observations["service_client"] = {
            "success": response.success,
            "message": response.message,
            "future_done": response_future.done(),
            "service_type": _qualified_type(service),
            "client_type": _qualified_type(client),
            "destroy_client": node.destroy_client(client),
            "destroy_service": node.destroy_service(service),
        }

        parameter_callbacks = []

        def accept_parameters(parameters):
            parameter_callbacks.append([
                {"name": parameter.name, "value": parameter.value}
                for parameter in parameters
            ])
            return SetParametersResult(successful=True)

        node.declare_parameter("corpus_value", 1)
        node.add_on_set_parameters_callback(accept_parameters)
        set_results = node.set_parameters([Parameter("corpus_value", value=7)])
        observations["parameters"] = {
            "value": node.get_parameter("corpus_value").value,
            "successful": [result.successful for result in set_results],
            "callbacks": parameter_callbacks,
            "listed": "corpus_value" in node.list_parameters([], 1).names,
        }

        lifecycle_node = None
        failing_lifecycle_node = None
        lifecycle_publisher = None
        lifecycle_subscription = None
        lifecycle_clients = {}
        lifecycle_executor_nodes = []
        try:
            lifecycle_node = CorpusLifecycleNode(
                "diff_application_lifecycle",
                context=context,
                start_parameter_services=False,
            )
            failing_lifecycle_node = FailingLifecycleNode(
                "diff_application_lifecycle_failure",
                context=context,
                enable_communication_interface=False,
                start_parameter_services=False,
            )
            lifecycle_messages = []
            lifecycle_publisher = lifecycle_node.create_lifecycle_publisher(
                String, "lifecycle_topic", 10)
            lifecycle_subscription = factory_node.create_subscription(
                String,
                "lifecycle_topic",
                lambda message: lifecycle_messages.append(message.data),
                10,
            )
            for lifecycle_executor_node in (lifecycle_node, factory_node):
                if executor.add_node(lifecycle_executor_node):
                    lifecycle_executor_nodes.append(lifecycle_executor_node)

            service_types = {
                "change_state": ChangeState,
                "get_state": GetState,
                "get_available_states": GetAvailableStates,
                "get_available_transitions": GetAvailableTransitions,
                "get_transition_graph": GetAvailableTransitions,
            }
            lifecycle_clients = {
                name: factory_node.create_client(
                    service_type,
                    "/diff_application_lifecycle/" + name,
                )
                for name, service_type in service_types.items()
            }
            if not all(
                client.wait_for_service(timeout_sec=TIMEOUT_S)
                for client in lifecycle_clients.values()
            ):
                raise AssertionError("lifecycle communication services were not discovered")

            initial_state = _call_service(
                lifecycle_clients["get_state"], GetState.Request(), executor)
            available_states = _call_service(
                lifecycle_clients["get_available_states"],
                GetAvailableStates.Request(),
                executor,
            )
            available_transitions = _call_service(
                lifecycle_clients["get_available_transitions"],
                GetAvailableTransitions.Request(),
                executor,
            )
            transition_graph = _call_service(
                lifecycle_clients["get_transition_graph"],
                GetAvailableTransitions.Request(),
                executor,
            )

            _spin_executor_until(
                executor,
                lambda: lifecycle_publisher.get_subscription_count() > 0,
            )
            lifecycle_publisher.publish(String(data="before-activation"))
            executor.spin_once(timeout_sec=0.1)
            before_activation_messages = list(lifecycle_messages)

            configure_result = lifecycle_node.trigger_configure()
            inactive_state = _call_service(
                lifecycle_clients["get_state"], GetState.Request(), executor)

            activate_request = ChangeState.Request()
            activate_request.transition.id = Transition.TRANSITION_ACTIVATE
            activate_response = _call_service(
                lifecycle_clients["change_state"], activate_request, executor)
            active_state = _call_service(
                lifecycle_clients["get_state"], GetState.Request(), executor)
            lifecycle_publisher.publish(String(data="while-active"))
            _spin_executor_until(
                executor,
                lambda: "while-active" in lifecycle_messages,
            )
            activated_during_active = lifecycle_publisher.is_activated

            deactivate_request = ChangeState.Request()
            deactivate_request.transition.id = Transition.TRANSITION_DEACTIVATE
            deactivate_response = _call_service(
                lifecycle_clients["change_state"], deactivate_request, executor)
            deactivated_state = _call_service(
                lifecycle_clients["get_state"], GetState.Request(), executor)
            lifecycle_publisher.publish(String(data="after-deactivation"))
            executor.spin_once(timeout_sec=0.1)

            cleanup_result = lifecycle_node.trigger_cleanup()
            unconfigured_state = _call_service(
                lifecycle_clients["get_state"], GetState.Request(), executor)
            shutdown_result = lifecycle_node.trigger_shutdown()
            finalized_state = _call_service(
                lifecycle_clients["get_state"], GetState.Request(), executor)

            failure_result = failing_lifecycle_node.trigger_configure()
            failure_shutdown_result = failing_lifecycle_node.trigger_shutdown()

            expected_callbacks = [
                ("configure", "unconfigured"),
                ("activate", "inactive"),
                ("deactivate", "active"),
                ("cleanup", "inactive"),
                ("shutdown", "unconfigured"),
            ]
            actual_callbacks = [
                (item["callback"], item["previous"][1])
                for item in lifecycle_node.transition_callbacks
            ]
            lifecycle_invariants = (
                initial_state.current_state.label == "unconfigured"
                and bool(available_states.available_states)
                and bool(available_transitions.available_transitions)
                and bool(transition_graph.available_transitions)
                and before_activation_messages == []
                and configure_result == TransitionCallbackReturn.SUCCESS
                and inactive_state.current_state.label == "inactive"
                and activate_response.success
                and active_state.current_state.label == "active"
                and activated_during_active
                and deactivate_response.success
                and deactivated_state.current_state.label == "inactive"
                and "before-activation" not in lifecycle_messages
                and "while-active" in lifecycle_messages
                and "after-deactivation" not in lifecycle_messages
                and not lifecycle_publisher.is_activated
                and cleanup_result == TransitionCallbackReturn.SUCCESS
                and unconfigured_state.current_state.label == "unconfigured"
                and shutdown_result == TransitionCallbackReturn.SUCCESS
                and finalized_state.current_state.label == "finalized"
                and actual_callbacks == expected_callbacks
                and failure_result == TransitionCallbackReturn.FAILURE
                and failing_lifecycle_node.configure_previous_state[1] == "unconfigured"
                and failure_shutdown_result == TransitionCallbackReturn.SUCCESS
            )
            if not lifecycle_invariants:
                raise AssertionError("lifecycle contract invariants were not satisfied")

            observations["lifecycle_identity"] = {
                "node_type": _qualified_type(lifecycle_node),
                "node_exact": type(lifecycle_node) is CorpusLifecycleNode,
                "node_base_preserved": (
                    CorpusLifecycleNode.__bases__ == (LifecycleNode,)),
                "context_preserved": lifecycle_node.context is context,
                "publisher_type": _qualified_type(lifecycle_publisher),
                "publisher_exact": type(lifecycle_publisher) is LifecyclePublisher,
            }
            observations["lifecycle_services"] = {
                "initial_state": [
                    initial_state.current_state.id,
                    initial_state.current_state.label,
                ],
                "available_states": [
                    [state.id, state.label]
                    for state in available_states.available_states
                ],
                "available_transitions": _transition_descriptions(
                    available_transitions.available_transitions),
                "transition_graph": _transition_descriptions(
                    transition_graph.available_transitions),
                "service_names": sorted(
                    client.srv_name for client in lifecycle_clients.values()),
            }
            observations["lifecycle_transitions"] = {
                "configure": configure_result.to_label(),
                "inactive_state": [
                    inactive_state.current_state.id,
                    inactive_state.current_state.label,
                ],
                "activate_service": activate_response.success,
                "active_state": [
                    active_state.current_state.id,
                    active_state.current_state.label,
                ],
                "deactivate_service": deactivate_response.success,
                "deactivated_state": [
                    deactivated_state.current_state.id,
                    deactivated_state.current_state.label,
                ],
                "cleanup": cleanup_result.to_label(),
                "unconfigured_state": [
                    unconfigured_state.current_state.id,
                    unconfigured_state.current_state.label,
                ],
                "shutdown": shutdown_result.to_label(),
                "finalized_state": [
                    finalized_state.current_state.id,
                    finalized_state.current_state.label,
                ],
                "callbacks": lifecycle_node.transition_callbacks,
            }
            observations["lifecycle_publisher"] = {
                "before_activation": before_activation_messages,
                "received": lifecycle_messages,
                "activated_during_active": activated_during_active,
                "activated_after_deactivate": lifecycle_publisher.is_activated,
            }
            observations["lifecycle_failure"] = {
                "configure": failure_result.to_label(),
                "previous_state": failing_lifecycle_node.configure_previous_state,
                "shutdown": failure_shutdown_result.to_label(),
            }

            lifecycle_destroyed = lifecycle_node.destroy_lifecycle_publisher(
                lifecycle_publisher)
            lifecycle_publisher = None
            subscription_destroyed = factory_node.destroy_subscription(
                lifecycle_subscription)
            lifecycle_subscription = None
            client_destruction = {
                name: factory_node.destroy_client(client)
                for name, client in lifecycle_clients.items()
            }
            lifecycle_clients = {}
            if not (
                lifecycle_destroyed
                and subscription_destroyed
                and all(client_destruction.values())
            ):
                raise AssertionError("lifecycle entity teardown was incomplete")
            observations["lifecycle_teardown"] = {
                "publisher": lifecycle_destroyed,
                "subscription": subscription_destroyed,
                "clients": client_destruction,
            }
        finally:
            for executor_node in lifecycle_executor_nodes:
                executor.remove_node(executor_node)
            for lifecycle_client in lifecycle_clients.values():
                factory_node.destroy_client(lifecycle_client)
            if lifecycle_subscription is not None:
                factory_node.destroy_subscription(lifecycle_subscription)
            if lifecycle_node is not None:
                if lifecycle_publisher is not None:
                    lifecycle_node.destroy_lifecycle_publisher(lifecycle_publisher)
                lifecycle_node.destroy_node()
            if failing_lifecycle_node is not None:
                failing_lifecycle_node.destroy_node()

        exception_name = None

        class CorpusCallbackError(RuntimeError):
            pass

        def raise_from_callback():
            raise CorpusCallbackError("callback-contract")

        exception_timer = node.create_timer(0.01, raise_from_callback)
        try:
            deadline = time.monotonic() + TIMEOUT_S
            while exception_name is None and time.monotonic() < deadline:
                try:
                    rclpy.spin_once(node, executor=executor, timeout_sec=0.1)
                except CorpusCallbackError as exc:
                    exception_name = type(exc).__name__
            if exception_name is None:
                raise AssertionError("timer callback exception was not propagated")
        finally:
            node.destroy_timer(exception_timer)
        observations["callback_exception"] = {
            "propagated_type": exception_name,
        }

        mte_node = rclpy.create_node(
            "diff_application_mte", context=context,
            start_parameter_services=False)
        mte_group = ReentrantCallbackGroup()
        mte_executor = MultiThreadedExecutor(num_threads=2, context=context)
        mte_calls = []
        mte_timer = None
        spin_thread = None
        try:
            def on_mte_timer():
                mte_calls.append("mte")
                mte_timer.cancel()

            mte_timer = mte_node.create_timer(
                0.01, on_mte_timer, callback_group=mte_group)
            mte_executor.add_node(mte_node)
            spin_thread = threading.Thread(target=mte_executor.spin, daemon=True)
            spin_thread.start()
            deadline = time.monotonic() + TIMEOUT_S
            while not mte_calls and time.monotonic() < deadline:
                time.sleep(0.01)
            if mte_calls != ["mte"]:
                raise AssertionError("MultiThreadedExecutor timer did not run")
            observations["multi_threaded_executor"] = {
                "calls": mte_calls,
                "callback_group_has_timer": mte_group.has_entity(mte_timer),
            }
        finally:
            mte_executor.shutdown(timeout_sec=1.0)
            if spin_thread is not None:
                spin_thread.join(timeout=1.0)
                if spin_thread.is_alive():
                    raise AssertionError("MultiThreadedExecutor spin thread did not stop")
            mte_executor.remove_node(mte_node)
            if mte_timer is not None:
                mte_node.destroy_timer(mte_timer)
            mte_node.destroy_node()

        sim_node = rclpy.create_node(
            "diff_application_sim_time",
            context=context,
            parameter_overrides=[Parameter("use_sim_time", value=True)],
            start_parameter_services=False,
        )
        clock_pub = factory_node.create_publisher(
            Clock,
            "/clock",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        clock_message = Clock()
        clock_message.clock.sec = 12
        clock_message.clock.nanosec = 345
        expected_time = 12_000_000_345
        clock_pub.publish(clock_message)
        _spin_until(
            sim_node,
            lambda: sim_node.get_clock().now().nanoseconds == expected_time,
            executor,
        )
        observations["sim_time"] = {
            "active": sim_node.get_clock().ros_time_is_active,
            "nanoseconds": sim_node.get_clock().now().nanoseconds,
            "destroy_clock_publisher": factory_node.destroy_publisher(clock_pub),
        }
        sim_node.destroy_node()

        spin_context = Context()
        spin_context.init(args=[])
        spin_node = None
        spin_executor = SingleThreadedExecutor(context=spin_context)
        spin_calls = []
        try:
            spin_node = rclpy.create_node(
                "diff_application_spin", context=spin_context,
                start_parameter_services=False)

            def stop_spin():
                spin_calls.append("spin")
                spin_context.try_shutdown()

            spin_node.create_timer(0.01, stop_spin)
            rclpy.spin(spin_node, executor=spin_executor)
            observations["spin"] = {
                "calls": spin_calls,
                "context_ok_after_spin": spin_context.ok(),
            }
        finally:
            if spin_node is not None:
                spin_node.destroy_node()
            spin_executor.shutdown(timeout_sec=1.0)
            if spin_context.ok():
                spin_context.try_shutdown()
            cleanup["spin_context_shutdown"] = not spin_context.ok()
    finally:
        executor.shutdown(timeout_sec=1.0)
        cleanup["executor_shutdown"] = True
        if factory_node is not None:
            factory_node.destroy_node()
        if node is not None:
            node.destroy_node()
        cleanup["nodes_destroyed"] = True
        if context.ok():
            context.try_shutdown()
        cleanup["context_shutdown"] = not context.ok()

    expectations = []
    backend_status = None
    if backend_module is not None:
        expectations = [
            {
                "kind": "operations",
                "backend": "cpp",
                "minimum": 1,
                "metadata": {"operation": "enable_cpp_acceleration"},
            },
            {
                "kind": "nodes",
                "backend": "python",
                "minimum": 1,
                "metadata": {"profile": "compatible"},
            },
            {
                "kind": "entities",
                "backend": "cpp",
                "minimum": 1,
                "metadata": {"entity_type": "publisher", "profile": "compatible"},
            },
            {
                "kind": "entities",
                "backend": "python",
                "minimum": 1,
                "metadata": {"entity_type": "subscription", "profile": "compatible"},
            },
            {
                "kind": "entities",
                "backend": "python",
                "minimum": 1,
                "metadata": {"entity_type": "timer", "profile": "compatible"},
            },
            {
                "kind": "entities",
                "backend": "python",
                "minimum": 1,
                "metadata": {"entity_type": "service", "profile": "compatible"},
            },
            {
                "kind": "entities",
                "backend": "python",
                "minimum": 1,
                "metadata": {"entity_type": "client", "profile": "compatible"},
            },
            {
                "kind": "entities",
                "backend": "python",
                "minimum": 1,
                "metadata": {"entity_type": "guard_condition", "profile": "compatible"},
            },
            {
                "kind": "operations",
                "backend": "python",
                "minimum": 1,
                "metadata": {"operation": "set_parameters", "profile": "compatible"},
            },
            {
                "kind": "operations",
                "backend": "python",
                "minimum": 1,
                "metadata": {"operation": "future", "profile": "compatible"},
            },
            {
                "kind": "operations",
                "backend": "python",
                "minimum": 1,
                "metadata": {"operation": "spin", "profile": "compatible"},
            },
            {
                "kind": "operations",
                "backend": "python",
                "minimum": 1,
                "metadata": {"operation": "spin_once", "profile": "compatible"},
            },
            {
                "kind": "operations",
                "backend": "python",
                "minimum": 1,
                "metadata": {"operation": "callback_exception", "profile": "compatible"},
            },
            {
                "kind": "operations",
                "backend": "python",
                "minimum": 1,
                "metadata": {"operation": "multi_threaded_spin", "profile": "compatible"},
            },
            {
                "kind": "entities",
                "backend": "python",
                "minimum": 1,
                "metadata": {
                    "entity_type": "lifecycle_node",
                    "communication_interface": True,
                    "profile": "compatible",
                },
            },
            {
                "kind": "entities",
                "backend": "python",
                "minimum": 1,
                "metadata": {
                    "entity_type": "lifecycle_node",
                    "communication_interface": False,
                    "profile": "compatible",
                },
            },
            {
                "kind": "entities",
                "backend": "cpp",
                "minimum": 1,
                "metadata": {
                    "entity_type": "publisher",
                    "topic": "/lifecycle_topic",
                    "profile": "compatible",
                },
            },
        ]
        backend_status = backend_module.status()
    backend_verified = (
        not expectations or not verify_backend_expectations(backend_status, expectations))
    return observations, expectations, backend_status, backend_verified, cleanup


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", required=True, choices=("stock", "activated"))
    args = parser.parse_args(argv)
    try:
        observations, expectations, status, verified, cleanup = _run(args.mode)
        result = {
            "schema": "rclcppyy.differential/v1",
            "scenario": "application_contract",
            "mode": args.mode,
            "outcome": "pass",
            "observations": observations,
            "backend_expectations": expectations,
            "backend_status": status,
            "backend_verified": verified,
            "cleanup": cleanup,
            "error": None,
        }
    except BaseException as exc:
        result = {
            "schema": "rclcppyy.differential/v1",
            "scenario": "application_contract",
            "mode": args.mode,
            "outcome": "error",
            "observations": {},
            "backend_expectations": [],
            "backend_status": None,
            "backend_verified": False,
            "cleanup": {},
            "error": {
                "type": type(exc).__name__,
                "message": str(exc),
                "traceback": traceback.format_exc(),
            },
        }
    print(encode_result(result), flush=True)
    return 0 if result["outcome"] == "pass" and result["backend_verified"] else 1


if __name__ == "__main__":
    sys.exit(main())
