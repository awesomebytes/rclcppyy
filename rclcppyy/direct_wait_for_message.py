"""Source-compatible ``rclpy.wait_for_message`` over direct C++ subscriptions."""

from __future__ import annotations

from functools import wraps
import threading
import time

from rclcppyy.policy import BackendUnavailableError


_WAIT_SLICE_SECONDS = 0.1


def _unsupported(reason):
    raise BackendUnavailableError(reason)


def prepare(original):
    """Return a direct implementation retaining the installed public contract."""

    @wraps(original)
    def wait_for_message(
        msg_type,
        node,
        topic: str,
        *,
        qos_profile=1,
        time_to_wait=-1,
    ):
        from rclpy.utilities import timeout_sec_to_nsec
        from rclcppyy.direct_cpp import DirectNode
        from rclcppyy.direct_executors import DirectSingleThreadedExecutor

        if not isinstance(node, DirectNode):
            _unsupported("direct_cpp wait_for_message requires a direct_cpp Node")
        timeout_nsec = timeout_sec_to_nsec(time_to_wait)
        context = node.context
        executor = node.executor
        owns_executor = executor is None
        if owns_executor:
            executor = DirectSingleThreadedExecutor(context=context)
        elif (
            executor.is_spinning and
            executor._spin_thread_id == threading.get_ident()
        ):
            _unsupported(
                "direct_cpp wait_for_message cannot recursively wait from its "
                "single-threaded executor callback")

        was_parked = getattr(executor, "_parked_node", None) is node
        added_node = False
        subscription = None
        received = []
        ready = threading.Event()

        def receive(message):
            received.append(message)
            ready.set()

        try:
            if not executor.is_spinning:
                added_node = executor.add_node(node)
            subscription = node.create_subscription(
                msg_type, topic, receive, qos_profile=qos_profile)
            deadline = (
                None if timeout_nsec < 0
                else time.monotonic() + timeout_nsec / 1_000_000_000
            )
            first_wait = True
            while context.ok() and not ready.is_set():
                remaining = (
                    None if deadline is None
                    else max(0.0, deadline - time.monotonic())
                )
                if remaining == 0.0 and not first_wait:
                    break
                wait_slice = (
                    _WAIT_SLICE_SECONDS if remaining is None
                    else min(_WAIT_SLICE_SECONDS, remaining)
                )
                if executor.is_spinning:
                    ready.wait(wait_slice)
                else:
                    try:
                        executor.spin_once(timeout_sec=wait_slice)
                    except RuntimeError as exception:
                        if str(exception) != "Executor is already spinning":
                            raise
                        ready.wait(wait_slice)
                first_wait = False
            if received:
                return True, received[0]
            return False, None
        finally:
            if subscription is not None:
                node.destroy_subscription(subscription)
            if added_node:
                if was_parked:
                    executor.park_node(node)
                else:
                    executor.remove_node(node)
            if owns_executor:
                executor.shutdown()

    return wait_for_message


__all__ = ["prepare"]
