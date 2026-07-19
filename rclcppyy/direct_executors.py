"""rclpy-shaped executors backed by the active native ``rclcpp`` context."""

from __future__ import annotations

import math
import threading
import time
from typing import Any

import cppyy

from rclcppyy.policy import BackendUnavailableError


def _runtime():
    from rclcppyy.direct_cpp import _runtime as active_runtime

    return active_runtime()


def _unsupported(reason: str):
    raise BackendUnavailableError(reason)


def _timeout_seconds(timeout_sec: Any) -> float | None:
    value = getattr(timeout_sec, "timeout", timeout_sec)
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError("timeout_sec must be a number or None")
    value = float(value)
    if not math.isfinite(value):
        if value < 0:
            return None
        raise ValueError("timeout_sec must be finite, negative, or None")
    return value


class DirectExecutor:
    """Base facade for one executor owned by the active ``NativeSession``."""

    _kind = "single_threaded"
    _threads = 1

    def __init__(self, *, context=None, num_threads=None) -> None:
        runtime = _runtime()
        if context is not None and context is not runtime.context:
            _unsupported("direct_cpp executors require the active direct context")
        if self._kind == "multi_threaded":
            _unsupported(
                "direct_cpp MultiThreadedExecutor requires native Python callback "
                "concurrency and exception propagation proof"
            )
        if num_threads is not None:
            _unsupported("direct_cpp SingleThreadedExecutor does not accept num_threads")
        self._runtime = runtime
        self._context = runtime.context
        self._native = runtime.require_session().create_executor(self._kind)
        # A blocking native wait must not monopolize the interpreter. cppyy
        # reacquires the GIL when an rclcpp callback enters its Python target.
        self._native.spin_once.__release_gil__ = True
        self._nodes: list[Any] = []
        self._nodes_lock = threading.RLock()
        self._spin_lock = threading.Lock()
        self._state_lock = threading.RLock()
        self._is_shutdown = False
        self._is_spinning = False
        runtime.register_executor(self)

    @property
    def context(self):
        return self._context

    @property
    def is_spinning(self) -> bool:
        with self._state_lock:
            return self._is_spinning

    @property
    def native_executor(self):
        if self._is_shutdown:
            raise RuntimeError("direct_cpp executor is shut down")
        return self._native

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.shutdown()

    def _validate_node(self, node):
        from rclcppyy.direct_cpp import DirectNode

        if not isinstance(node, DirectNode):
            raise TypeError("direct_cpp executor requires a direct_cpp Node")
        if node.context is not self._context:
            _unsupported("direct_cpp executor and node contexts do not match")
        native = node._require_node()
        if node not in self._runtime.nodes:
            raise ValueError("node is not owned by the active direct_cpp context")
        return native

    def add_node(self, node) -> bool:
        """Transfer a direct node to this exact native executor."""
        if self._is_shutdown:
            return False
        native = self._validate_node(node)
        with self._runtime.membership_lock:
            with self._nodes_lock:
                if node in self._nodes:
                    return False
            current = node.executor
            if current is not None and current is not self:
                current.remove_node(node)
            self._native.add_node(native)
            with self._nodes_lock:
                self._nodes.append(node)
            node._set_direct_executor(self)
            return True

    def remove_node(self, node) -> None:
        with self._runtime.membership_lock:
            with self._nodes_lock:
                try:
                    self._nodes.remove(node)
                except ValueError:
                    return
            native = getattr(node, "_direct_cpp_node", None)
            if native is not None and not self._runtime.session.closed:
                self._native.remove_node(native)

    def get_nodes(self) -> list[Any]:
        with self._nodes_lock:
            return list(self._nodes)

    def wake(self) -> None:
        if not self._is_shutdown:
            self._native.cancel()

    def create_future(self):
        from rclpy.task import Future

        return Future(executor=self)

    def create_task(self, callback, *args, **kwargs):
        _unsupported("direct_cpp executors do not yet support create_task()")

    def _enter_spin(self) -> None:
        if not self._spin_lock.acquire(blocking=False):
            raise RuntimeError("Executor is already spinning")
        with self._state_lock:
            if self._is_spinning:
                self._spin_lock.release()
                raise RuntimeError("Executor is already spinning")
            self._is_spinning = True

    def _exit_spin(self) -> None:
        with self._state_lock:
            self._is_spinning = False
        self._spin_lock.release()

    def _spin_once_impl(self, timeout_sec=None) -> None:
        if self._is_shutdown or not self._context.ok():
            return
        timeout = _timeout_seconds(timeout_sec)
        if timeout is None or timeout < 0:
            self._native.spin_once()
        else:
            duration = cppyy.gbl.std.chrono.nanoseconds(int(timeout * 1e9))
            self._native.spin_once(duration)
        for node in self.get_nodes():
            node._poll_direct_clients()

    def spin_once(self, timeout_sec=None) -> None:
        self._enter_spin()
        try:
            self._spin_once_impl(timeout_sec)
        finally:
            self._exit_spin()

    def spin(self) -> None:
        self._enter_spin()
        try:
            while self._context.ok() and not self._is_shutdown:
                self._spin_once_impl(None)
        finally:
            self._exit_spin()

    def spin_until_future_complete(self, future, timeout_sec=None) -> None:
        self._enter_spin()
        future.add_done_callback(lambda _future: self.wake())
        try:
            timeout = _timeout_seconds(timeout_sec)
            deadline = None if timeout is None or timeout < 0 else time.monotonic() + timeout
            while (
                self._context.ok()
                and not self._is_shutdown
                and not future.done()
                and not future.cancelled()
            ):
                remaining = None if deadline is None else max(0.0, deadline - time.monotonic())
                self._spin_once_impl(remaining)
                if deadline is not None and time.monotonic() >= deadline:
                    return
        finally:
            self._exit_spin()

    def spin_once_until_future_complete(self, future, timeout_sec=None) -> None:
        if future.done() or future.cancelled():
            return
        self.spin_once(timeout_sec=timeout_sec)

    def shutdown(self, timeout_sec=None) -> bool:
        del timeout_sec
        with self._state_lock:
            if self._is_shutdown:
                return True
            self._is_shutdown = True
        self._native.cancel()
        with self._runtime.membership_lock:
            for node in self.get_nodes():
                self.remove_node(node)
                if node.executor is self:
                    node._set_direct_executor(None)
        self._runtime.unregister_executor(self)
        return True

    def _runtime_shutdown(self) -> None:
        self.shutdown()


class DirectSingleThreadedExecutor(DirectExecutor):
    """A public facade over ``rclcpp::executors::SingleThreadedExecutor``."""


class DirectMultiThreadedExecutor(DirectExecutor):
    """Reserved until native concurrent Python callback behavior is proven."""

    _kind = "multi_threaded"


__all__ = [
    "DirectExecutor",
    "DirectMultiThreadedExecutor",
    "DirectSingleThreadedExecutor",
]
