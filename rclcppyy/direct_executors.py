"""rclpy-shaped executors backed by the active native ``rclcpp`` context."""

from __future__ import annotations

import math
import os
import threading
import time
from typing import Any
import warnings

import cppyy

from rclcppyy._surface import _DirectSurface
from rclcppyy.policy import BackendUnavailableError


def _runtime():
    from rclcppyy.direct_cpp import _runtime as active_runtime

    return active_runtime()


def _unsupported(reason: str):
    raise BackendUnavailableError(reason)


# Bound on how long _run_native_background() waits for a freshly launched
# ExecutorThread's std::thread to report itself running before failing
# loud. Native thread scheduling is normally near-instant; this only fires
# if something is badly wrong, never as a routine silent teardown path.
_NATIVE_SPIN_STARTUP_TIMEOUT_SEC = 10.0


def _resolve_num_threads(num_threads: Any) -> int:
    """Resolve ``num_threads`` with the exact stock ``MultiThreadedExecutor``
    semantics: ``None`` queries CPU affinity (falling back to ``cpu_count()``,
    then 2), and a resolved single thread warns (use SingleThreadedExecutor).
    """
    if num_threads is None:
        if hasattr(os, "sched_getaffinity"):
            num_threads = len(os.sched_getaffinity(0))
        else:
            num_threads = os.cpu_count()
        if num_threads is None:
            num_threads = 2
    if num_threads == 1:
        warnings.warn(
            "MultiThreadedExecutor is used with a single thread.\n"
            "Use the SingleThreadedExecutor instead."
        )
    return num_threads


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


class DirectExecutor(metaclass=_DirectSurface):
    """Base facade for one executor owned by the active ``NativeSession``."""

    _kind = "single_threaded"
    # Native thread count requested from create_executor(); 0 keeps today's
    # single-threaded default. DirectMultiThreadedExecutor.__init__ resolves
    # its own count and overrides this on the instance before calling super().
    _threads = 0
    _PARITY_HIDDEN = frozenset({"native_executor", "park_node"})

    def __init__(self, *, context=None) -> None:
        runtime = _runtime()
        if context is not None and context is not runtime.context:
            _unsupported("direct_cpp executors require the active direct context")
        self._runtime = runtime
        self._context = runtime.context
        self._native = runtime.require_session().create_executor(
            self._kind, threads=self._threads)
        # A blocking native wait must not monopolize the interpreter. cppyy
        # reacquires the GIL when an rclcpp callback enters its Python target.
        self._native.spin_once.__release_gil__ = True
        self._nodes: list[Any] = []
        self._nodes_snapshot: tuple[Any, ...] = ()
        self._parked_node = None
        self._nodes_lock = threading.RLock()
        self._spin_lock = threading.Lock()
        self._state_lock = threading.RLock()
        self._is_shutdown = False
        self._is_spinning = False
        self._spin_thread_id = None
        self._tasks_lock = threading.Lock()
        self._ready_tasks: list[Any] = []
        # Set by wake() and cleared by _drive_tasks(); a blocked native wait
        # is interrupted by cancel() below, but a background pump (the
        # multi-threaded executor) waits on this instead of a native call.
        self._wake_event = threading.Event()
        # Live only while a MultiThreadedExecutor background pump (spin() /
        # spin_until_future_complete()) is running; see _run_native_background.
        self._background_thread = None
        # True only while _run_native_background() is between calling
        # start_executor() and recording the result in _background_thread --
        # closes the wake-cancel window described on wake() below.
        self._native_spin_starting = False
        # Deferred node-teardown requests (Slice 2.5, docs/plans/
        # PLAN-mte-unlock.md Addendum Q3): a callback that destroys its own
        # node cannot wait for its own in-flight count to reach zero
        # without self-deadlocking, so DirectNode._defer_teardown() enqueues
        # here instead; the spin/pump loop drains this at the top of every
        # cycle on a thread that CAN safely block on quiescence.
        self._deferred_teardown_lock = threading.Lock()
        self._deferred_teardown_queue: list[Any] = []
        runtime.register_executor(self)

    @property
    def context(self):
        return self._context

    @property
    def is_spinning(self) -> bool:
        return self._is_spinning

    @property
    def native_executor(self):
        if self._is_shutdown:
            raise RuntimeError("direct_cpp executor is shut down")
        return self._native

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.shutdown()

    def can_execute(self, entity) -> bool:
        # Stock also gates on `not entity._executor_event`; direct entities
        # carry no such attribute, so this delegates only to the callback
        # group's advisory contract (rclcpp is the true scheduling
        # authority -- see DirectCallbackGroup).
        return entity.callback_group.can_execute(entity)

    def wait_for_ready_callbacks(self, *args, **kwargs):
        _unsupported(
            "direct_cpp executors have no Python-level wait set to enumerate; "
            "the native rclcpp executor owns readiness"
        )

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
                if self._parked_node is node:
                    self._parked_node = None
                    self._nodes.append(node)
                    self._nodes_snapshot = tuple(self._nodes)
                    node._set_direct_executor(self)
                    return True
                if self._parked_node is not None:
                    parked = self._parked_node
                    self._parked_node = None
                    parked_native = getattr(parked, "_direct_cpp_node", None)
                    if parked_native is not None:
                        self._native.remove_node(parked_native)
            current = node.executor
            if current is not None and current is not self:
                current.remove_node(node)
            self._native.add_node(native)
            with self._nodes_lock:
                self._nodes.append(node)
                self._nodes_snapshot = tuple(self._nodes)
            node._set_direct_executor(self)
            return True

    def remove_node(self, node) -> None:
        with self._runtime.membership_lock:
            with self._nodes_lock:
                try:
                    self._nodes.remove(node)
                except ValueError:
                    if self._parked_node is not node:
                        return
                    self._parked_node = None
                else:
                    self._nodes_snapshot = tuple(self._nodes)
            native = getattr(node, "_direct_cpp_node", None)
            if native is not None and not self._runtime.session.closed:
                self._native.remove_node(native)
            if node.executor is self:
                node._set_direct_executor(None)

    def park_node(self, node) -> None:
        """Logically remove one temporary global node without native churn."""
        with self._runtime.membership_lock:
            with self._nodes_lock:
                try:
                    self._nodes.remove(node)
                except ValueError:
                    return
                self._nodes_snapshot = tuple(self._nodes)
                if self._parked_node is not None and self._parked_node is not node:
                    raise RuntimeError("direct_cpp executor already has a parked node")
                self._parked_node = node

    def get_nodes(self) -> list[Any]:
        with self._nodes_lock:
            return list(self._nodes)

    def wake(self) -> None:
        self._wake_event.set()
        if self._is_shutdown:
            return
        if self._background_thread is not None or self._native_spin_starting:
            # A live (or just-launching) MultiThreadedExecutor background
            # pump dispatches subscription/timer/service callbacks natively
            # on its own C++ threads; cancel() would stop that dispatch
            # outright, so the wake event alone is enough to nudge the
            # Python pump loop. ``_native_spin_starting`` covers the window
            # between start_executor() launching the native std::thread and
            # ``_background_thread`` being recorded, during which a cancel()
            # here would otherwise stop a spin that has barely begun.
            return
        self._native.cancel()

    def create_future(self):
        from rclpy.task import Future

        return Future(executor=self)

    def create_task(self, callback, *args, **kwargs):
        from rclpy.task import Task

        task = Task(callback, args, kwargs, executor=self)
        self._call_task_in_next_spin(task)
        return task

    def _call_task_in_next_spin(self, task) -> None:
        """Enqueue ``task`` to run on the next drive and wake a blocked spin.

        ``rclpy.task.Task`` (stock, unmodified) calls this exact private
        method by name when a coroutine yields plain ``None`` (see
        ``Task._execute_coroutine_step``), so the name and single-``task``
        signature are load-bearing, not just an internal convenience.
        """
        with self._tasks_lock:
            self._ready_tasks.append(task)
        self.wake()

    def _drive_tasks(self) -> None:
        """Step every task that was ready at the start of this call, once.

        A plain-callable task always finishes in a single step. A coroutine
        task that suspends on a Future re-arms itself through the Future's
        own done-callback machinery, which resolves back into this
        executor's ``create_task``/``_call_task_in_next_spin`` -- so nothing
        here needs to know which kind of handler it is stepping.

        Stock's SingleThreadedExecutor re-raises a handler's exception on
        the calling thread immediately after running it (see
        ``_spin_once_impl``); matched here so a raising task is stock-parity
        visible to a `spin()`/`spin_once()` caller, not just to
        ``future.result()``. Any tasks not yet stepped in this batch are
        preserved (put back ahead of anything newly enqueued meanwhile) so a
        mid-batch exception never drops pending work.
        """
        self._wake_event.clear()
        with self._tasks_lock:
            ready = self._ready_tasks
            self._ready_tasks = []
        for index, task in enumerate(ready):
            task()
            if task.exception() is not None:
                with self._tasks_lock:
                    self._ready_tasks = ready[index + 1:] + self._ready_tasks
                raise task.exception()

    def _enter_spin(self) -> None:
        if not self._spin_lock.acquire(blocking=False):
            raise RuntimeError("Executor is already spinning")
        self._is_spinning = True
        self._spin_thread_id = threading.get_ident()

    def _exit_spin(self) -> None:
        self._spin_thread_id = None
        self._is_spinning = False
        self._spin_lock.release()

    def _poll_nodes(self) -> None:
        """Poll entities the native wait set never sees (clients/action
        clients/action servers -- see the dispatch-model note in the
        executor slice plan)."""
        for node in self._nodes_snapshot:
            if (
                node._direct_cpp_clients
                or node._direct_cpp_action_clients
                or node._direct_cpp_action_servers
            ):
                node._poll_direct_entities()

    def _drain_and_raise_callback_exceptions(self) -> None:
        """Drain the owned nodes' contained callback-exception sinks
        (defect A -- see docs/plans/PLAN-mte-unlock.md) and re-raise the
        first captured exception on the calling spin/pump thread, matching
        stock's ``future.result()`` re-raise semantics. Every node's sink
        is drained regardless -- only the first exception is raised, but
        none are left behind to resurface on a later drain.
        """
        first = None
        for node in self._nodes_snapshot:
            for exc, _origin_node in node._drain_callback_exceptions():
                if first is None:
                    first = exc
        if first is not None:
            raise first

    def _enqueue_deferred_teardown(self, teardown) -> None:
        """Called by ``DirectNode._defer_teardown`` when a callback destroys
        its own node (Slice 2.5): record the teardown for this spin/pump
        loop to run once genuinely quiescent, instead of running it here on
        the dispatching thread (which would self-deadlock waiting on its
        own in-flight count)."""
        with self._deferred_teardown_lock:
            self._deferred_teardown_queue.append(teardown)
        self.wake()

    def _drain_deferred_teardown(self) -> None:
        """Run any teardown requests enqueued by a self-destroying callback.
        Safe to block here (e.g. on ``DirectNode._wait_quiescent``): this
        runs on the spin/pump thread, never on the native worker thread
        that was dispatching when the destroy was requested."""
        with self._deferred_teardown_lock:
            pending = self._deferred_teardown_queue
            self._deferred_teardown_queue = []
        for teardown in pending:
            teardown()

    def _spin_once_impl(self, timeout_sec=None) -> None:
        if self._is_shutdown:
            return
        if not self._nodes_snapshot and self._parked_node is not None:
            # Direct calls on an otherwise empty global executor must not run a
            # node that was only cached between top-level spin_once calls.
            self.remove_node(self._parked_node)
        self._drain_deferred_teardown()
        # Ready tasks are driven before the native step: a task that resolves
        # the future a spin_until_future_complete() caller is waiting on
        # should end that loop without waiting on an unrelated native event.
        self._drive_tasks()
        timeout = _timeout_seconds(timeout_sec)
        if timeout is None or timeout < 0:
            self._native.spin_once()
        else:
            duration = cppyy.gbl.std.chrono.nanoseconds(int(timeout * 1e9))
            self._native.spin_once(duration)
        self._poll_nodes()
        # A contained callback exception (defect A) no longer crosses the
        # cppyy boundary from spin_once() above -- re-raise it here instead,
        # preserving today's propagate-out-of-spin_once() observable.
        self._drain_and_raise_callback_exceptions()

    def _run_native_background(self, stop_predicate) -> None:
        """Spin native concurrent dispatch on a managed C++ thread, pumping
        Python-side work (tasks, direct-entity polling) on the calling
        thread until ``stop_predicate()``, shutdown, or the native thread
        itself stops.

        Subscription/timer/service callbacks dispatch concurrently, fully
        inside ``rclcpp``, group-respecting -- this method never touches an
        entity callback directly. ``_wake_event.wait()`` is a GIL-releasing
        block so the native threads are never starved of the interpreter.
        """
        # start_executor() only enqueues the native std::thread's launch --
        # the OS can schedule it and it can set ExecutorThread::running_ true
        # before this Python frame's next line runs. Claim the "a background
        # pump owns this executor" state before that call so a wake() firing
        # in that window is covered even before ``thread`` exists to assign
        # to ``_background_thread`` (see wake() above).
        self._native_spin_starting = True
        thread = self._runtime.require_session().start_executor(self._native)
        # close() blocks on joining the native ExecutorThread's std::thread,
        # which itself may be waiting on a worker thread that needs the GIL
        # to finish running/returning from a Python callback. Without this,
        # a Python thread blocked inside close() holds the GIL forever and
        # that worker thread can never complete -- a permanent deadlock.
        thread._implementation.close.__release_gil__ = True
        self._background_thread = thread
        self._native_spin_starting = False
        poll_interval = 0.02
        # ExecutorThread's running_ flips true only once the OS has actually
        # scheduled its thread lambda (rclcpp_kit native.py), strictly after
        # start_executor() has already returned here -- so a False reading
        # before that happens means "not started yet", not "already
        # stopped". Exit the loop only once running was observed True and
        # has since gone False (a genuine end); a bounded startup deadline
        # keeps a native thread that never starts from hanging this pump
        # forever, failing loud instead of silently tearing the executor
        # down before it dispatches anything.
        started = False
        startup_deadline = time.monotonic() + _NATIVE_SPIN_STARTUP_TIMEOUT_SEC
        try:
            while self._context.ok() and not self._is_shutdown and not stop_predicate():
                if thread.running:
                    started = True
                elif started:
                    break
                elif time.monotonic() >= startup_deadline:
                    raise RuntimeError(
                        "direct_cpp MultiThreadedExecutor native worker "
                        "thread did not report itself running within %.1fs "
                        "of start_executor(); failing loud instead of "
                        "silently tearing down before any dispatch"
                        % _NATIVE_SPIN_STARTUP_TIMEOUT_SEC
                    )
                # A self-destroying callback (Slice 2.5) enqueued its own
                # node's teardown here instead of running it in place; this
                # pump thread can safely block waiting for that node's
                # in-flight count to reach zero, unlike the worker thread
                # that deferred it.
                self._drain_deferred_teardown()
                self._drive_tasks()
                self._poll_nodes()
                # A contained callback exception (defect A) is recorded to
                # the owning node's sink instead of crossing the cppyy
                # boundary off the native worker thread that ran it; drain
                # and re-raise it here on the pump thread. _record_callback_
                # exception() also wakes this loop, so a fresh exception is
                # usually seen well before the next poll_interval tick.
                self._drain_and_raise_callback_exceptions()
                self._wake_event.wait(poll_interval)
                self._wake_event.clear()
        finally:
            self._background_thread = None
            thread.close()
            if thread.exceptions:
                # Backstop only, for whatever still escapes the containment
                # shim applied at the create_subscription/create_timer/
                # create_service hand-offs -- e.g. a raise inside the
                # suite-internal owning-copy construction, which is not a
                # user callback and not covered by that shim (documented
                # differential, PLAN-mte-unlock.md risk 2). The escaping
                # exception's object/traceback is already lost by the time
                # it is counted here, so this can only report a count.
                raise RuntimeError(
                    "%d direct_cpp MultiThreadedExecutor callback(s) raised "
                    "on a native worker thread" % thread.exceptions
                )

    def spin_once(self, timeout_sec=None) -> None:
        self._enter_spin()
        try:
            self._spin_once_impl(timeout_sec)
        finally:
            self._exit_spin()

    def spin(self) -> None:
        self._enter_spin()
        try:
            if self._kind == "multi_threaded":
                self._run_native_background(lambda: False)
            else:
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
            if self._kind == "multi_threaded":
                def _future_complete_or_expired() -> bool:
                    if future.done() or future.cancelled():
                        return True
                    return deadline is not None and time.monotonic() >= deadline

                self._run_native_background(_future_complete_or_expired)
            else:
                while (
                    self._context.ok()
                    and not self._is_shutdown
                    and not future.done()
                    and not future.cancelled()
                ):
                    remaining = (
                        None if deadline is None
                        else max(0.0, deadline - time.monotonic())
                    )
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
        self._wake_event.set()
        self._native.cancel()
        background = self._background_thread
        if background is not None:
            background.close()
        with self._runtime.membership_lock:
            for node in self.get_nodes():
                self.remove_node(node)
                if node.executor is self:
                    node._set_direct_executor(None)
            if self._parked_node is not None:
                parked = self._parked_node
                self.remove_node(parked)
                if parked.executor is self:
                    parked._set_direct_executor(None)
        self._runtime.unregister_executor(self)
        if background is not None and background.exceptions:
            # Same backstop-only case as in _run_native_background above --
            # a raise the containment shim never saw, counted late.
            raise RuntimeError(
                "%d direct_cpp MultiThreadedExecutor callback(s) raised on a "
                "native worker thread before shutdown" % background.exceptions
            )
        return True

    def _runtime_shutdown(self) -> None:
        self.shutdown()


class DirectSingleThreadedExecutor(DirectExecutor):
    """A public facade over ``rclcpp::executors::SingleThreadedExecutor``."""

    # Owned directly (not just inherited from DirectExecutor) so the ledger's
    # dunder-ownership rule counts them, matching stock -- which owns its own
    # __init__ and inherits __enter__/__exit__ from an rclpy-owned ancestor
    # (an inheritance path this package's base class cannot supply).
    def __init__(self, *, context=None) -> None:
        super().__init__(context=context)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.shutdown()


class DirectMultiThreadedExecutor(DirectExecutor):
    """A facade over ``rclcpp::executors::MultiThreadedExecutor``.

    Concurrent-dispatch machinery: native worker threads, group-respecting
    (reentrant groups run in parallel, mutually-exclusive groups serialize --
    see ``_run_native_background``). Un-fail-closed in Slice 3 of
    ``docs/plans/PLAN-mte-unlock.md`` once the containment (Slice 2),
    zero-dispatch (Slice 1), and destroy-under-dispatch (Slice 2.5) fixes
    proved this safe under real concurrency.
    """

    _kind = "multi_threaded"

    def __init__(self, num_threads=None, *, context=None) -> None:
        # Set before super().__init__() so the base constructor's
        # create_executor() call requests the resolved native thread count;
        # the public __init__ signature stays exactly stock's (no extra
        # parameter), so this can only flow through as instance state.
        self._threads = _resolve_num_threads(num_threads)
        super().__init__(context=context)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.shutdown()


def _mirror_stock_executor_subclasses() -> None:
    """Mirror stock signatures onto the concrete executor subclasses.

    ``direct_cpp.activate()`` mirrors only "Executor" onto the base
    ``DirectExecutor`` before installing replacements (its own
    ``mirrored_classes`` frozenset, out of this lane's file boundary); the
    concrete subclasses were left unmirrored there because, historically,
    they defined no members of their own. Now that they own their own
    ``__init__``/``__enter__``/``__exit__``, those need mirroring too or the
    ledger sees a real (but spurious) signature divergence.

    This module is imported by ``direct_cpp.activate()`` before its
    replacement loop runs (`from rclcppyy.direct_executors import ...`
    precedes the loop that patches ``rclpy.executors.*``), so
    ``rclpy.executors.SingleThreadedExecutor``/``MultiThreadedExecutor`` are
    still genuinely stock at that point. The guard keeps this a no-op (never
    mirroring a direct class onto itself) if that ordering assumption is ever
    violated by a future refactor.
    """
    import rclpy.executors as _stock_executors

    from rclcppyy._signature_mirror import mirror_class

    stock_single = _stock_executors.SingleThreadedExecutor
    if not issubclass(stock_single, DirectExecutor):
        mirror_class(DirectSingleThreadedExecutor, stock_single)
    stock_multi = _stock_executors.MultiThreadedExecutor
    if not issubclass(stock_multi, DirectExecutor):
        mirror_class(DirectMultiThreadedExecutor, stock_multi)


_mirror_stock_executor_subclasses()


__all__ = [
    "DirectExecutor",
    "DirectMultiThreadedExecutor",
    "DirectSingleThreadedExecutor",
]
