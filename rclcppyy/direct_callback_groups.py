"""Callback-group facades whose scheduling authority stays in ``rclcpp``."""

from __future__ import annotations

import threading
import weakref

from rclcppyy._surface import _DirectSurface
from rclcppyy.policy import BackendUnavailableError


def _unsupported(reason: str):
    raise BackendUnavailableError(reason)


class DirectCallbackGroup(metaclass=_DirectSurface):
    """Base rclpy-shaped ownership facade for one native callback group."""

    _kind = None
    _PARITY_HIDDEN = frozenset({"discard_entity", "native_group", "node"})

    def __init__(self) -> None:
        self.entities = set()
        self._node_ref = None
        self._native_group = None
        self._bind_lock = threading.RLock()

    @classmethod
    def _default_for(cls, node, native_group):
        group = DirectMutuallyExclusiveCallbackGroup()
        group._node_ref = weakref.ref(node)
        group._native_group = native_group
        return group

    @property
    def native_group(self):
        if self._native_group is None:
            _unsupported("direct_cpp callback group has not been bound to a node")
        return self._native_group

    @property
    def node(self):
        return None if self._node_ref is None else self._node_ref()

    def _bind(self, node):
        from rclcppyy.direct_cpp import DirectNode, _runtime

        if not isinstance(node, DirectNode):
            raise TypeError("direct_cpp callback groups require a direct_cpp Node")
        if self._kind is None:
            _unsupported("the direct_cpp CallbackGroup base class cannot own entities")
        with self._bind_lock:
            owner = self.node
            if owner is not None and owner is not node:
                _unsupported(
                    "a direct_cpp callback group cannot be shared across nodes")
            if self._native_group is None:
                self._native_group = _runtime().require_session().create_callback_group(
                    node._require_node(), self._kind)
                self._node_ref = weakref.ref(node)
                node._retain_callback_group(self)
            return self._native_group

    def _unbind(self, node) -> None:
        with self._bind_lock:
            if self.node is not node:
                return
            self.entities.clear()
            self._native_group = None
            self._node_ref = None
            if hasattr(self, "_active_entity"):
                self._active_entity = None

    def add_entity(self, entity) -> None:
        self.entities.add(weakref.ref(entity))

    def discard_entity(self, entity) -> None:
        self.entities.discard(weakref.ref(entity))

    def has_entity(self, entity) -> bool:
        return weakref.ref(entity) in self.entities

    def can_execute(self, entity) -> bool:
        raise NotImplementedError()

    def beginning_execution(self, entity) -> bool:
        raise NotImplementedError()

    def ending_execution(self, entity) -> None:
        raise NotImplementedError()


class DirectReentrantCallbackGroup(DirectCallbackGroup):
    """A facade over an exact ``rclcpp`` reentrant callback group."""

    _kind = "reentrant"

    # Owned here (not just inherited) so the ledger's dunder-ownership rule
    # counts it: a required dunder counts for a class only when the class
    # itself owns it or an rclpy-owned ancestor does, and DirectCallbackGroup
    # is neither stock nor this class.
    def __init__(self) -> None:
        super().__init__()

    def can_execute(self, entity) -> bool:
        return True

    def beginning_execution(self, entity) -> bool:
        return True

    def ending_execution(self, entity) -> None:
        pass


class DirectMutuallyExclusiveCallbackGroup(DirectCallbackGroup):
    """A facade over an exact ``rclcpp`` mutually-exclusive callback group."""

    _kind = "mutually_exclusive"

    def __init__(self) -> None:
        super().__init__()
        self._active_entity = None
        self._execution_lock = threading.Lock()

    def can_execute(self, entity) -> bool:
        with self._execution_lock:
            assert self.has_entity(entity)
            return self._active_entity is None

    def beginning_execution(self, entity) -> bool:
        with self._execution_lock:
            assert self.has_entity(entity)
            if self._active_entity is None:
                self._active_entity = entity
                return True
            return False

    def ending_execution(self, entity) -> None:
        with self._execution_lock:
            assert self._active_entity is entity
            self._active_entity = None


__all__ = [
    "DirectCallbackGroup",
    "DirectMutuallyExclusiveCallbackGroup",
    "DirectReentrantCallbackGroup",
]
