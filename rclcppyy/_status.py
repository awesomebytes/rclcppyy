"""Bounded process-local reporting for backend selection decisions."""

import copy
import os
import threading
from collections import deque
from itertools import islice


_SCHEMA_VERSION = 1
_RECORD_KINDS = ("nodes", "entities", "operations")
_RECORD_PREFIXES = {"nodes": "node", "entities": "entity", "operations": "operation"}
_BACKENDS = ("cpp", "python", "unsupported")
_MAX_RECORDS_PER_KIND = 256
_MAX_METADATA_ITEMS = 32
_MAX_POLICIES = 16
_MAX_STRING_LENGTH = 512


def _bounded_string(value):
    text = str(value)
    if len(text) <= _MAX_STRING_LENGTH:
        return text
    return text[:_MAX_STRING_LENGTH - 3] + "..."


def _value_snapshot(value, depth=0):
    """Return a bounded JSON-compatible value without retaining ``value``."""
    if value is None or isinstance(value, (bool, float)):
        return value
    if isinstance(value, int):
        # Extremely large integers are valid Python but can make a status record
        # arbitrarily large. Normal runtime metadata remains an integer.
        if value.bit_length() <= 256:
            return value
        return _bounded_string(value)
    if isinstance(value, str):
        return _bounded_string(value)
    if depth >= 3:
        return _bounded_string(value)
    if isinstance(value, dict):
        result = {}
        for index, (key, item) in enumerate(value.items()):
            if index >= _MAX_METADATA_ITEMS:
                result["__truncated__"] = True
                break
            result[_bounded_string(key)] = _value_snapshot(item, depth + 1)
        return result
    if isinstance(value, (list, tuple, set, frozenset)):
        items = list(islice(value, _MAX_METADATA_ITEMS + 1))
        if isinstance(value, (set, frozenset)):
            items.sort(key=str)
        result = [
            _value_snapshot(item, depth + 1)
            for item in items[:_MAX_METADATA_ITEMS]
        ]
        if len(items) > _MAX_METADATA_ITEMS:
            result.append("...")
        return result
    return _bounded_string(value)


class _DecisionRegistry:
    def __init__(self, max_records_per_kind=_MAX_RECORDS_PER_KIND):
        self._lock = threading.RLock()
        self._max_records_per_kind = max_records_per_kind
        self._pid = os.getpid()
        self._generation = 0
        self._clear_locked()

    def _clear_locked(self):
        self._sequence = 0
        self._records = {
            kind: deque(maxlen=self._max_records_per_kind)
            for kind in _RECORD_KINDS
        }
        self._counts = {
            kind: dict.fromkeys(_BACKENDS, 0)
            for kind in _RECORD_KINDS
        }
        self._dropped = dict.fromkeys(_RECORD_KINDS, 0)

    def _ensure_process_locked(self):
        pid = os.getpid()
        if pid != self._pid:
            # A fork inherits module globals. Start a fresh registry in the child
            # so its report never claims decisions made by the parent process.
            self._pid = pid
            self._generation += 1
            self._clear_locked()

    def record(self, kind, backend, reason, policies=(), metadata=None):
        if kind not in _RECORD_KINDS:
            raise ValueError("unknown status record kind: %r" % (kind,))
        if backend not in _BACKENDS:
            raise ValueError("unknown backend: %r" % (backend,))

        safe_reason = _bounded_string(reason)
        safe_policies = [
            _bounded_string(policy)
            for policy in islice(policies, _MAX_POLICIES)
        ]
        safe_metadata = _value_snapshot(metadata or {})

        with self._lock:
            self._ensure_process_locked()
            self._sequence += 1
            record_id = "%s-%08d" % (_RECORD_PREFIXES[kind], self._sequence)
            records = self._records[kind]
            if len(records) == records.maxlen:
                self._dropped[kind] += 1
            records.append({
                "id": record_id,
                "backend": backend,
                "reason": safe_reason,
                "policies": safe_policies,
                "metadata": safe_metadata,
            })
            self._counts[kind][backend] += 1
            return record_id

    def snapshot(self):
        with self._lock:
            self._ensure_process_locked()
            result = {
                "schema_version": _SCHEMA_VERSION,
                "process_id": self._pid,
                "generation": self._generation,
                "limits": {
                    "records_per_kind": self._max_records_per_kind,
                },
                "counts": copy.deepcopy(self._counts),
                "dropped_records": dict(self._dropped),
            }
            for kind in _RECORD_KINDS:
                result[kind] = copy.deepcopy(list(self._records[kind]))
            return result

    def reset(self):
        with self._lock:
            self._ensure_process_locked()
            self._generation += 1
            self._clear_locked()


_REGISTRY = _DecisionRegistry()


def record_decision(kind, backend, reason, policies=(), metadata=None):
    """Record one backend decision and return its value-only identifier."""
    return _REGISTRY.record(kind, backend, reason, policies, metadata)


def status():
    """Return a stable, JSON-serializable snapshot of backend decisions."""
    return _REGISTRY.snapshot()


def reset_status_for_tests():
    """Clear process-local reporting state. Intended for isolated tests only."""
    _REGISTRY.reset()
