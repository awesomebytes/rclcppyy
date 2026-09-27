"""Metaclass that curates the introspected public surface of direct classes."""

from __future__ import annotations


class _DirectSurface(type):
    """Curate the introspected public surface of direct entity classes."""

    def __dir__(cls):
        hidden = frozenset(getattr(cls, "_PARITY_HIDDEN", ()))
        return [name for name in super().__dir__() if name not in hidden]
