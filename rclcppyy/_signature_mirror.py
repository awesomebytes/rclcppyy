"""Mirror stock rclpy signatures onto structurally-identical direct facades.

Several direct-backend facades in this package are genuine drop-ins for their
stock rclpy counterparts: identical parameter names, kinds, and defaults,
differing only in how the *annotation* renders (absent on the facade, or
stringified by this package's own ``from __future__ import annotations``).
Left alone, that purely cosmetic difference makes ``inspect.signature`` --
and everything that reads it, including ``help()``, IDEs, and the pinned
rclpy API ledger -- see a divergence that is not real.

This module fixes it at the source: it copies stock's exact ``Signature``
object onto the facade member, so every consumer of ``inspect.signature``
agrees. It only ever does so when two conditions both hold:

- structural identity -- the facade's own parameter names, kinds, and
  default reprs already match stock's, slot for slot (annotations aside);
- payload-free -- stock's signature does not reference a C++-payload type
  that this package's message installation has rebound (recognizable by a
  ``cppyy.gbl`` marker in its rendered form).

Anything else -- a facade whose call surface genuinely differs from stock, or
whose stock counterpart deals in a rebound payload type -- is left with its
own signature, so the ledger keeps seeing (and failing on) a real divergence.
"""

from __future__ import annotations

import inspect


_CPPYY_MARKER = "cppyy.gbl"


def _parameters_agree(direct_signature, stock_signature) -> bool:
    """True iff every parameter slot agrees on name, kind, and default repr."""
    direct_parameters = list(direct_signature.parameters.values())
    stock_parameters = list(stock_signature.parameters.values())
    if len(direct_parameters) != len(stock_parameters):
        return False
    for direct_parameter, stock_parameter in zip(direct_parameters, stock_parameters):
        if direct_parameter.name != stock_parameter.name:
            return False
        if direct_parameter.kind != stock_parameter.kind:
            return False
        direct_has_default = direct_parameter.default is not inspect.Parameter.empty
        stock_has_default = stock_parameter.default is not inspect.Parameter.empty
        if direct_has_default != stock_has_default:
            return False
        if direct_has_default and repr(direct_parameter.default) != repr(stock_parameter.default):
            return False
    return True


def _is_payload_free(stock_signature) -> bool:
    return _CPPYY_MARKER not in str(stock_signature)


def _mirror(direct_member, stock_member) -> bool:
    """Assign stock's signature onto ``direct_member`` iff the gate passes.

    Returns whether the mirror was applied. Never raises: an uninspectable
    member on either side is treated as a gate failure, not an error, so a
    facade with a genuinely stranger call surface is simply left alone.
    """
    try:
        direct_signature = inspect.signature(direct_member)
        stock_signature = inspect.signature(stock_member)
    except (TypeError, ValueError):
        return False
    if not _is_payload_free(stock_signature):
        return False
    if not _parameters_agree(direct_signature, stock_signature):
        return False
    direct_member.__signature__ = stock_signature
    return True


def _static_member(cls, name):
    try:
        return inspect.getattr_static(cls, name)
    except AttributeError:
        return None


def _callable_target(raw):
    """Resolve the plain function whose ``__signature__`` is read or set.

    Mirrors ``property``/``classmethod``/``staticmethod`` down to the
    function they wrap, matching how the ledger extractor itself resolves a
    class member's signature target. Anything else (a class attribute that
    is not a routine) is not a mirror candidate.
    """
    if isinstance(raw, property):
        return raw.fget, raw.fget is not None
    if isinstance(raw, (classmethod, staticmethod)):
        return raw.__func__, True
    if inspect.isroutine(raw):
        return raw, True
    return None, False


def mirror_function(direct_replacement, stock_original) -> bool:
    """Mirror one module-level function replacement's signature from stock."""
    return _mirror(direct_replacement, stock_original)


def mirror_class(direct_class, stock_class) -> None:
    """Mirror every member ``direct_class`` defines itself, gated per-member.

    Walks only ``direct_class``'s own ``vars()`` -- never an inherited
    member -- so a subclass that leaves an rclcppyy base's member untouched
    picks up that base's mirror automatically (the two share the same
    function object) rather than being revisited.

    Deliberately does not also assign ``direct_class.__signature__``:
    ``inspect.signature(a_class)`` reads a ``__signature__`` class attribute
    before it ever looks at ``__init__``, and a class attribute is inherited
    by every subclass -- including one outside this mirror's own scope (an
    rclpy-owned mixin subclass of a mirrored facade, say). Setting it here
    would leak the facade's mirrored constructor onto that unrelated
    subclass's class-symbol row even though the subclass defines its own
    ``__init__``. Mirroring only the ``__init__`` member is sufficient on its
    own: ``inspect.signature(cls)`` falls back to a class's own or inherited
    ``__init__`` when no ``__signature__`` is set, so the class-symbol row
    and the ``__init__`` member row still flip together for every class this
    function is actually called on, without touching classes it is not
    called on.
    """
    for name, raw in list(vars(direct_class).items()):
        direct_target, direct_applicable = _callable_target(raw)
        if not direct_applicable:
            continue
        stock_raw = _static_member(stock_class, name)
        if stock_raw is None:
            continue
        stock_target, stock_applicable = _callable_target(stock_raw)
        if not stock_applicable:
            continue
        _mirror(direct_target, stock_target)
