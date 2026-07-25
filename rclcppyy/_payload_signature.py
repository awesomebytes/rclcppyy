"""Hand-built ``Signature`` objects for payload-tainted parameter methods.

``rclcppyy._signature_mirror`` mirrors a facade member onto stock's exact
``Signature`` automatically, but only when stock's own signature is
payload-free (``_is_payload_free`` -- no ``cppyy.gbl`` marker in its rendered
form). ``rclpy.node.Node``'s parameter/descriptor methods reference
``rcl_interfaces.msg`` types this package rebinds to cppyy on activation, so
stock's signature for them *is* tainted by the time it can be observed (see
PLAN-wave7.md §4.1 for exactly why: ``rclpy.node`` is first imported only
inside ``activate()``, after ``direct_messages.install()`` has already
rebound ``rcl_interfaces.msg``'s attributes) -- the mirror correctly refuses
these, leaving ``DirectNode``'s own bare, unannotated signature in place.

This module builds the equivalent ``Signature`` by hand instead, from
*pristine* copies of the referenced classes -- captured by the caller before
``direct_messages.install()`` ever runs -- and assigns it directly to each
method's ``__signature__``, bypassing ``__annotations__``/PEP 563 entirely
(``inspect.signature()`` reads ``__signature__`` first, before ever
consulting ``__annotations__``).
"""

from __future__ import annotations

import inspect
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple, Union


_EMPTY = inspect.Parameter.empty
_POSITIONAL_OR_KEYWORD = inspect.Parameter.POSITIONAL_OR_KEYWORD

METHOD_NAMES = (
    "declare_parameter",
    "declare_parameters",
    "describe_parameter",
    "describe_parameters",
    "list_parameters",
    "add_on_set_parameters_callback",
    "remove_on_set_parameters_callback",
    "set_parameters",
    "set_parameters_atomically",
    "set_descriptor",
    "get_parameters_by_prefix",
)


def _parameter(name, *, default=_EMPTY, annotation=_EMPTY):
    return inspect.Parameter(
        name, _POSITIONAL_OR_KEYWORD, default=default, annotation=annotation)


def build_signatures(
    *,
    parameter_descriptor,
    parameter_value,
    set_parameters_result,
    list_parameters_result,
    parameter_class,
):
    """Return ``{method_name: inspect.Signature}`` for ``METHOD_NAMES``.

    Every annotation is a live class object (never a string), so
    ``str(inspect.signature(...))`` renders each's fully-qualified dotted
    path exactly as stock's own baseline does -- matching
    ``compatibility/rclpy-api-ledger-jazzy.json`` byte-for-byte.
    """
    self_ = _parameter("self")
    parameter_list_type = List[parameter_class]
    callback_annotation = Callable[[parameter_list_type], set_parameters_result]

    signatures = {}

    signatures["declare_parameter"] = inspect.Signature(
        [
            self_,
            _parameter("name", annotation=str),
            _parameter("value", default=None, annotation=Any),
            _parameter(
                "descriptor", default=None,
                annotation=Optional[parameter_descriptor]),
            _parameter("ignore_override", default=False, annotation=bool),
        ],
        return_annotation=parameter_class,
    )

    signatures["declare_parameters"] = inspect.Signature(
        [
            self_,
            _parameter("namespace", annotation=str),
            _parameter(
                "parameters",
                annotation=List[Union[
                    Tuple[str],
                    Tuple[str, parameter_class.Type],
                    Tuple[str, Any, parameter_descriptor],
                ]]),
            _parameter("ignore_override", default=False, annotation=bool),
        ],
        return_annotation=parameter_list_type,
    )

    signatures["describe_parameter"] = inspect.Signature(
        [self_, _parameter("name", annotation=str)],
        return_annotation=parameter_descriptor,
    )

    signatures["describe_parameters"] = inspect.Signature(
        [self_, _parameter("names", annotation=List[str])],
        return_annotation=List[parameter_descriptor],
    )

    signatures["list_parameters"] = inspect.Signature(
        [
            self_,
            _parameter("prefixes", annotation=List[str]),
            _parameter("depth", annotation=int),
        ],
        return_annotation=list_parameters_result,
    )

    signatures["add_on_set_parameters_callback"] = inspect.Signature(
        [self_, _parameter("callback", annotation=callback_annotation)],
        return_annotation=None,
    )

    signatures["remove_on_set_parameters_callback"] = inspect.Signature(
        [self_, _parameter("callback", annotation=callback_annotation)],
        return_annotation=None,
    )

    signatures["set_parameters"] = inspect.Signature(
        [self_, _parameter("parameter_list", annotation=parameter_list_type)],
        return_annotation=List[set_parameters_result],
    )

    signatures["set_parameters_atomically"] = inspect.Signature(
        [self_, _parameter("parameter_list", annotation=parameter_list_type)],
        return_annotation=set_parameters_result,
    )

    signatures["set_descriptor"] = inspect.Signature(
        [
            self_,
            _parameter("name", annotation=str),
            _parameter("descriptor", annotation=parameter_descriptor),
            _parameter(
                "alternative_value", default=None,
                annotation=Optional[parameter_value]),
        ],
        return_annotation=parameter_value,
    )

    signatures["get_parameters_by_prefix"] = inspect.Signature(
        [self_, _parameter("prefix", annotation=str)],
        return_annotation=Dict[str, Optional[Union[
            bool, int, float, str, bytes,
            Sequence[bool], Sequence[int], Sequence[float], Sequence[str],
        ]]],
    )

    assert set(signatures) == set(METHOD_NAMES), (
        "build_signatures/METHOD_NAMES drifted apart: %r" %
        (set(signatures) ^ set(METHOD_NAMES),))
    return signatures


def install_signatures(direct_node_class, **pristine_classes):
    """Assign each built signature onto ``direct_node_class``'s own member.

    Idempotent: re-assigning the same rendered ``Signature`` is a no-op in
    effect, so calling this more than once in a process is harmless.
    """
    signatures = build_signatures(**pristine_classes)
    for name, signature in signatures.items():
        getattr(direct_node_class, name).__signature__ = signature
    return signatures


__all__ = ["METHOD_NAMES", "build_signatures", "install_signatures"]
