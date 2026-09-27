"""Focused unit tests for the direct-facade signature mirror helper.

Synthetic classes only -- no ROS runtime, no activation. These pin the gate's
outcomes (skip on structural drift, skip on a cppyy-tainted stock signature,
mirror on a clean match, leave an unmatched member untouched) plus the
class/``__init__`` pairing and inheritance-sharing behavior the real facades
rely on.
"""

import inspect

from rclcppyy import _signature_mirror as mirror


class _StockPayloadType:
    """Stands in for a message type rebound to a cppyy proxy class.

    Real payload-tainted rows carry a ``__module__`` like
    ``cppyy.gbl.rcl_interfaces.msg`` once this package's message installation
    rebinds the stock message type (see rclcppyy's own ledger extractor,
    which detects the same rebinding by source and module).
    """


_StockPayloadType.__module__ = "cppyy.gbl.rcl_interfaces.msg"
_StockPayloadType.__qualname__ = "ParameterDescriptor"


def test_structural_drift_skips_the_mirror():
    class Stock:
        def method(self, value, extra=1):
            pass

    class Direct:
        def method(self, value):
            pass

    before = inspect.signature(Direct.method)
    applied = mirror.mirror_function(Direct.method, Stock.method)

    assert applied is False
    assert inspect.signature(Direct.method) == before


def test_default_value_drift_skips_the_mirror():
    class Stock:
        def method(self, value=1):
            pass

    class Direct:
        def method(self, value=2):
            pass

    before = inspect.signature(Direct.method)
    applied = mirror.mirror_function(Direct.method, Stock.method)

    assert applied is False
    assert inspect.signature(Direct.method) == before


def test_cppyy_tainted_stock_signature_skips_the_mirror():
    class Stock:
        def method(self, descriptor: _StockPayloadType = None):
            pass

    class Direct:
        def method(self, descriptor=None):
            pass

    before = inspect.signature(Direct.method)
    applied = mirror.mirror_function(Direct.method, Stock.method)

    assert applied is False
    assert inspect.signature(Direct.method) == before
    assert mirror._CPPYY_MARKER in str(inspect.signature(Stock.method))


def test_clean_structural_match_is_mirrored():
    class Stock:
        def method(self, value) -> bool:
            pass

    class Direct:
        def method(self, value):
            pass

    applied = mirror.mirror_function(Direct.method, Stock.method)

    assert applied is True
    assert inspect.signature(Direct.method) == inspect.signature(Stock.method)
    assert str(inspect.signature(Direct.method)) == "(self, value) -> bool"


def test_member_with_no_stock_counterpart_is_untouched():
    class Stock:
        def method(self, value):
            pass

    class Direct:
        def method(self, value):
            pass

        def facade_only(self):
            pass

    before = inspect.signature(Direct.facade_only)
    mirror.mirror_class(Direct, Stock)

    assert "__signature__" not in vars(Direct.facade_only)
    assert inspect.signature(Direct.facade_only) == before
    # The member that does have a stock counterpart still gets mirrored.
    assert inspect.signature(Direct.method) == inspect.signature(Stock.method)


def test_mirror_class_couples_init_member_and_class_symbol_rows():
    class Stock:
        def __init__(self, name, count=0):
            pass

    class Direct:
        def __init__(self, name, count=0):
            pass

    mirror.mirror_class(Direct, Stock)

    assert inspect.signature(Direct.__init__) == inspect.signature(Stock.__init__)
    assert inspect.signature(Direct) == inspect.signature(Stock)


def test_mirror_class_skips_init_on_parameter_set_drift():
    class Stock:
        def __init__(self, name, num_threads=None):
            pass

    class Direct:
        def __init__(self, name):
            pass

    before_init = inspect.signature(Direct.__init__)
    before_class = inspect.signature(Direct)
    mirror.mirror_class(Direct, Stock)

    assert inspect.signature(Direct.__init__) == before_init
    assert inspect.signature(Direct) == before_class


def test_subclass_inherits_a_base_mirror_without_separate_visitation():
    class Stock:
        def shared(self, value):
            pass

    class DirectBase:
        def shared(self, value):
            pass

    class DirectSub(DirectBase):
        """Defines nothing of its own; inherits ``shared`` unchanged."""

    mirror.mirror_class(DirectBase, Stock)

    # DirectSub was never separately visited, but it shares DirectBase's
    # function object for the inherited member, so it already reads mirrored.
    assert "shared" not in vars(DirectSub)
    assert inspect.signature(DirectSub.shared) == inspect.signature(Stock.shared)


def test_class_level_signature_does_not_leak_onto_an_unrelated_subclass():
    class Stock:
        def __init__(self, x, y=1) -> None:
            pass

    class Direct:
        def __init__(self, x, y=1):
            pass

    class Mixin:
        pass

    class Subclass(Mixin, Direct):
        """Stands in for LifecycleNode(LifecycleNodeMixin, Node): defines its
        own __init__, entirely outside this mirror's own scope."""

        def __init__(self, name, *, flag=True):
            pass

    mirror.mirror_class(Direct, Stock)

    assert inspect.signature(Direct.__init__) == inspect.signature(Stock.__init__)
    assert inspect.signature(Direct) == inspect.signature(Stock)
    # Subclass was never passed to mirror_class -- its own __init__ must read
    # exactly as it always did, not Direct's mirrored constructor.
    assert str(inspect.signature(Subclass)) == "(name, *, flag=True)"


def test_property_getter_is_mirrored_through_its_function():
    class Stock:
        @property
        def context(self) -> int:
            return 1

    class Direct:
        @property
        def context(self):
            return 1

    mirror.mirror_class(Direct, Stock)

    assert (
        inspect.signature(Direct.context.fget)
        == inspect.signature(Stock.context.fget)
    )
