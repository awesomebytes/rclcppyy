"""Slice 2 content-filtered subscriptions: product wiring proof
(PLAN-qos-events-product.md). Live cases use ROS_DOMAIN_ID 77 (this lane's
domain, hardcoded in the helper for isolation regardless of the invoking
environment).

``direct_cpp`` is hard-gated Jazzy/Cyclone-only (``_check_runtime``,
``rclcppyy/direct_cpp.py:2664``; documented since the profile's first slice,
``docs/RCLCPPYY_EVOLUTION_PLAN.md:628``) -- ``enable_cpp_acceleration(profile=
"direct_cpp")`` itself raises ``BackendUnavailableError`` before any node,
publisher, or subscription exists when the runtime RMW isn't
``rmw_cyclonedds_cpp``. That makes a *live, product-level* Fast DDS proof
unreachable: there is no way to construct any rclcppyy-accelerated entity
while Fast DDS is the runtime RMW. And ``rmw_cyclonedds_cpp`` itself has no
content-filter implementation (the suite's own ``ContentFilterUnsupported``
docstring: ``rmw_subscription_set/get_content_filter`` are literal
"unimplemented" stubs there) -- so on the one RMW the product ever runs
under, a content-filtered subscription created through rclcppyy always
fails closed today. There is no live positive-filtering path to prove at
the product layer; only the fail-closed one below.

The content-filter *mechanism* itself (that Fast DDS actually filters) is
already proven at the suite layer, independent of rclcppyy's Cyclone gate:
cppyy_kit's ``test_direct_content_filter.py`` /
``_direct_content_filter_fastdds_helper.py`` call
``rclcpp_kit.direct_entities.create_subscription`` directly inside the
suite's own ``native()`` session, which never passes through rclcppyy's
``_check_runtime`` at all.

So the positive-path coverage this slice can honestly add at the product
layer is the translation itself -- ``_extract_content_filter`` turning a
stock ``ContentFilterOptions`` into the exact tuple the suite expects --
plus the live proof that the suite's fail-closed guard is reachable through
the public API and translated to the product's own exception type.
"""
import json

import pytest

from _run_helper import format_output, run_helper
from rclcppyy.direct_cpp import ContentFilterUnsupportedError, _extract_content_filter
from rclpy.subscription_content_filter_options import ContentFilterOptions


CYCLONE_PREFIX = "CONTENT_FILTER_CYCLONE_REPORT="


def _report(process, prefix):
    lines = [
        line for line in process.stdout.splitlines() if line.startswith(prefix)
    ]
    assert len(lines) == 1, format_output(process)
    return json.loads(lines[0][len(prefix):])


def test_extract_content_filter_none_means_no_filter():
    assert _extract_content_filter(None) is None


def test_extract_content_filter_translates_stock_options_exactly():
    result = _extract_content_filter(
        ContentFilterOptions(
            filter_expression="data = %0 AND data < %1",
            expression_parameters=["1", "2"]))
    assert result == ("data = %0 AND data < %1", ("1", "2"))
    # The parameters side is a tuple even though stock's NamedTuple field is
    # a list -- the suite's own validator requires a sequence it can accept
    # as either, but the product always hands it a tuple.
    assert isinstance(result[1], tuple)


def test_extract_content_filter_coerces_empty_parameter_list_to_empty_tuple():
    result = _extract_content_filter(
        ContentFilterOptions(filter_expression="true", expression_parameters=[]))
    assert result == ("true", ())


def test_extract_content_filter_rejects_wrong_type():
    with pytest.raises(TypeError, match="ContentFilterOptions"):
        _extract_content_filter(object())


def test_content_filter_unsupported_error_is_a_runtime_error():
    assert issubclass(ContentFilterUnsupportedError, RuntimeError)


def test_content_filter_fails_closed_on_cyclone(monkeypatch):
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    process = run_helper("_content_filter_cyclone_helper.py", timeout=60)
    assert process.returncode == 0, format_output(process)
    report = _report(process, CYCLONE_PREFIX)

    assert report["rmw"] == "rmw_cyclonedds_cpp"
    assert report["raised_content_filter_unsupported"] is True
    assert report["raised_message_mentions_rmw"] is True
    assert report["subscription_created_without_raising"] is False
    assert report["control_subscription_ok"] is True
