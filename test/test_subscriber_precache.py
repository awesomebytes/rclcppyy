"""Correctness proof for the subscriber-side sub-message pre-cache
(``rclcpp_kit.direct_entities._submessage_precache``): before a subscriber
callback runs, the dispatch trampoline seeds the received message's
``__dict__`` with cppyy's own proxy for each composite (sub-message)
field, so repeat access inside the callback hits ``__dict__`` instead of
re-resolving the C++ member. Exercised over both dispatch trampolines that
hand a message to a subscriber callback: the default owning-copy path and
the opt-in ``subscription_shared_lease`` path.
"""

from _run_helper import format_output, run_helper


def test_subscriber_precache_shared_lease_dispatch():
    process = run_helper("_subscriber_precache_helper.py", "lease", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_PRECACHE_FIELD_CLASSIFICATION_OK" in process.stdout
    assert "DIRECT_CPP_PRECACHE_DISPATCH_LEASE_OK" in process.stdout
    assert "DIRECT_CPP_PRECACHE_RETAINED_LEASE_OK" in process.stdout


def test_subscriber_precache_owning_copy_dispatch():
    process = run_helper("_subscriber_precache_helper.py", "copy", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_PRECACHE_FIELD_CLASSIFICATION_OK" in process.stdout
    assert "DIRECT_CPP_PRECACHE_DISPATCH_COPY_OK" in process.stdout
    assert "DIRECT_CPP_PRECACHE_RETAINED_COPY_OK" in process.stdout
