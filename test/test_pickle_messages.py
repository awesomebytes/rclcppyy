"""Pickle support for C++-backed (``direct_cpp``) ROS messages.

cppyy does not implement the pickle protocol for its wrapped C++ instances, so
``enable_cpp_acceleration(profile="direct_cpp")`` used to break any caller code
that pickles a message -- ``multiprocessing``, a Redis-backed cache, etc. See
``rclcpp_kit.message_pickle`` for the implementation each scenario below
exercises through a real ``direct_cpp`` activation.
"""

import os

from _run_helper import format_output, run_helper


def test_pickle_roundtrip_for_each_message_type():
    proc = run_helper("_pickle_messages_helper.py")
    details = format_output(proc)
    assert proc.returncode == 0, details
    assert "PICKLE_MESSAGES_ROUNDTRIP_OK" in proc.stdout, details


def test_unpickling_without_activation_produces_a_stock_message(tmp_path):
    path = str(tmp_path / "pose_stamped.pickle")

    produced = run_helper("_pickle_messages_stock_fallback_helper.py", "produce", path)
    assert produced.returncode == 0, format_output(produced)
    assert "PICKLE_MESSAGES_PRODUCE_OK" in produced.stdout, format_output(produced)
    assert os.path.getsize(path) > 0

    consumed = run_helper("_pickle_messages_stock_fallback_helper.py", "consume", path)
    assert consumed.returncode == 0, format_output(consumed)
    assert "PICKLE_MESSAGES_STOCK_FALLBACK_OK" in consumed.stdout, format_output(consumed)


def test_multiprocessing_queue_roundtrip():
    proc = run_helper("_pickle_messages_multiprocessing_helper.py")
    details = format_output(proc)
    assert proc.returncode == 0, details
    assert "PICKLE_MESSAGES_MULTIPROCESSING_OK" in proc.stdout, details
