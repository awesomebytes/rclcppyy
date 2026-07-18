"""Correctness and failure-boundary proofs for the direct-C++ first slice."""

import os

from _run_helper import format_output, run_helper


def test_direct_cpp_source_compatible_pubsub_and_lifetime():
    process = run_helper("_direct_cpp_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_CONSTRUCTORS_OK" in process.stdout
    assert "DIRECT_CPP_FAIL_CLOSED_OK" in process.stdout
    assert "DIRECT_CPP_MESSAGES_OK" in process.stdout
    assert "DIRECT_CPP_TEARDOWN_OK" in process.stdout


def test_direct_cpp_native_timer_and_bounded_spin():
    process = run_helper("_direct_cpp_timer_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_TIMER_FAIL_CLOSED_OK" in process.stdout
    assert "DIRECT_CPP_TIMER_CONTROL_OK" in process.stdout
    assert "DIRECT_CPP_TIMER_EXCEPTION_OK" in process.stdout
    assert "DIRECT_CPP_SPIN_INTERRUPT_OK" in process.stdout


def test_direct_cpp_rejects_supported_message_imported_before_activation():
    process = run_helper("_direct_cpp_stale_import_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_STALE_IMPORT_OK" in process.stdout


def test_direct_cpp_profile_is_jazzy_cyclone_only():
    assert os.environ.get("ROS_DISTRO") == "jazzy"
    assert os.environ.get("RMW_IMPLEMENTATION") == "rmw_cyclonedds_cpp"
