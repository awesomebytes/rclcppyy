"""Correctness and failure-boundary proofs for DirectLifecycleNode (P1)."""

from _run_helper import format_output, run_helper


def test_direct_cpp_lifecycle_node_construction_transitions_and_differential():
    process = run_helper("_direct_cpp_lifecycle_helper.py", timeout=240)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_LIFECYCLE_REBOUND_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_CONSTRUCT_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_TRIGGER_STAR_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_INVALID_TRANSITION_DIFFERENTIAL_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_STOCK_CLIENT_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_SHUTDOWN_DIFFERENTIAL_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_TEARDOWN_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_NO_COMMUNICATION_INTERFACE_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_ALL_OK" in process.stdout


def test_direct_cpp_lifecycle_raising_callback_contained_and_recovers():
    process = run_helper(
        "_direct_cpp_lifecycle_error_recovery_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_LIFECYCLE_RAISE_CONTAINED_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_ERROR_RECOVERY_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_POST_ERROR_USABLE_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_ERROR_RECOVERY_ALL_OK" in process.stdout


def test_direct_cpp_lifecycle_destroy_under_transition_dispatch():
    process = run_helper(
        "_direct_cpp_lifecycle_destroy_under_dispatch_helper.py", timeout=300)
    assert process.returncode == 0, format_output(process)
    assert "LIFECYCLE_DESTROY_DISPATCH_ITER_49_OK" in process.stdout
    assert "LIFECYCLE_DESTROY_DISPATCH_ALL_OK" in process.stdout


def test_direct_cpp_lifecycle_publisher_gating_and_coexistence():
    process = run_helper("_direct_cpp_lifecycle_publisher_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_LIFECYCLE_PUBLISHER_CONSTRUCT_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_PUBLISHER_UNCONFIGURED_SUPPRESSED_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_PUBLISHER_INACTIVE_SUPPRESSED_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_PUBLISHER_ACTIVE_DELIVERED_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_PUBLISHER_DEACTIVATE_SUPPRESSED_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_PUBLISHER_COEXISTENCE_ACTIVATE_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_PUBLISHER_COEXISTENCE_DEACTIVATE_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_PUBLISHER_CLASS_REJECTED_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_PUBLISHER_DESTROY_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_PUBLISHER_ALL_OK" in process.stdout


def test_direct_cpp_lifecycle_data_plane_end_to_end():
    process = run_helper("_direct_cpp_lifecycle_dataplane_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_LIFECYCLE_DATAPLANE_ACTIONS_REJECTED_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_DATAPLANE_PUBLISHER_REJECTED_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_DATAPLANE_CONSTRUCT_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_DATAPLANE_SUBSCRIPTION_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_DATAPLANE_TIMER_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_DATAPLANE_SERVICE_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_DATAPLANE_PUBLISHER_GATED_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_DATAPLANE_POST_ACTIVATE_OK" in process.stdout
    assert "DIRECT_CPP_LIFECYCLE_DATAPLANE_ALL_OK" in process.stdout
