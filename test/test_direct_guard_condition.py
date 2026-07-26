"""Direct-node guard condition proofs (live, ROS_DOMAIN_ID=82)."""

from _run_helper import format_output, run_helper


def test_direct_cpp_guard_condition_dispatch_teardown_and_exception_containment():
    process = run_helper("_direct_cpp_guard_condition_helper.py", timeout=120)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_GUARD_CONDITION_CREATE_OK" in process.stdout
    assert "DIRECT_CPP_GUARD_CONDITION_TRIGGER_OK" in process.stdout
    assert "DIRECT_CPP_GUARD_CONDITION_RETRIGGER_OK" in process.stdout
    assert "DIRECT_CPP_GUARD_CONDITION_EXCEPTION_CONTAINED_OK" in process.stdout
    assert "DIRECT_CPP_GUARD_CONDITION_DESTROY_OK" in process.stdout
    assert "DIRECT_CPP_GUARD_CONDITION_TEARDOWN_OK" in process.stdout
