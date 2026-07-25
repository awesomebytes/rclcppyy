"""Direct-node clock jump-callback proofs (live, ROS_DOMAIN_ID=80)."""

from _run_helper import format_output, run_helper


def test_direct_cpp_clock_jump_callbacks_fire_and_contain_exceptions():
    process = run_helper("_direct_cpp_clock_jump_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_CLOCK_JUMP_ACTIVATED_OK" in process.stdout
    assert "DIRECT_CPP_CLOCK_JUMP_FORWARD_OK" in process.stdout
    assert "DIRECT_CPP_CLOCK_JUMP_UNREGISTER_OK" in process.stdout
    assert "DIRECT_CPP_CLOCK_JUMP_EXCEPTION_CONTAINED_OK" in process.stdout
    assert "DIRECT_CPP_CLOCK_JUMP_TEARDOWN_OK" in process.stdout
