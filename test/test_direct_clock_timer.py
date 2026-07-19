"""ROS-clock/sim-time create_timer default proof."""

from _run_helper import format_output, run_helper


def test_direct_cpp_create_timer_follows_the_node_ros_clock():
    process = run_helper("_direct_cpp_clock_timer_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CPP_CLOCK_TIMER_TYPE_OK" in process.stdout
    assert "DIRECT_CPP_CLOCK_TIMER_SIM_FROZEN_OK" in process.stdout
    assert "DIRECT_CPP_CLOCK_TIMER_SIM_TICK_OK" in process.stdout
    assert "DIRECT_CPP_CLOCK_TIMER_WALL_OK" in process.stdout
    assert "DIRECT_CPP_CLOCK_TIMER_DESTROY_OK" in process.stdout
    assert "DIRECT_CPP_CLOCK_TIMER_NO_CONVERSION_OK" in process.stdout
    assert "DIRECT_CPP_CLOCK_TIMER_TEARDOWN_OK" in process.stdout
