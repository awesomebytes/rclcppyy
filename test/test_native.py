from rclcpp_kit.native import NativeSession

import rclcppyy
from rclcppyy._status import reset_status_for_tests


def setup_function():
    reset_status_for_tests()


def test_native_factory_is_thin_and_status_visible():
    session = rclcppyy.native(["program", "--ros-args"])
    assert isinstance(session, NativeSession)
    assert session.capabilities.raw_rclcpp is True

    record = rclcppyy.status()["operations"][-1]
    assert record["backend"] == "cpp"
    assert record["policies"] == ["native", "explicit_opt_in"]
    assert record["metadata"]["operation"] == "native"
    assert record["metadata"]["arguments_count"] == 2
    assert record["metadata"]["capabilities"]["managed_context"] is True


def test_native_symbols_are_part_of_the_small_public_surface():
    assert rclcppyy.NativeSession is NativeSession
    assert rclcppyy.NativeCapabilities().loaned_messages == "publisher_runtime_query"
    assert callable(rclcppyy.publisher_capabilities)
