#!/usr/bin/env python3
"""Tests for rclcppyy.kits.control_kit (a Python ros2_control controller inside the
real controller_manager, via cppyy).

ros2_control is an optional dependency (the pixi `control` env), absent from the
default env. The whole module auto-skips when the controller_manager headers are not
installed, so the default `pixi run test` is unaffected. Run the real thing with
`pixi run -e control test-control`.

All tests share one process; bringup_control() is idempotent. The CM owns a
pal_statistics async publisher thread, so each CM is torn down (deactivate + reset)
before the next; the module-scoped `ros` fixture brings rclcpp up once. A benign
`class_loader` "SEVERE WARNING ... will NOT be unloaded" and pal_statistics "registry
not found" warnings may print; they are not errors (see the kit docstring / REPORT)."""
import os

import pytest

_HAVE_CONTROL = os.path.isdir(
    os.path.join(os.environ.get("CONDA_PREFIX", ""), "include", "controller_manager"))

pytestmark = pytest.mark.skipif(
    not _HAVE_CONTROL, reason="ros2_control not installed (use the control env)")

if _HAVE_CONTROL:
    from rclcppyy.bringup_rclcpp import bringup_rclcpp
    from rclcppyy.kits import control_kit as ck

JOINTS = ["joint1", "joint2"]
FWD = "forward_command_controller/ForwardCommandController"


@pytest.fixture(scope="module")
def ros():
    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    ck.bringup_control()
    return rclcpp


def _pd_controller_class():
    class PD(ck.ControllerInterface):
        def __init__(self, target, kp=0.4):
            super().__init__()
            self.target = target
            self.kp = kp
            self.n = 0
            self.last = None

        def on_init(self):
            return ck.CallbackReturn.SUCCESS

        def command_interface_configuration(self):
            return ck.interface_config(["%s/position" % j for j in JOINTS])

        def state_interface_configuration(self):
            return ck.interface_config(["%s/position" % j for j in JOINTS])

        def on_configure(self, s):
            return ck.CallbackReturn.SUCCESS

        def on_activate(self, s):
            return ck.CallbackReturn.SUCCESS

        def on_deactivate(self, s):
            return ck.CallbackReturn.SUCCESS

        def update(self, time_, period):
            self.n += 1
            cur = [ck.read_state(self, i) for i in range(ck.n_command_interfaces(self))]
            self.last = cur
            for i in range(len(cur)):
                ck.write_command(self, i, cur[i] + self.kp * (self.target[i] - cur[i]))
            return ck.return_type.OK

    return PD


def test_bringup_idempotent(ros):
    ns = ck.bringup_control()
    assert ns is ck.bringup_control()
    assert hasattr(ns, "ControllerInterface")


def test_mock_system_urdf_shape():
    urdf = ck.mock_system_urdf(["a", "b"])
    assert "mock_components/GenericSystem" in urdf
    assert '<joint name="a">' in urdf
    assert "command_interface" in urdf and "state_interface" in urdf


def test_ok_helper(ros):
    # a returned uint8 return_type crosses as a 1-char str; ok() reads it
    assert ck.ok("\x00") is True
    assert ck.ok("\x01") is False
    assert ck.ok(ck.return_type.OK) is True


def test_controller_manager_and_empty_loop(ros):
    rig = ck.make_controller_manager(ck.mock_system_urdf(JOINTS), update_rate=100)
    try:
        assert rig.cm.is_resource_manager_initialized()
        assert rig.cm.get_update_rate() == 100
        for _ in range(20):
            rig.update()
    finally:
        rig._teardown()


def test_stock_controller_lifecycle(ros):
    rig = ck.make_controller_manager(ck.mock_system_urdf(JOINTS), update_rate=100)
    try:
        rig.load_controller("fwd", FWD,
                            parameters={"joints": JOINTS, "interface_name": "position"})
        assert rig.configure("fwd")
        assert rig.activate(["fwd"])
        fwd = rig.cm.get_loaded_controllers()[0].c
        assert fwd.get_lifecycle_id() == 3          # PRIMARY_STATE_ACTIVE
        assert rig.deactivate(["fwd"])
        assert fwd.get_lifecycle_id() == 2          # PRIMARY_STATE_INACTIVE
    finally:
        rig._teardown()


def test_python_controller_converges(ros):
    PD = _pd_controller_class()
    rig = ck.make_controller_manager(ck.mock_system_urdf(JOINTS), update_rate=100)
    try:
        target = [0.5, -0.3]
        pd = PD(target)
        rig.add_python_controller(pd, "pd")
        assert rig.configure("pd")
        assert rig.activate(["pd"])
        rig.run(seconds=1.5, rate_hz=100)
        assert pd.n > 100                            # update() was dispatched to Python
        assert pd.last is not None
        assert abs(pd.last[0] - target[0]) < 1e-2    # PD converged on mock hardware
        assert abs(pd.last[1] - target[1]) < 1e-2
    finally:
        rig._teardown()


def test_interface_config_types(ros):
    # A uint8_t-backed enum read back from a struct member crosses as a 1-char str
    # (same quirk as a returned return_type -- see ok()); ord() it to compare.
    cfg = ck.interface_config(["j/position"], "individual")
    assert ord(cfg.type) == int(ck.interface_configuration_type.INDIVIDUAL)
    assert list(cfg.names) == ["j/position"]
    assert ord(ck.interface_config([], "none").type) == int(
        ck.interface_configuration_type.NONE)
