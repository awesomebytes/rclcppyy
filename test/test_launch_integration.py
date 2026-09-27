"""Tests for rclcppyy.launch's rclcppyy_env_actions() launch-file integration.

Mirrors the caution in test_hook.py / test_cli_tools.py: hook.install() is
redirected to a throwaway temp site dir so these tests never touch the real
environment's site-packages.
"""
import tempfile

import pytest
from launch import LaunchContext
from launch.actions import SetEnvironmentVariable
from launch.utilities import perform_substitutions

from rclcppyy import hook
from rclcppyy import launch as rclcppyy_launch
from rclcppyy._hook_boot import ENABLE_ENV, INTERFACES_ENV, OPTIMIZATIONS_ENV, PROFILE_ENV
from rclcppyy.cli import RMW_ENV, ZENOH_CONFIG_ENV, ZENOH_SHM_CONFIG


@pytest.fixture
def temp_site(monkeypatch):
    original_install = hook.install
    with tempfile.TemporaryDirectory() as site:
        monkeypatch.setattr(rclcppyy_launch.hook, "install",
                            lambda site_dir=None: original_install(site))
        yield site


def _resolved_env(actions):
    context = LaunchContext()
    return {
        perform_substitutions(context, action.name): perform_substitutions(context, action.value)
        for action in actions
    }


def test_returns_set_environment_variable_actions(temp_site):
    actions = rclcppyy_launch.rclcppyy_env_actions()
    assert actions
    assert all(isinstance(action, SetEnvironmentVariable) for action in actions)


def test_default_profile_sets_enable_hook(temp_site):
    actions = rclcppyy_launch.rclcppyy_env_actions()
    env = _resolved_env(actions)
    assert env[ENABLE_ENV] == "1"
    assert env[PROFILE_ENV] == "compatible"
    assert INTERFACES_ENV not in env
    assert OPTIMIZATIONS_ENV not in env


def test_profile_and_interfaces_are_forwarded(temp_site):
    actions = rclcppyy_launch.rclcppyy_env_actions(
        profile="direct_cpp",
        interfaces=["std_msgs/msg/Header"],
        optimizations=["subscription_shared_lease"],
    )
    env = _resolved_env(actions)
    assert env[ENABLE_ENV] == "1"
    assert env[PROFILE_ENV] == "direct_cpp"
    assert env[INTERFACES_ENV] == "std_msgs/msg/Header"
    assert env[OPTIMIZATIONS_ENV] == "subscription_shared_lease"


def test_installs_hook_by_default(temp_site):
    assert not hook.is_installed(temp_site)
    rclcppyy_launch.rclcppyy_env_actions()
    assert hook.is_installed(temp_site)


def test_can_skip_hook_install(temp_site):
    rclcppyy_launch.rclcppyy_env_actions(install_hook=False)
    assert not hook.is_installed(temp_site)


def test_rejects_unknown_profile(temp_site):
    with pytest.raises(ValueError, match="unknown acceleration profile"):
        rclcppyy_launch.rclcppyy_env_actions(profile="silent_fallback")


def test_rmw_zenoh_shm_is_forwarded(temp_site):
    actions = rclcppyy_launch.rclcppyy_env_actions(rmw="zenoh", shm=True)
    env = _resolved_env(actions)
    assert env[ENABLE_ENV] == "1"
    assert env[RMW_ENV] == "rmw_zenoh_cpp"
    assert env[ZENOH_CONFIG_ENV] == ZENOH_SHM_CONFIG


def test_rmw_fastrtps_is_forwarded_without_zenoh_config(temp_site):
    actions = rclcppyy_launch.rclcppyy_env_actions(rmw="fastrtps")
    env = _resolved_env(actions)
    assert env[RMW_ENV] == "rmw_fastrtps_cpp"
    assert ZENOH_CONFIG_ENV not in env


def test_shm_without_zenoh_rejected(temp_site):
    with pytest.raises(ValueError, match="--shm requires --rmw zenoh"):
        rclcppyy_launch.rclcppyy_env_actions(rmw="fastrtps", shm=True)
