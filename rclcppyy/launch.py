"""Launch-file integration for the RCLCPPYY_ENABLE_HOOK startup hook.

``rclcppyy_env_actions()`` gives every node in a launch file the same
acceleration ``rclcppyy-env``/``rclcppyy-run`` (see ``rclcppyy.cli``) give a
shell: the startup hook installed into the launching interpreter's
site-packages (``rclcppyy.hook``), plus ``SetEnvironmentVariable`` actions so
every process the launch file spawns -- Python or not, ``rclcppyy``-aware or
not -- inherits ``RCLCPPYY_ENABLE_HOOK=1`` and accelerates the instant it
imports ``rclpy`` (see ``rclcppyy._hook_boot``). No changes to the launched
nodes themselves are required; see ``launch/example_accelerated.launch.py``.
"""
from launch.actions import SetEnvironmentVariable

from rclcppyy import hook
from rclcppyy.cli import build_env


def rclcppyy_env_actions(profile="compatible", interfaces=(), optimizations=(), *,
                          install_hook=True):
    """Return ``SetEnvironmentVariable`` actions that enable rclcppyy acceleration.

    Add the returned actions to a ``LaunchDescription`` ahead of any
    ``Node``/``ExecuteProcess`` action so the child processes inherit the env.
    Unless ``install_hook=False``, also installs the startup hook into the
    launching interpreter's site-packages (idempotent; see ``rclcppyy.hook``) so
    the env vars actually have an effect on the launched processes.
    """
    if install_hook:
        hook.install()
    env = build_env(profile, interfaces, optimizations, enable=True)
    return [
        SetEnvironmentVariable(name=name, value=value)
        for name, value in env.items()
    ]
