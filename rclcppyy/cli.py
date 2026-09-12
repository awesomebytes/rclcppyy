"""Shell and process integration for the RCLCPPYY_ENABLE_HOOK startup hook.

``rclcppyy-env`` and ``rclcppyy-run`` (thin executables installed from
``scripts/rclcppyy_env.py`` / ``scripts/rclcppyy_run.py``, see CMakeLists.txt) are
both built on the same two things: the startup hook installed into the active
interpreter's site-packages (``rclcppyy.hook``) and the env vars that turn it on
for a process (``rclcppyy._hook_boot``). This module holds that shared logic so
neither script has to duplicate it:

    eval $(rclcppyy-env)                  # this shell now accelerates rclpy
    rclcppyy-run my_node.py               # equivalent to the two steps above
    rclcppyy-run ros2 topic hz /some_topic  # works for any command, not just .py
"""
import argparse
import os
import subprocess
import sys

from rclcppyy import hook
from rclcppyy._hook_boot import ENABLE_ENV, INTERFACES_ENV, OPTIMIZATIONS_ENV, PROFILE_ENV
from rclcppyy.policy import resolve_policy


def _split_csv(values):
    """Flatten repeated ``--flag a,b --flag c`` values into ``[a, b, c]``."""
    items = []
    for value in values or ():
        items.extend(item.strip() for item in value.split(",") if item.strip())
    return items


def build_env(profile="compatible", interfaces=(), optimizations=(), *, enable=True):
    """Return the ``{ENV_VAR: value}`` mapping that (de)activates the startup hook.

    Raises ``ValueError`` for an unknown ``profile`` (mirrors ``resolve_policy``).
    """
    if not enable:
        return {ENABLE_ENV: "0"}
    resolve_policy(profile)  # validates; raises ValueError on an unknown profile
    env = {ENABLE_ENV: "1", PROFILE_ENV: profile}
    if interfaces:
        env[INTERFACES_ENV] = ",".join(interfaces)
    if optimizations:
        env[OPTIMIZATIONS_ENV] = ",".join(optimizations)
    return env


def _add_common_args(parser):
    parser.add_argument(
        "--profile", default="compatible",
        help="activation profile, i.e. RCLCPPYY_HOOK_PROFILE (default: compatible)")
    parser.add_argument(
        "--interfaces", action="append", default=[],
        help="comma-separated RCLCPPYY_DIRECT_INTERFACES entries (repeatable)")
    parser.add_argument(
        "--optimizations", action="append", default=[],
        help="comma-separated RCLCPPYY_DIRECT_OPTIMIZATIONS entries (repeatable)")
    parser.add_argument(
        "--disable", action="store_true",
        help="print/apply RCLCPPYY_ENABLE_HOOK=0 instead of enabling it")


def _resolve_env(args, prog):
    try:
        return build_env(
            args.profile,
            _split_csv(args.interfaces),
            _split_csv(args.optimizations),
            enable=not args.disable,
        )
    except ValueError as exc:
        print("%s: %s" % (prog, exc), file=sys.stderr)
        return None


def env_main(argv=None):
    """Implementation of ``rclcppyy-env``: print shell-eval-able exports."""
    parser = argparse.ArgumentParser(
        prog="rclcppyy-env",
        description="Print shell exports that turn on rclcppyy's "
                    "RCLCPPYY_ENABLE_HOOK startup hook: eval $(rclcppyy-env)")
    _add_common_args(parser)
    args = parser.parse_args(argv)

    if not args.disable:
        try:
            hook.install()
        except Exception as exc:
            print("rclcppyy-env: failed to install startup hook: %s" % exc,
                  file=sys.stderr)
            return 1

    env = _resolve_env(args, "rclcppyy-env")
    if env is None:
        return 1
    for key, value in env.items():
        print("export %s=%s" % (key, value))
    return 0


def run_main(argv=None):
    """Implementation of ``rclcppyy-run``: run a command with the hook enabled."""
    parser = argparse.ArgumentParser(
        prog="rclcppyy-run",
        description="Run a command with rclcppyy's RCLCPPYY_ENABLE_HOOK startup "
                    "hook enabled: equivalent to `eval $(rclcppyy-env) && <command>`. "
                    "Put rclcppyy-run's own flags before the command.")
    _add_common_args(parser)
    parser.add_argument(
        "command", nargs=argparse.REMAINDER,
        help="command to run, e.g. a .py script or `ros2 topic hz ...`")
    args = parser.parse_args(argv)

    if not args.command:
        parser.error("no command given")

    if not args.disable:
        try:
            hook.install()
        except Exception as exc:
            print("rclcppyy-run: failed to install startup hook: %s" % exc,
                  file=sys.stderr)
            return 1

    activation_env = _resolve_env(args, "rclcppyy-run")
    if activation_env is None:
        return 1

    command = list(args.command)
    if command[0].endswith(".py"):
        command = [sys.executable] + command

    child_env = os.environ.copy()
    child_env.update(activation_env)
    result = subprocess.run(command, env=child_env)
    return result.returncode


if __name__ == "__main__":
    sys.exit(env_main())
