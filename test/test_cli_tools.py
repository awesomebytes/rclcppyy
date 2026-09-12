"""Tests for the rclcppyy-env / rclcppyy-run shell integration (rclcppyy.cli).

File-op tests redirect ``hook.install()`` to a throwaway temp site dir (same
caution as test_hook.py: never touch the real environment's site-packages). The
end-to-end test spawns a fresh interpreter that processes that temp dir's
installed ``.pth`` via ``site.addsitedir`` -- the same code path a real process
on PATH would run automatically -- and checks whether rclcppyy-run's env
actually routed rclpy through the C++ backend.
"""
import os
import subprocess
import sys
import tempfile

import pytest

from rclcppyy import cli
from rclcppyy import hook
from rclcppyy._hook_boot import ENABLE_ENV, INTERFACES_ENV, OPTIMIZATIONS_ENV, PROFILE_ENV


@pytest.fixture
def temp_site(monkeypatch):
    """Redirect every hook.install() call made through rclcppyy.cli to a temp dir."""
    original_install = hook.install
    with tempfile.TemporaryDirectory() as site:
        monkeypatch.setattr(cli.hook, "install", lambda site_dir=None: original_install(site))
        yield site


class TestBuildEnv:

    def test_defaults_to_compatible_profile(self):
        env = cli.build_env()
        assert env == {ENABLE_ENV: "1", PROFILE_ENV: "compatible"}

    def test_profile_interfaces_and_optimizations(self):
        env = cli.build_env(
            "direct_cpp", ["std_msgs/msg/Header"], ["subscription_shared_lease"])
        assert env == {
            ENABLE_ENV: "1",
            PROFILE_ENV: "direct_cpp",
            INTERFACES_ENV: "std_msgs/msg/Header",
            OPTIMIZATIONS_ENV: "subscription_shared_lease",
        }

    def test_disable_only_sets_zero(self):
        assert cli.build_env(enable=False) == {ENABLE_ENV: "0"}

    def test_rejects_unknown_profile(self):
        with pytest.raises(ValueError, match="unknown acceleration profile"):
            cli.build_env("silent_fallback")


class TestEnvMain:

    def test_prints_valid_shell_exports(self, temp_site, capsys):
        rc = cli.env_main(["--profile", "direct_cpp", "--interfaces",
                           "std_msgs/msg/Header"])
        assert rc == 0
        out = capsys.readouterr().out
        assert hook.is_installed(temp_site)

        # This is exactly "eval $(rclcppyy-env)": feed the printed exports to a
        # shell and read the vars back.
        proc = subprocess.run(
            ["bash", "-c", '%s\necho "$%s:$%s:$%s"' % (
                out, ENABLE_ENV, PROFILE_ENV, INTERFACES_ENV)],
            capture_output=True, text=True, timeout=10,
        )
        assert proc.returncode == 0, proc.stderr
        assert proc.stdout.strip() == "1:direct_cpp:std_msgs/msg/Header"

    def test_env_vars_are_sufficient_for_activation(self, temp_site, capsys):
        # The keys env_main prints must line up 1:1 with what _hook_boot.activate()
        # actually reads -- otherwise "eval $(rclcppyy-env)" would be a no-op.
        cli.env_main(["--profile", "compatible"])
        out = capsys.readouterr().out
        printed = {line.split("=", 1)[0][len("export "):]
                   for line in out.strip().splitlines()}
        assert printed <= {ENABLE_ENV, PROFILE_ENV, INTERFACES_ENV, OPTIMIZATIONS_ENV}
        assert ENABLE_ENV in printed

    def test_disable_prints_zero_and_skips_install(self, temp_site, capsys):
        rc = cli.env_main(["--disable"])
        assert rc == 0
        out = capsys.readouterr().out
        assert out.strip() == "export %s=0" % ENABLE_ENV
        # Nothing to turn off yet -- --disable must not install the hook.
        assert not hook.is_installed(temp_site)

    def test_rejects_unknown_profile(self, temp_site, capsys):
        rc = cli.env_main(["--profile", "bogus"])
        assert rc == 1
        assert "unknown acceleration profile" in capsys.readouterr().err


class TestRunMain:

    def test_runs_python_script_with_hook_env(self, temp_site, monkeypatch):
        captured = {}

        class _Result:
            returncode = 0

        def fake_run(command, env, **kwargs):
            captured["command"] = command
            captured["env"] = env
            return _Result()

        monkeypatch.setattr(cli.subprocess, "run", fake_run)
        rc = cli.run_main(["--profile", "direct_cpp", "my_node.py", "--foo", "bar"])

        assert rc == 0
        assert captured["command"] == [sys.executable, "my_node.py", "--foo", "bar"]
        assert captured["env"][ENABLE_ENV] == "1"
        assert captured["env"][PROFILE_ENV] == "direct_cpp"
        assert hook.is_installed(temp_site)

    def test_passes_through_non_python_commands(self, temp_site, monkeypatch):
        captured = {}

        class _Result:
            returncode = 3

        def fake_run(command, env, **kwargs):
            captured["command"] = command
            return _Result()

        monkeypatch.setattr(cli.subprocess, "run", fake_run)
        rc = cli.run_main(["ros2", "topic", "hz", "/some_topic"])

        assert rc == 3
        assert captured["command"] == ["ros2", "topic", "hz", "/some_topic"]

    def test_requires_a_command(self, temp_site):
        with pytest.raises(SystemExit):
            cli.run_main([])

    def test_activates_backend_via_helper_script(self, temp_site, capfd, monkeypatch):
        """End-to-end: rclcppyy-run's env really does route rclpy through the hook."""
        with tempfile.TemporaryDirectory() as scratch:
            helper = os.path.join(scratch, "check_backend.py")
            with open(helper, "w") as f:
                f.write(
                    "import site; site.addsitedir(%r)\n"
                    "import rclpy\n"
                    "from rclpy.node import Node\n"
                    "print('ROUTED' if Node.create_publisher.__name__ == "
                    "'_create_publisher_wrapper' else 'STOCK')\n" % temp_site
                )

            for var in (ENABLE_ENV, PROFILE_ENV, INTERFACES_ENV, OPTIMIZATIONS_ENV):
                monkeypatch.delenv(var, raising=False)

            real_run = subprocess.run

            def bounded_run(command, env, **kwargs):
                # Real rclcpp bringup in the child (the real cost of
                # acceleration) is slow; mirror test_hook.py's generous timeout.
                return real_run(command, env=env, timeout=180, **kwargs)

            monkeypatch.setattr(cli.subprocess, "run", bounded_run)

            rc = cli.run_main([helper])

            assert rc == 0
            assert "ROUTED" in capfd.readouterr().out
