#!/usr/bin/env python3
"""Tests for the opt-in RCLCPPYY_ENABLE_HOOK startup hook (rclcppyy.hook).

File-op tests (install/uninstall/status) run against a throwaway temp site dir, so
they never touch the real environment. The behaviour tests spawn a fresh
interpreter that processes the installed ``.pth`` from that temp dir via
``site.addsitedir`` (the same code path site.py runs at startup), then import rclpy
and check whether the C++ backend was applied:

  * RCLCPPYY_ENABLE_HOOK=1  -> rclpy.create_node / spin_once are the rclcppyy wrappers
  * unset                   -> stock rclpy, untouched
  * RCLCPPYY_ENABLE_HOOK=0   -> stock rclpy (only "1" enables)

The enabled case brings up rclcpp in the child (the real cost of acceleration), so
it gets a generous timeout; the disabled cases are cheap (no bringup).
"""
import os
import subprocess
import sys
import tempfile
import unittest

from rclcppyy import hook


def _run(code, env_extra, timeout):
    env = os.environ.copy()
    # Never let an env var inherited from the caller decide the outcome.
    env.pop("RCLCPPYY_ENABLE_HOOK", None)
    env.update(env_extra)
    return subprocess.run([sys.executable, "-c", code], capture_output=True,
                          text=True, timeout=timeout, env=env)


def _probe_code(site_dir):
    # addsitedir processes the .pth (-> activate() installs the hook) before the
    # import rclpy below, exactly as site.py would at interpreter startup.
    return (
        "import site; site.addsitedir(%r)\n"
        "import rclpy\n"
        "wrapped = rclpy.create_node.__name__ == '_create_node_wrapper' and \\\n"
        "          rclpy.spin_once.__name__ == '_spin_once_wrapper'\n"
        "print('WRAPPED' if wrapped else 'STOCK')\n" % site_dir
    )


class TestHookInstall(unittest.TestCase):

    def test_install_uninstall_status(self):
        with tempfile.TemporaryDirectory() as site:
            self.assertFalse(hook.is_installed(site))
            paths = hook.install(site)
            self.assertTrue(os.path.exists(paths["pth"]))
            self.assertTrue(os.path.exists(paths["boot"]))
            self.assertTrue(hook.is_installed(site))
            self.assertTrue(hook.status(site)["installed"])
            # Idempotent second install is a no-op that still verifies installed.
            hook.install(site)
            self.assertTrue(hook.is_installed(site))
            removed = hook.uninstall(site)
            self.assertEqual(len(removed), 2)
            self.assertFalse(hook.is_installed(site))

    def test_pth_line_imports_boot_and_guards(self):
        # The .pth must import the installed boot copy, call activate(), and swallow
        # any exception so it can never crash an interpreter start.
        self.assertIn("_rclcppyy_hook", hook._PTH_LINE)
        self.assertIn("activate()", hook._PTH_LINE)
        self.assertIn("except Exception", hook._PTH_LINE)


class TestHookBehaviour(unittest.TestCase):

    def test_enabled_accelerates_rclpy(self):
        with tempfile.TemporaryDirectory() as site:
            hook.install(site)
            proc = _run(_probe_code(site), {"RCLCPPYY_ENABLE_HOOK": "1"}, timeout=180)
            self.assertIn("WRAPPED", proc.stdout,
                          "\nstdout:\n%s\nstderr:\n%s" % (proc.stdout, proc.stderr))

    def test_unset_leaves_stock_rclpy(self):
        with tempfile.TemporaryDirectory() as site:
            hook.install(site)
            proc = _run(_probe_code(site), {}, timeout=60)
            self.assertIn("STOCK", proc.stdout,
                          "\nstdout:\n%s\nstderr:\n%s" % (proc.stdout, proc.stderr))

    def test_zero_leaves_stock_rclpy(self):
        # Only "1" enables; "0" (like any non-"1" value) leaves stock rclpy.
        with tempfile.TemporaryDirectory() as site:
            hook.install(site)
            proc = _run(_probe_code(site), {"RCLCPPYY_ENABLE_HOOK": "0"}, timeout=60)
            self.assertIn("STOCK", proc.stdout,
                          "\nstdout:\n%s\nstderr:\n%s" % (proc.stdout, proc.stderr))


if __name__ == "__main__":
    unittest.main()
