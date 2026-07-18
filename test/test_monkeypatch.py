#!/usr/bin/env python3
"""Compatible activation preserves stock contracts and accelerates publishing."""
import unittest

from _run_helper import run_helper, format_output


class TestMonkeypatch(unittest.TestCase):

    def test_enable_cpp_acceleration_path(self):
        proc = run_helper("_monkeypatch_helper.py")
        out = proc.stdout
        self.assertIn("MESSAGE_CONTRACT_OK", out, format_output(proc))
        self.assertIn("NODE_AUTHORITY_OK", out, format_output(proc))
        self.assertIn("ROUNDTRIP_OK", out, format_output(proc))
        self.assertIn("GRAPH_IDENTITY_OK", out, format_output(proc))
        self.assertIn("STATUS_OK", out, format_output(proc))
        self.assertIn("STOCK_TEARDOWN_OK", out, format_output(proc))
        self.assertIn("MONKEYPATCH_ALL_OK", out, format_output(proc))
        self.assertEqual(proc.returncode, 0, format_output(proc))


if __name__ == "__main__":
    unittest.main()
