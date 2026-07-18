#!/usr/bin/env python3
"""Focused contract tests for process-local backend reporting."""

import gc
import json
import threading
import unittest
import weakref

import rclcppyy
from rclcppyy._status import record_decision, reset_status_for_tests


class TestStatus(unittest.TestCase):

    def setUp(self):
        reset_status_for_tests()

    def tearDown(self):
        reset_status_for_tests()

    def test_empty_snapshot_has_stable_json_schema(self):
        snapshot = rclcppyy.status()

        self.assertEqual(snapshot["schema_version"], 1)
        self.assertGreater(snapshot["process_id"], 0)
        self.assertEqual(snapshot["limits"], {"records_per_kind": 256})
        self.assertEqual(snapshot["nodes"], [])
        self.assertEqual(snapshot["entities"], [])
        self.assertEqual(snapshot["operations"], [])
        self.assertEqual(
            snapshot["counts"],
            {
                kind: {"cpp": 0, "python": 0, "unsupported": 0}
                for kind in ("nodes", "entities", "operations")
            },
        )
        json.dumps(snapshot)

    def test_records_are_value_only_and_snapshots_are_isolated(self):
        class Payload:
            def __str__(self):
                return "payload-value"

        payload = Payload()
        payload_ref = weakref.ref(payload)
        record_id = record_decision(
            "entities",
            "cpp",
            "test decision",
            policies=("direct_cpp_message",),
            metadata={"payload": payload, "nested": {"enabled": True}},
        )
        del payload
        gc.collect()

        self.assertIsNone(payload_ref())
        snapshot = rclcppyy.status()
        self.assertEqual(snapshot["entities"][0]["id"], record_id)
        self.assertTrue(record_id.startswith("entity-"))
        self.assertEqual(snapshot["entities"][0]["metadata"]["payload"], "payload-value")
        snapshot["entities"][0]["metadata"]["payload"] = "mutated"
        self.assertEqual(
            rclcppyy.status()["entities"][0]["metadata"]["payload"],
            "payload-value",
        )

    def test_history_is_bounded_while_aggregate_counts_remain_exact(self):
        capacity = rclcppyy.status()["limits"]["records_per_kind"]
        total = capacity + 19
        for index in range(total):
            record_decision(
                "operations",
                "python",
                "bounded history test",
                metadata={"operation": "test", "index": index},
            )

        snapshot = rclcppyy.status()
        self.assertEqual(len(snapshot["operations"]), capacity)
        self.assertEqual(snapshot["counts"]["operations"]["python"], total)
        self.assertEqual(snapshot["dropped_records"]["operations"], total - capacity)
        self.assertEqual(snapshot["operations"][0]["metadata"]["index"], total - capacity)

    def test_concurrent_writers_are_serialized(self):
        writers = 8
        records_per_writer = 75

        def write_records(writer):
            for index in range(records_per_writer):
                record_decision(
                    "entities",
                    "cpp",
                    "thread safety test",
                    metadata={"writer": writer, "index": index},
                )

        threads = [threading.Thread(target=write_records, args=(index,)) for index in range(writers)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        snapshot = rclcppyy.status()
        self.assertEqual(
            snapshot["counts"]["entities"]["cpp"],
            writers * records_per_writer,
        )
        retained_ids = [record["id"] for record in snapshot["entities"]]
        self.assertEqual(len(retained_ids), len(set(retained_ids)))
        json.dumps(snapshot)

    def test_reset_clears_records_and_advances_generation(self):
        generation = rclcppyy.status()["generation"]
        record_decision("nodes", "unsupported", "test")

        reset_status_for_tests()

        snapshot = rclcppyy.status()
        self.assertEqual(snapshot["generation"], generation + 1)
        self.assertEqual(snapshot["nodes"], [])
        self.assertEqual(snapshot["counts"]["nodes"]["unsupported"], 0)

    def test_invalid_record_values_are_rejected(self):
        with self.assertRaises(ValueError):
            record_decision("callbacks", "cpp", "test")
        with self.assertRaises(ValueError):
            record_decision("operations", "gpu", "test")


if __name__ == "__main__":
    unittest.main()
