"""Validate physical command timing, camera associations and episode boundaries."""

import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from recording import Recorder, camera_observations, write_manifest
from replay import inspect_episode, rows
from synthetic_cameras import CAMERAS


class RobotRecordingTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.rec = Recorder(self.root)
        self.addCleanup(self.rec.close)
        self.arm = {"mode": "observe"}

    def test_delayed_hardware_feedback_is_saved_with_its_own_frame_references(self):
        start = time.monotonic_ns() - 30_000_000
        name = self.rec.start(host_ns=start, physical_arm=self.arm)["episode"]
        folder = self.root / name
        raw = {"session": "quest", "seq": 2}
        self.rec.sample(raw, {"hands": {}, "physical_arm": self.arm}, start)
        dispatch = time.monotonic_ns()
        token = self.rec.begin_robot_io(name)
        self.assertIsNotNone(token)
        closing = threading.Thread(target=self.rec.finish)
        closing.start()
        try:
            with self.rec._io:
                self.assertTrue(self.rec._io.wait_for(lambda: self.rec._io_closing, 2))
            self.assertIsNone(self.rec.begin_robot_io(name))
            self.assertTrue(closing.is_alive())
            row = dict(raw, arm="right", command="track", host_monotonic_ns=dispatch,
                       target_deg=[2.0]*7, measured_deg=[1.0]*7, feedback_host_ns=time.monotonic_ns())
            self.rec.robot_io(row, token)
            self.rec.robot_io(row, token)
        finally:
            closing.join(4)
        self.assertFalse(closing.is_alive())
        report = inspect_episode(folder, at=5)
        self.assertEqual(report["status"], "saved")
        self.assertEqual(report["missing_robot_io"], 0)
        self.assertEqual(report["streams"]["observations"], 2)
        self.assertEqual(report["snapshot"]["robot_io"]["right"], row)
        self.assertEqual(next(rows(folder/"feedback.jsonl"))["physical_arm"], self.arm)
        on_input, on_command = rows(folder/"observations.jsonl")
        self.assertEqual((on_input["stream"], on_input["seq"]), ("input", 2))
        self.assertEqual(on_input["cameras"], dict.fromkeys(CAMERAS))
        self.assertEqual(on_command["stream"], "robot_io")
        for cam, ref in on_command["cameras"].items():
            table = list(rows(folder/(cam+".frames.jsonl")))
            frame = table[ref["frame_index"]]
            self.assertEqual(ref["source_index"], frame["source_index"])
            self.assertEqual(ref["host_monotonic_ns"], frame["host_monotonic_ns"])
            self.assertEqual(ref["age_ns"], dispatch - frame["host_monotonic_ns"])
            self.assertGreaterEqual(ref["age_ns"], 0)
        on_command["cameras"]["overview"] = None
        (folder/"observations.jsonl").write_text(
            json.dumps(on_input)+"\n"+json.dumps(on_command)+"\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "invalid command/camera reference"):
            inspect_episode(folder)

    def test_abandoned_cycle_marks_trial_incomplete_and_never_crosses_into_next(self):
        first = self.rec.start(physical_arm=self.arm)["episode"]
        token = self.rec.begin_robot_io(first)
        with patch("recording.ROBOT_IO_DRAIN_S", .01):
            self.assertEqual(self.rec.finish()["last_episode_status"], "incomplete")
        self.assertEqual(inspect_episode(self.root/first)["missing_robot_io"], 1)
        second = self.rec.start(physical_arm=self.arm)["episode"]
        self.assertIsNone(self.rec.begin_robot_io(first))
        self.rec.robot_io({"arm":"right","host_monotonic_ns":time.monotonic_ns()}, token)
        self.assertEqual(self.rec.finish()["last_episode_status"], "saved")
        self.assertEqual(inspect_episode(self.root/second)["streams"]["robot_io"], 0)

    def test_finish_async_keeps_serving_and_closes_admission_before_disk_join(self):
        name = self.rec.start(physical_arm=self.arm)["episode"]
        token = self.rec.begin_robot_io(name)
        entered, release = threading.Event(), threading.Event()
        def slow_index(*args):
            entered.set()
            self.assertTrue(release.wait(4))
            yield from camera_observations(*args)

        with patch("recording.camera_observations", slow_index):
            saving = self.rec.finish_async("success")
            self.assertTrue(saving["finalizing"])
            self.assertFalse(saving["active"])
            self.assertIsNone(self.rec.begin_robot_io(name))
            # Already-admitted feedback still belongs to this episode.
            self.rec.robot_io({"arm":"left", "host_monotonic_ns":time.monotonic_ns(),
                               "session":"quest", "seq":7}, token)
            self.assertTrue(entered.wait(2))
            try:
                self.rec.sample({"session":"quest","seq":8}, {"hands":{}})
                self.rec.event("after-stop")
                with self.assertRaisesRegex(ValueError, "still saving"):
                    self.rec.start(physical_arm=self.arm)
                self.assertTrue(self.rec.status()["finalizing"])
                first_frame = self.rec.snapshot("overview")[1]["index"]
                deadline = time.monotonic() + 2
                while self.rec.snapshot("overview")[1]["index"] == first_frame:
                    self.assertLess(time.monotonic(), deadline, "camera preview froze while saving")
                    time.sleep(.01)
            finally:
                release.set()
            self.rec.close()
        report = inspect_episode(self.root/name)
        self.assertEqual((report["status"], report["outcome"]), ("saved", "success"))
        self.assertEqual(report["streams"]["input"], 0)
        self.assertEqual(report["streams"]["robot_io"], 1)
        self.assertNotIn("after-stop", [r["kind"] for r in rows(self.root/name/"events.jsonl")])

    def test_command_and_stop_while_hardware_files_are_opening(self):
        entered, release = threading.Event(), threading.Event()
        def slow_manifest(path, data):
            if "end_host_ns" not in data:
                entered.set()
                self.assertTrue(release.wait(4))
            write_manifest(path, data)
        with patch("recording.write_manifest", slow_manifest):
            status = self.rec.start(physical_arm=self.arm)
            name = status["episode"]
            self.assertTrue(status["active"])
            self.assertTrue(entered.wait(2))
            try:
                self.assertTrue(self.rec.status()["opening"])
                self.rec.sample({"session":"q","seq":1}, {"hands":{}})
                self.rec.event("before-disk")
                token = self.rec.begin_robot_io(name)
                self.assertIsNotNone(token)
                stamp = time.monotonic_ns()
                self.rec.robot_io({"arm":"right","session":"q","seq":1,"host_monotonic_ns":stamp}, token)
                self.assertTrue(self.rec.finish_async()["finalizing"])
                self.rec.sample({"session":"q","seq":2}, {"hands":{}})
                self.assertIsNone(self.rec.begin_robot_io(name))
            finally:
                release.set()
            self.rec.close()
        summary = inspect_episode(self.root/name, at=5)
        self.assertEqual(summary["status"], "saved")
        self.assertEqual(summary["streams"]["input"], 1)
        self.assertEqual(summary["streams"]["observations"], 2)
        self.assertEqual(summary["snapshot"]["robot_io"]["right"]["seq"], 1)
        self.assertEqual([r["kind"] for r in rows(self.root/name/"events.jsonl")],
                         ["record_start","before-disk","record_end"])

    def test_open_failure_reports_reserved_data_and_next_trial_is_clean(self):
        entered, release = threading.Event(), threading.Event()
        def unavailable(path, data):
            entered.set()
            self.assertTrue(release.wait(4))
            raise OSError("disk full")
        with patch("recording.write_manifest", unavailable):
            first = self.rec.start(physical_arm=self.arm)["episode"]
            self.assertTrue(entered.wait(2))
            try:
                self.rec.sample({"session":"q","seq":4}, {"hands":{}})
                self.rec.event("failed-trial-marker")
                self.rec.robot_io({"arm":"right","host_monotonic_ns":time.monotonic_ns()},
                                  self.rec.begin_robot_io(first))
            finally:
                release.set()
            deadline = time.monotonic() + 2
            while self.rec.status()["opening"]:
                self.assertLess(time.monotonic(), deadline)
                time.sleep(.01)
        status = self.rec.status()
        self.assertEqual(status["last_episode_status"], "start_failed")
        self.assertFalse(status["active"])
        self.assertEqual(status["dropped_input_records"], 1)
        self.assertIn("1 robot cycle(s) unsaved", status["error"])
        second = self.rec.start(physical_arm=self.arm)["episode"]
        self.rec.finish()
        self.assertEqual(inspect_episode(self.root/second)["streams"]["input"], 0)


if __name__ == "__main__":
    unittest.main()
