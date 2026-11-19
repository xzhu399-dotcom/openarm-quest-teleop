"""Exercise the terminal pose report with controller samples and a real trial recorder."""

from pathlib import Path
import queue
import re
from tempfile import TemporaryDirectory
import unittest

from pose_report import report_lines
from station import Station
from test_station import sample
from test_teleop import hand


class PoseReportTests(unittest.TestCase):
    def setUp(self):
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.trial_events = queue.SimpleQueue()
        self.station = Station(Path(directory.name), trial_events=self.trial_events)
        self.addCleanup(self.station.close)

    def send(self, seq, x=.2, trigger=0., grip=0., events=()):
        packet = sample(seq, events=events)
        packet["frame"] = "local"
        packet["hands"] = {
            "left": hand(p=(-.3, 1.3, -.6), q=(0., .6, 0., .8), profile="oculus-touch-v3"),
            "right": hand(p=(x, 1.2, -.4), q=(0., 0., .6, .8), grip=grip, trigger=trigger,
                          profile="oculus-touch-v3"),
        }
        return self.station.sample(packet)

    def assert_pose(self, line, xyz, xyzw):
        for tag, expected in (("XYZ m", xyz), ("XYZW", xyzw)):
            values = re.search(re.escape(tag) + r" \[([^\]]+)\]", line)
            self.assertIsNotNone(values, line)
            numbers = [float(x) for x in re.split(r"[ ,]+", values.group(1).strip())]
            self.assertEqual(numbers, [round(v, 3) for v in expected])

    def test_controller_telemetry_and_saved_trial(self):
        idle = "\n".join(report_lines(self.station.state()))
        self.assertIn("Recording IDLE", idle)
        self.send(1, events=[{"id": 1, "action": "record"}])
        self.send(2, grip=1)
        moved = self.send(3, x=.22, grip=1, trigger=.5)
        lines = report_lines(self.station.state())
        self.assertIn("Recording ACTIVE", "\n".join(lines))
        for label, pose in (("Headset", moved["head"]), ("Left handle", moved["hands"]["left"]["pose"]),
                            ("Right handle", moved["hands"]["right"]["pose"])):
            line = next(row for row in lines if label in row)
            self.assert_pose(line, pose["p"], pose["q"])
        idx = next(i for i, row in enumerate(lines) if "Right handle" in row)
        right = lines[idx:]
        delta = next(row for row in right if "from initial" in row)
        self.assert_pose(delta, [.02, 0, 0], [0, 0, 0, 1])
        target = next(row for row in right if "simulated tool target" in row)
        commanded = moved["hands"]["right"]["tool"]
        self.assert_pose(target, commanded["p"], commanded["q"])
        self.assertIn("closure 50%", "\n".join(right))
        self.assertIn("jaw 44.0 mm", "\n".join(right))

        ended = self.send(4, x=.22, events=[{"id": 2, "action": "record"}])
        report = "\n".join(report_lines(self.station.state()))
        self.assertIn("Recording IDLE", report)
        self.assertIn("last " + ended["recording"]["episode"] + " (saved)", report)
        self.assertIn("jaw 44.0 mm", report)
        self.assertEqual(ended["hands"]["right"]["tool"], commanded)

    def test_rapid_record_edges_are_reported_even_between_snapshots(self):
        taps = [{"id": 1, "action": "record"}, {"id": 2, "action": "record"}]
        state = self.send(1, events=taps)
        self.assertFalse(state["recording"]["active"])
        episode = state["recording"]["episode"]
        self.assertEqual(self.trial_events.get_nowait(), ("START", episode, "recording"))
        self.assertEqual(self.trial_events.get_nowait(), ("STOP", episode, "saved"))
        self.send(2, events=taps)
        with self.assertRaises(queue.Empty):
            self.trial_events.get_nowait()

    def test_shutdown_reports_interrupted_episode(self):
        state = self.send(1, events=[{"id": 1, "action": "record"}])
        episode = state["recording"]["episode"]
        self.assertEqual(self.trial_events.get_nowait(), ("START", episode, "recording"))
        self.station.close()
        self.assertEqual(self.trial_events.get_nowait(), ("STOP", episode, "interrupted"))


if __name__ == "__main__":
    unittest.main()
