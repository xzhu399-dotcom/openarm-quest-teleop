"""Exercise independent camera inputs through OpenCV's real file capture and JPEG codec."""

import copy
import json
from pathlib import Path
import tempfile
import time
import unittest

from live_cameras import load_camera_config
from recording import Recorder
from replay import inspect_episode, rows
from synthetic_cameras import CAMERAS


try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None


@unittest.skipUnless(cv2 is not None, "OpenCV camera tests need opencv-python")
class LiveCameraTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = self.root/"cameras.json"
        self.colors = dict(zip(CAMERAS, [(15,40,210), (15,210,40), (210,40,15)]))

    def config(self, short=None, names=CAMERAS):
        cameras = {}
        for cam in names:
            color = self.colors[cam]
            file = self.root/(cam+".avi")
            writer = cv2.VideoWriter(str(file),cv2.VideoWriter_fourcc(*"MJPG"),20,(96,72))
            self.assertTrue(writer.isOpened())
            try:
                for i in range(8 if cam == short else 80):
                    frame = np.full((72,96,3),color,np.uint8)
                    frame[:4, :i%70+1] = 255
                    writer.write(frame)
            finally:
                writer.release()
            cameras[cam] = {"file":file.name, "label":CAMERAS[cam]}
        self.data = {"width":96,"height":72,"fps":20,"cameras":cameras}
        self.path.write_text(json.dumps(self.data))
        return load_camera_config(self.path)

    def open_station_cameras(self, config):
        self.rec = Recorder(self.root/"episodes",camera_config=config)
        self.addCleanup(self.rec.close)
        self.wait_for(lambda:all(c["status"]=="live" for c in self.rec.status()["cameras"]))

    def wait_for(self, fn):
        deadline = time.monotonic()+4
        while not fn():
            self.assertLess(time.monotonic(),deadline)
            time.sleep(.01)

    def test_robot_command_links_all_three_saved_opencv_frames(self):
        self.open_station_cameras(self.config())
        name = self.rec.start(physical_arm={"mode":"observe"})["episode"]
        start_indices = [self.rec.snapshot(cam)[1]["index"] for cam in CAMERAS]
        self.wait_for(lambda: all(self.rec.snapshot(cam)[1]["index"] > first
                                  for cam, first in zip(CAMERAS, start_indices)))
        token = self.rec.begin_robot_io(name)
        stamp = time.monotonic_ns()
        self.rec.robot_io({"session":"xr","seq":5,"arm":"right","command":"observe",
                           "host_monotonic_ns":stamp,"feedback_host_ns":stamp,
                           "measured_deg":[4.0]*7,"target_deg":None}, token)
        self.rec.finish()
        folder = self.root/"episodes"/name
        self.assertEqual(inspect_episode(folder)["streams"]["observations"], 1)
        association = next(rows(folder/"observations.jsonl"))
        self.assertEqual(association["seq"], 5)
        for cam in CAMERAS:
            saved = list(rows(folder/(cam+".frames.jsonl")))
            latest = max(frame["frame_index"] for frame in saved if frame["host_monotonic_ns"] <= stamp)
            self.assertEqual(association["cameras"][cam]["frame_index"], latest)
        # Earlier OpenCV manifests implicitly required all three views.
        manifest_file = folder/"manifest.json"
        legacy = json.loads(manifest_file.read_text())
        legacy.pop("configured_cameras")
        manifest_file.write_text(json.dumps(legacy))
        self.assertEqual(set(inspect_episode(folder)["cameras"]), set(CAMERAS))
        del legacy["cameras"]["left_wrist"]
        manifest_file.write_text(json.dumps(legacy))
        with self.assertRaisesRegex(ValueError, "Expected camera streams"):
            inspect_episode(folder)

    def test_single_camera_physical_episode_records_only_configured_view(self):
        cam = "right_wrist"
        self.open_station_cameras(self.config(names=[cam]))
        self.assertEqual(self.rec.status()["selected_camera"], cam)
        self.assertEqual(self.rec.cycle_camera()["selected_camera"], cam)
        self.assertIsNone(self.rec.snapshot("overview"))
        name = self.rec.start(physical_arm={"mode":"observe"})["episode"]
        first = self.rec.snapshot()[1]["index"]
        self.wait_for(lambda: self.rec.snapshot()[1]["index"] > first)
        stamp = time.monotonic_ns()
        sample = {"session":"quest", "seq":1}
        self.rec.sample(sample, {"hands":{}}, stamp)
        self.rec.robot_io({**sample, "arm":"right", "command":"observe", "measured_deg":[2.0]*7,
                           "host_monotonic_ns":stamp}, self.rec.begin_robot_io(name))
        self.assertEqual(self.rec.finish()["last_episode_status"], "saved")
        folder = self.root / "episodes" / name
        report = inspect_episode(folder, at=5)
        self.assertEqual(set(report["cameras"]), {cam})
        self.assertEqual([p.stem for p in folder.glob("*.mjpg")], [cam])
        self.assertEqual(report["snapshot"]["robot_io"]["right"]["seq"], 1)
        associations = list(rows(folder/"observations.jsonl"))
        self.assertEqual([a["stream"] for a in associations], ["input", "robot_io"])
        for a in associations:
            self.assertEqual(set(a["cameras"]), {cam})
            self.assertIsNotNone(a["cameras"][cam])
        manifest_file = folder / "manifest.json"
        manifest = json.loads(manifest_file.read_text())
        self.assertEqual(manifest["configured_cameras"], [cam])
        manifest["configured_cameras"].append("overview")
        manifest_file.write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError, "Expected camera streams"):
            inspect_episode(folder)

    def test_two_camera_preview_cycles_only_configured_views(self):
        self.open_station_cameras(self.config(names=["overview", "right_wrist"]))
        self.assertEqual(self.rec.snapshot()[1]["camera_id"], "overview")
        self.assertEqual(self.rec.cycle_camera()["selected_camera"], "right_wrist")
        self.assertEqual(self.rec.cycle_camera()["selected_camera"], "overview")

    def test_preview_switch_records_every_camera_and_exports_color(self):
        self.open_station_cameras(self.config())
        rec = self.rec
        name = rec.start("three camera trial")["episode"]
        for cam,color in self.colors.items():
            jpeg,meta = rec.snapshot()
            self.assertEqual((meta["camera_id"],meta["mime"]),(cam,"image/jpeg"))
            decoded = cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
            self.assertLess(np.max(np.abs(decoded[30,30].astype(int)-color)),10)
            rec.cycle_camera()
        self.assertEqual(rec.status()["selected_camera"],"overview")
        self.wait_for(lambda:rec.snapshot()[1]["index"] >= 5)
        done = rec.finish()
        self.assertEqual(done["last_episode_status"],"saved",done)
        path = self.root/"episodes"/name
        summary = inspect_episode(path,done["elapsed_s"],self.root/"stills")
        rows = {cam:[json.loads(v) for v in (path/(cam+".frames.jsonl")).read_text().splitlines()] for cam in CAMERAS}
        for cam,info in summary["cameras"].items():
            self.assertEqual(info["source"],"file")
            self.assertGreater(info["frames"],1)
            self.assertIsNone(info["capture_issue"])
            self.assertTrue(all("crc32" in row and isinstance(row["source_ns"],int) for row in rows[cam]))
            exported = cv2.imread(str(self.root/"stills"/(cam+".png")))
            self.assertLess(np.max(np.abs(exported[30,30].astype(int)-self.colors[cam])),10)
        corrupt = path/"overview.mjpg"
        with corrupt.open("r+b") as stream:
            stream.seek(100)
            value = stream.read(1)[0]
            stream.seek(100)
            stream.write(bytes([value ^ 1]))
        with self.assertRaisesRegex(ValueError,"damaged JPEG"):
            inspect_episode(path)

    def test_failed_camera_keeps_other_views_live_and_preserves_incomplete_trial(self):
        self.open_station_cameras(self.config(short="left_wrist"))
        rec = self.rec
        path = self.root/"episodes"/rec.start()["episode"]
        rec.cycle_camera()
        self.wait_for(lambda:rec.status()["camera_stalled"])
        self.assertIsNone(rec.snapshot())
        self.assertEqual(rec.status()["camera_error"],"End of video file")
        rec.cycle_camera()
        frame = rec.snapshot()[1]["index"]
        self.wait_for(lambda:rec.snapshot()[1]["index"] > frame+1)
        self.assertFalse(rec.status()["camera_stalled"])
        result = rec.finish()
        self.assertEqual(result["last_episode_status"],"incomplete")
        self.assertIn("left_wrist",result["error"])
        info = inspect_episode(path)
        self.assertGreater(info["cameras"]["right_wrist"]["frames"],info["cameras"]["left_wrist"]["frames"])
        self.assertEqual(info["cameras"]["left_wrist"]["capture_issue"],"End of video file")
        # No frames at all from the left camera in this second trial.
        path = self.root/"episodes"/rec.start()["episode"]
        self.wait_for(lambda:rec.snapshot()[1]["index"] > frame+4)
        self.assertEqual(rec.finish()["last_episode_status"],"incomplete")
        self.assertEqual(inspect_episode(path)["cameras"]["left_wrist"]["frames"],0)

    def test_config_validation(self):
        self.config()
        for invalid in ({}, {"side": {"device": 0}}):
            self.path.write_text(json.dumps({"cameras":invalid}))
            with self.assertRaisesRegex(ValueError, "Configure at least one camera"):
                load_camera_config(self.path)
        for key,new in [("device",True),("label","Invalid\nlabel"),("file","right_wrist.avi")]:
            data = copy.deepcopy(self.data)
            if key=="device":
                del data["cameras"]["overview"]["file"]
            data["cameras"]["overview"][key]=new
            self.path.write_text(json.dumps(data))
            with self.assertRaises(ValueError):
                load_camera_config(self.path)


if __name__ == "__main__":
    unittest.main()
