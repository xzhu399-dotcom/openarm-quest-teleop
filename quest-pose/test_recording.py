import contextlib
import json
import tempfile
import time
import unittest
from pathlib import Path

from recording import Recorder
from replay import inspect_episode
from robot_sim import HOME, RobotArm, Y, turn
from synthetic_cameras import CAMERAS, SIGNATURE, render


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.resources = contextlib.ExitStack()
        self.addCleanup(self.resources.close)
        self.root = Path(self.resources.enter_context(tempfile.TemporaryDirectory())) / "episodes"
        self.rec = Recorder(self.root)
        self.resources.callback(self.rec.close)

    def test_standalone_reference_preserves_current_session_only(self):
        path = self.root / "initial_poses.json"
        left = {"p": [-0.2, 1, 0], "q": [0, 0, 0, 1]}
        right = {"p": [0.3, 1, 0], "q": [0, 0, 1, 0]}
        state = {
            "session": "one", "seq": 15, "time_ms": 34, "arrival_s": 5.0,
            "age_s": 0.125, "axes": "+X right, +Y up, -Z forward", "frame": "local",
            "hands": {"left": {"initial": left}, "right": {"initial": right}}, "captured": [],
        }
        self.assertIsNone(self.rec.save_reference(state))
        self.assertFalse(path.exists())
        state["captured"] = ["right"]
        self.assertEqual(self.rec.save_reference(state), path)
        first = json.loads(path.read_text())
        self.assertEqual(first["initial"], {"left": left, "right": right})
        self.assertEqual(first["captured"], ["right"])
        self.assertEqual(first["arrival_host_ns"], 5_000_000_000)
        self.assertEqual(first["capture_host_ns"], 5_125_000_000)
        self.assertEqual(first["source_time_ms"], 34)
        self.assertEqual(first["units"]["q"], "unit quaternion XYZW")
        self.assertFalse(self.rec.status()["active"])
        state.update(session="two", seq=1, frame="mouse-local")
        state["hands"]["left"]["initial"] = None
        self.rec.save_reference(state)
        self.rec.close()
        updated = json.loads(path.read_text())
        self.assertEqual(updated["initial"], {"right": right})
        self.assertEqual(updated["frame"], "mouse-local")
        self.assertEqual(updated["session"], "two")
        self.assertEqual(updated["seq"], 1)
        self.assertEqual(list(self.root.iterdir()), [path])

    def test_switch_mark_record_and_replay(self):
        start = time.monotonic_ns()
        name = self.rec.start("test grasp", host_ns=start)["episode"]
        raw = {"session":"s","seq":1,"time_ms":1}
        tool = {"p":[0.25,1,-0.45],"q":[0,0,0,1]}
        state = {"clock_offset_estimate_s":2.5, "hands":{"right":{
            "tool":tool,"initial":tool,"delta":{"p":[0,0,0],"q":[0,0,0,1]},"closure":0.4,
        }}}
        self.rec.sample(raw, state, start)
        state["hands"]["right"]["tool"]["p"][0] = 1234
        self.rec.mark("grasp")
        self.assertEqual(self.rec.cycle_camera()["selected_camera"], "left_wrist")
        self.rec.event("hold", {"reason":"stale input"})
        frames = [self.rec.snapshot(cam) for cam in CAMERAS]
        self.assertEqual(len(set(f[0] for f in frames)), 3)
        self.assertEqual(self.rec.snapshot()[1]["camera_id"], "left_wrist")
        deadline = time.monotonic() + 2
        while self.rec.snapshot("overview")[1]["index"] == frames[0][1]["index"]:
            self.assertLess(time.monotonic(), deadline)
            time.sleep(0.01)
        status = self.rec.finish("success")
        self.assertFalse(status["active"])
        ep = self.root/name
        summary = inspect_episode(ep, at=status["elapsed_s"], frame_dir=self.root/"stills")
        self.assertEqual(summary["outcome"], "success")
        self.assertEqual(summary["streams"], {"input":1,"actions":1,"feedback":1,"events":5})
        self.assertEqual(summary["snapshot"]["action"]["elapsed_s"], 0)
        self.assertEqual(summary["snapshot"]["action"]["hands"]["right"]["tool"]["p"][0], 0.25)
        for cam in CAMERAS:
            self.assertGreaterEqual(summary["cameras"][cam]["frames"], 2)
            self.assertTrue((self.root/"stills"/(cam+".png")).read_bytes().startswith(SIGNATURE))
        manifest = json.loads((ep / "manifest.json").read_text())
        self.assertEqual(manifest["start_host_ns"], start)
        self.assertEqual(manifest["input_clock_offset_estimate_s"], {"s":2.5})
        self.assertEqual(manifest["task"], "test grasp")
        with (ep/"actions.jsonl").open() as f:
            action = json.load(f)
        self.assertEqual(action["motor_commands"], [])
        self.assertIsNone(action["ik"])

    def test_shutdown_preserves_interrupted_episode(self):
        ep = self.root/self.rec.start()["episode"]
        self.rec.close()
        summary = inspect_episode(ep)
        self.assertEqual(summary["status"], "interrupted")
        self.assertEqual(summary["streams"]["events"], 2)
        self.assertEqual(len(summary["cameras"]), 3)

    def test_wrist_camera_follows_jaw_and_wrist_orientation(self):
        arm = RobotArm("right")
        def views(closure):
            snapshot = {"right":arm.snapshot(closure)}
            return [render(cam, 0, 0.0, snapshot) for cam in ("overview", "right_wrist")]
        opened = views(0.0)
        closed = views(1.0)
        self.assertTrue(all(a != b for a,b in zip(opened,closed)))
        self.assertTrue(arm.solve({"p":HOME["right"], "q":turn(Y,0.25)}))
        rotated = views(1.0)
        self.assertTrue(all(a != b for a,b in zip(closed,rotated)))

    def test_detects_corrupt_video_and_timestamps(self):
        ep = self.root/self.rec.start()["episode"]
        self.rec.finish()
        table = ep/"left_wrist.frames.jsonl"
        original = table.read_bytes()
        table.write_text("")
        with self.assertRaisesRegex(ValueError, "differ in length"):
            inspect_episode(ep)
        table.write_bytes(original)
        path = ep/"right_wrist.png"
        data = bytearray(path.read_bytes())
        data[-20] ^= 1
        path.write_bytes(data)
        with self.assertRaisesRegex(ValueError, "damaged"):
            inspect_episode(ep)


if __name__ == "__main__":
    unittest.main()
