"""Exercise the same receiver endpoints used by the mouse and Quest pages."""

import http.client
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
import time
import unittest
from unittest.mock import patch

from quest_pose_receiver import make_server
from replay import inspect_episode
from station import Station


def sample(seq, x=0.0, grip=0, events=(), session="operator"):
    pose = {"p": [x, 1.2, -0.4], "q": [0, 0, 0, 1]}
    return {
        "session": session, "seq": seq, "time_ms": time.monotonic() * 1000,
        "frame": "mouse-local", "visible": True, "head": {"p": [0, 1.6, 0], "q": [0, 0, 0, 1]},
        "hands": {"left": None, "right": {"pose": pose, "profile": "mouse", "handedness": "right",
                  "buttons": [{"value": 0, "pressed": False}, {"value": grip, "pressed": bool(grip)}],
                  "axes": []}}, "events": list(events),
    }


class ReceiverTests(unittest.TestCase):
    def setUp(self):
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)
        self.station = Station(self.directory)
        self.addCleanup(self.station.close)
        self.server = make_server("127.0.0.1", 0, self.station)
        self.thread = threading.Thread(target=self.server.serve_forever, args=(0.01,))
        self.thread.start()
        self.addCleanup(self.stop_server)
        self.http = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=3)
        self.addCleanup(self.http.close)

    def stop_server(self):
        self.server.shutdown()
        self.thread.join()
        self.server.server_close()

    def request(self, path, data=None, extra=None):
        headers = {"Content-Type": "application/json", **(extra or {})}
        self.http.request("POST" if data is not None else "GET", path,
                          json.dumps(data) if data is not None else None, headers)
        res = self.http.getresponse()
        body = res.read()
        return res.status, res.headers, json.loads(body) if body and res.headers.get_content_type() == "application/json" else body

    def ok(self, path, data=None):
        status, _, value = self.request(path, data)
        self.assertEqual(status, 200, value)
        return value

    def test_owner_release_capture_and_malformed_sample(self):
        first = self.ok("/sample", sample(1))
        self.assertTrue(first["simulated"])
        self.assertIsNone(first["hands"]["left"]["initial"])
        self.assertEqual(self.request("/sample", sample(2, session="other"))[0], 409)
        nxt = self.ok("/sample", sample(2, x=0.05))
        self.assertAlmostEqual(nxt["hands"]["right"]["delta"]["p"][0], 0.05)
        captured = self.ok("/control", {"action":"capture"})
        self.assertEqual(captured["captured"], ["right"])
        self.assertEqual(captured["hands"]["right"]["delta"]["p"], [0.0]*3)
        saved = json.loads((self.directory / "initial_poses.json").read_text(encoding="utf-8"))
        self.assertEqual(saved["initial"]["right"], captured["hands"]["right"]["initial"])
        packet = sample(3)
        packet["hands"]["right"]["pose"]["q"] = [0, 0, 0, 0]
        self.assertEqual(self.request("/sample", packet)[0], 400)
        self.assertTrue(self.ok("/state")["held"])
        self.assertIsNone(self.ok("/control", {"action": "release", "session": "operator"})["owner"])
        self.assertEqual(self.request("/sample", sample(4))[0], 409)
        replacement = self.ok("/sample", sample(1, session="other"))
        self.assertTrue(replacement["held"])
        self.assertFalse(self.ok("/control", {"action":"resume"})["held"])

    def test_record_edges_and_camera_switch_are_replayable(self):
        start = [{"id":1, "action":"record"}]
        first = self.ok("/sample", sample(1, events=start))
        self.assertTrue(first["recording"]["active"])
        self.assertFalse(self.ok("/sample", sample(1, events=start))["accepted"])
        self.ok("/sample", sample(2, x=0.5, grip=1))
        move = self.ok("/sample", sample(3, x=0.56, grip=1))
        tool = move["hands"]["right"]["tool"]
        self.assertAlmostEqual(tool["p"][0] - first["hands"]["right"]["tool"]["p"][0], 0.06)
        view = self.ok("/control", {"action":"view"})
        status, hdr, png = self.request("/camera")
        self.assertEqual(status, 200)
        self.assertTrue(png.startswith(b"\x89PNG"))
        self.assertEqual(hdr["X-Camera-Id"], view["camera"]["id"])
        self.ok("/control", {"action":"displayed", "camera_id":view["camera"]["id"],
                             "index":int(hdr["X-Frame-Index"]), "session":"operator"})
        self.ok("/control", {"action":"mark"})
        last = self.ok("/sample", sample(4, x=0.8, grip=0, events=[{"id":2, "action":"record"}]))
        self.assertEqual(last["hands"]["right"]["tool"], tool)
        self.assertFalse(last["recording"]["active"])
        episode = self.directory / last["recording"]["episode"]
        result = inspect_episode(episode, at=0.2)
        self.assertEqual(result["streams"]["input"], 4)
        self.assertEqual(set(result["cameras"]), {"overview", "left_wrist", "right_wrist"})
        self.assertTrue(all(c["frames"] for c in result["cameras"].values()))
        events = [json.loads(line)["kind"] for line in (episode / "events.jsonl").read_text().splitlines()]
        self.assertEqual(events, ["record_start", "trial_start_pose", "view_requested",
                                 "view_displayed", "marker", "trial_finish", "record_end"])
        self.assertGreaterEqual(result["snapshot"]["action"]["elapsed_s"], 0)

    def test_controller_finish_return_is_acknowledged_and_recorded(self):
        def event(i, action):
            return [{"id":i, "action":action}]
        self.ok("/sample", sample(1, events=event(1,"record")))
        self.ok("/sample", sample(2, grip=1))
        moved = self.ok("/sample", sample(3,x=.035,grip=1))
        raw = sample(4, x=.06, grip=0, events=event(2,"finish_return")+event(3,"mark"))
        ended = self.ok("/sample", raw)
        self.assertEqual(ended["trial_return"]["status"], "returning")
        self.assertEqual(ended["ack"], 3)
        self.assertFalse(ended["recording"]["active"])
        self.assertEqual(ended["hands"]["right"]["tool"], moved["hands"]["right"]["tool"])
        self.assertFalse(self.ok("/sample", raw)["accepted"])
        episode = self.directory / ended["recording"]["episode"]
        events = [json.loads(r) for r in (episode/"events.jsonl").read_text().splitlines()]
        by_kind = {e["kind"]:e["details"] for e in events}
        self.assertEqual(by_kind["trial_finish"]["choice"], "return")
        self.assertEqual(set(by_kind["trial_start_pose"]["joints_rad"]), {"left","right"})
        self.assertIn("marker", by_kind)
        self.ok("/sample", sample(5,x=.035,grip=1))
        self.assertEqual(self.ok("/state")["trial_return"]["status"], "cancelled")
        actions = [json.loads(r) for r in (episode/"actions.jsonl").read_text().splitlines()]
        self.assertEqual([a["seq"] for a in actions], [1,2,3,4])

    def test_both_ending_choices_freeze_on_boundary_and_grip_refuses_return(self):
        for action, status in (("record","stayed"),("finish_return","skipped")):
            with self.subTest(action=action):
                self.ok("/control", {"action":"record"})
                seq = (self.ok("/state")["seq"] or 0)+1
                self.ok("/sample", sample(seq))
                before = self.ok("/sample", sample(seq+1, grip=1))
                packet = sample(seq+2, x=.10, grip=1, events=[{"id":before["ack"]+1, "action":action}])
                after = self.ok("/sample", packet)
                self.assertEqual(after["trial_return"]["status"],status)
                self.assertEqual(before["hands"]["right"]["robot"]["joint_angles_rad"],
                                 after["hands"]["right"]["robot"]["joint_angles_rad"])
                self.assertFalse(after["hands"]["right"]["clutched"])
                high = self.ok("/sample",sample(seq+3,x=.11,grip=1))
                self.assertFalse(high["hands"]["right"]["clutched"])

    def test_held_final_sample_can_save_but_never_queues_a_return(self):
        self.ok("/control", {"action":"record"})
        self.ok("/sample", sample(1))
        self.ok("/sample", sample(2,grip=1))
        moved = self.ok("/sample", sample(3,x=.035,grip=1))
        raw = sample(4,x=.04,grip=0,events=[{"id":1,"action":"finish_return"}])
        raw["view_valid"] = False
        ended = self.ok("/sample", raw)
        self.assertEqual(ended["trial_return"]["status"], "skipped")
        self.assertEqual(ended["trial_return"]["reason"], "camera display unavailable")
        self.assertEqual(ended["recording"]["last_episode_status"],"saved")
        self.assertTrue(self.ok("/control", {"action":"resume"})["held"])
        self.ok("/sample", sample(5))
        self.assertFalse(self.ok("/control", {"action":"resume"})["held"])
        after = self.ok("/sample", sample(6))
        self.assertEqual(after["hands"]["right"]["tool"], moved["hands"]["right"]["tool"])

    def test_selected_camera_loss_cancels_return_and_gates_resume(self):
        self.ok("/sample", sample(1,events=[{"id":1,"action":"record"}]))
        self.ok("/sample", sample(2,grip=1))
        self.ok("/sample", sample(3,x=.035,grip=1))
        end = self.ok("/sample",sample(4,events=[{"id":2,"action":"finish_return"}]))
        self.assertEqual(end["trial_return"]["status"],"returning")
        camera_status = self.station.recorder.status()
        camera_status.update(camera_mode="opencv",camera_stalled=True)
        with patch.object(self.station.recorder, "status", return_value=camera_status):
            held = self.ok("/sample",sample(5))
            self.assertEqual(held["trial_return"]["status"],"cancelled")
            self.assertEqual(held["reason"],"selected camera feed unavailable")
            self.assertTrue(self.ok("/control", {"action":"resume"})["held"])
            camera_status["camera_stalled"] = False
            self.assertFalse(self.ok("/control", {"action":"resume"})["held"])
        after = self.ok("/sample",sample(6))
        self.assertEqual(after["hands"]["right"]["tool"],end["hands"]["right"]["tool"])

    def test_failed_start_is_acknowledged_and_retrying_same_edge_does_not_start(self):
        packet = sample(1,events=[{"id":1,"action":"record"}])
        with patch.object(self.station.recorder, "start", side_effect=OSError("disk unavailable")):
            failure = self.ok("/sample",packet)
        self.assertEqual(failure["ack"],1)
        self.assertEqual(failure["trial_return"],{"status":"start_failed","reason":"disk unavailable"})
        self.assertFalse(failure["recording"]["active"])
        packet.update(seq=2)
        self.assertFalse(self.ok("/sample",packet)["recording"]["active"])

    def test_incomplete_save_blocks_return(self):
        self.ok("/sample",sample(1,events=[{"id":1,"action":"record"}]))
        self.ok("/sample",sample(2,grip=1))
        moved = self.ok("/sample",sample(3,x=.035,grip=1))
        finish = self.station.recorder.finish
        def incomplete(outcome):
            return dict(finish(outcome),last_episode_status="incomplete",error="camera lost")
        with patch.object(self.station.recorder,"finish",side_effect=incomplete):
            ended = self.ok("/sample",sample(4,events=[{"id":2,"action":"finish_return"}]))
        self.assertEqual(ended["trial_return"]["status"],"incomplete")
        self.assertTrue(ended["held"])
        self.assertFalse(ended["recording"]["active"])
        self.assertEqual(self.ok("/sample",sample(5))["hands"]["right"]["tool"],moved["hands"]["right"]["tool"])

    def test_stale_input_logged_without_polling(self):
        self.ok("/control", {"action":"record"})
        self.ok("/sample", sample(0))
        # No HTTP state read or new input should be needed to observe this transition.
        time.sleep(self.station.engine.timeout_s + 0.15)
        status = self.station.recorder.finish()
        events = [json.loads(line) for line in (self.directory / status["episode"] / "events.jsonl").read_text().splitlines()]
        self.assertTrue(any(e["kind"] == "hold" and e["details"]["reason"] == "stale input" for e in events))

    def test_release_between_deliveries_discards_previous_clutch_anchor(self):
        self.ok("/sample", sample(1))
        self.ok("/sample", sample(2, grip=1))
        before = self.ok("/sample", sample(3, x=0.03, grip=1))["hands"]["right"]["tool"]
        for i, (x, grip) in enumerate([(0.6, 1), (0.65, 0), (0.7, 1)], 4):
            packet = sample(i, x=x, grip=grip)
            packet["hands"]["right"]["grip_epoch"] = 1
            hand = self.ok("/sample", packet)["hands"]["right"]
            self.assertEqual(hand["tool"], before)
        self.assertTrue(hand["clutched"])

    def test_page_assets_and_transport_guards(self):
        for path in ("/", "/app.js", "/scene.js", "/xr.js", "/xr_robot.js", "/style.css", "/legacy.html"):
            self.assertTrue(self.ok(path))
        self.assertEqual(self.request("/sample", {"junk":"x"*17000})[0], 413)
        self.assertEqual(self.request("/control", {"action":"record"}, {"Origin":"https://elsewhere.invalid"})[0], 403)
        self.assertEqual(self.request("/state", extra={"Host":"elsewhere.invalid"})[0], 403)
        self.assertEqual(self.request("/../station.py")[0], 404)
        self.assertEqual(self.request("/pose", {"left":None, "right":[1,2]})[0], 400)
        self.server.token = "pairing-test-key"
        self.assertEqual(self.request("/state")[0], 403)
        status, hdr, _ = self.request("/?key=pairing-test-key")
        self.assertEqual(status, 303)
        self.assertIn("HttpOnly", hdr["Set-Cookie"])
        self.assertEqual(self.request("/state", extra={"Cookie":"station_key=pairing-test-key"})[0], 200)


if __name__ == "__main__":
    unittest.main()
