"""Exercise the same receiver endpoints used by the mouse and Quest pages."""

import http.client
from contextlib import nullcontext, redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
import time
import unittest
import sys
from unittest.mock import patch

from quest_pose_receiver import main as receiver_main, make_server
from replay import inspect_episode
from station import Station
from test_teleop import hand


def sample(seq, x=0.0, grip=0, events=(), session="operator"):
    pose = {"p": [x, 1.2, -0.4], "q": [0, 0, 0, 1]}
    return {
        "session": session, "seq": seq, "time_ms": time.monotonic() * 1000,
        "frame": "mouse-local", "visible": True, "head": {"p": [0, 1.6, 0], "q": [0, 0, 0, 1]},
        "hands": {"left": None, "right": {"pose": pose, "profile": "mouse", "handedness": "right",
                  "buttons": [{"value": 0, "pressed": False}, {"value": grip, "pressed": bool(grip)}],
                  "axes": []}}, "events": list(events),
    }


def quest_sample(seq, x=0., grip=0, *, b=False, events=(), session="operator"):
    packet = sample(seq, events=events, session=session)
    packet["frame"] = "local-floor"
    right = hand((x, 1.2, -.4), grip=grip, b=b, profile="oculus-touch-v3")
    right.update(handedness="right", source="xr-1", mapping="xr-standard")
    packet["hands"]["right"] = right
    return packet


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

    def test_quest_grip_moves_through_b_and_recording_boundaries(self):
        first = self.ok("/sample", quest_sample(1, grip=1))
        origin = first["hands"]["right"]["tool"]["p"][0]
        self.assertTrue(first["grip_only"])
        self.assertTrue(first["hands"]["right"]["clutched"])
        self.assertFalse(self.ok("/control", {"action": "hold"})["held"])

        # An older Quest page can still send its queued B event and raw button together.
        events = [{"id": 1, "action": "hold"}, {"id": 2, "action": "record"}]
        started = self.ok("/sample", quest_sample(2, .01, 1, b=True, events=events))
        self.assertFalse(started["held"])
        self.assertEqual(started["ack"], 2)
        self.assertTrue(started["recording"]["active"])
        self.assertAlmostEqual(started["hands"]["right"]["tool"]["p"][0], origin + .01)
        moving = self.ok("/sample", quest_sample(3, .02, 1, b=True))
        self.assertAlmostEqual(moving["hands"]["right"]["tool"]["p"][0], origin + .02)

        saved = self.ok("/sample", quest_sample(4, .03, 1, events=[{"id": 3, "action": "record"}]))
        self.assertEqual(saved["recording"]["last_episode_status"], "saved")
        self.assertAlmostEqual(saved["hands"]["right"]["tool"]["p"][0], origin + .03)
        after = self.ok("/sample", quest_sample(5, .04, 1))
        self.assertAlmostEqual(after["hands"]["right"]["tool"]["p"][0], origin + .04)
        released = self.ok("/sample", quest_sample(6, .20))
        self.assertFalse(released["hands"]["right"]["clutched"])
        self.assertEqual(released["hands"]["right"]["tool"], after["hands"]["right"]["tool"])

    def test_physical_home_request_is_explicit_fresh_and_bound_to_the_controller(self):
        self.station.physical_arm = {"mode":"drive","arms":{"right":{"enabled":True}}, "error":None}

        def packet(seq, event_id=None, age_ms=0):
            raw = quest_sample(seq)
            for side, source in (("right","xr-1"), ("left","xr-2")):
                raw["hands"][side] = dict(raw["hands"]["right"], handedness=side, source=source,
                                          target_ray_mode="tracked-pointer")
            if event_id:
                raw["events"] = [{"id": event_id, "action": "restore_robot_initial",
                                  "time_ms": raw["time_ms"] - age_ms}]
            return raw

        self.assertEqual(self.request("/sample", sample(1))[0], 409)
        state = self.ok("/sample", packet(1))
        self.assertFalse(state["grip_only"])
        self.assertTrue(state["local_controls"])
        self.assertEqual(self.request("/control", {"action":"restore_initial"})[0], 409)
        with self.assertRaises(PermissionError):
            self.station.control({"action":"restore_robot_initial", "session":state["session"]}, local=False)
        delayed = self.ok("/sample", packet(2, 1, 500))
        self.assertIsNone(delayed["hardware_return"])
        self.assertIn("expired", delayed["hardware_return_notice"])
        accepted = self.ok("/sample", packet(3, 2))
        intent = accepted["hardware_return"]
        self.assertEqual(intent["sources"], {"left":"xr-2","right":"xr-1"})
        self.assertEqual(self.ok("/sample", packet(4, 2))["hardware_return"], intent)
        self.station.physical_arm["arms"]["right"]["phase"] = "returning"
        self.assertEqual(self.ok("/sample", packet(5, 3))["hardware_return"], intent)
        changed = packet(6)
        changed["hands"]["left"]["source"] = "xr-3"
        self.assertIsNone(self.ok("/sample", changed)["hardware_return"])
        self.station.physical_arm = dict(self.station.physical_arm, mode="shadow")
        self.assertEqual(self.request("/control", {"action":"restore_robot_initial"})[0], 409)
        self.assertIsNone(self.ok("/sample", packet(7, 4))["hardware_return"])

    def test_disk_open_failure_allows_next_record_tap_to_start_a_new_hardware_trial(self):
        self.station.physical_arm = {"mode":"shadow", "arms":{"right":{"enabled":False}}, "error":None}
        with patch("recording._Episode", side_effect=OSError("disk unavailable")):
            first = self.ok("/control", {"action":"record"})["recording"]["episode"]
            deadline = time.monotonic() + 3
            while self.station.recorder.status()["last_episode_status"] != "start_failed":
                self.assertLess(time.monotonic(), deadline)
                time.sleep(.01)
        second = self.ok("/control", {"action":"record"})["recording"]
        self.assertTrue(second["active"])
        self.assertNotEqual(first, second["episode"])

    def test_quest_reanchors_after_tracking_loss_and_reconnect_with_grip_down(self):
        self.ok("/sample", quest_sample(1, grip=1))
        tool = self.ok("/sample", quest_sample(2, .02, 1))["hands"]["right"]["tool"]
        hidden = quest_sample(3, .30, 1)
        hidden["visible"] = False
        paused = self.ok("/sample", hidden)
        self.assertFalse(paused["hands"]["right"]["clutched"])
        self.assertEqual(paused["hands"]["right"]["tool"], tool)

        recovered = self.ok("/sample", quest_sample(4, .60, 1))
        self.assertFalse(recovered["held"])
        self.assertEqual(recovered["hands"]["right"]["tool"], tool)
        moving = self.ok("/sample", quest_sample(5, .61, 1))["hands"]["right"]["tool"]
        self.assertAlmostEqual(moving["p"][0], tool["p"][0] + .01)

        self.ok("/control", {"action": "release", "session": "operator"})
        reconnected = self.ok("/sample", quest_sample(1, -.50, 1, session="new"))
        self.assertFalse(reconnected["held"])
        self.assertEqual(reconnected["hands"]["right"]["tool"], moving)
        moved = self.ok("/sample", quest_sample(2, -.48, 1, session="new"))["hands"]["right"]["tool"]
        self.assertAlmostEqual(moved["p"][0], moving["p"][0] + .02)

        time.sleep(self.station.engine.timeout_s + .1)
        stale = self.ok("/state")
        self.assertFalse(stale["hands"]["right"]["tracked"])
        anchored = self.ok("/sample", quest_sample(3, .75, 1, session="new"))
        self.assertEqual(anchored["hands"]["right"]["tool"], moved)
        fresh = self.ok("/sample", quest_sample(4, .76, 1, session="new"))
        self.assertAlmostEqual(fresh["hands"]["right"]["tool"]["p"][0], moved["p"][0] + .01)

    def test_quest_legacy_return_gesture_saves_without_moving_released_arms(self):
        started = self.ok("/sample", quest_sample(1, events=[{"id": 1, "action": "record"}]))
        self.ok("/sample", quest_sample(2, grip=1))
        moved = self.ok("/sample", quest_sample(3, .03, 1))["hands"]["right"]["tool"]
        self.assertNotEqual(moved, started["hands"]["right"]["tool"])
        saved = self.ok("/sample", quest_sample(4, .03, events=[{"id": 2, "action": "finish_return"}]))
        self.assertEqual(saved["recording"]["last_episode_status"], "saved")
        self.assertEqual(saved["trial_return"]["status"], "stayed")
        self.assertEqual(self.ok("/sample", quest_sample(5, .10))["hands"]["right"]["tool"], moved)

    def test_restore_initial_button_and_chord_preserve_recording_and_marker(self):
        control = {"action": "restore_initial"}
        self.assertEqual(self.ok("/control", control)["trial_return"]["status"], "skipped")
        first = self.ok("/sample", quest_sample(1))
        ready = self.ok("/control", control)
        self.assertEqual(ready["trial_return"]["status"], "completed")
        self.assertFalse(ready["recording"]["active"])

        self.ok("/sample", quest_sample(2, grip=1))
        self.ok("/sample", quest_sample(3, .03, 1))
        start = self.ok("/sample", quest_sample(4, events=[{"id":1, "action":"record"}]))
        chord = {"id":2, "action":"restore_initial"}
        returning = self.ok("/sample", quest_sample(5, events=[chord]))
        self.assertEqual(returning["trial_return"]["status"], "returning")
        self.assertEqual(returning["trial_return"]["target"], "home")
        self.assertEqual(returning["ack"], 2)
        self.assertEqual(returning["camera"]["id"], first["camera"]["id"])
        self.assertTrue(returning["recording"]["active"])
        self.assertEqual(returning["recording"]["episode"], start["recording"]["episode"])
        self.ok("/sample", quest_sample(6, events=[chord]))
        end = self.ok("/sample", quest_sample(7, events=[{"id":3, "action":"record"}]))
        self.assertEqual(end["recording"]["last_episode_status"], "saved")
        after = self.ok("/control", control)
        self.assertFalse(after["recording"]["active"])
        self.assertEqual(after["trial_return"]["status"], "returning")
        episode = self.directory / end["recording"]["episode"]
        event_kinds = [json.loads(line)["kind"] for line in (episode/"events.jsonl").read_text().splitlines()]
        self.assertEqual(event_kinds.count("restore_initial"), 1)
        self.assertNotIn("marker", event_kinds)

    def test_physical_bench_rejects_restore_without_starting_a_path(self):
        self.station.gripper_bench = {"arm":"right", "mode":"shadow"}
        self.ok("/sample", quest_sample(1))
        self.assertEqual(self.request("/control", {"action":"restore_initial"})[0], 409)
        state = self.ok("/sample", quest_sample(2, events=[{"id":1, "action":"restore_initial"}]))
        self.assertEqual(state["ack"], 1)
        self.assertEqual(state["trial_return"]["status"], "idle")
        self.assertFalse(state["recording"]["active"])

    def test_quest_camera_loss_and_unreachable_goal_do_not_latch_motion(self):
        first = self.ok("/sample", quest_sample(1, grip=1))
        origin = first["hands"]["right"]["tool"]["p"][0]
        status = self.station.recorder.status()
        status.update(camera_mode="opencv", camera_stalled=True)
        packet = quest_sample(2, .02, 1)
        packet["view_valid"] = False
        with patch.object(self.station.recorder, "status", return_value=status):
            moving = self.ok("/sample", packet)
        self.assertFalse(moving["held"])
        before = moving["hands"]["right"]
        self.assertAlmostEqual(before["tool"]["p"][0], origin + .02)
        far = self.ok("/sample", quest_sample(3, 2., 1))["hands"]["right"]
        self.assertEqual(far["tool"], before["tool"])
        self.assertEqual(far["robot"]["joint_angles_rad"], before["robot"]["joint_angles_rad"])
        back = self.ok("/sample", quest_sample(4, .04, 1))["hands"]["right"]
        self.assertAlmostEqual(back["tool"]["p"][0], origin + .04)

    def test_physical_gripper_snapshot_is_separate_from_simulated_pose(self):
        hardware = {"arm":"right", "mode":"shadow", "phase":"waiting", "enabled":False,
                    "measured_deg":4.25, "target_deg":None, "feedback_host_ns":time.monotonic_ns()}
        self.station.gripper_bench = hardware
        status, _, _ = self.request("/sample", sample(1))
        self.assertEqual(status, 409)
        first = quest_sample(1, events=[{"id":1, "action":"record"}])
        running = self.ok("/sample", first)
        self.assertFalse(running["grip_only"])
        self.assertEqual(running["gripper_bench"], hardware)
        self.assertTrue(running["recording"]["active"])
        end = self.ok("/control", {"action":"record"})
        self.assertGreater(end["trial_epoch"], running["trial_epoch"])
        held = self.ok("/control", {"action":"hold"})
        self.assertGreater(held["safety_epoch"], end["safety_epoch"])
        self.ok("/sample", quest_sample(2))
        self.assertFalse(self.ok("/control", {"action": "resume"})["held"])
        stopped = self.ok("/sample", quest_sample(3, grip=1, b=True))
        self.assertEqual(stopped["reason"], "operator hold")
        after_b = self.ok("/sample", quest_sample(4, .04, 1))
        self.assertTrue(after_b["held"])
        self.assertEqual(after_b["hands"]["right"]["tool"], stopped["hands"]["right"]["tool"])
        episode = self.directory / end["recording"]["episode"]
        self.assertEqual(inspect_episode(episode)["physical_gripper"]["mode"], "shadow")
        feedback = json.loads((episode / "feedback.jsonl").read_text().splitlines()[0])
        self.assertEqual(feedback["physical_gripper"], hardware)
        self.assertIsNone(feedback["hands"]["right"]["measured_joints"])
        self.assertTrue(feedback["simulated"])

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


class ReceiverModeTests(unittest.TestCase):
    def test_hardware_selection_through_receiver_entry_point(self):
        import quest_pose_receiver as receiver
        stop = threading.Event()
        stop.set()  # A worker can fault before the server starts listening for requests.
        for mode, execute in (("robot-readonly", False), ("robot-drive", True)):
            with self.subTest(mode=mode), TemporaryDirectory() as temp:
                args = ["receiver", "--host", "127.0.0.1", "--port", "0", "--mode", mode,
                        "--robot-calibration", "lab.json", "--episodes", temp]
                with patch.object(sys, "argv", args), \
                        patch.object(receiver, "make_tls_context", return_value=None), \
                        patch.object(receiver.ThreadingHTTPServer, "handle_request") as request, \
                        patch("robot_hardware.robot_hardware", return_value=nullcontext(stop)) as bridge, \
                        redirect_stdout(io.StringIO()):
                    receiver_main()
                request.assert_not_called()
                self.assertEqual(bridge.call_count, 1)
                self.assertEqual(bridge.call_args.args[1:], (Path("lab.json"), execute))

    def test_idle_robot_receiver_exits_after_worker_fault(self):
        from quest_pose_receiver import serve_robot
        stop = threading.Event()
        with make_server("127.0.0.1", 0, None, robot=True) as server:
            server.token = "sample-robot-token"
            thread = threading.Thread(target=serve_robot, args=(server, stop), daemon=True)
            thread.start()
            conn = http.client.HTTPConnection(*server.server_address, timeout=2)
            try:
                conn.request("GET", "/?key=sample-robot-token")
                response = conn.getresponse()
                self.assertEqual(response.status, 303)
                cookie = response.getheader("Set-Cookie").split(";", 1)[0]
                response.read()
                self.assertFalse(cookie.startswith("station_key="))
                conn.request("GET", "/", headers={"Cookie": f"station_key=simulator-token; {cookie}"})
                response = conn.getresponse()
                self.assertEqual(response.status, 200)
                response.read()
            finally:
                conn.close()
                stop.set()
                thread.join(1)
            self.assertFalse(thread.is_alive())

    def test_hardware_mode_requires_explicit_configuration_and_quest_session(self):
        for args in (["--host", "127.0.0.1", "--mode", "robot-drive"],
                     ["--sim", "--mode", "robot-drive", "--robot-calibration", "lab.json"],
                     ["--host", "127.0.0.1", "--mode", "robot-drive", "--robot-calibration", "lab.json",
                      "--pairing-code-file", "pin"],
                     ["--host", "127.0.0.1", "--robot-calibration", "lab.json"]):
            with self.subTest(args=args), patch.object(sys, "argv", ["receiver", *args]), \
                    redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                receiver_main()
            self.assertEqual(error.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
