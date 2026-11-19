"""Offline acceptance tests for the common Quest and mouse packet path."""

import math
import unittest

from teleop import IDENTITY, Teleop, difference, tool_goal


def pose(p=(0, 1, -0.4), q=IDENTITY):
    return {"p": list(p), "q": list(q)}


def hand(p=(0, 1, -0.4), q=IDENTITY, grip=0, trigger=0, profile="mouse", b=False):
    values = [trigger, grip, 0, 0, 0, int(b)]
    return {"pose": pose(p, q), "profile": profile, "buttons":
            [{"value": v, "pressed": v >= 0.5} for v in values], "axes": [0, 0, 0, 0]}


def axis_rotation(axis, degrees):
    angle = math.radians(degrees) / 2
    return [v * math.sin(angle) for v in axis] + [math.cos(angle)]


class RelativePoseTest(unittest.TestCase):
    def setUp(self):
        self.engine = Teleop()
        self.seq = 0
        self.now = 100.0

    def send(self, right=None, **changes):
        self.seq += 1
        self.now += 0.02
        sample = {"session": "desk", "seq": self.seq, "time_ms": self.seq * 20,
                  "frame": "mouse-local", "visible": True, "head": pose((0, 1.6, 0)),
                  "hands": {"right": right, "left": None}}
        sample.update(changes)
        return self.engine.apply(sample, self.now)

    def right(self, **kwargs):
        return self.send(hand(**kwargs))["hands"]["right"]

    def assert_vector(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for value, want in zip(actual, expected):
            self.assertAlmostEqual(value, want)

    def test_first_capture_and_clutch_are_distinct(self):
        initial = self.right()["initial"]
        idle = self.right(p=(0.1, 1, -0.4))
        engaged = self.right(p=(0.1, 1, -0.4), grip=1)
        self.assertEqual(idle["tool"], engaged["tool"])
        self.assertEqual(engaged["initial"], initial)
        self.assertEqual(engaged["anchor"], idle["pose"])
        moved = self.right(p=(0.1, 1, -0.5), grip=1)
        self.assert_vector(moved["delta"]["p"], [0.1, 0, -0.1])
        self.assert_vector(moved["clutch_delta"]["p"], [0, 0, -0.1])
        self.assert_vector(moved["tool"]["p"], [0.25, 1, -0.55])
        refused = self.engine.capture(self.now)
        self.assertEqual(refused["captured"], [])
        self.assertEqual(refused["hands"]["right"]["initial"], initial)
        released = self.right(p=(0.3, 1, -0.5))
        capture = self.engine.capture(self.now)
        self.assertEqual(capture["captured"], ["right"])
        captured = capture["hands"]["right"]
        self.assertEqual(captured["tool"], moved["tool"])
        self.assertEqual(captured["initial"], released["pose"])
        self.assert_vector(captured["delta"]["p"], [0, 0, 0])

    def test_head_only_motion_and_other_arm(self):
        engine = self.engine
        r, left = hand(), hand(p=(-0.2, 1, -0.5))
        self.send(hands={"right": r, "left": left})
        clutched = {"right": hand(grip=1), "left": left}
        old = self.send(hands=clutched)
        rotated = pose((1, 1.3, 2), axis_rotation((0, 1, 0), 100))
        new = self.send(hands=clutched, head=rotated)
        for side in ("left", "right"):
            self.assertEqual(new["hands"][side]["tool"], old["hands"][side]["tool"])
        self.right(p=(0.15, 1, -0.4), grip=1)
        state = engine.snapshot(self.now)
        self.assertNotEqual(state["hands"]["right"]["tool"], old["hands"]["right"]["tool"])
        self.assertEqual(state["hands"]["left"]["tool"], old["hands"]["left"]["tool"])

    def test_rotation_in_place_and_quaternion_sign(self):
        self.right()
        origin = self.right(grip=1)["tool"]
        q = axis_rotation((1, 0, 0), 70)
        rotated = self.right(q=q, grip=1)
        self.assertEqual(rotated["tool"]["p"], origin["p"])
        self.assert_vector(rotated["tool"]["q"], q)
        flipped = self.right(q=[-v for v in q], grip=1)
        self.assert_vector(flipped["tool"]["q"], q)
        self.assert_vector(flipped["delta"]["q"], q)

    def test_rotation_is_world_relative_and_alignment_transforms_both_deltas(self):
        start = axis_rotation((0, 0, 1), 90)
        # A 90-degree world-X rotation applied *after* the hand's initial Z rotation.
        end = [0.5, -0.5, 0.5, 0.5]
        delta = difference(pose((0.2, 1, -0.4), end), pose(q=start))
        result = tool_goal(delta, pose((0.25, 1, -0.45)), start, scale=0.5)
        self.assert_vector(result["p"], [0.25, 1.1, -0.45])
        self.assert_vector(result["q"], axis_rotation((0, 1, 0), 90))

    def test_grasp_preserved_on_release_and_mismatched_reclutch(self):
        self.right()
        self.right(grip=1)
        held = self.right(grip=1, trigger=0.8)
        self.assertEqual(held["closure"], 0.8)
        released = self.right(p=(0.5, 1, -0.2), trigger=0.05)
        self.assertEqual(released["closure"], 0.8)
        reclutched = self.right(p=(0.5, 1, -0.2), trigger=0.05, grip=1)
        self.assertEqual(held["tool"], reclutched["tool"])
        self.assertEqual(reclutched["closure"], 0.8)
        self.assertFalse(reclutched["trigger_live"])
        self.assertEqual(self.right(p=(0.5, 1, -0.2), grip=1, trigger=0.4)["closure"], 0.8)
        self.assertEqual(self.right(p=(0.5, 1, -0.2), grip=1, trigger=0.9)["closure"], 0.9)

    def test_loss_of_only_one_hand_requires_fresh_edge_on_that_side(self):
        self.send(hands={"right": hand(), "left": hand()})
        old = self.send(hands={"right": hand(grip=1), "left": hand(grip=1)})
        missing = self.send(hands={"right": hand(p=(0.1, 1, -0.4), grip=1), "left": None})
        self.assertFalse(missing["held"])
        self.assertTrue(missing["hands"]["right"]["clutched"])
        left = self.send(hands={"right": hand(), "left": hand(p=(0.4, 1, -0.1), grip=1)})["hands"]["left"]
        self.assertFalse(left["clutched"])
        self.assertEqual(left["tool"], old["hands"]["left"]["tool"])

    def test_stale_input_latches_and_requires_release_after_resume(self):
        self.right()
        self.right(grip=1)
        old = self.right(p=(0.1, 1.1, -0.5), grip=1)["tool"]
        self.now += 0.5
        self.assertEqual(self.engine.snapshot(self.now)["reason"], "stale input")
        self.assertTrue(self.engine.resume(self.now)["held"])
        # A fresh browser clock has advanced while tracking was stopped.
        self.seq += 25
        self.assertTrue(self.send(hand(grip=1))["held"])
        self.assertFalse(self.engine.resume(self.now)["held"])
        self.assertFalse(self.right(grip=1)["clutched"])
        self.right()
        self.assertEqual(self.right(grip=1)["tool"], old)

    def test_lifecycle_faults_are_distinct_and_clear_anchors(self):
        cases = [({"visible": False}, "input hidden"), ({"reset": True}, "reference frame reset"),
                 ({"frame": "local"}, "reference frame reset"), ({"head": None}, "head tracking lost"),
                 ({"hands": {}}, "controller tracking lost"), ({"session": "other"}, "session changed")]
        for change, reason in cases:
            with self.subTest(change=change):
                self.setUp()
                self.right()
                engaged = self.right(grip=1)
                state = self.send(hand(grip=1), **change)
                self.assertTrue(state["held"])
                self.assertEqual(state["reason"], reason)
                right = state["hands"]["right"]
                self.assertIsNone(right["anchor"])
                self.assertEqual(right["tool"], engaged["tool"])
                if reason == "reference frame reset":
                    self.assertIsNone(right["initial"])

    def test_disconnect_and_reconnect_with_grip_down(self):
        self.right()
        old = self.right(grip=1)
        state = self.engine.disconnect(self.now)
        self.assertEqual(state["reason"], "input disconnected")
        self.assertIsNone(state["hands"]["right"]["pose"])
        self.assertTrue(self.engine.resume(self.now)["held"])
        after = self.send(hand(grip=1), session="new")
        self.assertTrue(after["held"])
        self.assertEqual(after["ack"], 0)
        self.engine.resume(self.now)
        still_down = self.send(hand(grip=1, p=(1.2, 1, -0.4)), session="new")
        self.assertFalse(still_down["hands"]["right"]["clutched"])
        self.assertEqual(still_down["hands"]["right"]["tool"], old["tool"])

    def test_duplicate_reordered_and_old_source_data(self):
        self.right()
        original = self.send(hand(grip=1))
        extra = [{"id": 1, "action": "record"}]
        for change in ({"seq": original["seq"]}, {"seq": 1}, {"time_ms": 0}):
            result = self.send(hand(p=(0.4, 1, -0.2), grip=1), events=extra, **change)
            self.assertFalse(result["accepted"])
            self.assertEqual(result["arrival_s"], original["arrival_s"])
            self.assertEqual(result["events"], [])
        for _ in range(22):
            result = self.send(hand(grip=1), time_ms=original["time_ms"])
        self.assertEqual(result["reason"], "delayed input")
        self.assertGreater(result["source_lag_s"], 0.4)

    def test_event_ack_retries_view_and_record_independent_of_clutch(self):
        self.right()
        actions = [{"id": 1, "action": "view"}, {"id": 2, "action": "record"}]
        old = self.send(hand(), events=actions[1:])
        self.assertEqual(old["ack"], 0)
        result = self.send(hand(), events=actions)
        self.assertEqual(result["events"], actions)
        self.assertEqual(result["ack"], 2)
        self.assertEqual(result["hands"]["right"]["tool"], old["hands"]["right"]["tool"])
        self.assertEqual(self.send(hand(), events=actions)["events"], [])
        self.assertEqual(self.send(hand(), events=[{"id": 3, "action": "hold"}])["reason"], "operator hold")
        self.send(hand(b=True))
        self.assertTrue(self.engine.resume(self.now)["held"])
        self.send(hand(b=False))
        self.assertFalse(self.engine.resume(self.now)["held"])

    def test_invalid_sample_freezes_previous_command(self):
        self.right()
        self.right(grip=1)
        tool = self.right(p=(0.1, 1, -0.4), grip=1)["tool"]
        for invalid in [hand(q=[0, 0, 0, 0]), hand(p=(float("nan"), 0, 0)), hand(trigger=2)]:
            with self.assertRaises(ValueError):
                self.send(invalid)
            self.assertEqual(self.engine.snapshot(self.now)["reason"], "invalid sample")
            self.assertEqual(self.engine.snapshot(self.now)["hands"]["right"]["tool"], tool)

    def test_reach_rejection_drops_goal_and_freezes_aperture(self):
        self.right()
        old = self.right(grip=1)
        far = self.right(p=(2, 1, -0.4), trigger=1, grip=1)
        self.assertEqual(far["status"], "outside simulated reach")
        self.assertEqual(far["closure"], old["closure"])
        self.assertFalse(far["clutched"])
        self.assertEqual(self.right(grip=1)["tool"], old["tool"])
        self.right(p=(0.4, 1, 0))
        self.assertEqual(self.right(p=(0.4, 1, 0), grip=1)["tool"], old["tool"])

    def test_hysteresis_and_unknown_profile(self):
        self.right()
        self.assertFalse(self.right(grip=0.5)["clutched"])
        self.assertTrue(self.right(grip=0.8)["clutched"])
        self.assertTrue(self.right(grip=0.5)["clutched"])
        self.assertFalse(self.right(grip=0.3)["clutched"])
        self.assertEqual(self.right(profile="unknown")["status"], "unmapped")
        self.assertFalse(self.right(grip=1, profile="unknown")["clutched"])


if __name__ == "__main__":
    unittest.main()
