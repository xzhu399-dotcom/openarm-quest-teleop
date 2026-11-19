"""Operate the simulated arm with deterministic controller and host clocks."""

import math
import unittest

from teleop import Teleop
from test_teleop import hand, pose


class TrialTests(unittest.TestCase):
    def setUp(self):
        self.engine = Teleop(gripper_mode="stick")
        self.seq = 0
        self.now = 100.
        self.quest_simulation = False

    def send(self, x=0., grip=0., stick=0., trigger=0., left=False, dt=.02, left_x=0., left_grip=0., **changes):
        self.seq += 1
        self.now += dt
        right = hand(p=(x,1,-.4), grip=grip, trigger=trigger, profile="oculus-touch-v3")
        right["axes"][3] = stick
        left_hand = hand(p=(left_x,1,-.4), grip=left_grip) if left else None
        sample = {"session":"quest", "seq":self.seq, "time_ms":(self.now-100)*1000,
                  "frame":"local", "visible":True, "head":pose(), "hands":{"right":right, "left":left_hand}}
        sample.update(changes)
        return self.engine.apply(sample, self.now, quest_simulation=self.quest_simulation)

    def test_restore_initial_uses_startup_joints_during_trial_and_keeps_jaws(self):
        self.quest_simulation = True
        reference = self.send(left=True)["hands"]
        self.send(grip=1, left=True, left_grip=1)
        self.send(x=.04, grip=1, stick=1, left=True, left_x=-.03, left_grip=1)
        self.send(x=.04, left=True, left_x=-.03)
        trial_start = self.engine.trial.begin(self.engine.hands, self.engine.session)["joints_rad"]
        self.engine.capture(self.now)
        for side in ("left", "right"):
            self.assertNotEqual(trial_start[side], reference[side]["robot"]["joint_angles_rad"])
        moved = self.engine.restore_initial(self.now)
        self.assertEqual(moved["trial_return"]["target"], "home")
        self.assertEqual(moved["trial_return"]["status"], "returning")
        previous = trial_start["right"]
        for _ in range(400):
            state = self.send(left=True)
            current = state["hands"]["right"]["robot"]["joint_angles_rad"]
            self.assertLessEqual(max(abs(a-b) for a,b in zip(previous,current)), math.radians(30)*.02 + 1e-12)
            self.assertEqual(state["hands"]["right"]["closure"], moved["hands"]["right"]["closure"])
            previous = current
            if state["trial_return"]["status"] == "completed":
                break
        self.assertEqual(state["trial_return"]["status"], "completed")
        for side in ("left", "right"):
            self.assertEqual(state["hands"][side]["robot"]["joint_angles_rad"], reference[side]["robot"]["joint_angles_rad"])
        self.assertEqual(self.engine.trial.start, trial_start)
        self.assertTrue(self.send(grip=1, left=True)["hands"]["right"]["clutched"])

    def test_restore_initial_cancels_on_grip_or_tracking_loss_in_grip_only_mode(self):
        for change in ({"grip":1}, {"left":False}, {"dt":.5}, {"visible":False}, {"reset":True}):
            with self.subTest(change=change):
                self.setUp()
                self.quest_simulation = True
                self.send(grip=1, left=True)
                self.send(x=.04, grip=1, left=True)
                self.assertEqual(self.engine.restore_initial(self.now)["trial_return"]["status"], "skipped")
                self.send(left=True)
                self.assertEqual(self.engine.restore_initial(self.now)["trial_return"]["status"], "returning")
                before = self.send(left=True)["hands"]["right"]["robot"]["joint_angles_rad"]
                state = self.send(**{"left":True, **change})
                self.assertEqual(state["trial_return"]["status"], "cancelled")
                self.assertEqual(state["trial_return"]["target"], "home")
                self.assertEqual(state["hands"]["right"]["robot"]["joint_angles_rad"], before)
                self.assertEqual(self.send(left=True)["hands"]["right"]["robot"]["joint_angles_rad"], before)

    def test_joystick_holds_aperture_while_arm_moves_and_trigger_is_released(self):
        self.send()
        self.send(grip=1)
        closed = self.send(grip=1, stick=1, dt=.1)["hands"]["right"]
        self.assertAlmostEqual(closed["closure"], .05)
        for trigger in (.9,0.,1.):
            moving = self.send(x=.03, grip=1, stick=.1, trigger=trigger)["hands"]["right"]
            self.assertEqual(moving["closure"], closed["closure"])
        self.assertNotEqual(moving["tool"]["p"], closed["tool"]["p"])
        opened = self.send(grip=1,stick=-1,dt=.02)["hands"]["right"]
        self.assertAlmostEqual(opened["closure"],.04)
        self.send()
        self.send(grip=1,stick=1)
        self.assertEqual(self.send(grip=1,stick=1)["hands"]["right"]["closure"],opened["closure"])
        self.send(grip=1,stick=0)
        self.assertGreater(self.send(grip=1,stick=1)["hands"]["right"]["closure"],opened["closure"])
        stale = self.send(grip=1,stick=1,dt=.6)
        self.assertTrue(stale["held"])

    def begin_return(self, left=False):
        self.send(left=left)
        start = self.engine.trial.begin(self.engine.hands,self.engine.session)
        self.send(grip=1,left=left)
        moved = self.send(x=.04,grip=1,stick=1,left=left)["hands"]["right"]
        self.send(x=.04,left=left)
        self.engine.finish_trial(True)
        self.assertEqual(self.engine.trial.result["status"], "returning")
        return start["joints_rad"],moved

    def test_return_to_trial_start_has_bounded_speed_and_fixed_aperture(self):
        initial, moved = self.begin_return()
        previous = moved["robot"]["joint_angles_rad"]
        for _ in range(400):
            state = self.send()
            right = state["hands"]["right"]
            actual = right["robot"]["joint_angles_rad"]
            self.assertLessEqual(max(abs(a-b) for a,b in zip(previous,actual)),math.radians(30)*.02 + 1e-12)
            self.assertEqual(right["closure"],moved["closure"])
            self.assertEqual(state["hands"]["left"]["robot"]["joint_angles_rad"],initial["left"])
            previous = actual
            if state["trial_return"]["status"] == "completed":
                break
        self.assertEqual(state["trial_return"]["status"],"completed")
        self.assertEqual(actual,initial["right"])
        self.assertTrue(self.send(grip=1)["hands"]["right"]["clutched"])

    def test_return_cancellation_never_restarts(self):
        for change in ({"dt":.6},{"view_valid":False},{"reset":True},{"visible":False},
                       {"grip":1},{"events":[{"id":1,"action":"hold"}]},{"left":False}):
            with self.subTest(change=change):
                self.setUp()
                self.begin_return(left=True)
                before = self.send(left=True)["hands"]["right"]["robot"]["joint_angles_rad"]
                state = self.send(**dict({"left":True},**change))
                self.assertEqual(state["trial_return"]["status"],"cancelled")
                self.assertEqual(state["hands"]["right"]["robot"]["joint_angles_rad"],before)
                self.assertEqual(self.send(left=True)["hands"]["right"]["robot"]["joint_angles_rad"],before)

    def test_invalid_or_disconnected_input_aborts_return(self):
        self.begin_return()
        with self.assertRaises(ValueError):
            self.send(view_valid="broken")
        self.assertEqual(self.engine.trial.result["status"],"cancelled")
        self.setUp()
        self.begin_return()
        self.assertEqual(self.engine.disconnect(self.now)["trial_return"]["status"],"cancelled")
