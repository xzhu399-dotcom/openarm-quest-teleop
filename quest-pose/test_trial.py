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

    def send(self, x=0., grip=0., stick=0., trigger=0., left=False, dt=.02, **changes):
        self.seq += 1
        self.now += dt
        right = hand(p=(x,1,-.4), grip=grip, trigger=trigger, profile="oculus-touch-v3")
        right["axes"][3] = stick
        sample = {"session":"quest", "seq":self.seq, "time_ms":(self.now-100)*1000,
                  "frame":"local", "visible":True, "head":pose(), "hands":{"right":right, "left":hand() if left else None}}
        sample.update(changes)
        return self.engine.apply(sample, self.now)

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
