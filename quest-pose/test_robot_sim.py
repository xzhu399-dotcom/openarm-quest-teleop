"""Regressions for articulated OpenArm v1.0 feedback and finite workspace."""

import math
import unittest

from robot_sim import ANGLE_TOL, HOME, POSITION_TOL, RobotArm, X, Y, Z, pose_error, turn
from teleop import Teleop
from test_teleop import hand, pose


class ArticulatedArmTests(unittest.TestCase):
    def assert_pose_close(self, goal, tcp):
        dp, dr = pose_error(goal, tcp)
        self.assertLess(math.hypot(*dp), POSITION_TOL)
        self.assertLess(math.hypot(*dr), ANGLE_TOL)

    def test_forward_against_pinned_urdf_reference(self):
        # Fixtures from an independent product of 4x4 transforms parsed from
        # 624e3dd/v1.urdf, converted from the URDF base to this XR workspace.
        for side, sign in (("right", 1), ("left", -1)):
            arm = RobotArm(side)
            angles = [sign*.21, sign*.73, -sign*.13, 1.3, sign*.42, -sign*.22, sign*.35]
            _, _, tcp = arm.forward(angles)
            reference = {"p": [sign*.34927326423, .94045748344, -.50616264516],
                         "q": [.14298825262, -sign*.11356229991, sign*.18497761678, .96562997310]}
            dp, dr = pose_error(reference, tcp)
            self.assertLess(math.hypot(*dp), 1e-10)
            self.assertLess(math.hypot(*dr), 1e-10)

    def test_wrist_rotates_about_tcp_and_jaws_follow_it(self):
        for side in ("right", "left"):
            for axis in (X, Y, Z):
                with self.subTest(side=side, axis=axis):
                    arm = RobotArm(side)
                    start = arm.snapshot(0)
                    previous = start["joint_angles_rad"]
                    for degree in range(1, 26):
                        goal = {"p": HOME[side], "q": turn(axis, math.radians(degree))}
                        self.assertTrue(arm.solve(goal))
                        self.assertTrue(all(lo <= a <= hi for a, (lo, hi) in zip(arm.angles, arm.limits)))
                        self.assertLess(max(abs(a-b) for a, b in zip(arm.angles, previous)), 0.15)
                        previous = arm.angles
                    self.assertNotEqual(arm.angles, start["joint_angles_rad"])
                    self.assert_pose_close(goal, arm.tcp)
                    for closure in (0, .4, 1):
                        state = arm.snapshot(closure)
                        fingertips = state["fingers_m"]
                        midpoint = [(a+b)/2 for a, b in zip(*fingertips)]
                        self.assertLess(math.dist(midpoint, HOME[side]), POSITION_TOL)
                        self.assertAlmostEqual(math.dist(*fingertips), .088*(1-closure))
                        self.assertAlmostEqual(math.dist(*fingertips), state["jaw_opening_m"])

    def test_unreachable_controller_sample_preserves_joints_and_grasp(self):
        engine = Teleop()
        def send(seq, p, grip, trigger=0):
            sample = {"session": "sim", "seq": seq, "frame": "local", "time_ms": seq*20,
                      "visible": True, "head": pose(), "hands": {"right": hand(
                          p=p, grip=grip, trigger=trigger, profile="oculus-touch-v3")}}
            return engine.apply(sample, now=seq*.02)["hands"]["right"]
        anchor = [0.1, 1.05, -.2]
        send(1, anchor, 0)
        send(2, anchor, 1)
        before = send(3, [.13, 1.07, -.23], 1, .6)
        self.assertTrue(before["clutched"])
        self.assert_pose_close(before["tool"], before["robot"]["tcp"])
        frozen = send(4, [.82, 1.05, -.2], 1, 0)
        self.assertFalse(frozen["clutched"])
        self.assertEqual(frozen["robot"]["status"], "wrist beyond arm reach")
        for key in ("joint_angles_rad", "tcp", "jaw_opening_m"):
            self.assertEqual(frozen["robot"][key], before["robot"][key])
        self.assertEqual(send(5, anchor, 1)["tool"], before["tool"])
        send(6, anchor, 0)
        self.assertEqual(send(7, anchor, 1)["robot"]["joint_angles_rad"], before["robot"]["joint_angles_rad"])

    def test_limits_reject_unreachable_orientation_near_workspace_center(self):
        arm = RobotArm("right")
        angles, tcp = arm.angles, arm.tcp
        self.assertFalse(arm.solve({"p": HOME["right"], "q": turn(Y, math.radians(70))}))
        self.assertEqual(arm.status, "no local IK solution within joint limits")
        self.assertEqual(arm.angles, angles)
        self.assertEqual(arm.tcp, tcp)
