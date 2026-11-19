"""Pure offline key-state tests. No server, browser, USB or motion is started."""
import math
import unittest

from openarm_keyboard import InputState, jog_step
from openarm_motor_test import build_parser, validate


class KeyboardChecks(unittest.TestCase):
    def test_keyboard_is_default_and_preview_only(self):
        args = validate(build_parser().parse_args(['--motor', '3']))
        self.assertTrue(args.keyboard)
        self.assertFalse(args.execute)
        self.assertEqual(args.range_deg, 10)
        self.assertFalse(build_parser().parse_args(['--motor', '1', '--sweep']).keyboard)

    def test_press_release_reverse(self):
        state = InputState()
        state.update(1, 1, False, 1)
        self.assertEqual(state.sample(1.01), (True, False, 1))
        state.update(2, 0, False, 1.02)
        self.assertEqual(state.sample(1.03), (True, False, 0))
        state.update(3, -1, False, 1.04)
        self.assertEqual(state.sample(1.05), (True, False, -1))

    def test_old_packet_cannot_restore_held_key(self):
        state = InputState()
        state.update(2, 0, False, 1)
        state.update(1, 1, False, 1.01)
        self.assertEqual(state.sample(1.02)[2], 0)

    def test_heartbeat_loss_latches_stop(self):
        state = InputState()
        state.update(1, 1, False, 1)
        self.assertEqual(state.sample(1.31), (True, True, 0))
        state.update(2, 1, False, 1.32)
        self.assertEqual(state.sample(1.33), (True, True, 0))

    def test_explicit_stop_latches(self):
        state = InputState()
        state.update(1, -1, False, 1)
        state.update(2, 0, True, 1.01)
        state.update(3, 1, False, 1.02)
        self.assertEqual(state.sample(1.03), (True, True, 0))

    def test_speed_direction_release_and_limits(self):
        origin = 1.0
        speed, travel = math.radians(1), math.radians(10)
        positive = jog_step(origin, 1, .02, speed, origin, travel)
        negative = jog_step(origin, -1, .02, speed, origin, travel)
        self.assertAlmostEqual(positive - origin, speed * .02)
        self.assertAlmostEqual(origin - negative, speed * .02)
        self.assertEqual(jog_step(positive, 0, .02, speed, origin, travel), positive)
        self.assertEqual(jog_step(origin+travel, 1, .02, speed, origin, travel), origin+travel)
        self.assertEqual(jog_step(origin-travel, -1, .02, speed, origin, travel), origin-travel)
        with self.assertRaises(RuntimeError):
            jog_step(origin, 1, .2, speed, origin, travel)

    def test_range_validation(self):
        args = validate(build_parser().parse_args(['--motor', '8']))
        self.assertEqual(args.range_deg, 1)
        for value in ['nan', 'inf', '0', '31']:
            with self.assertRaises(ValueError):
                validate(build_parser().parse_args(['--motor', '1', '--range-deg', value]))


if __name__ == '__main__':
    unittest.main()
