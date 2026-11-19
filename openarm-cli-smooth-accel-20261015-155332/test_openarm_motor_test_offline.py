"""Offline protocol/guard checks. No USB imports, enumeration, or motor access."""
import ast
import contextlib
import importlib.util
import io
import math
from pathlib import Path
import struct
import subprocess
import sys
import unittest
from unittest.mock import patch

PATH = Path(__file__).with_name("openarm_motor_test.py")
spec = importlib.util.spec_from_file_location("motor_test_offline", PATH)
motor = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = motor
spec.loader.exec_module(motor)  # __main__ is never executed; no USB imported.


class OfflineChecks(unittest.TestCase):
    def setUp(self):
        # Failure-path tests use fake motors; do not save synthetic hardware logs.
        saver = patch.object(motor.SessionLog, 'save')
        saver.start()
        self.addCleanup(saver.stop)

    def args(self, *extra):
        return motor.validate(motor.build_parser().parse_args(["--motor", "1", *extra]))

    def test_defaults_and_each_motor(self):
        for number in range(1, 9):
            args = motor.validate(motor.build_parser().parse_args(["--motor", str(number)]))
            self.assertFalse(args.execute)
            self.assertEqual(args.speed_deg, 1)
            self.assertEqual(args.delta_deg, 0.5 if number == 8 else 2)

    def test_invalid_values_rejected(self):
        for extra in [("--speed-deg", "20"), ("--speed-deg", "nan"),
                      ("--delta-deg", "inf"), ("--delta-deg", "0"),
                      ("--delta-deg", "6"), ("--kp", "20")]:
            with self.assertRaises(ValueError):
                self.args(*extra)

    def test_smooth_trajectory_limits_both_directions(self):
        for delta in [math.radians(2), math.radians(-2), math.radians(0.5)]:
            speed = math.radians(1)
            duration = 1.875 * abs(delta) / speed
            q = [motor.smooth_reference(1, delta, duration * i / 1000, duration)
                 for i in range(1001)]
            self.assertAlmostEqual(q[0], 1)
            self.assertAlmostEqual(q[-1], 1 + delta)
            self.assertLessEqual(max(abs(b - a) / (duration / 1000)
                                     for a, b in zip(q, q[1:])), speed * 1.00001)
            self.assertTrue(all(min(1, 1 + delta) - 1e-12 <= v <= max(1, 1 + delta) + 1e-12
                                for v in q))

    def test_mit_packet_known_zero_and_gains(self):
        limits = motor.Limits(12.5, 30, 10)
        raw = motor.mit_frame(0, 5, 0.5, limits)
        self.assertEqual(len(raw), 8)
        p = (raw[0] << 8) | raw[1]
        v = (raw[2] << 4) | (raw[3] >> 4)
        kp = ((raw[3] & 15) << 8) | raw[4]
        kd = (raw[5] << 4) | (raw[6] >> 4)
        tau = ((raw[6] & 15) << 8) | raw[7]
        self.assertEqual((p, v, kp, kd, tau), (32768, 2048, 41, 410, 2048))
        self.assertNotEqual(raw, b"\xff" * 7 + b"\xfe")

    def test_state_unpack_and_reject_wrong_motor(self):
        state = motor.parse_state(bytes.fromhex("118000800800191a"), 1,
                                  motor.Limits(12.5, 30, 10))
        self.assertEqual(state["status"], 1)
        self.assertAlmostEqual(state["q"], 12.5 / 65535)
        self.assertEqual(state["temperature"], 26)
        with self.assertRaises(ValueError):
            motor.parse_state(bytes.fromhex("118000800800191a"), 2,
                              motor.Limits(12.5, 30, 10))

    def test_feedback_guards(self):
        limits = motor.Limits(12.5, 30, 10)
        base = dict(status=1, q=0, v=0, temperature=25)
        motor.check_state(base, 0, 0, 0.03, limits, True)
        for change in [dict(status=9), dict(temperature=60), dict(v=1), dict(q=1)]:
            with self.assertRaises(RuntimeError):
                motor.check_state(dict(base, **change), 0, 0, 0.03, limits, True)

    def test_overspeed_diagnostic_keeps_original_threshold(self):
        limits = motor.Limits(12.5,45,54)
        state = dict(status=1,q=-.05,v=math.radians(7),temperature=24,
                     raw_hex='117f008038001818')
        with self.assertRaises(RuntimeError) as caught:
            motor.check_state(state,-.05,-.049,.1,limits,True)
        message=str(caught.exception)
        self.assertIn('+7.000°/s', message)
        self.assertIn('117f008038001818',message)
        self.assertIn('不要直接提高阈值',message)

    def test_trace_records_boundaries_and_limits_memory(self):
        trace=motor.SessionLog(1)
        for i in range(3100):
            trace.record('sample',index=i)
        self.assertEqual(trace.total_events,3100)
        self.assertEqual(len(trace.events),3000)
        self.assertEqual(trace.events[-1]['index'],3099)

    def test_cleanup_after_enable_transfer_failure(self):
        class FakeLink:
            def __init__(self):
                self.sent = []
                self.closed = False
            def param(self, mid, rid, as_float=False):
                return {7: 17, 8: 1, 9: 100, 10: 1, 21: 12.5, 22: 30., 23: 10.}[rid]
            def state(self, *unused):
                return dict(status=0, q=0., v=0., temperature=25)
            def send(self, cid, data):
                self.sent.append((cid, data))
                if data == b"\xff" * 7 + b"\xfc":
                    raise IOError("simulated interrupted enable transfer")
            def close(self):
                self.closed = True
        fake = FakeLink()
        with patch.object(motor, "Link", return_value=fake), patch.object(motor.time, "sleep"), \
                patch.object(motor.signal, "signal"), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(IOError):
                motor.execute(self.args("--sweep"))
        self.assertIn((1, b"\xff" * 7 + b"\xfd"), fake.sent)
        self.assertTrue(fake.closed)
        self.assertTrue(all(cid == 1 for cid, _ in fake.sent))

    def test_source_parses_and_usb_is_deferred(self):
        tree = ast.parse(PATH.read_text())
        top_imports = [n for n in tree.body if isinstance(n, (ast.Import, ast.ImportFrom))]
        self.assertFalse(any("usb" in ast.unparse(n) for n in top_imports))
        # Other diagnostic test modules may import PyUSB; check a fresh process.
        result = subprocess.run(
            [sys.executable, '-c',
             "import openarm_motor_test, sys; assert 'usb.core' not in sys.modules"],
            cwd=PATH.parent, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_position_velocity_packet_and_speed_limit(self):
        args = self.args()
        args.control_mode = 2
        cid, data = motor.position_command(args, -.07, motor.Limits(12.5, 45, 54))
        self.assertEqual(cid, 0x101)
        position, velocity = struct.unpack('<ff', data)
        self.assertAlmostEqual(position, -.07, places=6)
        self.assertAlmostEqual(velocity, math.pi / 180, places=8)
        self.assertEqual(len(data), 8)
        args.control_mode = 1
        self.assertEqual(motor.position_command(args, 0, motor.Limits(12.5, 45, 54))[0], 1)

    def test_watchdog_conversion_and_bounds(self):
        self.assertEqual(motor.watchdog_ticks(500), 10000)
        for bad in (0, 249, 1001, float('nan'), 500.0):
            with self.assertRaises(ValueError):
                motor.watchdog_ticks(bad)

    def test_mode_two_watchdog_is_restored_after_failed_enable(self):
        class FakeLink:
            def __init__(self):
                self.sent = []
                self.writes = []
                self.closed = False
            def param(self, mid, rid, as_float=False):
                return {7:17, 8:1, 9:0, 10:2, 21:12.5, 22:45., 23:54.}[rid]
            def state(self, *unused):
                return dict(status=0, q=-.07, v=0., temperature=24)
            def set_watchdog(self, mid, ticks):
                self.writes.append((mid,ticks))
            def send(self, cid, data):
                self.sent.append((cid,data))
                if data == b'\xff'*7+b'\xfc':
                    raise IOError('simulated enable transfer failure')
            def close(self):
                self.closed = True
        fake = FakeLink()
        with patch.object(motor, 'Link', return_value=fake), patch.object(motor.time, 'sleep'), \
                patch.object(motor.signal, 'signal'), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(IOError):
                motor.execute(self.args('--sweep', '--watchdog-ms', '500'))
        self.assertEqual(fake.writes, [(1,10000),(1,0)])
        self.assertEqual(fake.sent[0][0], 0x101)
        self.assertIn((1,b'\xff'*7+b'\xfd'),fake.sent)
        self.assertTrue(fake.closed)

    def test_zero_timeout_without_option_does_not_enable(self):
        class FakeLink:
            def param(self, mid, rid, as_float=False):
                return {7:17,8:1,9:0,10:2,21:12.5,22:45.,23:54.}[rid]
            def send(self, *unused):
                raise AssertionError('Must not send control frames')
            def close(self):
                pass
        with patch.object(motor, 'Link', return_value=FakeLink()), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError,'watchdog-ms'):
                motor.execute(self.args('--sweep'))


if __name__ == "__main__":
    unittest.main()
