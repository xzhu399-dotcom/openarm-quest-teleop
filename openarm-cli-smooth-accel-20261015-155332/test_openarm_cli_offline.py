"""All transport/state is fake. These tests never enumerate USB or enable hardware."""
import contextlib
import copy
import io
import json
import math
import os
from pathlib import Path
import pty
import struct
import tempfile
import termios
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import openarm_cli as cli
from openarm_motor_test import select_adapter


class Device:
    def __init__(self, serial):
        self.serial_number = serial


class FakeLink:
    def __init__(self):
        self.events = []
        self.trace = None
        self.enabled = set()
        self.timeouts = {i: 0 for i in range(1, 9)}
        self.fail_enable = None
        self.fail_watchdog_once = None
        self.unreadable = None
        self.velocity = 0
        self.closed = False

    def param(self, motor, rid, as_float=False):
        return {7: motor + 16, 8: motor, 9: self.timeouts[motor], 10: 2,
                21: 12.5, 22: 45., 23: 54.}[rid]

    def state(self, motor, reply, limits):
        self.events.append(('state', motor))
        if motor == self.unreadable:
            raise RuntimeError('fake feedback timeout')
        return dict(status=int(motor in self.enabled), q=-.07, v=self.velocity,
                    temperature=25, raw_hex='fake')

    def send(self, cid, data):
        self.events.append(('send', cid, data))
        if data == b'\xff' * 7 + b'\xfc':
            self.enabled.add(cid)
            if cid == self.fail_enable:
                raise IOError('fake enable ACK/transfer failure')
        elif data == b'\xff' * 7 + b'\xfd':
            self.enabled.discard(cid)

    def set_watchdog(self, motor, ticks):
        self.events.append(('watchdog', motor, ticks))
        self.timeouts[motor] = ticks
        if motor == self.fail_watchdog_once:
            self.fail_watchdog_once = None
            raise IOError('fake watchdog ACK loss')

    def close(self):
        self.closed = True


class CliChecks(unittest.TestCase):
    def setUp(self):
        self.config = cli.load_config(cli.CONFIG)
        self.arm = copy.deepcopy(self.config['arms']['left'])
        self.fake = FakeLink()
        for patcher in (patch.object(cli, 'Link', return_value=self.fake),
                        patch.object(cli.SessionLog, 'save'),
                        patch.object(cli.time, 'sleep')):
            patcher.start()
            self.addCleanup(patcher.stop)
        self.output = contextlib.redirect_stdout(io.StringIO())
        self.output.__enter__()
        self.addCleanup(self.output.__exit__, None, None, None)

    def pose(self):
        return dict(serial=self.arm['serial'], joints=list(cli.POSE_JOINTS),
                    positions_rad={str(i): .1 for i in cli.POSE_JOINTS},
                    motor_parameters={str(i): {'mode': 2, 'limits': {'p': 12.5, 'v': 45., 't': 54.}}
                                      for i in cli.POSE_JOINTS})

    def test_serial_selection_ignores_usb_enumeration_order(self):
        left = Device(self.config['arms']['left']['serial'])
        right = Device(self.config['arms']['right']['serial'])
        for devices in ([left, right], [right, left]):
            self.assertIs(select_adapter(devices, left.serial_number), left)
            self.assertIs(select_adapter(devices, right.serial_number), right)
        with self.assertRaises(RuntimeError):
            select_adapter([right], left.serial_number)
        with self.assertRaises(RuntimeError):
            select_adapter([left, left], left.serial_number)

    def test_motion_preview_never_connects(self):
        self.assertEqual(cli.main(['--arm', 'left', 'jog', '--joint', '4', '--speed', '2']), 0)
        cli.Link.assert_not_called()

    def test_only_hand_joint_accepts_faster_jog_preview(self):
        self.assertEqual(cli.main(['--arm', 'left', 'jog', '--joint', '8', '--speed', '10']), 0)
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(cli.main(['--arm', 'left', 'jog', '--joint', '1', '--speed', '10']), 1)
        cli.Link.assert_not_called()

    def test_missing_pose_and_invalid_speed_never_connect(self):
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(cli.main(['--arm', 'left', 'play', 'not-recorded', '--execute']), 1)
            self.assertEqual(cli.main(['--arm', 'left', 'jog', '--joint', '1', '--speed', 'nan', '--execute']), 1)
        cli.Link.assert_not_called()

    def test_key_repeat_expires_and_stop_wins(self):
        keys = cli.JogInput()
        keys.update('a', 1)
        self.assertEqual(keys.sample(1.1), 1)
        self.assertEqual(keys.sample(1.181), 0)
        keys.update('d', 2)
        self.assertEqual(keys.sample(2.1), -1)
        keys.update('a ad', 2.1)
        self.assertEqual(keys.sample(2.11), 0)

    def test_no_artificial_ten_degree_window_and_explicit_bounds(self):
        q = 0
        for _ in range(1200):
            q = cli.reference_step(q, 1, .02, 5, (-2, 2))
        self.assertAlmostEqual(q, 2)
        with self.assertRaises(RuntimeError):
            cli.reference_step(q, 1, .21, 1, (-2, 2))

    def test_synchronized_pose_profile_speed_and_endpoints(self):
        start, target = {1: -.2, 2: .4}, {1: 1.2, 2: -.3}
        speed = 2
        duration = 1.875 * 1.4 / math.radians(speed)
        dt = duration / 1000
        samples = [cli.pose_reference(start, target, dt * k, speed)[0] for k in range(1001)]
        self.assertEqual(samples[0], start)
        for i in start:
            self.assertAlmostEqual(samples[-1][i], target[i])
            self.assertLessEqual(max(abs(b[i] - a[i]) / dt for a, b in zip(samples, samples[1:])),
                                 math.radians(speed) * 1.00001)

    def test_jog_ramp_speed_acceleration_stop_reverse_and_bounds(self):
        for motor, cap, accel in ((4, 5, 2.5), (8, 10, 5)):
            ramp = cli.JogRamp()
            q = 0
            for n in range(80):
                previous, velocity = q, ramp.velocity
                q = ramp.step(q, 1, .05, cap, (-10, 10), motor)
                self.assertLessEqual(abs(q - previous) / .05, math.radians(cap) + 1e-12)
                self.assertLessEqual(abs(ramp.velocity - velocity), accel * .05 + 1e-12)
                if n == 39:
                    self.assertAlmostEqual(ramp.velocity, cap)
            # Reverse starts slowly in the new direction; no old-direction travel.
            previous = q
            q = ramp.step(q, -1, .05, cap, (-10, 10), motor)
            self.assertAlmostEqual(q - previous, -math.radians(.5 * accel * .05**2))
            self.assertEqual(ramp.step(q, 0, .05, cap, (-10, 10), motor), q)
            self.assertEqual(ramp.velocity, 0)
            q = ramp.step(0, 1, .2, cap, (-.0001, .0001), motor)
            self.assertEqual(q, .0001)
            self.assertEqual(ramp.velocity, 0)
            with self.assertRaises(RuntimeError):
                ramp.step(q, 1, .201, cap, (-1, 1), motor)
            with self.assertRaises(ValueError):
                ramp.step(q, 1, .05, cap + .1, (-1, 1), motor)

    def test_ramp_cap_reached_mid_step_and_speed_reduction(self):
        ramp = cli.JogRamp()
        q = ramp.step(0, 1, .2, .1, (-1, 1), 4)
        self.assertAlmostEqual(math.degrees(q), .018)
        ramp.velocity = 5
        q = ramp.step(0, 1, .1, 1, (-1, 1), 4)
        self.assertAlmostEqual(math.degrees(q), .1)
        self.assertEqual(ramp.velocity, 1)

    def test_short_pose_acceleration_is_bounded(self):
        dt = .005
        start, target = {4: 0}, {4: math.radians(2)}
        samples = [cli.pose_reference(start, target, n * dt, 5)[0][4] for n in range(1001)]
        velocities = [(b - a) / dt for a, b in zip(samples, samples[1:])]
        accelerations = [(b - a) / dt for a, b in zip(velocities, velocities[1:])]
        self.assertAlmostEqual(samples[-1], target[4])
        self.assertLessEqual(max(map(abs, velocities)), math.radians(5))
        self.assertLessEqual(max(map(abs, accelerations)), math.radians(cli.ARM_ACCEL) * 1.00001)

    def test_frame_speed_is_radians_and_no_mode_write(self):
        meta = {'mode': 2, 'limits': cli.Limits(12.5, 45, 54)}
        cid, frame = cli.frame_for(4, meta, -.2, 5)
        q, v = struct.unpack('<ff', frame)
        self.assertEqual(cid, 0x104)
        self.assertAlmostEqual(q, -.2)
        self.assertAlmostEqual(v, math.radians(5))
        with self.assertRaises(ValueError):
            cli.frame_for(4, meta, 0, 6)

    def test_read_only_session_has_no_sends_or_parameter_writes(self):
        with cli.Session(self.arm, 'left', range(1, 9)):
            pass
        self.assertFalse(any(e[0] in ('send', 'watchdog') for e in self.fake.events))
        cli.Link.assert_called_once_with(serial=self.arm['serial'])

    def test_enable_failure_disables_all_attempted_before_waiting(self):
        self.fake.fail_enable = 2
        with self.assertRaises(IOError):
            with cli.Session(self.arm, 'left', (1, 2, 3)) as session:
                session.start(1)
        self.assertFalse(self.fake.enabled)
        stops = [n for n, e in enumerate(self.fake.events) if e[0] == 'send' and e[2] == b'\xff' * 7 + b'\xfd']
        self.assertEqual(self.fake.events[stops[0]][1], 1)
        self.assertEqual(self.fake.events[stops[1]][1], 2)
        self.assertEqual(stops[1], stops[0] + 1)
        self.assertEqual([self.fake.timeouts[i] for i in (1, 2, 3)], [0, 0, 0])
        self.assertTrue(self.fake.closed)

    def test_uncertain_watchdog_write_restored_without_enable(self):
        self.fake.fail_watchdog_once = 2
        with self.assertRaises(IOError):
            with cli.Session(self.arm, 'left', (1, 2)) as session:
                session.start(1)
        self.assertFalse(any(e[0] == 'send' for e in self.fake.events))
        self.assertEqual(self.fake.timeouts[1], 0)
        self.assertEqual(self.fake.timeouts[2], 0)

    def test_unconfirmed_motor_retains_watchdog_but_other_motor_restored(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(RuntimeError):
            with cli.Session(self.arm, 'left', (1, 2)) as session:
                session.start(1)
                self.fake.unreadable = 1
        self.assertEqual(self.fake.timeouts[1], 10000)
        self.assertEqual(self.fake.timeouts[2], 0)

    def test_feedback_fault_sends_no_further_position_commands(self):
        with self.assertRaises(RuntimeError):
            with cli.Session(self.arm, 'left', (1, 2)) as session:
                session.start(1)
                self.fake.velocity = 1
                before = len([e for e in self.fake.events if e[0] == 'send'])
                try:
                    session.cycle(dict(session.refs), 1, 1)
                finally:
                    self.assertEqual(before, len([e for e in self.fake.events if e[0] == 'send']))

    def test_saved_pose_is_bound_to_arm_and_parameter_mapping(self):
        pose = self.pose()
        self.arm['poses']['vertical'] = pose
        cli.read_pose(self.arm, 'vertical')
        with cli.Session(self.arm, 'left', cli.POSE_JOINTS) as session:
            session.verify_pose(pose)
            pose['motor_parameters']['1']['limits']['p'] = 10
            with self.assertRaises(RuntimeError):
                session.verify_pose(pose)
        pose['serial'] = 'different-arm'
        with self.assertRaises(ValueError):
            cli.read_pose(self.arm, 'vertical')

    def test_pose_save_preserves_other_arm_and_requires_explicit_replace(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'poses.json'
            path.write_text(json.dumps(self.config))
            cli.save_pose(path, 'left', self.arm['serial'], 'vertical', self.pose())
            updated = cli.load_config(path)
            self.assertEqual(updated['arms']['right'], self.config['arms']['right'])
            with self.assertRaises(ValueError):
                cli.save_pose(path, 'left', self.arm['serial'], 'vertical', self.pose())
            with self.assertRaises(RuntimeError):
                cli.save_pose(path, 'left', 'wrong-device', 'new', self.pose())

    def test_pending_quit_does_not_enable(self):
        session, terminal = Mock(), Mock()
        terminal.poll.return_value = 'q'
        cli.control(session, terminal, SimpleNamespace(command='jog', speed=1), [])
        session.start.assert_not_called()

    def test_play_completes_sequence_and_holds_until_quit(self):
        clock = [0.0]
        def sleep(seconds):
            clock[0] += seconds
        terminal = Mock()
        terminal.poll.side_effect = [''] * 170 + ['q']
        args = SimpleNamespace(command='play', speed=1, names=['vertical', 'forearm-raised'])
        output = io.StringIO()
        with patch.object(cli.time, 'monotonic', side_effect=lambda: clock[0]), \
                patch.object(cli.time, 'sleep', side_effect=sleep), \
                contextlib.redirect_stdout(output):
            with cli.Session(self.arm, 'left', cli.POSE_JOINTS) as session:
                target = {i: -.07 for i in cli.POSE_JOINTS}
                cli.control(session, terminal, args, [target, target])
                self.assertEqual(self.fake.enabled, set(cli.POSE_JOINTS))
        self.assertIn('已到达姿态：vertical', output.getvalue())
        self.assertIn('已到达姿态：forearm-raised', output.getvalue())
        self.assertIn('动作完成，正在保持', output.getvalue())
        self.assertFalse(self.fake.enabled)

    def test_terminal_reads_keys_and_restores_settings_on_error(self):
        master, slave = pty.openpty()
        try:
            before = termios.tcgetattr(slave)
            with os.fdopen(os.dup(slave), 'r') as stream, patch.object(cli.sys, 'stdin', stream):
                with self.assertRaises(RuntimeError):
                    with cli.Terminal() as terminal:
                        os.write(master, b'a')
                        cli.select.select([slave], [], [], .2)
                        self.assertEqual(terminal.poll(), 'a')
                        raise RuntimeError('fake failure')
            after = termios.tcgetattr(slave)
            # macOS sets this kernel input-reprocessing flag on canonical restore.
            after[3] &= ~getattr(termios, 'PENDIN', 0)
            before[3] &= ~getattr(termios, 'PENDIN', 0)
            self.assertEqual(after, before)
        finally:
            os.close(master)
            os.close(slave)


if __name__ == '__main__':
    unittest.main()
