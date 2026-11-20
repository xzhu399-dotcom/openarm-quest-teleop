"""Persistent dual-arm control tests with fake USB; no hardware is accessed."""
from contextlib import ExitStack, nullcontext, redirect_stderr, redirect_stdout
import copy
import io
import math
import struct
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import openarm_cli as cli
import openarm_console as console
from test_openarm_cli_offline import FakeLink, use_test_config


class ConsoleChecks(unittest.TestCase):
    def setUp(self):
        self.config = use_test_config(self)
        self.links = {side: FakeLink() for side in ('left', 'right')}
        self.clock = [0.0]
        def link_for(serial):
            return next(self.links[s] for s in self.links if self.config['arms'][s]['serial'] == serial)
        def sleep(seconds):
            self.clock[0] += seconds
        patches = [patch.object(cli, 'Link', side_effect=link_for),
                   patch.object(cli.SessionLog, 'save'),
                   patch.object(cli.time, 'monotonic', side_effect=lambda: self.clock[0]),
                   patch.object(cli.time, 'sleep', side_effect=sleep)]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)
        output = redirect_stdout(io.StringIO())
        output.__enter__()
        self.addCleanup(output.__exit__, None, None, None)

    def sessions(self, stack):
        return {s: stack.enter_context(cli.Session(self.config['arms'][s], s, range(1, 9)))
                for s in self.links}

    def pose(self, session, offset=.1):
        return dict(serial=session.arm['serial'], joints=list(cli.POSE_JOINTS),
                    positions_rad={str(i): -.07 + (offset if i == 4 else 0) for i in cli.POSE_JOINTS},
                    motor_parameters={str(i): dict(mode=2, limits=vars(session.meta[i]['limits']))
                                      for i in cli.POSE_JOINTS})

    def test_preview_both_never_connects(self):
        self.assertEqual(cli.main(['--arm', 'both', 'console', '--speed', '.5']), 0)
        cli.Link.assert_not_called()

    def test_hand_speed_preview_and_invalid_values_never_connect(self):
        self.assertEqual(cli.main(['--arm', 'both', 'console', '--speed', '.5', '--hand-speed', '10']), 0)
        with redirect_stderr(io.StringIO()):
            for value in ('nan', '-1', '10.1'):
                self.assertEqual(cli.main(['--arm', 'both', 'console', '--hand-speed', value, '--execute']), 1)
        cli.Link.assert_not_called()

    def test_hand_speed_reaches_only_j8_packets_and_guard(self):
        with ExitStack() as stack:
            sessions = self.sessions(stack)
            console.prepare_all(sessions, .5, 10)
            model = console.ConsoleState(sessions, .5, 0, hand_speed=10)
            for side, session in sessions.items():
                session.cycle(model.targets[side], model.speeds_for(side), 1)
                packets = [e for e in self.links[side].events if e[0] == 'send' and 0x101 <= e[1] <= 0x108]
                for _, cid, payload in packets:
                    speed_rad = struct.unpack('<ff', payload)[1]
                    self.assertAlmostEqual(speed_rad, math.radians(10 if cid == 0x108 else .5))
                state = dict(status=1, q=-.07, v=math.radians(-11.333), temperature=25, raw_hex='test')
                cli.check_feedback(8, session.meta[8], state, -.07, model.speeds_for(side), True)
                with self.assertRaisesRegex(RuntimeError, '速度保护'):
                    cli.check_feedback(1, session.meta[1], state, -.07, model.speeds_for(side), True)
                state['v'] = math.radians(-30)
                with self.assertRaisesRegex(RuntimeError, '速度保护'):
                    cli.check_feedback(8, session.meta[8], state, -.07, model.speeds_for(side), True)

    def test_hand_jog_and_adjustment_stay_separate_from_arm_speed(self):
        with ExitStack() as stack:
            sessions = self.sessions(stack)
            model = console.ConsoleState(sessions, .5, 0, hand_speed=10)
            model.switch('left', 8, .1)
            model.handle('', .4, Mock())
            model.handle('a', .5, Mock())
            before = copy.deepcopy(model.targets)
            model.advance(.5, .05)
            self.assertAlmostEqual(model.targets['left'][8] - before['left'][8], math.radians(.00625))
            self.assertEqual(model.targets['right'], before['right'])
            model.handle('+', .55, Mock())
            self.assertEqual(model.selected_speed, 10)
            self.assertEqual(model.hand_speeds['right'], 10)
            self.assertEqual(model.speed, .5)
            model.switch('left', 4, .6)
            self.assertEqual(model.selected_speed, .5)
            sessions['left'].arm['poses']['vertical'] = self.pose(sessions['left'])
            model.switch('left', 8, .7)
            model.begin_pose(['vertical'], .8)
            self.assertEqual(model.plan['base_speed'], .5)
            model.advance(.9, .1)
            self.assertLessEqual(abs(model.targets['left'][4] - before['left'][4]), math.radians(.5) * .1)

    def test_incomplete_speed_map_rejected_before_motion_commands(self):
        with ExitStack() as stack:
            session = self.sessions(stack)['left']
            with self.assertRaisesRegex(ValueError, '缺少速度'):
                session.prepare({8: 10})
            self.assertFalse(any(e[0] in ('send', 'watchdog') for e in self.links['left'].events))

    def test_preflight_no_reply_names_arm_and_never_enables(self):
        errors = io.StringIO()
        with patch.object(cli, 'arm_lock', side_effect=lambda serial: nullcontext()), \
                patch.object(cli.Terminal, 'check_input'), \
                patch.object(self.links['left'], 'param', side_effect=RuntimeError('Motor 1, register 7: no reply')), \
                redirect_stderr(errors):
            result = cli.main(['--arm', 'both', 'console', '--execute'])
        self.assertEqual(result, 1)
        self.assertIn('left', errors.getvalue())
        self.assertIn(self.config['arms']['left']['serial'], errors.getvalue())
        self.assertIn('尚未发送任何电机使能指令', errors.getvalue())
        for link in self.links.values():
            self.assertFalse(link.enabled)
            self.assertFalse(any(e[0] in ('send', 'watchdog') for e in link.events))

    def test_all_modes_validated_before_watchdog_writes(self):
        with ExitStack() as stack:
            sessions = self.sessions(stack)
            sessions['right'].meta[8]['mode'] = 1
            with self.assertRaisesRegex(RuntimeError, '模式'):
                console.prepare_all(sessions, 1)
            for link in self.links.values():
                self.assertFalse(any(e[0] in ('send', 'watchdog') for e in link.events))

    def test_both_arms_prepare_before_either_enables(self):
        events = []
        sessions = {}
        for side in ('left', 'right'):
            session = Mock(meta={1: {'mode': 2, 'limits': cli.Limits(12.5, 45, 54)}},
                           joints=(1,), refs={1: .1})
            session.prepare.side_effect = lambda speed, s=side: events.append(('prepare', s))
            session.enable_prepared.side_effect = lambda s=side: events.append(('enable', s))
            sessions[side] = session
        console.prepare_all(sessions, 1)
        self.assertEqual(events, [('prepare', 'left'), ('prepare', 'right'),
                                  ('enable', 'left'), ('enable', 'right')])

    def test_jog_changes_only_selected_target_and_selection_never_disables(self):
        with ExitStack() as stack:
            sessions = self.sessions(stack)
            console.prepare_all(sessions, 1)
            model = console.ConsoleState(sessions, 1, 0)
            model.handle('', .2, Mock())
            before = copy.deepcopy(model.targets)
            model.handle('a', .3, Mock())
            model.advance(.3, .05)
            self.assertGreater(model.targets['left'][1], before['left'][1])
            self.assertEqual(model.targets['right'], before['right'])
            for i in range(2, 9):
                self.assertEqual(model.targets['left'][i], before['left'][i])
            model.handle('r4a', .35, Mock())
            selected = copy.deepcopy(model.targets)
            model.advance(.4, .05)
            self.assertEqual(model.targets, selected)
            self.assertEqual(model.ramp.velocity, 0)
            model.handle('a', .45, Mock())  # Still held: cannot move newly selected joint.
            model.advance(.5, .05)
            self.assertEqual(model.targets, selected)
            model.handle('', .7, Mock())
            model.handle('d', .75, Mock())
            model.advance(.75, .05)
            self.assertLess(model.targets['right'][4], selected['right'][4])
            self.assertEqual(model.targets['left'], selected['left'])
            for link in self.links.values():
                self.assertEqual(link.enabled, set(range(1, 9)))
                self.assertFalse(any(e[0] == 'send' and e[2] == b'\xff' * 7 + b'\xfd' for e in link.events))

    def test_console_sends_hold_to_all_16_on_every_cycle(self):
        with ExitStack() as stack:
            sessions = self.sessions(stack)
            terminal = Mock()
            terminal.poll.side_effect = ['', '', '', 'r', '', '3', '', 'q']
            writer = Mock()
            writer.poll.return_value = None
            console.console_loop(sessions, terminal, writer, SimpleNamespace(speed=1))
            for link in self.links.values():
                self.assertEqual(link.enabled, set(range(1, 9)))
                counts = {i: len([e for e in link.events if e[0] == 'send' and e[1] == i + 0x100])
                          for i in range(1, 9)}
                self.assertGreater(counts[1], 5)
                self.assertEqual(len(set(counts.values())), 1)
            console.stop_all(sessions)
            self.assertTrue(all(not link.enabled for link in self.links.values()))

    def test_partial_second_arm_enable_failure_stops_first_arm_too(self):
        self.links['right'].fail_enable = 3
        with self.assertRaises(IOError), ExitStack() as stack:
            sessions = self.sessions(stack)
            try:
                console.prepare_all(sessions, 1)
            finally:
                console.stop_all(sessions)
                self.assertTrue(all(not link.enabled for link in self.links.values()))
        for link in self.links.values():
            self.assertTrue(link.closed)
            self.assertTrue(all(value == 0 for value in link.timeouts.values()))

    def test_runtime_fault_exits_whole_console_and_disables_both_arms(self):
        terminal = Mock()
        polls = [0]
        def poll():
            polls[0] += 1
            if polls[0] == 3:
                self.links['right'].velocity = 1  # Feedback fault after both are holding.
            return ''
        terminal.poll.side_effect = poll
        with patch.object(cli, 'arm_lock', side_effect=lambda serial: nullcontext()), \
                patch.object(cli, 'Terminal', return_value=nullcontext(terminal)), \
                patch.object(cli.sys.stdin, 'isatty', return_value=True), \
                redirect_stderr(io.StringIO()):
            result = cli.main(['--arm', 'both', 'console', '--execute'])
        self.assertEqual(result, 1)
        for link in self.links.values():
            self.assertFalse(link.enabled)
            self.assertTrue(link.closed)
            self.assertTrue(all(value == 0 for value in link.timeouts.values()))

    def test_saved_pose_moves_arm_but_not_gripper_or_other_arm(self):
        with ExitStack() as stack:
            sessions = self.sessions(stack)
            model = console.ConsoleState(sessions, 1, 0)
            sessions['left'].arm['poses']['vertical'] = self.pose(sessions['left'])
            before = copy.deepcopy(model.targets)
            model.begin_pose(['vertical'], 1)
            model.advance(1.1, .1)
            first = model.targets['left'][4]
            self.assertGreater(first, before['left'][4])
            model.speed = 3
            model.advance(1.2, .1)
            self.assertLessEqual(model.targets['left'][4] - first, math.radians(3) * .1)
            self.assertEqual(model.targets['left'][8], before['left'][8])
            self.assertEqual(model.targets['right'], before['right'])

    def test_space_cancels_action_but_holds_targets(self):
        with ExitStack() as stack:
            sessions = self.sessions(stack)
            model = console.ConsoleState(sessions, 1, 0)
            sessions['left'].arm['poses']['vertical'] = self.pose(sessions['left'])
            model.begin_pose(['vertical'], .2)
            model.advance(.3, .1)
            before = copy.deepcopy(model.targets)
            model.handle(' a', .4, Mock())
            model.advance(.4, .1)
            self.assertEqual(model.targets, before)
            self.assertIsNone(model.plan)

    def test_console_defaults_and_deadman_restart(self):
        args = cli.parser().parse_args(['--arm', 'both', 'console'])
        self.assertEqual((args.speed, args.hand_speed, args.execute), (5, 10, False))
        with ExitStack() as stack:
            model = console.ConsoleState(self.sessions(stack), 5, 0, hand_speed=10)
            model.handle('', .2, Mock())
            for n in range(40):
                now = .3 + n * .05
                model.handle('a', now, Mock())
                model.advance(now, .05)
            self.assertAlmostEqual(model.ramp.velocity, 5)
            before = copy.deepcopy(model.targets)
            model.advance(now + .19, .19)
            self.assertEqual(model.targets, before)
            self.assertEqual(model.ramp.velocity, 0)
            model.handle('a', now + .2, Mock())
            model.advance(now + .2, .05)
            self.assertAlmostEqual(model.ramp.velocity, .125)
            model.handle(' ', now + .21, Mock())
            before = copy.deepcopy(model.targets)
            model.advance(now + .25, .05)
            self.assertEqual(model.targets, before)
            self.assertEqual(model.ramp.velocity, 0)

    def test_expired_lease_restarts_even_when_new_key_arrives_same_cycle(self):
        with ExitStack() as stack:
            model = console.ConsoleState(self.sessions(stack), 5, 0, hand_speed=10)
            model.handle('', .2, Mock())
            model.handle('a', .3, Mock())
            model.advance(.3, .05)
            model.ramp.velocity = 5
            model.handle('a', .49, Mock())
            model.advance(.49, .19)
            self.assertAlmostEqual(model.ramp.velocity, 2.5 * .19)

    def test_reference_log_is_snapshot_and_tracking_guard_still_stops(self):
        with ExitStack() as stack:
            session = self.sessions(stack)['left']
            session.start(5)
            targets = dict(session.refs)
            session.cycle(targets, 5, 1)
            recorded = [e for e in session.trace.events if e['event'] == 'reference'][-1]
            original = targets[4]
            targets[4] += .1
            self.assertEqual(recorded['targets'][4], original)
            session.refs[4] += math.radians(3.01)
            with self.assertRaisesRegex(RuntimeError, '跟踪误差超过 3°'):
                session.cycle(targets, 5, 1)

    def test_pose_resume_starts_fresh_ramp_without_position_jump(self):
        with ExitStack() as stack:
            sessions = self.sessions(stack)
            model = console.ConsoleState(sessions, 5, 0, hand_speed=10)
            sessions['left'].arm['poses']['vertical'] = self.pose(sessions['left'])
            model.begin_pose(['vertical'], 0)
            for n in range(20):
                model.advance(n * .05, .05)
            model.handle('p', 1.1, Mock())
            before = copy.deepcopy(model.targets)
            model.advance(1.2, .1)
            self.assertEqual(model.targets, before)
            model.handle('\r', 1.3, Mock())
            model.advance(1.3, .05)
            self.assertLess(abs(model.targets['left'][4] - before['left'][4]), math.radians(.01))

    def test_record_while_holding_without_any_disable(self):
        with ExitStack() as stack:
            sessions = self.sessions(stack)
            console.prepare_all(sessions, 1)
            for session in sessions.values():
                session.read_states()
            model = console.ConsoleState(sessions, 1, 0)
            pose = model.pose_snapshot(1)
            self.assertEqual(pose['joints'], list(range(1, 8)))
            self.assertNotIn('8', pose['positions_rad'])
            self.assertEqual(pose['serial'], sessions['left'].arm['serial'])
            self.assertTrue(all(len(link.enabled) == 8 for link in self.links.values()))
            model.last_motion['left'] = .9
            with self.assertRaises(ValueError):
                model.pose_snapshot(1)

    def test_background_save_does_not_block_holding_loop(self):
        entered, release = threading.Event(), threading.Event()
        def slow_save(*args):
            entered.set()
            release.wait(2)
        writer = console.PoseWriter('unused')
        try:
            with patch.object(cli, 'save_pose', side_effect=slow_save):
                writer.submit('left', 'serial', 'vertical', {}, False)
                self.assertTrue(entered.wait(1))
                self.assertIsNone(writer.poll())
                with self.assertRaises(ValueError):
                    writer.submit('left', 'serial', 'other', {}, False)
                release.set()
                writer.finish()
                self.assertEqual(writer.poll(), ('left', 'vertical', {}, None))
        finally:
            release.set()
            writer.finish()


if __name__ == '__main__':
    unittest.main()
