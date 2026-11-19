"""Exercise the physical J8 commissioning logic offline with synthetic WebXR input."""

import json
import math
from contextlib import ExitStack, nullcontext, redirect_stderr, redirect_stdout
from io import StringIO
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from gripper_bench import BenchControl, bind_arm, gripper_bench, load_calibration
from teleop import Teleop
from test_teleop import hand, pose


class FakeJ8:
    def __init__(self):
        self.meta = {8: {"mode": 2, "bounds": tuple(map(math.radians, [2., 10.]))}}
        self.states = {8: {"q": math.radians(5.)}}
        self.refs = {8: self.states[8]["q"]}
        self.attempted = []
        self.trace = Mock()
        self.calls = []
        self.measured_after_cycle = threading.Event()
        self.fail_after_command = False

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.calls.append(("exit", threading.current_thread()))

    def stationary(self):
        self.calls.append(("stationary",))

    def read_states(self, deadline):
        self.calls.append(("read",))
        if self.measured_after_cycle.is_set() and self.fail_after_command:
            raise OSError("encoder unavailable")

    def prepare(self, speed):
        self.calls.append(("prepare", speed))

    def enable_prepared(self):
        self.attempted.append(8)
        self.calls.append(("enable",))

    def cycle(self, targets, speed, factor):
        self.calls.append(("cycle", targets, speed, factor))
        self.refs = targets.copy()
        self.states[8]["q"] = targets[8]
        self.measured_after_cycle.set()

    def request_disable(self):
        self.calls.append(("disable", threading.current_thread()))


class BenchTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.path = self.root / "bench.json"
        self.settings = {
            "arm": "right", "adapter_serial": "0011AABB",
            "hardware_config": "hardware.json", "open_deg": 3.0,
            "closed_deg": 9.0, "max_speed_deg_s": 0.5,
        }
        self.arms = {"arms": {"right": {"serial": "0011AABB", "limits_deg": {"8": [2.0, 10.0]}, "poses": {}}}}
        (self.root / "hardware.json").write_text(json.dumps(self.arms), encoding="utf-8")
        self.reset_quest()

    def reset_quest(self, mode="trigger"):
        self.engine = Teleop(gripper_mode=mode)
        self.seq = 0
        self.now = 100.

    def config(self, **changes):
        self.path.write_text(json.dumps(dict(self.settings, **changes)), encoding="utf-8")
        return load_calibration(self.path)

    def sample(self, grip=0, closure=.5, epoch=0, stick=0., **changes):
        self.seq += 1
        self.now += .05
        raw = hand(grip=grip, profile="oculus-touch-v3")
        raw.update(source="xr-1", profiles=["oculus-touch-v3"], mapping="xr-standard",
                   target_ray_mode="tracked-pointer", handedness="right", grip_epoch=epoch)
        raw["axes"][3] = stick
        packet = {"session": "headset", "frame": "local-floor", "seq": self.seq,
                  "time_ms": (self.now - 100.) * 1000, "visible": True, "head": pose(),
                  "hands": {"left": None, "right": raw}}
        packet.update(changes)
        state = self.engine.apply(packet, self.now)
        state["owner"] = state["session"]
        state["hands"]["right"]["closure"] = closure
        state["recording"] = {"active": False, "episode": None, "last_episode_status": None}
        return state

    def step(self, bench, measurement=5., **kwargs):
        state = self.sample(**kwargs)
        return bench.step(state, measurement, self.now)

    def test_rejects_bad_calibration_and_unverified_encoder_range(self):
        for changes in ({"arm": "both"}, {"open_deg": 9.}, {"closed_deg": float("nan")},
                        {"max_speed_deg_s": 0}, {"max_speed_deg_s": 1.1},
                        {"max_speed_deg_s": True}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.config(**changes)
        cfg = self.config()
        bind_arm(cfg, self.arms)
        for serial, limits in (("DIFFERENT", {"8": [2., 10.]}), ("0011AABB", {}),
                               ("0011AABB", {"8": [5., 7.]})):
            arm = {"serial": serial, "limits_deg": limits, "poses": {}}
            with self.subTest(serial=serial, limits=limits), self.assertRaises(ValueError):
                bind_arm(cfg, {"arms": {"right": arm}})

    def test_anchor_and_speed_limit_in_each_encoder_direction(self):
        for open_deg, closed_deg in ((3., 9.), (9., 3.)):
            with self.subTest(open_deg=open_deg):
                self.reset_quest()
                bench = BenchControl(self.config(open_deg=open_deg, closed_deg=closed_deg))
                self.assertIsNone(self.step(bench))
                target = self.step(bench, grip=1)
                self.assertAlmostEqual(target, 5.)
                moved = self.step(bench, grip=1, closure=.9)
                self.assertGreater((moved - target) * math.copysign(1, closed_deg-open_deg), 0.)
                self.assertLessEqual(abs(moved - target), .5*.05 + 1e-9)
                for _ in range(3):
                    old = moved
                    moved = self.step(bench, measurement=old, grip=1, closure=.9)
                    self.assertAlmostEqual(moved, old, msg="centering must discard any unmade movement")
                self.step(bench, measurement=moved, epoch=1, closure=0.)
                next_target = self.step(bench, measurement=moved, grip=1, epoch=1, closure=0.)
                self.assertAlmostEqual(next_target, moved)

    def test_stops_at_calibrated_endpoint_in_both_directions(self):
        for open_deg, closed_deg in ((3., 9.), (9., 3.)):
            with self.subTest(open_deg=open_deg):
                self.reset_quest()
                bench = BenchControl(self.config(open_deg=open_deg, closed_deg=closed_deg))
                measured = closed_deg - math.copysign(.01, closed_deg - open_deg)
                self.step(bench, measurement=measured)
                self.step(bench, measurement=measured, grip=1)
                target = self.step(bench, measurement=measured, grip=1, closure=.6)
                self.assertAlmostEqual(target, closed_deg)
                self.assertAlmostEqual(self.step(bench, measurement=target, grip=1, closure=.8), target)

    def test_held_grip_with_a_new_epoch_does_not_cross_a_missed_release(self):
        bench = BenchControl(self.config())
        self.step(bench)
        self.step(bench, grip=1)
        before = self.step(bench, grip=1, closure=.6)
        state = self.sample(grip=1, closure=.9, epoch=1)
        result = bench.step(state, before, self.now)
        self.assertIn(result, (None, before))
        self.assertFalse(bench.engaged)

    def test_duplicate_poll_does_not_repeat_a_jaw_increment(self):
        bench = BenchControl(self.config())
        self.step(bench)
        self.step(bench, grip=1)
        state = self.sample(grip=1, closure=.7)
        target = bench.step(state, 5., self.now)
        repeated = bench.step(state, target, self.now + .015)
        self.assertIn(repeated, (None, target))

    def test_stick_controls_encoder_velocity_when_simulated_jaw_is_at_its_limit(self):
        for open_deg, closed_deg in ((3., 9.), (9., 3.)):
            with self.subTest(open_deg=open_deg):
                self.reset_quest("stick")
                bench = BenchControl(self.config(open_deg=open_deg, closed_deg=closed_deg))
                self.step(bench, closure=1.)
                self.assertAlmostEqual(self.step(bench, grip=1, stick=0., closure=1.), 5.)
                direction = math.copysign(1., closed_deg-open_deg)
                before = 5.
                # Full and half-speed positive commands toward "closed"; negative opens.
                for stick, expected_rate in ((1., 1.), (.575, .5), (0., 0.), (-1., -1.)):
                    target = self.step(bench, measurement=before, grip=1, stick=stick, closure=1.)
                    self.assertAlmostEqual(target, before + direction*.5*.05*expected_rate)
                    before = target
                self.assertEqual(self.step(bench, measurement=before, stick=1., epoch=1), before)
                self.assertEqual(self.step(bench, measurement=before, stick=-1., epoch=1), before)

    def test_stick_held_on_engagement_must_center_before_j8_can_move(self):
        self.reset_quest("stick")
        bench = BenchControl(self.config())
        self.step(bench, stick=1.)
        for stick in (1., .5, .3):
            self.assertIn(self.step(bench, grip=1, stick=stick), (None, 5.))
        self.assertAlmostEqual(self.step(bench, grip=1, stick=.14), 5.)
        state = self.sample(grip=1, stick=1.)
        target = bench.step(state, 5., self.now)
        self.assertGreater(target, 5.)
        self.assertEqual(bench.step(state, target, self.now+.015), target)
        self.assertAlmostEqual(self.step(bench, measurement=target, grip=1, stick=0.), target)

    def test_already_squeezed_and_simulated_sources_cannot_engage(self):
        for change in ("already-held", "mouse", "untracked", "gaze"):
            with self.subTest(change=change):
                self.reset_quest()
                bench = BenchControl(self.config())
                for grip in (0, 1):
                    state = self.sample(grip=1 if change=="already-held" else grip)
                    right = state["hands"]["right"]
                    if change == "mouse":
                        right.update(profile="mouse", source="mouse")
                        state["frame"] = "mouse-local"
                    elif change == "untracked":
                        right.update(pose=None, tracked=False)
                    elif change == "gaze":
                        right["target_ray_mode"] = "gaze"
                    self.assertIsNone(bench.step(state, 5., self.now))
                self.assertFalse(bench.activated)

    def test_active_bench_stops_on_hold_stale_lost_source_and_session(self):
        for change in ("hold", "stale", "session", "source", "hidden", "lost", "safety_epoch", "button_b"):
            with self.subTest(change=change):
                self.reset_quest()
                bench = BenchControl(self.config())
                self.step(bench)
                self.step(bench, grip=1)
                state = self.sample(grip=1)
                if change == "hold":
                    state.update(held=True, reason="operator hold")
                elif change == "stale":
                    state.update(age_s=1., arrival_s=self.now-1.)
                elif change == "session":
                    state.update(session="new-headset", owner="new-headset")
                elif change == "source":
                    state["hands"]["right"]["source"] = "xr-2"
                elif change == "hidden":
                    state["visible"] = False
                elif change == "safety_epoch":
                    state["safety_epoch"] = 1
                elif change == "button_b":
                    state["hands"]["right"]["buttons"][5]["pressed"] = True
                else:
                    state["hands"]["right"].update(pose=None, tracked=False)
                with self.assertRaises(RuntimeError):
                    bench.step(state, 5., self.now)

    def test_record_edges_and_return_require_a_new_clutch(self):
        bench = BenchControl(self.config())
        self.step(bench)
        self.step(bench, grip=1)
        prior = self.step(bench, grip=1, closure=.6)
        state = self.sample(grip=1, closure=.7)
        state["trial_epoch"] = 1
        self.assertEqual(bench.step(state, prior, self.now), prior)
        self.assertFalse(bench.engaged)
        state = self.sample(grip=0, epoch=1)
        state["trial_return"]["status"] = "returning"
        bench.step(state, prior, self.now)
        state = self.sample(grip=1, epoch=1)
        self.assertEqual(bench.step(state, prior, self.now), prior)
        self.assertFalse(bench.engaged)

    def prepare_runtime(self, *, release_during_prepare=False):
        self.config()
        session = FakeJ8()
        cli = SimpleNamespace(
            interrupt_handlers=nullcontext, arm_lock=lambda serial: nullcontext(),
            Session=Mock(return_value=session), load_config=lambda path: self.arms,
            check_feedback=Mock())
        sampled = threading.Event()
        signalled = threading.Event()

        def state():
            preparing = any(c[0] == "prepare" for c in session.calls)
            grip = int(self.seq > 0 and not (preparing and release_during_prepare))
            data = self.sample(grip=grip, closure=min(.9, .1*self.seq))
            if self.seq >= 4:
                sampled.set()
            return data

        station = SimpleNamespace(lock=threading.RLock(), state=state)
        environment = ExitStack()
        self.addCleanup(environment.close)
        environment.enter_context(patch("gripper_bench.sys.platform", "darwin"))
        environment.enter_context(patch("gripper_bench.sys.stdin.isatty", return_value=True))
        environment.enter_context(patch.dict(sys.modules, {"openarm_cli": cli}))
        environment.enter_context(patch("gripper_bench.os.kill", side_effect=lambda *_: signalled.set()))
        environment.enter_context(redirect_stdout(StringIO()))
        environment.enter_context(redirect_stderr(StringIO()))
        return station, session, cli, sampled, signalled

    def test_shadow_reads_real_session_interface_without_writes(self):
        station, session, cli, sampled, _ = self.prepare_runtime()
        with gripper_bench(station, self.path):
            self.assertTrue(sampled.wait(3), "bench did not sample the scripted grip")
        cli.Session.assert_called_once_with(self.arms["arms"]["right"], "right", (8,))
        self.assertGreaterEqual(sum(c[0] == "read" for c in session.calls), 4)
        self.assertFalse(set(c[0] for c in session.calls) & {"enable","prepare","cycle","disable"})
        self.assertTrue(all(c.args[0] == 8 and c.args[-1] is False for c in cli.check_feedback.call_args_list))
        self.assertEqual(session.calls[-1], ("exit", threading.main_thread()))

    def test_live_encoder_fault_requests_disable_and_exits_session_on_main_thread(self):
        station, session, cli, _, signalled = self.prepare_runtime()
        session.fail_after_command = True
        with patch("builtins.input", return_value="ENABLE right J8"):
            with self.assertRaisesRegex(RuntimeError, "encoder unavailable"):
                with gripper_bench(station, self.path, execute=True):
                    self.assertTrue(signalled.wait(3), "a worker failure must wake the receiver")
        cli.Session.assert_called_once_with(self.arms["arms"]["right"], "right", (8,))
        self.assertEqual(session.attempted, [8])
        self.assertTrue(all(set(c[1]) == {8} for c in session.calls if c[0] == "cycle"))
        self.assertEqual(session.calls[-2][0], "disable")
        self.assertEqual(session.calls[-1], ("exit", threading.main_thread()))

    def test_disable_error_on_worker_still_wakes_main_for_session_cleanup(self):
        station, session, _, _, signalled = self.prepare_runtime()
        session.fail_after_command = True
        session.request_disable = Mock(side_effect=OSError("failed to send immediate disable"))
        with patch("builtins.input", return_value="ENABLE right J8"):
            with self.assertRaises(RuntimeError):
                with gripper_bench(station, self.path, execute=True):
                    self.assertTrue(signalled.wait(3))
        self.assertTrue(session.request_disable.called)
        self.assertEqual(session.calls[-1], ("exit", threading.main_thread()))

    def test_local_decline_and_release_during_prepare_cannot_enable(self):
        station, session, _, _, signalled = self.prepare_runtime(release_during_prepare=True)
        with patch("builtins.input", return_value="no"):
            with self.assertRaises(ValueError):
                with gripper_bench(station, self.path, execute=True):
                    self.fail("declined local command opened a live bench")
        self.assertNotIn("prepare", [c[0] for c in session.calls])
        with patch("builtins.input", return_value="ENABLE right J8"):
            with self.assertRaisesRegex(RuntimeError, "clutch"):
                with gripper_bench(station, self.path, execute=True):
                    self.assertTrue(signalled.wait(3))
        self.assertEqual(session.attempted, [])
        self.assertNotIn("cycle", [c[0] for c in session.calls])


if __name__ == "__main__":
    unittest.main()
