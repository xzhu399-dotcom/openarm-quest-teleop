"""Physical-arm behavior through the real station and a deterministic fake CAN session."""

from contextlib import nullcontext
from copy import deepcopy
import json
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from robot_hardware import ArmCalibration, ArmControl, bind_session, load_calibration, physical_sessions
from station import Station
from test_station import quest_sample


class MotorSession:
    def __init__(self, cal, home):
        self.joints = cal.joints
        self.arm = {"serial":cal.serial,"limits_deg":{str(j):[-180.,180.] for j in self.joints}}
        self.meta = {j:{"mode":2,"bounds":(-math.pi,math.pi)} for j in self.joints}
        self.refs = dict(home)
        self.states = {j:{"q":q,"v":0.,"torque":0.,"temperature":22.,"status":0} for j,q in home.items()}
        self.feedback_ns = {}
        self.attempted = []
        self.closed = self.disabled = False
        self.fail_after = None
        self.commands = []
        self.read_states()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.disabled = self.closed = True

    def stationary(self):
        self.read_states()

    def read_states(self, deadline=None):
        for j in self.joints:
            self.feedback_ns[j] = time.monotonic_ns()
            self.states[j].update(q=self.refs[j],status=int(bool(self.attempted)))

    def prepare(self, speed, on_preload):
        time.sleep(.05)
        for j in self.joints:
            on_preload(j,self.refs[j],time.monotonic_ns())

    def enable_prepared(self, guard, on_enabled):
        for j in self.joints:
            guard()
            self.attempted.append(j)
            on_enabled(j,time.monotonic_ns())

    def send_references(self, targets, speeds, factor, begun, select_target, on_sent):
        for j in self.joints:
            value = select_target(j, targets[j], self.refs[j])
            self.refs[j] = value
            on_sent(j, value, time.monotonic_ns())
            if self.fail_after == j:
                raise OSError("injected partial bus failure")
        self.commands.append(dict(self.refs))

    def request_disable(self):
        self.disabled = True


class RobotTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        self.station = Station(self.root)
        self.addCleanup(self.station.close)
        self.cal = ArmCalibration("serial1",[3.]*7,[1,-1,1,-1,1,-1,1],4.,
                                  {"open_deg":-5., "closed_deg":5., "max_speed_deg_s":1.})
        self.calfile = self.root/"config.json"
        self.calfile.write_text(json.dumps({"hardware_config":"robot.json","arms":{"right":{
            "adapter_serial":self.cal.serial,"encoder_zero_deg":self.cal.zero,"encoder_sign":self.cal.sign,
            "max_speed_deg_s":4.,"gripper":self.cal.gripper}}}))
        self.home = {i:math.radians(3)+self.cal.sign[i-1]*q
                     for i,q in enumerate(self.station.engine.initial_joints["right"],1)}
        self.home[8] = 0.
        self.bus = MotorSession(self.cal,self.home)
        def interpolate(origin, target, elapsed, speed):
            distance = max(abs(target[j]-q) for j,q in origin.items())
            t = min(1.,elapsed*math.radians(speed)/max(distance,1e-12))
            return {j:q+(target[j]-q)*t for j,q in origin.items()}, t == 1
        self.cli = SimpleNamespace(load_config=lambda path:{"arms":{"right":self.bus.arm}},
                                   Session=lambda *args:self.bus,arm_lock=lambda serial:nullcontext(),
                                   interrupt_handlers=nullcontext, check_feedback=lambda *args:None,
                                   pose_reference=interpolate)
        self.seq = 0
        self.epoch = 0

    def send(self,x=0.,grip=0.,b=False,left=None,events=()):
        self.seq += 1
        packet = quest_sample(self.seq,x,grip,b=b,events=events)
        packet["hands"]["right"].update(grip_epoch=self.epoch, target_ray_mode="tracked-pointer")
        if left is not None:
            other = quest_sample(self.seq,*left)["hands"]["right"]
            other.update(handedness="left",source="xr-2",grip_epoch=self.epoch,target_ray_mode="tracked-pointer")
            packet["hands"]["left"] = other
        return self.station.sample(packet)

    def drive_until(self, predicate, **inputs):
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            self.send(**inputs)
            if predicate():
                return
            time.sleep(.012)
        self.fail(f"Timed out waiting for fake arm: {self.station.physical_arm}")

    def saved_rows(self, name, keep_tracking=False):
        deadline = time.monotonic()+3
        state = self.station.recorder.status()
        while state.get("finalizing") and time.monotonic()<deadline:
            if keep_tracking:
                self.send(x=.015)
            time.sleep(.01)
            state = self.station.recorder.status()
        self.assertEqual(state["last_episode_status"],"saved",state.get("error"))
        return [json.loads(s) for s in (self.root/name/"robot_io.jsonl").read_text().splitlines()]

    def test_binding_rejects_missing_limits_mode_and_unknown_direction(self):
        path, arms = load_calibration(self.calfile)
        self.assertEqual(arms["right"].model_joints(self.home),self.cal.model_joints(self.home))
        invalid = json.loads(self.calfile.read_text())
        invalid["arms"]["right"]["encoder_sign"][1] = 0
        self.calfile.write_text(json.dumps(invalid))
        with self.assertRaisesRegex(ValueError,"directions"):
            load_calibration(self.calfile)
        self.bus.meta[4]["mode"] = 0
        with self.assertRaisesRegex(ValueError,"mode"):
            bind_session(self.station,"right",self.cal,self.bus)
        self.bus.arm["limits_deg"].pop("1")
        with self.assertRaisesRegex(ValueError,"limits"):
            bind_session(self.station,"right",self.cal,self.bus)

    def test_shadow_reads_and_aligns_startup_without_enable_or_write(self):
        with physical_sessions(self.station,self.root,{"right":self.cal},self.cli,False):
            self.send()
            time.sleep(.07)
            self.drive_until(lambda:self.station.physical_arm["arms"]["right"]["phase"] == "tracking",grip=1.)
            self.assertFalse(self.bus.attempted)
            self.assertFalse(self.bus.commands)
            self.assertAlmostEqual(self.station.engine.hands["right"].closure,.5)
            state = self.station.physical_arm["arms"]["right"]
            self.assertEqual(len(state["initial_deg"]),8)
            self.assertGreater(state["feedback_host_ns"],0)
        self.assertTrue(self.bus.closed)

    def test_first_enable_after_encoder_shift_is_refused(self):
        with patch("builtins.input",return_value="ENABLE right ARM"), patch("robot_hardware.os.kill"):
            with self.assertRaisesRegex(RuntimeError,"moved before first enable"):
                with physical_sessions(self.station,self.root,{"right":self.cal},self.cli,True):
                    self.send()
                    time.sleep(.07)
                    self.bus.refs[1] += math.radians(.75)
                    self.drive_until(lambda:bool(self.station.physical_arm["error"]),grip=1.)
        self.assertFalse(self.bus.attempted)
        self.assertTrue(self.bus.closed)

    def test_drive_records_actual_can_writes_and_measured_return(self):
        with patch("builtins.input",return_value="ENABLE right ARM"), patch("robot_hardware.os.kill") as signal_fault:
            with physical_sessions(self.station,self.root,{"right":self.cal},self.cli,True):
                # An already squeezed controller cannot arm; a release is required.
                self.send(grip=1.)
                time.sleep(.07)
                self.assertFalse(self.bus.attempted)
                self.epoch += 1
                self.send()
                self.station.control({"action":"record"},local=True)
                time.sleep(.065)
                self.drive_until(lambda:self.station.physical_arm["arms"]["right"]["enabled"],grip=1.)
                self.drive_until(lambda:any(abs(self.bus.refs[j]-self.home[j]) > math.radians(.1)
                                            for j in range(1,8)),grip=1.,x=.015)
                self.epoch += 1
                self.send(x=.015)
                self.drive_until(lambda:self.station.physical_arm["arms"]["right"]["phase"] == "holding",x=.015)
                self.station.control({"action":"restore_robot_initial"},local=True)
                self.drive_until(lambda:self.station.physical_arm["arms"]["right"]["phase"] == "returned",x=.015)
                for j in range(1,8):
                    self.assertAlmostEqual(self.bus.states[j]["q"],self.home[j],delta=math.radians(.5))
                name = self.station.control({"action":"record"},local=True)["recording"]["episode"]
                rows = self.saved_rows(name,keep_tracking=True)
                self.assertTrue(any(r["command"]=="track" and len(r["sent_joints"])==8 for r in rows))
                self.assertTrue(any(len(r.get("enabled_joints",{})) == len(r.get("preloaded_joints",{})) == 8 for r in rows))
                self.assertTrue(any(r["command"]=="return" for r in rows))
                self.assertTrue(all(r["start_ns"]<=r["host_monotonic_ns"]<=r["end_ns"] for r in rows))
                self.assertTrue(all(len(r["measured_deg"])==8 and r["seq"] for r in rows))
            signal_fault.assert_not_called()
        self.assertTrue(self.bus.closed)

    def test_lost_input_faults_drive_and_confirms_disable(self):
        with patch("builtins.input",return_value="ENABLE right ARM"), patch("robot_hardware.os.kill"):
            with self.assertRaisesRegex(RuntimeError,"stopped"):
                with physical_sessions(self.station,self.root,{"right":self.cal},self.cli,True):
                    self.send()
                    time.sleep(.07)
                    self.drive_until(lambda:self.station.physical_arm["arms"]["right"]["enabled"],grip=1.)
                    self.send(grip=1.,b=True)
                    deadline = time.monotonic()+2
                    while not self.bus.disabled and time.monotonic()<deadline:
                        time.sleep(.01)
                    self.assertTrue(self.bus.disabled)
        self.assertTrue(self.bus.closed)
        self.assertFalse(self.station.physical_arm["arms"]["right"]["enabled"])

    def test_bimanual_returns_both_and_disables_both_on_partial_bus_fault(self):
        left = deepcopy(self.cal)
        left.serial, left.gripper = "serial2", None
        left_home = {i:math.radians(3)+left.sign[i-1]*q
                     for i,q in enumerate(self.station.engine.initial_joints["left"],1)}
        other = MotorSession(left,left_home)
        buses = {"right":self.bus,"left":other}
        self.cli.Session = lambda arm,side,joints:buses[side]
        self.cli.load_config = lambda path:{"arms":{s:bus.arm for s,bus in buses.items()}}
        arms = {"right":self.cal,"left":left}
        with patch("builtins.input", return_value="ENABLE left right ARM"), patch("robot_hardware.os.kill"):
            with self.assertRaisesRegex(RuntimeError,"partial bus failure"):
                with physical_sessions(self.station,self.root,arms,self.cli,True):
                    self.send(left=(0.,0.),events=[{"id":1,"action":"record"}])
                    time.sleep(.07)
                    self.drive_until(lambda:all(h["enabled"] for h in self.station.physical_arm["arms"].values()),
                                     grip=1.,left=(0.,1.))
                    self.drive_until(lambda:any(abs(other.refs[j]-left_home[j])>math.radians(.1) for j in other.joints),
                                     grip=1.,left=(-.02,1.))
                    self.assertEqual(self.bus.refs,self.home)
                    self.drive_until(lambda:any(abs(self.bus.refs[j]-self.home[j])>math.radians(.1) for j in self.bus.joints),
                                     grip=1.,x=.02,left=(-.02,1.))
                    self.epoch += 1
                    self.drive_until(lambda:all(h["phase"]=="holding" for h in self.station.physical_arm["arms"].values()),
                                     x=.02,left=(-.02,0.))
                    self.station.control({"action":"restore_robot_initial"},local=True)
                    self.drive_until(lambda:all(h["phase"]=="returned" for h in self.station.physical_arm["arms"].values()),
                                     x=.02,left=(-.02,0.))
                    self.bus.fail_after = 3
                    deadline = time.monotonic()+2
                    while not all(bus.disabled for bus in buses.values()) and time.monotonic()<deadline:
                        self.send(left=(0.,0.))
                        time.sleep(.01)
                    self.assertTrue(all(bus.disabled for bus in buses.values()))
            last = self.send(left=(0.,0.),events=[{"id":2,"action":"record"}])
        self.assertTrue(all(bus.closed for bus in buses.values()))
        rows = self.saved_rows(last["recording"]["episode"])
        self.assertEqual({r["arm"] for r in rows if r["command"]=="return"},set(arms))
        fault = next(r for r in rows if r["error"] == "injected partial bus failure")
        self.assertEqual(fault["arm"],"right")
        self.assertEqual(list(fault["sent_joints"]),["1","2","3"])
        self.assertIsNone(fault["target_deg"])

    def test_return_cancellation_and_missed_grip_release_cannot_restart_old_request(self):
        initial = self.send()
        initial["hardware_return"] = None
        control = ArmControl("right",self.cal,self.home)
        bounds = {j:(-math.pi,math.pi) for j in self.home}
        def step(state):
            control.step(state,self.home,bounds,.05,state["arrival_s"],self.cli,control.activated)
        step(initial)
        step(self.send(grip=1.))
        step(self.send(grip=1.,x=.02))
        self.epoch += 1
        start = self.send()
        req = dict(id=1,session=start["session"],seq=start["seq"],trial_epoch=start["trial_epoch"],
                   grip_epochs={s:h["grip_epoch"] for s,h in start["hands"].items()},
                   sources={"right":start["hands"]["right"]["source"]},requested_s=start["arrival_s"])
        start["hardware_return"] = req
        # Give the return room to travel, then inject a grip release and regrip missed by the bus loop.
        control.ref[1] += .1
        step(start)
        self.assertEqual(control.phase,"returning")
        self.epoch += 1
        interrupted = self.send(grip=1.)
        interrupted["hardware_return"] = req
        step(interrupted)
        self.assertEqual(control.phase,"cancelled")
        ref = dict(control.ref)
        released = self.send()
        released["hardware_return"] = req
        step(released)
        self.assertEqual(control.ref,ref)
        self.assertIsNone(control.request)
