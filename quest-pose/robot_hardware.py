"""Calibrated Quest joint teleoperation using the lab Mac's serial-bound OpenArm sessions."""

from contextlib import contextmanager, ExitStack
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
import signal
import sys
import threading
import time

from gripper_bench import HARDWARE, MAX_CYCLE, MAX_INPUT_AGE, PERIOD, _tracked_touch, finite


JOINTS = tuple(range(1, 8))


@dataclass
class ArmCalibration:
    serial: str
    zero: list
    sign: list
    speed: float
    gripper: dict

    @property
    def joints(self):
        return JOINTS + (8,) if self.gripper else JOINTS

    @property
    def speeds(self):
        result = {j: self.speed for j in JOINTS}
        if self.gripper:
            result[8] = self.gripper["max_speed_deg_s"]
        return result

    def model_joints(self, positions):
        return [self.sign[i] * (positions[i+1] - math.radians(self.zero[i])) for i in range(7)]

    def encoder_targets(self, hand):
        angles = hand["robot"]["joint_angles_rad"]
        targets = {j: math.radians(self.zero[j-1]) + self.sign[j-1]*angles[j-1] for j in JOINTS}
        if self.gripper:
            lo, hi = self.gripper["open_deg"], self.gripper["closed_deg"]
            targets[8] = math.radians(lo + (hi-lo)*hand["closure"])
        return targets


def load_calibration(path):
    path = Path(path).resolve()
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or set(data) != {"hardware_config", "arms"}:
        raise ValueError("Hardware calibration needs hardware_config and arms.")
    if not isinstance(data["hardware_config"], str) or not data["hardware_config"].strip():
        raise ValueError("Set hardware_config to the local serial and encoder limits file.")
    entries = data["arms"]
    if not isinstance(entries, dict) or not entries or not set(entries) <= {"left","right"}:
        raise ValueError("Configure the connected left arm, right arm, or both.")
    arms = {}
    for side, arm in entries.items():
        required = {"adapter_serial","encoder_zero_deg","encoder_sign","max_speed_deg_s"}
        if not isinstance(arm, dict) or not required <= arm.keys() <= required | {"gripper"}:
            raise ValueError(f"{side}: supply the verified serial, seven encoder zeros and signs, and speed.")
        serial, zero, sign, speed = (arm[k] for k in ("adapter_serial","encoder_zero_deg","encoder_sign","max_speed_deg_s"))
        if not isinstance(serial, str) or not re.fullmatch(r"[A-Za-z0-9_-]+",serial) or serial.upper().startswith("REPLACE"):
            raise ValueError(f"{side}: supply the verified CANable serial.")
        if not isinstance(zero,list) or len(zero)!=7 or not all(finite(x) for x in zero):
            raise ValueError(f"{side}: supply seven measured encoder offsets in degrees.")
        if not isinstance(sign,list) or len(sign)!=7 or not all(finite(x) and x in (-1,1) for x in sign):
            raise ValueError(f"{side}: supply seven verified encoder directions (+1 or -1).")
        if not finite(speed) or not .1 <= speed <= 5:
            raise ValueError(f"{side}: choose a lab-approved arm speed within the CLI range 0.1–5 deg/s.")
        jaw = arm.get("gripper")
        if "gripper" in arm and (not isinstance(jaw, dict) or set(jaw) != {"open_deg","closed_deg","max_speed_deg_s"}
                or not all(finite(v) for v in jaw.values()) or jaw["open_deg"] == jaw["closed_deg"]
                or not .1 <= jaw["max_speed_deg_s"] <= 1):
            raise ValueError(f"{side}: optional J8 requires distinct measured endpoints and 0.1–1 deg/s.")
        arms[side] = ArmCalibration(serial, zero, sign, speed, jaw)
    if len({a.serial for a in arms.values()}) != len(arms):
        raise ValueError("Each physical arm requires its own CANable serial.")
    return path.parent / data["hardware_config"], arms


def bind_session(station, side, cal, session):
    for j in cal.joints:
        bounds = session.arm["limits_deg"].get(str(j))
        if not isinstance(bounds, list) or len(bounds) != 2 or not all(finite(v) for v in bounds) or bounds[0] >= bounds[1]:
            raise ValueError(f"{side} J{j}: measured encoder limits are required.")
        lo, hi = map(math.radians,bounds)
        meta = session.meta[j]
        if meta["mode"] not in (1,2) or (j == 8 and meta["mode"] != 2):
            raise ValueError(f"{side} J{j}: unsupported motor mode {meta['mode']}.")
        if cal.gripper and j == 8:
            lo, hi = sorted(math.radians(cal.gripper[k]) for k in ("open_deg","closed_deg"))
        if not meta["bounds"][0] <= lo < hi <= meta["bounds"][1]:
            raise ValueError(f"{side} J{j}: calibrated travel exceeds the drive's configured range.")
        if j in JOINTS:
            model_bounds = station.engine.hands[side].robot.limits[j-1]
            model_low, model_high = sorted(math.radians(cal.zero[j-1])+cal.sign[j-1]*a for a in model_bounds)
            lo, hi = max(lo,model_low), min(hi,model_high)
            if lo >= hi:
                raise ValueError(f"{side} J{j}: modeled and approved travel have no overlap.")
        meta["bounds"] = (lo,hi)
    session.stationary()
    initial = dict(session.refs)
    if any(not session.meta[j]["bounds"][0] <= initial[j] <= session.meta[j]["bounds"][1] for j in cal.joints):
        raise ValueError(f"{side}: startup encoder lies outside the approved travel.")
    with station.lock:
        hand = station.engine.hands[side]
        angles = cal.model_joints(initial)
        hand.robot.set_joints(angles)
        hand.tool = hand.robot.tcp
        station.engine.initial_joints[side] = angles
        if cal.gripper:
            hand.closure = (math.degrees(initial[8])-cal.gripper["open_deg"])/(cal.gripper["closed_deg"]-cal.gripper["open_deg"])
    return initial


def fresh(state, side, now):
    age = now - state["arrival_s"] if state and finite(state.get("arrival_s")) else math.inf
    return bool(state and state["owner"] and state["owner"] == state["session"] and state["head"]
                and state["visible"] and state["view_valid"] and not state["held"]
                and state["frame"] in ("local", "local-floor") and finite(state["source_lag_s"])
                and 0 <= age and age + state["source_lag_s"] <= MAX_INPUT_AGE
                and _tracked_touch(state["hands"][side]) and _tracked_touch(state["hands"]["right"])
                and not state["hands"]["right"]["buttons"][5]["pressed"])


def identity(state, side):
    return (state["session"], state["frame"], state["safety_epoch"],
            *((state["hands"][s]["source"],state["hands"][s]["profile"]) for s in (side,"right")))


def released(hand):
    buttons = hand["buttons"]
    return not buttons or (len(buttons) >= 2 and buttons[1]["value"] <= .35)


class _Closing(Exception):
    """The context is exiting and the main thread will verify disable."""


def return_allowed(state, request):
    return bool(request and state["hardware_return"] == request and request["session"] == state["session"]
                and request["trial_epoch"] == state["trial_epoch"] and
                all(_tracked_touch(state["hands"][s]) and state["hands"][s]["source"] == source
                    for s, source in request["sources"].items()) and
                all(released(h) and request["grip_epochs"][s] == h["grip_epoch"] for s,h in state["hands"].items()))


class ArmControl:
    def __init__(self, side, cal, initial):
        self.side, self.cal, self.home = side, cal, dict(initial)
        self.ref = dict(initial)
        self.identity = self.boundary = self.seq = None
        self.model_anchor = self.encoder_anchor = None
        self.ready = self.engaged = self.activated = False
        self.request = self.last_request = self.origin = None
        self.elapsed = self.settle = self.stable = 0.
        self.phase = "waiting"

    def step(self, state, measured, bounds, dt, now, cli, enabled):
        if not fresh(state,self.side,now):
            self.ready = self.engaged = False
            self.request = None
            if enabled:
                raise RuntimeError("Quest input lost, stale, held, or right B pressed.")
            self.identity = None
            self.activated = False
            self.phase = "waiting"
            return
        key = identity(state,self.side)
        if key != self.identity:
            if enabled:
                raise RuntimeError("Quest session, controller or safety state changed.")
            self.identity = key
            self.ready = self.engaged = False
            self.request = None
            self.activated = False
        hand = state["hands"][self.side]
        boundary = (state["trial_epoch"], hand["grip_epoch"])
        if boundary != self.boundary:
            self.ready = self.engaged = False
        self.boundary = boundary
        request = state.get("hardware_return")
        if request and request["id"] != self.last_request:
            self.last_request = request["id"]
            if (self.activated and self.request is None and 0 <= now-request["requested_s"] <= MAX_INPUT_AGE
                    and return_allowed(state,request)):
                self.request, self.origin = request, dict(self.ref)
                self.elapsed = self.settle = self.stable = 0.
                self.engaged = self.ready = False
                self.phase = "returning"
        if self.request:
            if not return_allowed(state,self.request):
                self.request = None
                self.phase = "cancelled"
                self.ready = self.engaged = False
                return
            if self.seq == state["seq"]:
                return
            self.seq = state["seq"]
            self.elapsed += dt
            target, reached = cli.pose_reference({j:self.origin[j] for j in JOINTS},
                                                  {j:self.home[j] for j in JOINTS}, self.elapsed, self.cal.speed)
            self.ref.update(target)
            if reached:
                self.settle += dt
                if all(abs(measured[j]-self.home[j]) <= math.radians(.5) for j in JOINTS):
                    self.stable += dt
                else:
                    self.stable = 0.
                if self.stable >= .2:
                    self.phase, self.request = "returned", None
                elif self.settle > 1.:
                    raise RuntimeError("Hardware return failed to settle at the initial encoder positions.")
            return
        if released(hand) or not hand["clutched"]:
            self.engaged = False
            self.ready = released(hand)
            if self.phase not in ("returned","cancelled"):
                self.phase = "holding" if self.activated else "waiting"
            return
        if self.seq == state["seq"]:
            return
        self.seq = state["seq"]
        wanted = self.cal.encoder_targets(hand)
        if not all(finite(v) for v in wanted.values()):
            raise ValueError("Invalid simulated joint target.")
        if not self.engaged:
            if not self.ready:
                return
            self.engaged = self.activated = True
            self.ready = False
            self.phase = "tracking"
            if not enabled:
                self.ref = dict(measured)
            self.model_anchor = wanted
            self.encoder_anchor = dict(self.ref)
            return
        for j, value in wanted.items():
            limit = math.radians(self.cal.speeds[j])*dt
            goal = self.encoder_anchor[j] + value - self.model_anchor[j]
            delta = max(-limit,min(limit,goal-self.ref[j]))
            low, high = bounds[j]
            self.ref[j] = min(high,max(low,self.ref[j]+delta))
        self.phase = "tracking"


def feedback(session):
    joints = session.joints
    return {
        "feedback_host_ns": max(session.feedback_ns.values()),
        "joint_feedback_ns": [session.feedback_ns[j] for j in joints],
        "measured_deg": [math.degrees(session.states[j]["q"]) for j in joints],
        "velocity_deg_s": [math.degrees(session.states[j]["v"]) for j in joints],
        "torque_nm": [session.states[j]["torque"] for j in joints],
        "temperature_c": [session.states[j]["temperature"] for j in joints],
        "motor_status": [session.states[j]["status"] for j in joints],
    }


def run_arm(side, station, cal, cli, session, initial, execute, stop, publish):
    control = ArmControl(side, cal, initial)
    bounds = {j:session.meta[j]["bounds"] for j in session.joints}
    speeds = cal.speeds
    enabled_at = None
    previous = time.monotonic()
    while not stop.is_set():
        # No station lock is taken after reserving a recorder row; finishing a trial
        # can wait for this cycle to complete while holding the station lock.
        if not station.lock.acquire(timeout=MAX_INPUT_AGE):
            raise RuntimeError("Station failed to deliver a timely hardware state.")
        try:
            state = station.state()
            episode = state["recording"]["episode"] if state["recording"]["active"] else None
            token = station.recorder.begin_robot_io(episode)
        finally:
            station.lock.release()
        start_ns = time.monotonic_ns()
        begun = start_ns/1e9
        enabled = enabled_at is not None
        row = dict(arm=side, session=state["session"], seq=state["seq"], phase=control.phase,
                   start_ns=start_ns, host_monotonic_ns=start_ns, target_deg=None,
                   joints=list(session.joints), mode="drive" if execute else "shadow", command="observe",
                   sent_joints={}, enabled=enabled, error=None)
        try:
            session.read_states(deadline=begun+MAX_CYCLE)
            row.update(feedback(session))
            measured = {j:session.states[j]["q"] for j in session.joints}
            for j in session.joints:
                ref = session.refs[j] if enabled else measured[j]
                cli.check_feedback(j, session.meta[j], session.states[j], ref, speeds[j], enabled)
                if not bounds[j][0] <= measured[j] <= bounds[j][1]:
                    raise RuntimeError(f"{side} J{j}: measured encoder outside approved limits.")
            now = time.monotonic()
            if now-begun > MAX_CYCLE or (enabled and now-previous > MAX_CYCLE):
                raise RuntimeError("Hardware feedback or command cycle exceeded its deadline.")
            return_lease = control.request
            control.step(state, measured, bounds, min(now-previous,MAX_CYCLE), now, cli, enabled)
            return_lease = control.request or return_lease
            previous = now
            # Use newly published input for the final interlock, without spending
            # another network sample waiting in a recorder/HTTP critical section.
            def latest_input():
                live = station.hardware_input
                if stop.is_set():
                    raise _Closing()
                if not fresh(live,side,time.monotonic()) or identity(live,side) != control.identity:
                    raise RuntimeError("Hardware command lost its input lease.")
                return live

            def still_clutched(live):
                hand = live["hands"][side]
                return (control.boundary == (live["trial_epoch"],hand["grip_epoch"])
                        and hand["clutched"] and not released(hand))

            if execute and control.engaged and not enabled:
                row["command"] = "prepare"
                def preloaded(j, value, stamp):
                    if "preloaded_joints" not in row:
                        row["host_monotonic_ns"] = stamp
                        row["preloaded_joints"] = {}
                    row["preloaded_joints"][str(j)] = {"target_deg":math.degrees(value),"host_ns":stamp,
                                                       "completed_ns":time.monotonic_ns()}

                def enabled_joint(j, stamp):
                    row.setdefault("enabled_joints",{})[str(j)] = {"host_ns":stamp,"completed_ns":time.monotonic_ns()}

                session.prepare(speeds,on_preload=preloaded)
                session.read_states(deadline=time.monotonic()+MAX_CYCLE)
                row.update(feedback(session))
                for j in session.joints:
                    current = session.states[j]
                    cli.check_feedback(j,session.meta[j],current,session.refs[j],speeds[j],False)
                    if not bounds[j][0] <= current["q"] <= bounds[j][1] or abs(current["q"]-session.refs[j]) > math.radians(.2):
                        raise RuntimeError(f"{side} J{j}: moved after stationary preparation.")
                    if abs(current["q"]-initial[j]) > math.radians(.5):
                        raise RuntimeError(f"{side} J{j}: arm moved before first enable. Support it and restart to capture its new home.")

                def allow_enable():
                    live = latest_input()
                    if live["seq"] == state["seq"] or not still_clutched(live):
                        raise RuntimeError("A fresh, uninterrupted side grip is required to enable.")

                allow_enable()
                row.update(command="enable",host_monotonic_ns=time.monotonic_ns())
                session.enable_prepared(guard=allow_enable,on_enabled=enabled_joint)
                enabled_at = previous = time.monotonic()
                live = latest_input()
                control.ref = dict(session.refs)
                control.model_anchor = cal.encoder_targets(live["hands"][side])
                control.encoder_anchor = dict(session.refs)
                control.seq = live["seq"]
                row["target_deg"] = [math.degrees(session.refs[j]) for j in session.joints]
            elif enabled:
                def select_target(j, target, hold):
                    if station.hardware_input is None:
                        # A record boundary invalidates the old sample before blocking on disk.
                        # Sending a hold for this one cycle allows collection to finish.
                        return hold
                    live = latest_input()
                    if return_lease and not return_allowed(live, return_lease):
                        return hold
                    if control.engaged and not still_clutched(live):
                        return hold
                    return target

                def sent(j, value, stamp):
                    if not row["sent_joints"]:
                        row["host_monotonic_ns"] = stamp
                    row["sent_joints"][str(j)] = {"target_deg":math.degrees(value), "host_ns":stamp,
                                                  "completed_ns":time.monotonic_ns()}

                row["command"] = "return" if return_lease and control.phase in ("returned","returning") else "track" if control.engaged else "hold"
                session.send_references(control.ref,speeds,min(1.,now-enabled_at),begun,select_target,sent)
                control.ref = dict(session.refs)
                row["target_deg"] = [math.degrees(session.refs[j]) for j in session.joints]
            row["phase"] = control.phase
            row["enabled"] = enabled_at is not None
            row["requested_deg"] = [math.degrees(control.ref[j]) for j in session.joints]
            publish(side,phase=control.phase,enabled=row["enabled"],target_deg=row["requested_deg"],
                    measured_deg=row["measured_deg"],feedback_host_ns=row["feedback_host_ns"],detail="")
        except _Closing:
            row.update(command="stop",phase="stopped")
            if session.attempted:
                session.request_disable()
            return
        except BaseException as exc:
            row.update(error=str(exc),phase="fault",enabled=None if session.attempted else False)
            row["enable_attempted"] = list(session.attempted)
            if session.attempted:
                session.request_disable()
            raise
        finally:
            row["end_ns"] = time.monotonic_ns()
            station.publish_robot_io(row,token)
        stop.wait(max(0., PERIOD - (time.monotonic()-begun)))


@contextmanager
def robot_hardware(station, path, execute=False):
    config_path, arms = load_calibration(path)
    if sys.platform != "darwin" or threading.current_thread() is not threading.main_thread():
        raise RuntimeError("Connect the CANable adapters to the lab Mac and run the receiver on its main thread.")
    if execute and not sys.stdin.isatty():
        raise RuntimeError("Drive mode requires arming in the local Mac terminal.")
    sys.path.insert(0, str(HARDWARE))
    try:
        import openarm_cli as cli
        with physical_sessions(station, config_path, arms, cli, execute):
            yield
    finally:
        sys.path.remove(str(HARDWARE))


@contextmanager
def physical_sessions(station, config_path, arms, cli, execute):
    hardware = cli.load_config(config_path)["arms"]
    stop = threading.Event()
    status_lock = threading.Lock()
    sessions, initial, workers, errors = {}, {}, [], []
    mode = "drive" if execute else "shadow"
    station.physical_arm = {"mode":mode,"arms":{},"error":None}

    def publish(side, **changes):
        with status_lock:
            current = station.physical_arm
            arm_states = dict(current["arms"])
            arm_states[side] = {**arm_states.get(side,{}), **changes}
            station.physical_arm = dict(current, arms=arm_states, error=(
                arm_states[side].get("detail") if changes.get("phase") == "fault" else current["error"]))

    def worker(side):
        try:
            run_arm(side,station,arms[side],cli,sessions[side],initial[side],execute,stop,publish)
        except BaseException as exc:
            errors.append(exc)
            notify = not stop.is_set()
            stop.set()
            publish(side, phase="fault", enabled=None if sessions[side].attempted else False, detail=str(exc))
            if notify:
                os.kill(os.getpid(),signal.SIGTERM)
        finally:
            if sessions[side].attempted:
                sessions[side].request_disable()

    with cli.interrupt_handlers(), ExitStack() as resources:
        try:
            for side, cal in arms.items():
                if hardware[side]["serial"] != cal.serial:
                    raise ValueError(f"{side}: CANable serial does not match hardware_config.")
                resources.enter_context(cli.arm_lock(cal.serial))
                session = resources.enter_context(cli.Session(hardware[side],side,cal.joints))
                sessions[side] = session
                initial[side] = bind_session(station,side,cal,session)
                positions = [math.degrees(initial[side][j]) for j in cal.joints]
                publish(side,joints=list(cal.joints),phase="waiting",enabled=False,initial_deg=positions,
                        measured_deg=positions,target_deg=None,feedback_host_ns=max(session.feedback_ns.values()),detail="")
                print(f"{side} J1–J{cal.joints[-1]} | stationary startup encoders (deg): "
                      + ", ".join(f"{q:+.2f}" for q in positions), flush=True)
            if execute:
                phrase = "ENABLE " + " ".join(sorted(arms)) + " ARM"
                if input(f"Support the arm(s) and clear the return path. Type {phrase} to arm the Quest grips: ") != phrase:
                    raise ValueError("Physical arm enable was not acknowledged.")
            for side in arms:
                thread = threading.Thread(target=worker, args=(side,), name=f"quest-arm-{side}")
                thread.start()
                workers.append(thread)
            yield
        finally:
            stop.set()
            for thread in workers:
                while thread.is_alive():
                    try:
                        thread.join(.1)
                    except KeyboardInterrupt:
                        continue
            # A normal Session exit reports unconfirmed disables as exceptions.
            # Invoke it even on interruption or on partial multi-arm setup.
            try:
                resources.close()
            except BaseException as exc:
                for side in sessions:
                    publish(side,phase="fault",enabled=None,detail=f"Disable/cleanup unconfirmed: {exc}")
                raise
            for side in sessions:
                if side in station.physical_arm["arms"]:
                    failed = station.physical_arm["arms"][side]["phase"] == "fault"
                    publish(side, phase="fault" if failed else "stopped", enabled=False)
            if errors:
                raise RuntimeError(f"Physical arm session stopped: {errors[0]}") from errors[0]
