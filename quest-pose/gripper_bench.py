"""Supervised, serial-bound Quest test for one physical OpenArm J8 actuator on macOS."""

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

from teleop import STICK_DEAD_ZONE


HARDWARE = Path(__file__).resolve().parent.parent / "openarm-cli-smooth-accel-20261015-155332"
TOUCH = {"oculus-touch", "oculus-touch-v2", "oculus-touch-v3"}
PERIOD = .05
MAX_CYCLE = .20
MAX_INPUT_AGE = .15


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


@dataclass(frozen=True)
class Calibration:
    arm: str
    adapter_serial: str
    hardware_config: Path
    open_deg: float
    closed_deg: float
    max_speed_deg_s: float

    @property
    def bounds(self):
        return min(self.open_deg, self.closed_deg), max(self.open_deg, self.closed_deg)

    def check_position(self, angle):
        low, high = self.bounds
        if not finite(angle) or not low <= angle <= high:
            raise RuntimeError("Measured J8 encoder is outside the calibrated test segment.")


def load_calibration(path):
    path = Path(path).resolve()
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or set(data) != set(Calibration.__dataclass_fields__):
        raise ValueError("Calibration needs arm, adapter_serial, hardware_config, open_deg, closed_deg and max_speed_deg_s.")
    if data["arm"] not in ("left", "right"):
        raise ValueError("Select one calibrated arm: left or right.")
    serial = data["adapter_serial"]
    if (not isinstance(serial, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", serial)
            or serial.upper().startswith("REPLACE")):
        raise ValueError("Supply the verified adapter serial.")
    if not isinstance(data["hardware_config"], str) or not data["hardware_config"].strip():
        raise ValueError("Supply the path to the local hardware configuration.")
    if not all(finite(data[k]) for k in ("open_deg", "closed_deg", "max_speed_deg_s")):
        raise ValueError("Measure the J8 segment endpoints and set a finite bench speed.")
    if data["open_deg"] == data["closed_deg"] or not .1 <= data["max_speed_deg_s"] <= 1.:
        raise ValueError("J8 endpoints must differ and bench speed must be 0.1 to 1.0 degrees/s.")
    data["hardware_config"] = (path.parent / data["hardware_config"]).resolve()
    return Calibration(**data)


def bind_arm(cal, config):
    arm = config["arms"][cal.arm]
    if arm["serial"] != cal.adapter_serial:
        raise ValueError("Selected side and serial do not match the hardware configuration.")
    bounds = arm["limits_deg"].get("8")
    low, high = cal.bounds
    if (not isinstance(bounds, list) or len(bounds) != 2 or not all(finite(v) for v in bounds)
            or not bounds[0] <= low < high <= bounds[1]):
        raise ValueError("Both J8 test endpoints must be inside measured limits_deg for joint 8.")
    return arm


def _tracked_touch(hand):
    return (hand["tracked"] and hand["pose"] and hand["profile"] in TOUCH
            and re.fullmatch(r"xr-[0-9]+", hand["source"]) and hand["mapping"] == "xr-standard"
            and hand["target_ray_mode"] == "tracked-pointer" and len(hand["buttons"]) >= 6)


class BenchControl:
    """Integrate only new closure changes. A clipped increment never becomes queued travel."""

    def __init__(self, calibration):
        self.cal = calibration
        self.identity = self.boundary = self.epoch = self.seq = None
        self.ready = self.engaged = self.activated = False
        self.stick_ready = False
        self.ref = self.closure = self.last_now = None

    def _inputs(self, state):
        hand = state["hands"][self.cal.arm]
        right = state["hands"]["right"]
        age, lag = state["age_s"], state["source_lag_s"]
        live = (state["owner"] and state["owner"] == state["session"] and state["head"]
                and state["frame"] in ("local", "local-floor") and state["visible"] and state["view_valid"]
                and not state["held"] and _tracked_touch(hand) and _tracked_touch(right)
                and not right["buttons"][5]["pressed"] and finite(age) and finite(lag)
                and 0 <= age + lag <= MAX_INPUT_AGE)
        identity = (state["session"], state["frame"], hand["source"], hand["profile"],
                    right["source"], right["profile"], state.get("safety_epoch"))
        if not live or (self.identity is not None and self.identity != identity):
            if self.identity is not None:
                raise RuntimeError("Quest tracking, input freshness, controller source or station hold changed.")
            return None
        if self.identity is None:
            if hand["buttons"][1]["value"] > .35:
                return None
            self.identity = identity
        return hand

    @staticmethod
    def _stick(hand):
        axes = hand["axes"]
        if len(axes) < 4 or not finite(axes[3]) or abs(axes[3]) > 1:
            raise RuntimeError("Missing or invalid physical Touch thumbstick axis.")
        return axes[3]

    @staticmethod
    def _boundary(state):
        rec = state["recording"]
        return (state.get("trial_epoch"), rec["episode"], rec["active"], state["trial_return"]["status"])

    def step(self, state, measured_deg, now):
        self.cal.check_position(measured_deg)
        dt = 0. if self.last_now is None else now - self.last_now
        if not finite(dt) or dt < 0 or (self.activated and dt > MAX_CYCLE):
            raise RuntimeError("Physical gripper control cycle exceeded its deadline.")
        self.last_now = now
        hand = self._inputs(state)
        if hand is None:
            return None
        boundary = self._boundary(state)
        if self.boundary != boundary or self.epoch != hand["grip_epoch"]:
            self.engaged = self.ready = False
        self.boundary, self.epoch = boundary, hand["grip_epoch"]
        released = hand["buttons"][1]["value"] <= .35
        if released or not hand["clutched"] or state["trial_return"]["status"] == "returning":
            self.engaged = False
            self.ready = released and state["trial_return"]["status"] != "returning"
            self.seq = state["seq"]
            return self.ref
        if self.seq == state["seq"]:
            return self.ref
        self.seq = state["seq"]
        closure = hand["closure"]
        if not finite(closure) or not 0 <= closure <= 1:
            raise RuntimeError("Invalid gripper closure in station state.")
        stick = self._stick(hand) if state["gripper_mode"] == "stick" else None
        if not self.engaged:
            if not self.ready:
                return self.ref
            self.ref, self.closure = measured_deg, closure
            self.stick_ready = stick is not None and abs(stick) <= STICK_DEAD_ZONE
            self.engaged = self.activated = True
            self.ready = False
        else:
            span = self.cal.closed_deg - self.cal.open_deg
            limit = self.cal.max_speed_deg_s * dt
            if stick is None:
                delta = (closure - self.closure) * span
            elif abs(stick) <= STICK_DEAD_ZONE:
                self.stick_ready = True
                delta = 0.
            elif self.stick_ready:
                # Stick commands an encoder velocity, independent of the simulated aperture.
                fraction = (abs(stick) - STICK_DEAD_ZONE) / (1 - STICK_DEAD_ZONE)
                delta = math.copysign(limit * fraction, stick * span)
            else:
                delta = 0.
            low, high = self.cal.bounds
            self.ref = min(high, max(low, self.ref + min(limit, max(-limit, delta))))
            self.closure = closure
        return self.ref

    def confirm_enable(self, state, measured_deg, now):
        # Session.prepare() makes two stationary checks and can take longer than a normal tick.
        hand = self._inputs(state)
        if (not hand or not hand["clutched"] or hand["buttons"][1]["value"] <= .35
                or self.epoch != hand["grip_epoch"] or self.boundary != self._boundary(state)
                or self.seq == state["seq"]):
            raise RuntimeError("Fresh continued clutch required after J8 preparation; restarting is required.")
        self.cal.check_position(measured_deg)
        self.ref, self.closure, self.seq, self.last_now = measured_deg, hand["closure"], state["seq"], now
        if state["gripper_mode"] == "stick" and abs(self._stick(hand)) <= STICK_DEAD_ZONE:
            self.stick_ready = True


def _station_state(station):
    if not station.lock.acquire(timeout=MAX_INPUT_AGE):
        raise RuntimeError("Workstation input was blocked during the J8 cycle.")
    try:
        return station.state()
    finally:
        station.lock.release()


def run_bench(station, cal, cli, session, execute, stop):
    """Worker owns all USB I/O until exit; the main thread owns the Session's cleanup."""
    control = BenchControl(cal)
    enabled = False
    while not stop.is_set():
        start = time.monotonic()
        session.read_states(deadline=start + MAX_CYCLE)
        measured = math.degrees(session.states[8]["q"])
        stamp = time.monotonic_ns()
        check_ref = session.refs[8] if enabled else session.states[8]["q"]
        cli.check_feedback(8, session.meta[8], session.states[8], check_ref, cal.max_speed_deg_s, enabled)
        cal.check_position(measured)
        state = _station_state(station)
        now = time.monotonic()
        if now - start > MAX_CYCLE:
            raise RuntimeError("Delayed J8 encoder or workstation state.")
        target = control.step(state, measured, now)
        if stop.is_set():
            break
        if execute and control.engaged and not enabled:
            session.prepare(cal.max_speed_deg_s)
            # Preparation preloads a measured hold target while J8 is disabled.
            # A disturbance after its stationary checks invalidates that target.
            session.read_states(deadline=time.monotonic() + MAX_CYCLE)
            measured = math.degrees(session.states[8]["q"])
            stamp = time.monotonic_ns()
            cli.check_feedback(8, session.meta[8], session.states[8], session.refs[8], cal.max_speed_deg_s, False)
            if abs(measured - math.degrees(session.refs[8])) > .2:
                raise RuntimeError("J8 moved after the stationary check; enable cancelled.")
            state = _station_state(station)
            control.confirm_enable(state, measured, time.monotonic())
            if stop.is_set():
                break
            session.enable_prepared()
            enabled = True
            target = measured
        elif enabled:
            session.cycle({8: math.radians(target)}, cal.max_speed_deg_s, 1.)
            # cycle() checks another feedback frame internally. Publish the timed read
            # above; the end of a CAN write is not a feedback timestamp.
            cal.check_position(math.degrees(session.states[8]["q"]))
        phase = "tracking" if control.engaged else ("holding" if control.activated else "waiting")
        detail = "Side grip engaged" if control.engaged else "Release, then squeeze the side grip to take control"
        station.gripper_bench = dict(station.gripper_bench, phase=phase, enabled=enabled, detail=detail,
                                     measured_deg=measured, target_deg=target, feedback_host_ns=stamp)
        stop.wait(max(0., PERIOD - (time.monotonic() - start)))


@contextmanager
def _physical_session(station, cli, arm, side):
    # The legacy Session signals an unconfirmed disable by raising on a clean __exit__.
    # Close explicitly so it also verifies cleanup when the receiver exits on Ctrl+C/fault.
    with ExitStack() as resources:
        session = resources.enter_context(cli.Session(arm, side, (8,)))
        try:
            yield session
        except BaseException as exc:
            session.trace.record("failure", detail=str(exc), type=exc.__class__.__name__)
            raise
        finally:
            try:
                resources.close()
            except BaseException as exc:
                station.gripper_bench = dict(station.gripper_bench, phase="fault", enabled=None,
                                             detail=f"J8 cleanup unconfirmed: {exc}")
                raise
            status = station.gripper_bench
            phase = "fault" if status["phase"] == "fault" else "stopped"
            detail = status["detail"] if phase == "fault" else "Bench closed"
            station.gripper_bench = dict(status, phase=phase, enabled=False, detail=detail)


@contextmanager
def gripper_bench(station, config_path, execute=False):
    """Open only J8 on the identified Mac adapter; enable requires a local operator and fresh grip."""
    cal = load_calibration(config_path)
    if sys.platform != "darwin" or threading.current_thread() is not threading.main_thread():
        raise RuntimeError("The physical J8 bench requires the receiver on the main thread of the lab Mac.")
    if execute and not sys.stdin.isatty():
        raise RuntimeError("J8 enable needs an operator in the local receiver terminal.")
    # Link also imports the CLI's sibling canable_query module when the USB session opens.
    sys.path.insert(0, str(HARDWARE))
    try:
        import openarm_cli as cli
        with _bench_session(station, cal, cli, execute):
            yield
    finally:
        sys.path.remove(str(HARDWARE))


@contextmanager
def _bench_session(station, cal, cli, execute):
    arm = bind_arm(cal, cli.load_config(cal.hardware_config))
    status = dict(mode="execute" if execute else "shadow", arm=cal.arm, phase="waiting", enabled=False,
                  measured_deg=None, target_deg=None, feedback_host_ns=None, detail="Waiting for tracked Quest and release")
    station.gripper_bench = status
    stop, done, start = threading.Event(), threading.Event(), threading.Event()
    errors = []
    with cli.interrupt_handlers(), cli.arm_lock(cal.adapter_serial), _physical_session(station, cli, arm, cal.arm) as session:
        meta = session.meta[8]
        if meta["mode"] != 2 or not all(meta["bounds"][0] <= math.radians(x) <= meta["bounds"][1] for x in cal.bounds):
            raise ValueError("J8 must already be in mode 2 and both calibrated endpoints inside its encoder range.")
        meta["bounds"] = tuple(math.radians(x) for x in cal.bounds)
        session.stationary()
        measured = math.degrees(session.states[8]["q"])
        cal.check_position(measured)
        station.gripper_bench = dict(status, measured_deg=measured, feedback_host_ns=time.monotonic_ns())
        print(f"J8 {cal.arm} {status['mode']} | measured {measured:.3f} deg | "
              f"open {cal.open_deg:g}, closed {cal.closed_deg:g} deg | speed {cal.max_speed_deg_s:g} deg/s", flush=True)
        if execute:
            phrase = f"ENABLE {cal.arm} J8"
            if input(f"Support the arm and clear the jaw. Type {phrase} to allow the next new Quest clutch: ") != phrase:
                raise ValueError("Local J8 enable was not acknowledged.")

        def worker():
            try:
                start.wait()
                if not stop.is_set():
                    run_bench(station, cal, cli, session, execute, stop)
            except BaseException as exc:
                errors.append(exc)
                detail = str(exc)
                try:
                    if session.attempted:
                        session.request_disable()
                except Exception as disable_error:
                    detail += f" | immediate disable unconfirmed: {disable_error}"
                station.gripper_bench = dict(station.gripper_bench, phase="fault", detail=detail)
                if not stop.is_set():
                    # Interrupt the server's select() so normal main-thread Session cleanup runs.
                    os.kill(os.getpid(), signal.SIGTERM)
            finally:
                done.set()

        thread = threading.Thread(target=worker, name="quest-j8")
        try:
            thread.start()
            start.set()
            yield
        finally:
            stop.set()
            start.set()
            # Never close the USB session on the main thread while a worker call is in flight.
            while thread.ident is not None and not done.is_set():
                try:
                    done.wait(.1)
                except KeyboardInterrupt:
                    continue
            if thread.ident is not None:
                thread.join()
            if errors:
                raise RuntimeError(f"J8 bench stopped: {errors[0]}") from errors[0]
