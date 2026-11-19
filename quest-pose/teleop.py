"""Relative-pose arm simulator. Positions are metres and quaternions are XYZW.

This module has no hardware interface. The server serializes calls to Teleop.
"""

from copy import deepcopy
import json
import math
import time

from robot_sim import HOME, RobotArm, inverse, product
from trial import Trial

SIDES = ("left", "right")
IDENTITY = [0.0, 0.0, 0.0, 1.0]
CONTROL_PROFILES = {"mouse", "oculus-touch", "oculus-touch-v2", "oculus-touch-v3"}
GRIP_ON, GRIP_OFF = 0.65, 0.35
TRIGGER_PICKUP = 0.025
STICK_DEAD_ZONE = 0.15
STICK_RATE = 0.5  # normalized aperture range per second at full deflection


def number(value, name, low, high):
    if type(value) not in (int, float) or not low <= value <= high:
        raise ValueError(f"{name} must be a finite number in [{low}, {high}]")
    return float(value)


def integer(value, name, low=0):
    if type(value) is not int or not low <= value < 2**53:
        raise ValueError(f"invalid {name}")
    return value


def vector(value, length, name, low, high):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name} must contain {length} numbers")
    return [number(x, name, low, high) for x in value]


def quaternion(value):
    q = vector(value, 4, "quaternion XYZW", -1.02, 1.02)
    norm = math.hypot(*q)
    if abs(norm - 1) > 0.02:
        raise ValueError("expected a unit quaternion XYZW")
    return [x / norm for x in q]


def aligned(q, reference):
    if sum(a*b for a, b in zip(q, reference)) < 0:
        return [-x for x in q]
    return q


def difference(current, origin):
    return {"p": [v - u for v, u in zip(current["p"], origin["p"])],
            "q": product(current["q"], inverse(origin["q"]))}


def tool_goal(delta, anchor, alignment, scale):
    """Map a tracking-space hand delta onto the anchored tool in robot space."""
    inv_a = inverse(alignment)
    dp = product(product(alignment, delta["p"] + [0]), inv_a)[:3]
    p = [u + scale*d for u, d in zip(anchor["p"], dp)]
    dq = product(product(alignment, delta["q"]), inv_a)
    return {"p": p, "q": product(dq, anchor["q"])}


def parse_pose(data):
    if data is None:
        return None
    if not isinstance(data, dict):
        raise ValueError("pose must be an object or null")
    return {"p": vector(data.get("p"), 3, "tracking position (m)", -10000, 10000),
            "q": quaternion(data.get("q"))}


def parse_hand(hand, side):
    if hand is None:
        return {"pose": None, "handedness": side, "profile": "", "source": "", "profiles": [],
                "mapping": "", "target_ray_mode": "", "grip_epoch": 0, "buttons": [], "axes": []}
    if not isinstance(hand, dict) or hand.get("handedness", side) != side:
        raise ValueError(f"invalid {side} controller")
    profile, source = hand.get("profile"), hand.get("source", "")
    mapping, target_ray_mode = hand.get("mapping", ""), hand.get("target_ray_mode", "")
    profiles = hand.get("profiles", [])
    if not isinstance(profiles, list) or len(profiles) > 32:
        raise ValueError("invalid controller profiles")
    if any(not isinstance(v, str) or len(v) > 128 for v in [profile, source, mapping, target_ray_mode, *profiles]):
        raise ValueError("invalid controller metadata")
    raw_buttons, raw_axes = hand.get("buttons"), hand.get("axes")
    if not isinstance(raw_buttons, list) or len(raw_buttons) > 32:
        raise ValueError("expected up to 32 raw buttons")
    if not isinstance(raw_axes, list) or len(raw_axes) > 16:
        raise ValueError("expected up to 16 axes")
    buttons = []
    for button in raw_buttons:
        if not isinstance(button, dict) or type(button.get("pressed")) is not bool:
            raise ValueError("expected a raw pressed flag")
        parsed = {"pressed": button["pressed"], "value": number(button.get("value"), "button value", 0, 1)}
        if "touched" in button:
            if type(button["touched"]) is not bool:
                raise ValueError("expected a raw touched flag")
            parsed["touched"] = button["touched"]
        buttons.append(parsed)
    return {"pose": parse_pose(hand.get("pose")), "handedness": side, "profile": profile,
            "source": source, "profiles": list(profiles), "mapping": mapping,
            "target_ray_mode": target_ray_mode, "grip_epoch": integer(hand.get("grip_epoch", 0), "grip epoch"),
            "buttons": buttons,
            "axes": [number(a, "axis", -1, 1) for a in raw_axes]}


def parse_sample(data):
    if not isinstance(data, dict):
        raise ValueError("sample must be an object")
    # The raw packet, including extra browser metadata, is written to a strict JSON stream.
    json.dumps(data, allow_nan=False)
    session = data.get("session")
    if not isinstance(session, str) or not 1 <= len(session) <= 128:
        raise ValueError("invalid session ID")
    frame, visible, reset = data.get("frame"), data.get("visible"), data.get("reset", False)
    if frame not in ("local", "local-floor", "mouse-local"):
        raise ValueError("expected a stationary tracking frame")
    view_valid = data.get("view_valid", True)
    if any(type(v) is not bool for v in (visible, reset, view_valid)):
        raise ValueError("visible, reset and view_valid must be booleans")
    hands = data.get("hands")
    if not isinstance(hands, dict) or set(hands) - set(SIDES):
        raise ValueError("invalid hands")
    events = data.get("events", [])
    if not isinstance(events, list) or len(events) > 64:
        raise ValueError("expected up to 64 button events")
    clean_events = []
    for event in events:
        if not isinstance(event, dict) or event.get("action") not in ("record", "finish_return", "restore_initial", "hold", "view", "mark"):
            raise ValueError("invalid button event")
        clean_events.append({"id": integer(event.get("id"), "event ID", 1), "action": event["action"]})
    return {"session": session, "seq": integer(data.get("seq"), "sequence"),
            "time_ms": number(data.get("time_ms"), "source timestamp", 0, 2**53),
            "frame": frame, "visible": visible, "reset": reset, "view_valid":view_valid,
            "head": parse_pose(data.get("head")),
            "hands": {side: parse_hand(hands.get(side), side) for side in SIDES}, "events": clean_events}


class Hand:
    def __init__(self, side):
        self.home = HOME[side]
        self.tool = {"p": list(self.home), "q": IDENTITY.copy()}
        self.robot = RobotArm(side)
        self.closure = 0.0
        self.initial = None
        self.raw = parse_hand(None, side)
        self.status = "untracked"
        self.release()

    def release(self):
        self.clutched = False
        self.ready = False
        self.anchor = None
        self.tool_anchor = None
        self.trigger_live = False
        self.stick_live = False
        self.last_trigger = None

    def forget_tracking(self):
        self.release()
        self.raw["pose"] = None
        self.status = "untracked"

    def observe(self, raw, tracking):
        # A release and re-grip can both occur while an earlier POST is in flight.
        if any(raw[key] != self.raw[key] for key in ("profile", "source", "grip_epoch")):
            self.release()
        previous = self.raw["pose"]
        self.raw = raw
        pose = raw["pose"]
        if not tracking or pose is None:
            self.forget_tracking()
            return
        orientation_reference = previous or self.initial
        if orientation_reference:
            pose["q"] = aligned(pose["q"], orientation_reference["q"])
        if self.initial is None:
            self.initial = pose

    @property
    def usable(self):
        return (self.raw["pose"] is not None and self.raw["profile"] in CONTROL_PROFILES
                and (self.raw["profile"] == "mouse" or self.raw["mapping"] in ("", "xr-standard"))
                and len(self.raw["buttons"]) >= 2)

    @property
    def grip_engaged(self):
        buttons = self.raw["buttons"]
        return len(buttons) >= 2 and buttons[1]["value"] > GRIP_OFF

    def update(self, held, alignment, scale, gripper_mode, dt, grip_only=False):
        if not self.usable or held:
            self.release()
            self.status = ("held" if held else "unmapped") if self.raw["pose"] else "untracked"
            return
        trigger, grip = [b["value"] for b in self.raw["buttons"][:2]]
        axes = self.raw["axes"]
        stick = axes[3] if len(axes) > 3 else None
        pose = self.raw["pose"]
        if grip <= GRIP_OFF:
            self.release()
            self.ready = True
            self.status = "released"
            return
        if not self.clutched:
            if (not grip_only and not self.ready) or grip < GRIP_ON:
                if self.status != "outside simulated reach":
                    self.status = "waiting for grip" if self.ready else "release grip"
                return
            self.clutched = True
            self.ready = False
            self.anchor = pose
            self.tool_anchor = self.tool
            self.last_trigger = trigger
            self.trigger_live = gripper_mode == "trigger" and abs(trigger - self.closure) <= TRIGGER_PICKUP
            self.stick_live = stick is not None and abs(stick) <= STICK_DEAD_ZONE
            self.status = "clutched"
            return  # Engaging always preserves both the pose and aperture on this sample.

        goal = tool_goal(difference(pose, self.anchor), self.tool_anchor, alignment, scale)
        goal["q"] = aligned(goal["q"], self.tool["q"])
        if not self.robot.solve(goal):
            if not grip_only:
                self.release()
            self.status = "outside simulated reach"
            return
        self.tool = goal
        self.status = "clutched"

        if gripper_mode == "stick":
            if stick is None:
                self.stick_live = False
            elif abs(stick) <= STICK_DEAD_ZONE:
                self.stick_live = True
            elif self.stick_live:
                speed = (abs(stick) - STICK_DEAD_ZONE) / (1 - STICK_DEAD_ZONE)
                self.closure = max(0., min(1., self.closure + math.copysign(STICK_RATE*dt*speed, stick)))
            return
        if not self.trigger_live:
            crossed = (self.last_trigger - self.closure) * (trigger - self.closure) <= 0
            self.trigger_live = crossed or abs(trigger - self.closure) <= TRIGGER_PICKUP
        if self.trigger_live:
            self.closure = trigger
        self.last_trigger = trigger

    def snapshot(self):
        pose = self.raw["pose"]
        return {**self.raw, "tracked": pose is not None, "initial": self.initial,
                "delta": difference(pose, self.initial) if pose and self.initial else None,
                "anchor": self.anchor, "tool_anchor": self.tool_anchor,
                "clutch_delta": difference(pose, self.anchor) if pose and self.anchor else None,
                "clutched": self.clutched, "tool": self.tool, "closure": self.closure,
                "trigger_live": self.trigger_live, "stick_live": self.stick_live, "status": self.status,
                "robot": self.robot.snapshot(self.closure)}


class Teleop:
    """One operator's latest sample and two independent simulated tool targets."""

    def __init__(self, scale=1.0, alignment=IDENTITY, timeout_s=0.4, gripper_mode="trigger"):
        if gripper_mode not in ("trigger", "stick"):
            raise ValueError("gripper mode must be trigger or stick")
        self.gripper_mode = gripper_mode
        self.trial = Trial()
        self.view_valid = True
        self.scale = number(scale, "motion scale", 0, 10)
        self.alignment = quaternion(alignment)
        self.timeout_s = number(timeout_s, "sample timeout (s)", 0.01, 10)
        self.session = self.seq = self.frame = self.head = None
        self.time_ms = self.arrival_s = self.clock_offset = self.source_lag = self.period = None
        self.visible = False
        self.grip_only = False
        self.held = False
        self.reason = ""
        self.ack = 0
        self.hands = {side: Hand(side) for side in SIDES}
        self.initial_joints = {side: list(hand.robot.angles) for side, hand in self.hands.items()}

    def _hold(self, reason):
        self.trial.cancel(reason)
        self.held, self.reason = not self.grip_only, reason
        for hand in self.hands.values():
            hand.release()
            hand.status = "held" if hand.raw["pose"] else "untracked"

    def _lose_tracking(self):
        self.head = None
        for hand in self.hands.values():
            hand.forget_tracking()

    def _expire(self, now):
        if self.arrival_s is None or self.head is None:
            return
        if now - self.arrival_s + self.source_lag > self.timeout_s:
            self._hold("stale input")
            self._lose_tracking()

    def _start_session(self, session):
        if self.session is not None:
            self._hold("session changed")
        self.session, self.seq, self.frame, self.ack = session, None, None, 0
        self.time_ms = self.arrival_s = self.clock_offset = self.source_lag = self.period = None
        self._lose_tracking()
        for hand in self.hands.values():
            hand.initial = None

    def _update_timing(self, sample, now):
        if self.arrival_s is not None and now > self.arrival_s:
            period = now - self.arrival_s
            self.period = period if self.period is None else 0.85*self.period + 0.15*period
        self.time_ms = sample["time_ms"]
        self.arrival_s = now
        offset = now - self.time_ms / 1000
        self.clock_offset = offset if self.clock_offset is None else min(self.clock_offset, offset)
        self.source_lag = offset - self.clock_offset

    def _events(self, events):
        applied = []
        for event in events:
            if event["id"] == self.ack + 1:
                self.ack = event["id"]
                applied.append(event)
        return applied

    def _b_pressed(self):
        raw = self.hands["right"].raw
        return raw["profile"] in CONTROL_PROFILES and len(raw["buttons"]) > 5 and raw["buttons"][5]["pressed"]

    def apply(self, data, now=None, *, quest_simulation=False):
        """Consume one packet. Replays and out-of-order packets do not refresh its lifetime."""
        now = time.monotonic() if now is None else now
        self._expire(now)
        try:
            sample = parse_sample(data)
        except ValueError:
            if isinstance(data, dict) and self.session and data.get("session") == self.session:
                self._hold("invalid sample")
                self._lose_tracking()
            raise
        new_session = sample["session"] != self.session
        if not new_session and self.seq is not None and (sample["seq"] <= self.seq or sample["time_ms"] < self.time_ms):
            return self.snapshot(now)

        self.grip_only = quest_simulation and sample["frame"] in ("local", "local-floor")
        if new_session:
            self._start_session(sample["session"])
        if self.grip_only:
            self.held, self.reason = False, ""
        self.seq = sample["seq"]
        # Clamp long intervals and require advancement on both clocks. A gap,
        # duplicate, clock jump or a queued sample cannot integrate a large step.
        dt = 0. if self.arrival_s is None else max(0., min(0.1, now-self.arrival_s,
                                                           (sample["time_ms"]-self.time_ms)/1000))
        self._update_timing(sample, now)
        reset = sample["reset"] or self.frame is not None and sample["frame"] != self.frame
        self.frame, self.visible, self.view_valid = sample["frame"], sample["visible"], sample["view_valid"]
        self.head = sample["head"]
        if reset:
            for hand in self.hands.values():
                hand.initial = None
            invalid = "reference frame reset"
        elif not self.visible:
            invalid = "input hidden"
        elif not self.view_valid and not self.grip_only:
            invalid = "camera display unavailable"
        elif self.source_lag > self.timeout_s:
            invalid = "delayed input"
        elif self.head is None:
            invalid = "head tracking lost"
        else:
            invalid = ""
        for side, hand in self.hands.items():
            hand.observe(sample["hands"][side], not invalid)
        if invalid:
            self._hold(invalid)
            self._lose_tracking()
        elif all(hand.raw["pose"] is None for hand in self.hands.values()):
            self._hold("controller tracking lost")
        events = self._events(sample["events"])
        if not self.grip_only and (self._b_pressed() or any(e["action"] == "hold" for e in events)):
            self._hold("operator hold")
        self.trial.observe(self.hands, self.session)
        commands = [e["action"] for e in events if e["action"] in ("record", "finish_return")]
        if self.trial.path is not None:
            if "record" in commands:
                self.trial.cancel("A trial command interrupted the return.")
            else:
                self.trial.advance(self.hands, dt)
            if self.trial.path is None:
                self.release_controls()
        elif not self.grip_only and ((self.trial.start is not None and commands) or "record" in commands[:-1]):
            # A delayed packet can contain both the start edge and a later end edge.
            self.release_controls()
        else:
            for hand in self.hands.values():
                hand.update(self.held, self.alignment, self.scale, self.gripper_mode, dt, self.grip_only)
        return self._snapshot(now, True, events)

    def release_controls(self):
        for hand in self.hands.values():
            hand.release()
            if hand.usable:
                hand.ready = not self.held and not hand.grip_engaged
                hand.status = "held" if self.held else ("released" if hand.ready else "release grip")

    def finish_trial(self, return_requested):
        if not self.grip_only:
            self.release_controls()
        blocked = self.reason if self.held else ""
        self.trial.finish(self.hands, self.session, return_requested, blocked)

    def restore_initial(self, now=None):
        now = time.monotonic() if now is None else now
        self._expire(now)
        blocked = self.reason if self.held else ""
        if not self.visible or self.head is None:
            blocked = blocked or "Start a tracked input session before restoring the arms."
        self.trial.restore_initial(self.hands, self.initial_joints, blocked)
        return self.snapshot(now)

    def hold(self, reason="operator hold", now=None):
        if not self.grip_only:
            self._hold(reason)
        return self.snapshot(now)

    def disconnect(self, now=None):
        self._hold("input disconnected")
        self._lose_tracking()
        self.visible = False
        return self.snapshot(now)

    def resume(self, now=None):
        now = time.monotonic() if now is None else now
        self._expire(now)
        if self.held and self.head and self.visible and any(h.usable for h in self.hands.values()) and not self._b_pressed():
            self.held, self.reason = False, ""
            for hand in self.hands.values():
                hand.release()
                hand.status = "release grip" if hand.usable else "untracked"
        return self._snapshot(now, False, [])

    def capture(self, now=None):
        """Record tracked, released hands as new reporting origins. Leave tool targets fixed."""
        now = time.monotonic() if now is None else now
        self._expire(now)
        captured = []
        for side, hand in self.hands.items():
            buttons = hand.raw["buttons"]
            if hand.raw["pose"] and not hand.clutched and len(buttons) >= 2 and buttons[1]["value"] <= GRIP_OFF:
                hand.initial = hand.raw["pose"]
                captured.append(side)
        return {**self._snapshot(now, False, []), "captured": captured}

    def _snapshot(self, now, accepted, events):
        return deepcopy({"simulated": True, "session": self.session, "seq": self.seq,
                         "gripper_mode":self.gripper_mode, "trial_return":self.trial.result,
                         "view_valid":self.view_valid,
                         "frame": self.frame, "axes": "+X right, +Y up, -Z forward", "visible": self.visible,
                         "accepted": accepted, "grip_only": self.grip_only, "held": self.held, "reason": self.reason,
                         "head": self.head, "hands": {s: h.snapshot() for s, h in self.hands.items()},
                         "time_ms": self.time_ms, "arrival_s": self.arrival_s,
                         "age_s": None if self.arrival_s is None else max(0, now - self.arrival_s),
                         "rate_hz": 1 / self.period if self.period else None,
                         "clock_offset_estimate_s": self.clock_offset, "source_lag_s": self.source_lag,
                         "ack": self.ack, "events": events})

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        self._expire(now)
        return self._snapshot(now, False, [])
