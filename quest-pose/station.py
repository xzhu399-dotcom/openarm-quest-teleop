"""Serialize the input owner, simulated targets and episode boundaries."""

from collections import deque
from threading import Event, RLock, Thread
import time

from recording import OUTCOMES, Recorder
from teleop import Teleop


class Conflict(ValueError):
    """An operator action cannot be applied in the current state."""


class Station:
    def __init__(self, episodes, gripper_mode="trigger", camera_config=None, trial_events=None):
        self.engine = Teleop(gripper_mode=gripper_mode)
        self.recorder = Recorder(episodes, self.engine.snapshot()["hands"],
                                 camera_config=camera_config, trial_events=trial_events)
        self.lock = RLock()
        self.owner = None
        self.last_arrival = 0
        self.retired = deque(maxlen=256)
        self.last_hold = (False, None)
        self.stop = Event()
        self.watchdog = Thread(target=self._watch, name="input-watchdog", daemon=True)
        self.watchdog.start()

    def close(self):
        self.stop.set()
        self.watchdog.join()
        with self.lock:
            self.recorder.close()

    def _watch(self):
        while not self.stop.wait(0.05):
            with self.lock:
                self._camera_ready()
                self._hold_transition(self.engine.snapshot())

    def _camera_ready(self, incoming=False):
        status = self.recorder.status()
        if (self.owner or incoming) and status.get("camera_mode") == "opencv" and status["camera_stalled"]:
            if not self.engine.held:
                self.engine.hold("selected camera feed unavailable")
            return False
        return True

    def _hold_transition(self, state):
        hold = (state["held"], state["reason"] if state["held"] else None)
        if hold != self.last_hold:
            self.last_hold = hold
            self.recorder.event("hold" if hold[0] else "resume", {
                "reason": hold[1], "session": state["session"], "seq": state["seq"],
                "hands": state["hands"],
            })

    def _response(self, state):
        self._hold_transition(state)
        status = self.recorder.status()
        age = status["camera_age_ms"]
        return dict(state, owner=self.owner, simulated=True, recording=status, camera={
            "id": status["selected_camera"], "label": status["camera_label"],
            "index": status["camera_frame"], "age_s": None if age is None else age / 1000,
        })

    def state(self):
        with self.lock:
            return self._response(self.engine.snapshot())

    def sample(self, raw):
        session = raw.get("session")
        with self.lock:
            if session in self.retired:
                raise Conflict("This input session ended. Start a new input session.")
            if self.owner and session != self.owner and time.monotonic() - self.last_arrival < 2:
                raise Conflict("Another input source is active. Stop it before switching.")
            self._camera_ready(incoming=True)
            state = self.engine.apply(raw)
            if state["accepted"]:
                if self.owner and self.owner != session:
                    self.retired.append(self.owner)
                self.owner = session
                self.last_arrival = state["arrival_s"]
                self._hold_transition(state)
                state = self._record_sample(raw, state)
            return self._response(state)

    def _start_trial(self, task="", stamp=None):
        try:
            self.recorder.start(task, host_ns=stamp)
        except (OSError, ValueError) as error:
            self.engine.hold("trial start failed")
            self.engine.trial.result = {"status":"start_failed", "reason":str(error)}
            return
        owner = self.owner if self.engine.head is not None else None
        reference = self.engine.trial.begin(self.engine.hands, owner)
        self.recorder.event("trial_start_pose", {"session": owner,
                            "gripper_mode": self.engine.gripper_mode, **reference}, host_ns=stamp)

    def _end_trial(self, action, state, raw=None, outcome="unreviewed"):
        if outcome not in OUTCOMES:
            raise ValueError("Unknown episode outcome")
        self._camera_ready()
        self.engine.snapshot()  # Check freshness for workstation-originated endings, too.
        self.engine.finish_trial(return_requested=action == "finish_return")
        fresh = self.engine.snapshot()
        fresh.update(accepted=state["accepted"], events=state["events"])
        stamp = round(state["arrival_s"] * 1e9) if raw is not None else time.monotonic_ns()
        self.recorder.event("trial_finish", {"choice":"return" if action == "finish_return" else "stay",
                            "jaw_policy":"hold", **fresh["trial_return"]}, host_ns=stamp)
        try:
            if raw is not None:
                self.recorder.sample(raw, fresh, stamp)
            saved = self.recorder.finish(outcome)
        except (OSError, ValueError) as error:
            self.engine.hold("trial save failed")
            self.engine.trial.result = {"status":"save_failed", "reason":str(error)}
        else:
            if saved.get("last_episode_status", "saved") == "saved" and not saved.get("error"):
                return fresh
            self.engine.hold("trial recording incomplete")
            self.engine.trial.result = {"status":"incomplete", "reason":saved.get("error") or "Incomplete trial."}
        fresh = self.engine.snapshot()
        fresh.update(accepted=state["accepted"], events=state["events"])
        return fresh

    def _record_sample(self, raw, state):
        stamp = round(state["arrival_s"] * 1e9)
        pending_end = None
        for event in state["events"]:
            action = event["action"]
            if action in ("record", "finish_return"):
                if pending_end is not None:
                    state = self._end_trial(pending_end, state, raw)
                    pending_end = None
                if self.recorder.status()["active"] or self.engine.trial.start is not None:
                    # Log markers and view changes observed in this final sample before closing.
                    pending_end = action
                elif action == "record":
                    self._start_trial(stamp=stamp)
            elif action == "view":
                self.recorder.cycle_camera()
            elif action == "mark":
                self.recorder.mark()
        if pending_end is not None:
            return self._end_trial(pending_end, state, raw)
        if state["trial_return"] != self.engine.trial.result:
            state = dict(self.engine.snapshot(), accepted=state["accepted"], events=state["events"])
        self.recorder.sample(raw, state, stamp)
        return state

    def control(self, data, local):
        action = data.get("action")
        with self.lock:
            if not local and action != "hold" and (
                action == "resume" or not self.owner or data.get("session") != self.owner
            ):
                raise PermissionError("Use the workstation for this action.")
            if action == "capture":
                state = self.engine.capture()
                if state["captured"]:
                    self.recorder.save_reference(state)
                    self.recorder.event("capture", {"sides": state["captured"], "hands": state["hands"]})
            elif action == "resume":
                state = self.engine.resume() if self._camera_ready() else self.engine.snapshot()
            elif action == "hold":
                state = self.engine.hold("operator hold")
            elif action == "release":
                if data.get("session") and data["session"] != self.owner:
                    raise Conflict("This input session is no longer active.")
                state = self.engine.disconnect()
                if self.owner:
                    self.retired.append(self.owner)
                self.owner = None
            else:
                if action in ("record", "finish_return"):
                    if self.recorder.status()["active"] or self.engine.trial.start is not None:
                        self._end_trial(action, self.engine.snapshot(), outcome=data.get("outcome", "unreviewed"))
                    elif action == "record":
                        self._start_trial(data.get("task", ""))
                elif action == "view":
                    self.recorder.cycle_camera()
                elif action == "mark":
                    self.recorder.mark()
                elif action == "displayed":
                    if data.get("session") != self.owner:
                        raise Conflict("Display acknowledgement belongs to a previous session.")
                    camera = self.recorder.snapshot()
                    if (camera and data.get("camera_id") == camera[1]["camera_id"]
                            and type(data.get("index")) is int and 0 <= data["index"] <= camera[1]["index"]):
                        self.recorder.event("view_displayed", {
                            "camera_id": data["camera_id"], "index": data["index"],
                        })
                else:
                    raise ValueError("Unknown control action.")
                state = self.engine.snapshot()
            return self._response(state)
