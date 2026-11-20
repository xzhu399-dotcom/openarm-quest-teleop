"""Serialize the input owner, simulated targets and episode boundaries."""

from collections import deque
from threading import Event, RLock, Thread
import time

from recording import OUTCOMES, Recorder
from teleop import Teleop
from gripper_bench import MAX_INPUT_AGE, _tracked_touch


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
        self.safety_epoch = 0
        self.trial_epoch = 0
        self.gripper_bench = None
        self.physical_arm = None
        self.hardware_input = None
        self.hardware_return = None
        self.hardware_return_notice = ""
        self._return_id = 0
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

    def _recording_status(self):
        status = self.recorder.status()
        if (self.physical_arm is not None and not status["active"] and not status.get("finalizing")
                and status["last_episode_status"] in ("start_failed", "interrupted", "incomplete")):
            # Hardware input kept running while the writer opened or closed the files.
            # Retire a failed episode so the next A tap starts a fresh collection.
            self.engine.trial.start = None
        return status

    def _camera_ready(self, incoming=False):
        status = self._recording_status()
        if (self.owner or incoming) and status.get("camera_mode") == "opencv" and status["camera_stalled"]:
            if not self.engine.held and not self.engine.grip_only:
                self.engine.hold("selected camera feed unavailable")
            return False
        return True

    def _hold_transition(self, state):
        hold = (state["held"], state["reason"] if state["held"] else None)
        if hold != self.last_hold:
            self.last_hold = hold
            if hold[0]:
                self.safety_epoch += 1
                self.hardware_return = None
            self.recorder.event("hold" if hold[0] else "resume", {
                "reason": hold[1], "session": state["session"], "seq": state["seq"],
                "hands": state["hands"],
            })

    def _response(self, state):
        self._hold_transition(state)
        if self.hardware_return:
            hands = state["hands"]
            if (state["session"] != self.hardware_return["session"] or not state["visible"] or not state["head"]
                    or any(hands[s]["source"] != src or not _tracked_touch(hands[s])
                           for s, src in self.hardware_return["sources"].items())
                    or any(hand.grip_engaged or hand.raw["grip_epoch"] != self.hardware_return["grip_epochs"][s]
                           for s, hand in self.engine.hands.items())):
                self.hardware_return = None
        status = self._recording_status()
        age = status["camera_age_ms"]
        response = dict(state, owner=self.owner, simulated=True, safety_epoch=self.safety_epoch,
                    physical_arm=self.physical_arm, hardware_return=self.hardware_return,
                    hardware_return_notice=self.hardware_return_notice,
                    trial_epoch=self.trial_epoch, gripper_bench=self.gripper_bench, recording=status, camera={
            "id": status["selected_camera"], "label": status["camera_label"],
            "index": status["camera_frame"], "age_s": None if age is None else age / 1000,
        })
        if self.physical_arm is not None:
            # Replace published snapshots atomically. The CAN worker can check the latest input
            # during a bus cycle even while finish() is waiting for that cycle's recording.
            self.hardware_input = response
        return response

    def state(self):
        with self.lock:
            return self._response(self.engine.snapshot())

    def sample(self, raw):
        session = raw.get("session")
        with self.lock:
            hardware = self.gripper_bench is not None or self.physical_arm is not None
            if hardware and raw.get("frame") == "mouse-local":
                raise Conflict("Physical hardware requires Quest headset input.")
            if session in self.retired:
                raise Conflict("This input session ended. Start a new input session.")
            if self.owner and session != self.owner and time.monotonic() - self.last_arrival < 2:
                raise Conflict("Another input source is active. Stop it before switching.")
            self._camera_ready(incoming=True)
            try:
                state = self.engine.apply(raw, quest_simulation=not hardware)
            except ValueError:
                if self.physical_arm is not None and session == self.owner:
                    self.hardware_input = None
                    self.hardware_return = None
                raise
            if state["accepted"]:
                if self.owner and self.owner != session:
                    self.retired.append(self.owner)
                self.owner = session
                self.last_arrival = state["arrival_s"]
                self._hold_transition(state)
                state = self._record_sample(raw, state)
            return self._response(state)

    def _start_trial(self, task="", stamp=None):
        if self._recording_status().get("finalizing"):
            return
        self.trial_epoch += 1
        self.hardware_return = None
        if self.physical_arm is not None:
            self._response(self.engine.snapshot())
        try:
            self.recorder.start(task, host_ns=stamp, physical_gripper=self.gripper_bench,
                                physical_arm=self.physical_arm)
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
        self.trial_epoch += 1
        self.hardware_return = None
        if self.physical_arm is not None:
            self._response(self.engine.snapshot())
        self._camera_ready()
        self.engine.snapshot()  # Check freshness for workstation-originated endings, too.
        return_requested = action == "finish_return" and not self.engine.grip_only and self.physical_arm is None
        self.engine.finish_trial(return_requested=return_requested)
        fresh = self.engine.snapshot()
        fresh.update(accepted=state["accepted"], events=state["events"])
        stamp = round(state["arrival_s"] * 1e9) if raw is not None else time.monotonic_ns()
        self.recorder.event("trial_finish", {"choice":"return" if return_requested else "stay",
                            "jaw_policy":"side_grip" if self.engine.grip_only else "hold",
                            **fresh["trial_return"]}, host_ns=stamp)
        try:
            if raw is not None:
                self._save_sample(raw, fresh, stamp)
            # Keep the incoming Quest/stop samples flowing while the motor episode
            # flushes its bus transactions, video indices and manifest.
            saved = (self.recorder.finish_async(outcome) if self.physical_arm is not None
                     else self.recorder.finish(outcome))
        except (OSError, ValueError) as error:
            self.engine.hold("trial save failed")
            self.engine.trial.result = {"status":"save_failed", "reason":str(error)}
        else:
            if saved.get("finalizing"):
                return fresh
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
                if self._recording_status()["active"] or self.engine.trial.start is not None:
                    # Log markers and view changes observed in this final sample before closing.
                    pending_end = action
                elif action == "record":
                    self._start_trial(stamp=stamp)
            elif action == "view":
                self.recorder.cycle_camera()
            elif action == "mark":
                self.recorder.mark()
            elif ((action == "restore_initial" and self.physical_arm is None and self.gripper_bench is None)
                  or (action == "restore_robot_initial" and self.physical_arm is not None)):
                if pending_end is not None:
                    state = self._end_trial(pending_end, state, raw)
                    pending_end = None
                if action == "restore_robot_initial":
                    self._restore_robot(event, stamp)
                else:
                    self._restore_initial(stamp)
        if pending_end is not None:
            return self._end_trial(pending_end, state, raw)
        if state["trial_return"] != self.engine.trial.result:
            state = dict(self.engine.snapshot(), accepted=state["accepted"], events=state["events"])
        self._save_sample(raw, state, stamp)
        return state

    def _save_sample(self, raw, state, stamp):
        if self.gripper_bench is not None:
            state = dict(state, gripper_bench=self.gripper_bench)
        if self.physical_arm is not None:
            state = dict(state, physical_arm=self.physical_arm)
        self.recorder.sample(raw, state, stamp)

    def _restore_initial(self, stamp=None):
        if self.gripper_bench is not None or self.physical_arm is not None:
            raise Conflict("Restoring the initial arm position is available only in simulation.")
        state = self.engine.restore_initial()
        self.recorder.event("restore_initial", state["trial_return"], host_ns=stamp)
        return state

    def _restore_robot(self, event=None, stamp=None):
        robot = self.physical_arm
        if not robot or robot["mode"] != "drive":
            if event is None:
                raise Conflict("Physical return requires robot-drive mode on the robot computer.")
            return self.engine.snapshot()
        state = self.engine.snapshot()
        required = set(robot["arms"]) | {"right"}
        if event is not None:
            required.add("left")
        hands = state["hands"]
        reason = ""
        if (not self.owner or self.owner != state["session"] or state["held"] or not state["head"]
                or not state["visible"] or not state["view_valid"]
                or state["age_s"] + state["source_lag_s"] > MAX_INPUT_AGE
                or not all(_tracked_touch(hands[side]) for side in required)):
            reason = "Physical return needs fresh, visible Quest tracking and a clear hold."
        elif any(hand.grip_engaged for hand in self.engine.hands.values()) or self.engine._b_pressed():
            reason = "Release both side grips and the stop button before returning the robot."
        elif event is not None and not 0 <= state["time_ms"] - event["time_ms"] <= MAX_INPUT_AGE * 1000:
            reason = "Return gesture expired. Press A and X together again."
        elif not any(arm.get("enabled") is True for arm in robot["arms"].values()):
            reason = "Engage a configured arm with its side grip before requesting physical return."
        if reason:
            self.hardware_return_notice = reason
            self.recorder.event("hardware_return_skipped", {"reason": reason}, host_ns=stamp)
            if event is None:
                raise Conflict(reason)
            return state
        if self.hardware_return:
            # A second tap may arrive before the workers publish their first
            # returning status, or after the faster preview has already finished.
            returning = any(arm.get("phase") in ("returning", "returned") for arm in robot["arms"].values())
            if (returning or state["trial_return"]["status"] == "returning"
                    or time.monotonic() - self.hardware_return["requested_s"] <= MAX_INPUT_AGE):
                return state
        state = self.engine.restore_initial()
        if state["trial_return"]["status"] in ("returning", "completed"):
            self._return_id += 1
            self.hardware_return = {
                "id": self._return_id, "session": self.owner, "seq": state["seq"],
                "requested_s": time.monotonic(), "trial_epoch": self.trial_epoch,
                "grip_epochs": {side: hand["grip_epoch"] for side, hand in hands.items()},
                "sources": {side: hands[side]["source"] for side in required},
            }
            self.hardware_return_notice = ""
            self.recorder.event("hardware_return_requested", self.hardware_return, host_ns=stamp)
        return state

    def publish_robot_io(self, row, token):
        # Recorder.finish may hold the station lock while draining reserved bus cycles.
        self.recorder.robot_io(row, token)

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
            elif action == "restore_initial":
                state = self._restore_initial()
            elif action == "restore_robot_initial":
                if not local:
                    raise PermissionError("Use the robot computer for this button, or press A and X on the Quest.")
                state = self._restore_robot()
            elif action == "resume":
                state = self.engine.resume() if self._camera_ready() else self.engine.snapshot()
            elif action == "hold":
                state = self.engine.hold("operator hold")
            elif action == "release":
                if data.get("session") and data["session"] != self.owner:
                    raise Conflict("This input session is no longer active.")
                state = self.engine.disconnect()
                self.hardware_return = None
                if self.owner:
                    self.retired.append(self.owner)
                self.owner = None
            else:
                if action in ("record", "finish_return"):
                    status = self._recording_status()
                    if status.get("finalizing"):
                        raise Conflict("The previous trial is still saving.")
                    if status["active"] or self.engine.trial.start is not None:
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
