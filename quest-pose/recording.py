"""Workstation-side camera preview and asynchronous demonstration recording."""

import copy
from bisect import bisect_right
from collections import deque
import datetime as dt
import json
import os
import queue
import threading
import time
import uuid
import zlib
from contextlib import ExitStack
from pathlib import Path

from synthetic_cameras import Animation, CAMERAS, FPS, HEIGHT, WIDTH, render, still_image
from live_cameras import CameraBank


OUTCOMES = {"unreviewed", "success", "failure", "aborted"}
PERIOD_NS = 1_000_000_000 // FPS
ROBOT_IO_DRAIN_S = 3.0


def write_manifest(path, data):
    tmp = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        with tmp.open("x", encoding="utf-8") as stream:
            json.dump(data, stream, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        tmp.replace(path)
    finally:
        tmp.unlink(missing_ok=True)


def write_row(stream, data):
    stream.write(json.dumps(data, separators=(",", ":"), allow_nan=False) + "\n")


def camera_observations(directory, manifest):
    """Join by capture time, after draining the cameras, using only saved frames."""
    tables = {}
    for cam, info in manifest["cameras"].items():
        times, source_indices = [], []
        with (directory / info["timestamps"]).open(encoding="utf-8") as frames:
            for line in frames:
                frame = json.loads(line)
                times.append(frame["host_monotonic_ns"])
                source_indices.append(frame["source_index"])
        tables[cam] = times, source_indices
    for source in ("input", "robot_io"):
        if source not in manifest["streams"]:
            continue
        with (directory / manifest["streams"][source]).open(encoding="utf-8") as stream:
            for index, line in enumerate(stream):
                item = json.loads(line)
                ns = item["host_monotonic_ns"]
                cameras = {}
                for cam, (times, source_indices) in tables.items():
                    i = bisect_right(times, ns) - 1
                    cameras[cam] = None if i < 0 else {
                        "frame_index": i, "source_index": source_indices[i],
                        "host_monotonic_ns": times[i], "age_ns": ns - times[i],
                    }
                yield {"stream": source, "row_index": index, "session": item.get("session"),
                       "seq": item.get("seq"), "host_monotonic_ns": ns, "cameras": cameras}


class _Start:
    def __init__(self, root, start_ns):
        self.utc = dt.datetime.now(dt.timezone.utc)
        self.path = root / (self.utc.strftime("%Y%m%dT%H%M%S") + "-" + uuid.uuid4().hex[:10])
        self.start_ns = start_ns
        self.frames = deque(maxlen=2*FPS*len(CAMERAS))
        self.lost_frames = set()

    def frame(self, cam, meta, data):
        # OpenCV has its own capture ring. Buffer the synthetic preview across a slow open.
        if len(self.frames) == self.frames.maxlen:
            self.lost_frames.add(self.frames[0][0])
        self.frames.append((cam, meta, data))


class _Episode:
    def __init__(self, start, task, model, camera_config=None, physical_gripper=None, physical_arm=None):
        self.path, self.start_ns = start.path, start.start_ns
        self.path.mkdir(parents=True)
        self.resources = ExitStack()
        self.streams, self.animations, self.tables, self.last_source_index = {}, {}, {}, {}
        self.jpegs = {}
        self.manifest = {
            "schema": 1, "status": "interrupted", "simulated": True, "task": task,
            "robot_source":"simulated", "camera_mode":"opencv" if camera_config else "synthetic",
            "outcome":"unreviewed", "started_utc":start.utc.isoformat(), "start_host_ns":self.start_ns,
            "clock":"time.monotonic_ns", "input_clock":"browser performance.now() in ms",
            "camera_clock":"synthetic capture uses the host clock (offset 0 ns)",
            "input_clock_offset_estimate_s":{},
            "input_clock_note":"Per-session minimum of host arrival seconds minus browser sample seconds; includes minimum transport delay.",
            "units":{"position":"metre","orientation":"quaternion XYZW","closure":"normalized"},
            "calibration":"station-identity-tracking-to-sim-base", "model":model,
            "streams":{k:k+".jsonl" for k in ("input","actions","feedback","events")},
            "cameras":{}, "counts":{k:0 for k in ("input","actions","feedback","events")},
            "dropped_input_records":0,
        }
        if physical_gripper:
            self.manifest["physical_gripper"] = {
                "arm": physical_gripper["arm"], "joint": "J8", "mode": physical_gripper["mode"],
                "units": "encoder degrees", "feedback": "feedback.jsonl physical_gripper",
                "clock": "feedback_host_ns is the host monotonic time of the latest encoder read",
            }
        if physical_arm is not None:
            self.manifest.update(robot_source="simulator_and_physical_arm", physical_arm=physical_arm,
                                 missing_robot_io=0)
            self.manifest["streams"]["robot_io"] = "robot_io.jsonl"
            self.manifest["counts"]["robot_io"] = 0
            self.manifest["streams"]["observations"] = "observations.jsonl"
            self.manifest["counts"]["observations"] = 0
            self.manifest["observation_alignment"] = (
                "Each input receipt and hardware cycle references the latest saved frame from each camera "
                "at or before its host_monotonic_ns. Null means no earlier frame in this episode. "
                "row_index is zero-based in the named stream. Robot I/O links to input by session and seq.")
        try:
            for name, file in self.manifest["streams"].items():
                self.streams[name] = self._lines(file)
            if camera_config:
                self.manifest["camera_clock"] = ("Host monotonic timestamp immediately after OpenCV read. "
                    "file source_ns is media time where available, device exposure time is unknown")
            for cam, label in CAMERAS.items():
                self.tables[cam] = self._lines(cam+".frames.jsonl")
                if camera_config:
                    self.jpegs[cam] = self.resources.enter_context((self.path / (cam+".mjpg")).open("xb"))
                    spec = {**camera_config["cameras"][cam], "video":cam+".mjpg","format":"MJPEG",
                            "width":camera_config["width"], "height":camera_config["height"],
                            "nominal_fps":camera_config["fps"]}
                else:
                    self.animations[cam] = Animation(self.path / (cam+".png"))
                    self.resources.callback(self.animations[cam].close)
                    spec = {"label":label, "source":"synthetic", "device_id":None, "video":cam+".png",
                            "format":"APNG", "width":WIDTH,"height":HEIGHT,"nominal_fps":FPS}
                self.manifest["cameras"][cam] = {**spec, "timestamps":cam+".frames.jsonl", "frames":0,"dropped_frames":0}
            # A process exit before finish leaves an inspectable, interrupted episode.
            write_manifest(self.path / "manifest.json", self.manifest)
        except BaseException:
            self.resources.close()
            raise

    def _lines(self, file):
        return self.resources.enter_context((self.path / file).open("x", encoding="utf-8", buffering=1))

    def row(self, name, data):
        write_row(self.streams[name], data)
        self.manifest["counts"][name] += 1

    def frame(self, cam, meta, compressed):
        camera = self.manifest["cameras"][cam]
        index = camera["frames"]
        extra = {}
        if cam in self.jpegs:
            out = self.jpegs[cam]
            offset = out.tell()
            out.write(compressed)
            out.flush()
            extra["crc32"] = zlib.crc32(compressed)
            previous = camera.get("last_host_ns",self.start_ns)
            if meta["host_monotonic_ns"]-previous > max(600_000_000,4e9/camera["nominal_fps"]):
                camera.setdefault("capture_issue","Gap in delivered camera frames")
            camera["last_host_ns"] = meta["host_monotonic_ns"]
        else:
            offset = self.animations[cam].append(compressed)
        write_row(self.tables[cam], {
            "frame_index":index, "source_index":meta["index"],
            "host_monotonic_ns":meta["host_monotonic_ns"], "source_ns":meta.get("source_ns",meta["host_monotonic_ns"]),
            "offset":offset, "size":len(compressed), **extra,
        })
        if cam in self.last_source_index:
            camera["dropped_frames"] += max(0, meta["index"] - self.last_source_index[cam] - 1)
        self.last_source_index[cam] = meta["index"]
        camera["frames"] += 1

    def sample(self, raw, state, ns):
        stamp = {"host_monotonic_ns":ns,"session":raw.get("session"),"seq":raw.get("seq")}
        offset = state.get("clock_offset_estimate_s")
        if offset is not None and stamp["session"] is not None:
            self.manifest["input_clock_offset_estimate_s"][stamp["session"]] = offset
        self.row("input", {**stamp, "sample":raw})
        feedback, ik = {}, {}
        for side, hand in state.get("hands", {}).items():
            if not isinstance(hand, dict):
                continue
            robot = hand.get("robot")
            feedback[side] = {"tcp":robot["tcp"] if robot else hand.get("tool"),
                              "closure":hand.get("closure"), "measured_joints":None}
            if robot:
                feedback[side]["robot"] = robot
                ik[side] = {"joint_angles_rad":robot["joint_angles_rad"], "status":robot["status"]}
        self.row("actions", {**stamp, "state":state, "ik":ik or None, "motor_commands":[]})
        row = {**stamp, "simulated":True, "hands":feedback}
        if state.get("gripper_bench") is not None:
            row["physical_gripper"] = state["gripper_bench"]
        if state.get("physical_arm") is not None:
            row["physical_arm"] = state["physical_arm"]
        self.row("feedback", row)

    def finish(self, status, outcome, dropped, error=None, stop_ns=None):
        ns = time.monotonic_ns() if stop_ns is None else stop_ns
        if "observations" in self.streams:
            # _finish has flushed live camera buffers, including slow JPEG encodes.
            for row in camera_observations(self.path, self.manifest):
                self.row("observations", row)
        self.manifest.update(status=status, outcome=outcome, end_host_ns=ns,
                             duration_s=(ns-self.start_ns)/1e9, dropped_input_records=dropped)
        if error:
            self.manifest["error"] = error
        self.resources.close()
        write_manifest(self.path / "manifest.json", self.manifest)


class Recorder:
    """One worker writes every episode stream. Preview reads return the latest frame."""

    def __init__(self, root, initial_hands=None, camera_config=None, trial_events=None):
        self.root = Path(root)
        self._trial_events = trial_events
        self._state = threading.Lock()
        self._api = threading.RLock()
        self._io = threading.Condition()
        self._io_episode = None
        self._io_closing = False
        self._io_pending = set()
        self._missing_io = 0
        self._queue = queue.Queue(maxsize=544)
        self._sample_slots = threading.BoundedSemaphore(512)
        self._active = None
        self._opening = False
        self._opening_samples = self._opening_io = self._opening_events = self._dropped_events = 0
        self._finalizing = False
        self._stop_requested_ns = None
        self._finish_thread = None
        self._last_name = None
        self._last_elapsed = 0
        self._last_status = None
        self._error = None
        self._selected = "overview"
        self._frames = {}
        self._robots = {side: hand["robot"] for side, hand in (initial_hands or {}).items() if hand.get("robot")}
        self._dropped = 0
        self._closed = False
        self._source_index = 0
        self._origin = time.monotonic_ns()
        self._camera_config = camera_config
        self._bank = CameraBank(camera_config) if camera_config else None
        self._period_ns = int(1e9 / camera_config["fps"]) if camera_config else PERIOD_NS
        self._worker = threading.Thread(target=self._run, name="episode-recorder", daemon=True)
        self._worker.start()

    def status(self):
        with self._state:
            now = time.monotonic_ns()
            active, cam = self._active, self._selected
            if self._bank:
                views = self._bank.statuses()
            else:
                views = {}
                for cid, label in CAMERAS.items():
                    meta = self._frames.get(cid, (None, {}))[1]
                    age = (now-meta["host_monotonic_ns"])/1e6 if meta else None
                    status = "waiting" if age is None else "stalled" if age > 4*PERIOD_NS/1e6 else "live"
                    views[cid] = {"id":cid, "label":label, "source":"synthetic", "status":status,
                                  "error":None,"age_ms":age,"index":meta.get("index")}
            selected = views[cam]
            return {
                "active":active is not None and not self._finalizing, "finalizing":self._finalizing,
                "opening":self._opening,
                "episode":self._last_name,
                "elapsed_s":((self._stop_requested_ns or now)-active.start_ns)/1e9 if active else self._last_elapsed,
                "last_episode_status":self._last_status, "camera_mode":"opencv" if self._bank else "synthetic",
                "selected_camera":cam, "camera_label":selected["label"], "camera_source":selected["source"],
                "camera_frame":selected["index"], "camera_age_ms":selected["age_ms"],
                "camera_stalled":selected["status"] != "live", "camera_error":selected["error"],
                "cameras":list(views.values()),
                "error":self._error, "dropped_input_records":self._dropped,
            }

    def snapshot(self, camera_id=None):
        with self._state:
            if self._bank:
                return self._bank.snapshot(camera_id or self._selected)
            frame = self._frames.get(camera_id or self._selected)
            return (frame[0], frame[1].copy()) if frame else None

    def save_reference(self, state):
        """Persist successful explicit captures for inspection; return the path or None."""
        captured = state.get("captured")
        if not captured:
            return None
        references = {
            side: copy.deepcopy(hand["initial"])
            for side, hand in state["hands"].items() if hand.get("initial") is not None
        }
        reference = {
            "schema": 1,
            "session": state["session"],
            "seq": state["seq"],
            "source_time_ms": state["time_ms"],
            "source_clock": "browser performance.now()",
            "arrival_host_ns": round(state["arrival_s"] * 1e9),
            "capture_host_ns": round((state["arrival_s"] + state["age_s"]) * 1e9),
            "host_clock": "time.monotonic_ns",
            "captured_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
            "frame": state["frame"],
            "axes": state["axes"],
            "units": {"p": "metre", "q": "unit quaternion XYZW"},
            "orientation": "q rotates controller grip-space axes into the tracking frame",
            "captured": list(captured),
            "initial": references,
        }
        self._sync("save_reference", reference)
        return self.root / "initial_poses.json"

    def _sync(self, name, payload):
        with self._api:
            if self._closed:
                raise RuntimeError("Recorder is closed")
            with self._state:
                if self._finalizing and name in ("start", "save_reference"):
                    raise ValueError("The previous episode is still saving")
                if self._opening and name == "save_reference":
                    raise ValueError("The episode files are still opening")
            reply = queue.Queue(maxsize=1)
            self._queue.put((name, payload, reply))
            result = reply.get()
            if isinstance(result, Exception):
                raise result
            return self.status()

    def start(self, task="", host_ns=None, physical_gripper=None, physical_arm=None):
        now = time.monotonic_ns()
        if host_ns is not None and (type(host_ns) is not int or not 0 <= host_ns <= now):
            raise ValueError("Episode start must be a host monotonic timestamp in nanoseconds")
        start = _Start(self.root, now if host_ns is None else host_ns)
        payload = (start, str(task)[:250], physical_gripper, copy.deepcopy(physical_arm))
        if physical_arm is None:
            return self._sync("start", payload)
        with self._api:
            if self._closed:
                raise RuntimeError("Recorder is closed")
            with self._io, self._state:
                if self._finalizing:
                    raise ValueError("The previous episode is still saving")
                if self._active is None:
                    try:
                        self._queue.put_nowait(("open", payload, None))
                    except queue.Full as exc:
                        raise ValueError("The recording queue is full; retry starting the trial") from exc
                    self._active, self._opening = start, True
                    self._last_name, self._last_status, self._error = start.path.name, None, None
                    self._dropped = self._dropped_events = self._opening_samples = self._opening_io = self._opening_events = 0
                    start.capture_ns, start.robots, start.camera = time.monotonic_ns(), self._robots, self._selected
                    self._io_episode, self._io_closing, self._missing_io = start.path.name, False, 0
            return self.status()

    def begin_robot_io(self, episode_id):
        """Reserve an in-flight hardware cycle so Stop includes its actual outcome."""
        with self._io:
            if not episode_id or episode_id != self._io_episode or self._io_closing:
                return None
            token = object()
            self._io_pending.add(token)
            return token

    def robot_io(self, row, token):
        """Publish the motor worker's actual bus timestamps, targets and readbacks."""
        if token is None:
            return
        with self._io:
            if token not in self._io_pending:
                return
            try:
                self._queue.put_nowait(("robot_io", (self._io_episode, copy.deepcopy(row)), None))
                if self._opening:
                    self._opening_io += 1
            except queue.Full:
                self._missing_io += 1
            finally:
                self._io_pending.remove(token)
                self._io.notify_all()

    def _drain_robot_io(self, expected=None):
        # Called on the API thread. Bus callbacks never need _api or the station lock.
        with self._io:
            episode = self._io_episode
            if episode is None or (expected is not None and episode != expected):
                return
            self._io_closing = True
            self._io.wait_for(lambda: not self._io_pending or self._io_episode != episode, ROBOT_IO_DRAIN_S)
            if self._io_episode != episode:
                return
            self._missing_io += len(self._io_pending)
            self._io_pending.clear()
            self._io_episode = None

    def finish(self, outcome="unreviewed"):
        if outcome not in OUTCOMES:
            raise ValueError("Unknown episode outcome")
        with self._api:
            if self._finish_thread:
                self._finish_thread.join()
            self._drain_robot_io()
            return self._sync("finish", outcome)

    def finish_async(self, outcome="unreviewed"):
        """Stop admitting samples immediately; keep serving robot input while saving."""
        if outcome not in OUTCOMES:
            raise ValueError("Unknown episode outcome")
        with self._api:
            if self._closed:
                raise RuntimeError("Recorder is closed")
            with self._io, self._state:
                start_finish = self._active is not None and not self._finalizing
                if start_finish:
                    self._stop_requested_ns = time.monotonic_ns()
                    self._finalizing = self._io_closing = True
                    episode = self._active.path.name
            if start_finish:
                # Reserved I/O must be allowed to enqueue before the writer's finish command.
                def finish_after_io():
                    self._drain_robot_io(episode)
                    handoff = queue.Queue(maxsize=1)
                    self._queue.put(("detach_episode", episode, handoff))
                    ep = handoff.get()
                    if ep is not None:
                        try:
                            self._finish(ep, "saved", outcome)
                        except Exception as exc:
                            with self._state:
                                if self._last_name == episode:
                                    self._error = str(exc)
                self._finish_thread = threading.Thread(target=finish_after_io, name="finish-robot-episode", daemon=True)
                self._finish_thread.start()
            return self.status()

    def close(self):
        with self._api:
            if self._closed:
                return
            try:
                if self._finish_thread:
                    self._finish_thread.join()
                self._drain_robot_io()
                self._sync("close", None)
            finally:
                self._closed = True
                self._worker.join()
                if self._bank:
                    self._bank.close()

    def sample(self, raw, state, host_ns=None):
        ns = time.monotonic_ns() if host_ns is None else host_ns
        with self._api:
            if self._closed:
                return
            robots = {side: hand["robot"] for side, hand in state.get("hands", {}).items()
                      if isinstance(hand, dict) and hand.get("robot")}
            with self._state:
                # Each station snapshot owns its nested data. The camera worker only reads it.
                self._robots = robots
                active = self._active is not None and not self._finalizing
                episode_id = self._active.path.name if active else None
                if active and self._opening:
                    self._opening_samples += 1
            if active:
                # Reserve space for button events even when the sample writer falls behind.
                if self._sample_slots.acquire(blocking=False):
                    try:
                        payload = (episode_id, copy.deepcopy(raw), copy.deepcopy(state), ns)
                        self._queue.put_nowait(("sample", payload, None))
                        return
                    except queue.Full:
                        self._sample_slots.release()
                with self._state:
                    if self._active and self._active.path.name == episode_id:
                        self._dropped += 1

    def event(self, kind, details=None, host_ns=None):
        with self._api:
            with self._state:
                active = self._active is not None and not self._finalizing
                episode_id = self._active.path.name if active else None
                if active and self._opening:
                    self._opening_events += 1
            if active:
                event = {
                    "host_monotonic_ns":time.monotonic_ns() if host_ns is None else host_ns,
                    "kind":str(kind), "details":copy.deepcopy(details) if details is not None else {},
                }
                if self._io_episode is not None:
                    try:
                        self._queue.put_nowait(("event", (episode_id, event), None))
                    except queue.Full:
                        with self._state:
                            self._dropped_events += 1
                else:
                    self._queue.put(("event", (episode_id, event), None))

    def mark(self, label="marker"):
        self.event("marker", {"label":str(label)[:250]})
        return self.status()

    def cycle_camera(self):
        with self._api:
            with self._state:
                ids = list(CAMERAS)
                self._selected = ids[(ids.index(self._selected)+1) % len(ids)]
                selected = self._selected
            self.event("view_requested", {"camera":selected})
            return self.status()

    def _capture(self, episode, skipped=0, capture_ns=None, robots=None):
        if self._bank:
            if episode:
                for cam, info in episode.manifest["cameras"].items():
                    frames, health, last_error = self._bank.since(cam, episode.start_ns, episode.last_source_index.get(cam,-1))
                    for jpeg, meta in frames:
                        if self._stop_requested_ns is None or meta["host_monotonic_ns"] <= self._stop_requested_ns:
                            episode.frame(cam, meta, jpeg)
                    if health["status"] != "live" and self._stop_requested_ns is None:
                        info.setdefault("capture_issue",health["error"] or health["status"])
                    elif episode.start_ns <= last_error[0] <= (self._stop_requested_ns or time.monotonic_ns()):
                        info.setdefault("capture_issue",last_error[1])
            return
        self._source_index += skipped
        if robots is None:
            with self._state:
                robots = self._robots
        for cam, label in CAMERAS.items():
            ns = time.monotonic_ns() if capture_ns is None else capture_ns
            data = render(cam, self._source_index, (ns-self._origin)/1e9, robots)
            meta = {"camera_id":cam,"label":label,"index":self._source_index,"host_monotonic_ns":ns}
            with self._state:
                self._frames[cam] = (still_image(data), meta)
            if episode and (self._stop_requested_ns is None or ns <= self._stop_requested_ns):
                episode.frame(cam, meta, data)
        self._source_index += 1

    def _finish(self, ep, status, outcome, reason=None):
        committed_status = "interrupted"
        try:
            issues = []
            if "robot_io" in ep.streams:
                with self._io:
                    # A writer failure may initiate finalization directly on this worker.
                    self._missing_io += len(self._io_pending)
                    self._io_pending.clear()
                    self._io_episode = None
                    ep.manifest["missing_robot_io"] = self._missing_io
                    self._io.notify_all()
                if ep.manifest["missing_robot_io"]:
                    issues.append(f"Missing {ep.manifest['missing_robot_io']} hardware I/O cycle(s)")
                if self._dropped:
                    issues.append(f"Dropped {self._dropped} controller sample(s)")
                ep.manifest["dropped_event_records"] = self._dropped_events
                if self._dropped_events:
                    issues.append(f"Dropped {self._dropped_events} collection event(s)")
            if self._bank:
                self._capture(ep)
                for cam, info in ep.manifest["cameras"].items():
                    if not info["frames"]:
                        info.setdefault("capture_issue","No camera frames during trial")
                    elif (self._stop_requested_ns is not None and self._stop_requested_ns - info["last_host_ns"]
                          > max(600_000_000, 4e9/info["nominal_fps"])):
                        info.setdefault("capture_issue", "Gap after last delivered camera frame")
                    if info["dropped_frames"]:
                        info.setdefault("capture_issue","Capture buffer overflow")
            for cam, info in ep.manifest["cameras"].items():
                if info.get("capture_issue"):
                    issues.append(f"{cam}: {info['capture_issue']}")
            if status == "saved" and issues:
                status, reason = "incomplete", "; ".join(issues)
                with self._state:
                    self._error = reason
            end_ns = self._stop_requested_ns or time.monotonic_ns()
            ep.row("events", {"host_monotonic_ns":end_ns,
                              "kind":"record_end","details":{"status":status,"outcome":outcome}})
            ep.finish(status, outcome, self._dropped, reason, end_ns)
            committed_status = status
        finally:
            try:
                ep.resources.close()
            finally:
                with self._state:
                    self._active = None
                    self._finalizing = False
                    self._last_status = committed_status
                    self._last_elapsed = ((self._stop_requested_ns or time.monotonic_ns())-ep.start_ns)/1e9
                    self._stop_requested_ns = None
                if self._trial_events is not None:
                    self._trial_events.put(("STOP", ep.path.name, committed_status))

    def _error_during_capture(self, exc, ep):
        with self._state:
            self._error = str(exc)
        if ep is not None and self._active is ep:
            try:
                self._finish(ep, "interrupted", "unreviewed", str(exc))
            except (ValueError, OSError):
                pass  # The interrupted manifest created on open is still present.

    def _open_hardware(self, start, task, model, physical_arm):
        if self._bank:
            # CameraBank continues to acquire and buffer all three physical sources.
            return _Episode(start, task, model, self._camera_config, physical_arm=physical_arm)
        ready = queue.Queue(maxsize=1)
        def open_files():
            try:
                ready.put(_Episode(start, task, model, physical_arm=physical_arm))
            except Exception as exc:
                ready.put(exc)
        threading.Thread(target=open_files, name="open-robot-episode", daemon=True).start()
        self._capture(start, capture_ns=start.capture_ns, robots=start.robots)
        while True:
            try:
                ep = ready.get(timeout=self._period_ns/1e9)
                break
            except queue.Empty:
                self._capture(start)
        if isinstance(ep, Exception):
            raise ep
        try:
            for frame in start.frames:
                ep.frame(*frame)
            start.frames.clear()
            for cam in start.lost_frames:
                ep.manifest["cameras"][cam]["capture_issue"] = "Capture buffer overflow while opening episode"
        except BaseException:
            ep.resources.close()
            raise
        return ep

    def _failed_open(self, start, exc):
        with self._io, self._state:
            if self._active is not start:
                return
            self._dropped = self._opening_samples
            lost_io = self._opening_io + len(self._io_pending) + self._missing_io
            self._error = (f"Could not open episode {start.path.name}: {exc}. "
                           f"{self._dropped} controller sample(s), {self._opening_events} event(s) "
                           f"and {lost_io} robot cycle(s) unsaved.")
            self._last_status, self._active = "start_failed", None
            self._last_elapsed = ((self._stop_requested_ns or time.monotonic_ns()) - start.start_ns)/1e9
            self._opening = self._finalizing = False
            self._stop_requested_ns = self._io_episode = None
            self._io_pending.clear()
            self._io.notify_all()
        if self._trial_events is not None:
            self._trial_events.put(("STOP", start.path.name, "start_failed"))

    def _run(self):
        deadline = self._origin
        # The writer exclusively owns ep until all admitted rows have been
        # processed. A hardware save then owns its files on a separate thread.
        ep = None
        while True:
            now = time.monotonic_ns()
            if now >= deadline:
                skipped = (now-deadline)//self._period_ns
                try:
                    self._capture(ep, skipped)
                except Exception as exc:
                    self._error_during_capture(exc, ep)
                    ep = None
                deadline += (skipped+1)*self._period_ns
            try:
                name, payload, reply = self._queue.get(timeout=max(0, (deadline-time.monotonic_ns())/1e9))
            except queue.Empty:
                continue
            result = None
            try:
                if name == "open" or (name == "start" and self._active is None):
                    with self._state:
                        model = next((robot["model"] for robot in self._robots.values()), "Unspecified simulation")
                    reservation, task, physical_gripper, physical_arm = payload
                    ep = (self._open_hardware(reservation, task, model, physical_arm) if name == "open"
                          else _Episode(reservation, task, model, self._camera_config, physical_gripper))
                    with self._state:
                        if name == "start":
                            self._dropped = 0
                        self._error, self._active, self._opening = None, ep, False
                        self._last_name = ep.path.name
                        self._last_status = None
                    if self._trial_events is not None:
                        self._trial_events.put(("START", ep.path.name, "recording"))
                    ep.row("events", {"host_monotonic_ns":ep.start_ns, "kind":"record_start",
                                      "details":{"camera":reservation.camera if name == "open" else self._selected}})
                    if self._bank or name == "start":
                        self._capture(ep)
                    deadline = time.monotonic_ns() + self._period_ns
                elif name == "sample" and ep and ep.path.name == payload[0]:
                    ep.sample(*payload[1:])
                elif name == "robot_io" and ep and ep.path.name == payload[0]:
                    ep.row("robot_io", payload[1])
                elif name == "event" and ep and ep.path.name == payload[0]:
                    ep.row("events", payload[1])
                elif name == "save_reference":
                    self.root.mkdir(parents=True, exist_ok=True)
                    write_manifest(self.root / "initial_poses.json", payload)
                elif name == "finish" and ep:
                    self._finish(ep, "saved", payload)
                    ep = None
                elif name == "detach_episode" and ep and ep.path.name == payload:
                    result, ep = ep, None
                elif name == "close" and ep:
                    self._finish(ep, "interrupted", "unreviewed")
            except Exception as exc:
                result = exc
                if name == "open" and ep is None:
                    self._failed_open(payload[0], exc)
                elif name != "save_reference":
                    self._error_during_capture(exc, ep)
                    ep = None
            finally:
                if name == "sample":
                    self._sample_slots.release()
            if reply is not None:
                reply.put(result)
            if name == "close":
                return
