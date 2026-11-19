"""Workstation-side camera preview and asynchronous demonstration recording."""

import copy
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


class _Episode:
    def __init__(self, root, task, start_ns, model, camera_config=None):
        utc = dt.datetime.now(dt.timezone.utc)
        self.path = root / (utc.strftime("%Y%m%dT%H%M%S") + "-" + uuid.uuid4().hex[:10])
        self.path.mkdir(parents=True)
        self.start_ns = start_ns
        self.resources = ExitStack()
        self.streams, self.animations, self.tables, self.last_source_index = {}, {}, {}, {}
        self.jpegs = {}
        self.manifest = {
            "schema": 1, "status": "interrupted", "simulated": True, "task": task,
            "robot_source":"simulated", "camera_mode":"opencv" if camera_config else "synthetic",
            "outcome":"unreviewed", "started_utc":utc.isoformat(), "start_host_ns":self.start_ns,
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
        self.row("feedback", {**stamp, "simulated":True, "hands":feedback})

    def finish(self, status, outcome, dropped, error=None):
        ns = time.monotonic_ns()
        self.manifest.update(status=status, outcome=outcome, end_host_ns=ns,
                             duration_s=(ns-self.start_ns)/1e9, dropped_input_records=dropped)
        if error:
            self.manifest["error"] = error
        self.resources.close()
        write_manifest(self.path / "manifest.json", self.manifest)


class Recorder:
    """One worker writes every episode stream. Preview reads return the latest frame."""

    def __init__(self, root, initial_hands=None, camera_config=None):
        self.root = Path(root)
        self._state = threading.Lock()
        self._api = threading.RLock()
        self._queue = queue.Queue(maxsize=544)
        self._sample_slots = threading.BoundedSemaphore(512)
        self._active = None
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
                "active":active is not None, "episode":self._last_name,
                "elapsed_s":(now-active.start_ns)/1e9 if active else self._last_elapsed,
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
            reply = queue.Queue(maxsize=1)
            self._queue.put((name, payload, reply))
            result = reply.get()
            if isinstance(result, Exception):
                raise result
            return self.status()

    def start(self, task="", host_ns=None):
        now = time.monotonic_ns()
        if host_ns is not None and (type(host_ns) is not int or not 0 <= host_ns <= now):
            raise ValueError("Episode start must be a host monotonic timestamp in nanoseconds")
        return self._sync("start", (str(task)[:250], now if host_ns is None else host_ns))

    def finish(self, outcome="unreviewed"):
        if outcome not in OUTCOMES:
            raise ValueError("Unknown episode outcome")
        return self._sync("finish", outcome)

    def close(self):
        with self._api:
            if self._closed:
                return
            try:
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
                active = self._active is not None
            if active:
                # Reserve space for button events even when the sample writer falls behind.
                if self._sample_slots.acquire(blocking=False):
                    try:
                        payload = (copy.deepcopy(raw), copy.deepcopy(state), ns)
                        self._queue.put_nowait(("sample", payload, None))
                        return
                    except queue.Full:
                        self._sample_slots.release()
                with self._state:
                    self._dropped += 1

    def event(self, kind, details=None, host_ns=None):
        with self._api:
            with self._state:
                active = self._active is not None
            if active:
                event = {
                    "host_monotonic_ns":time.monotonic_ns() if host_ns is None else host_ns,
                    "kind":str(kind), "details":copy.deepcopy(details) if details is not None else {},
                }
                self._queue.put(("event", event, None))

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

    def _capture(self, episode, skipped=0):
        if self._bank:
            if episode:
                for cam, info in episode.manifest["cameras"].items():
                    frames, health, last_error = self._bank.since(cam, episode.start_ns, episode.last_source_index.get(cam,-1))
                    for jpeg, meta in frames:
                        episode.frame(cam, meta, jpeg)
                    if health["status"] != "live":
                        info.setdefault("capture_issue",health["error"] or health["status"])
                    elif last_error[0] >= episode.start_ns:
                        info.setdefault("capture_issue",last_error[1])
            return
        self._source_index += skipped
        with self._state:
            robots = self._robots
        for cam, label in CAMERAS.items():
            ns = time.monotonic_ns()
            data = render(cam, self._source_index, (ns-self._origin)/1e9, robots)
            meta = {"camera_id":cam,"label":label,"index":self._source_index,"host_monotonic_ns":ns}
            with self._state:
                self._frames[cam] = (still_image(data), meta)
            if episode:
                episode.frame(cam, meta, data)
        self._source_index += 1

    def _finish(self, ep, status, outcome, reason=None):
        committed_status = "interrupted"
        try:
            if self._bank:
                self._capture(ep)
                issues = []
                for cam, info in ep.manifest["cameras"].items():
                    if not info["frames"]:
                        info.setdefault("capture_issue","No camera frames during trial")
                    if info["dropped_frames"]:
                        info.setdefault("capture_issue","Capture buffer overflow")
                    if info.get("capture_issue"):
                        issues.append(f"{cam}: {info['capture_issue']}")
                if status == "saved" and issues:
                    status, reason = "incomplete", "; ".join(issues)
                    with self._state:
                        self._error = reason
            ep.row("events", {"host_monotonic_ns":time.monotonic_ns(),
                              "kind":"record_end","details":{"status":status,"outcome":outcome}})
            ep.finish(status, outcome, self._dropped, reason)
            committed_status = status
        finally:
            try:
                ep.resources.close()
            finally:
                with self._state:
                    self._active = None
                    self._last_status = committed_status
                    self._last_elapsed = (time.monotonic_ns()-ep.start_ns)/1e9

    def _error_during_capture(self, exc):
        with self._state:
            self._error = str(exc)
        if self._active:
            try:
                self._finish(self._active, "interrupted", "unreviewed", str(exc))
            except (ValueError, OSError):
                pass  # The interrupted manifest created on open is still present.

    def _run(self):
        deadline = self._origin
        while True:
            now = time.monotonic_ns()
            if now >= deadline:
                skipped = (now-deadline)//self._period_ns
                try:
                    self._capture(self._active, skipped)
                except Exception as exc:
                    self._error_during_capture(exc)
                deadline += (skipped+1)*self._period_ns
            try:
                name, payload, reply = self._queue.get(timeout=max(0, (deadline-time.monotonic_ns())/1e9))
            except queue.Empty:
                continue
            result = None
            try:
                ep = self._active
                if name == "start" and not ep:
                    with self._state:
                        model = next((robot["model"] for robot in self._robots.values()), "Unspecified simulation")
                    ep = _Episode(self.root, *payload, model, self._camera_config)
                    with self._state:
                        self._error, self._active, self._dropped = None, ep, 0
                        self._last_name = ep.path.name
                        self._last_status = None
                    ep.row("events", {"host_monotonic_ns":ep.start_ns, "kind":"record_start",
                                      "details":{"camera":self._selected}})
                    self._capture(ep)
                    deadline = time.monotonic_ns() + self._period_ns
                elif name == "sample" and ep:
                    ep.sample(*payload)
                elif name == "event" and ep:
                    ep.row("events", payload)
                elif name == "save_reference":
                    self.root.mkdir(parents=True, exist_ok=True)
                    write_manifest(self.root / "initial_poses.json", payload)
                elif name == "finish" and ep:
                    self._finish(ep, "saved", payload)
                elif name == "close" and ep:
                    self._finish(ep, "interrupted", "unreviewed")
            except Exception as exc:
                result = exc
                if name != "save_reference":
                    self._error_during_capture(exc)
            finally:
                if name == "sample":
                    self._sample_slots.release()
            if reply is not None:
                reply.put(result)
            if name == "close":
                return
