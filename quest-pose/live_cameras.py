"""Open one OpenCV reader per camera. Capture clocks are workstation delivery clocks."""

from collections import deque
import json
import math
from pathlib import Path
import threading
import time

from synthetic_cameras import CAMERAS


def require_opencv():
    try:
        import cv2
    except ImportError as exc:
        raise ValueError("OpenCV camera capture requires: python -m pip install opencv-python") from exc
    return cv2


def load_camera_config(path):
    path = Path(path)
    data = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(data, dict) or set(data) - {"width", "height", "fps", "cameras"}:
        raise ValueError("Expected width, height, fps and cameras in the camera config")
    config = {}
    for key, default, low, high in (("width",640,16,1920), ("height",480,16,1080), ("fps",15,1,60)):
        value = data.get(key, default)
        if type(value) is not int or not low <= value <= high:
            raise ValueError(f"Camera {key} must be an integer between {low} and {high}")
        config[key] = value
    inputs = data.get("cameras")
    if not isinstance(inputs, dict) or set(inputs) != set(CAMERAS):
        raise ValueError("Configure overview, left_wrist and right_wrist")
    sources, used = {}, set()
    for cam, default_label in CAMERAS.items():
        item = inputs[cam]
        if not isinstance(item, dict) or set(item) - {"label","device","file"} or ("file" in item) == ("device" in item):
            raise ValueError(f"{cam} requires one device or file")
        label = item.get("label",default_label)
        if not isinstance(label, str) or not 1 <= len(label) <= 48 or not label.isascii() or not label.isprintable() or not label.strip():
            raise ValueError(f"{cam} label must contain 1 to 48 printable ASCII characters")
        kind = "file" if "file" in item else "device"
        source = item[kind]
        if kind == "file":
            if not isinstance(source,str) or not source:
                raise ValueError(f"{cam} file must be a local video filename")
            source = str((path.parent / source).resolve())
            if not Path(source).is_file():
                raise ValueError(f"{cam} video file does not exist: {source}")
        elif type(source) is int:
            if not 0 <= source <= 255:
                raise ValueError(f"{cam} device index must be in 0..255")
        elif not isinstance(source,str) or not Path(source).is_absolute():
            raise ValueError(f"{cam} device must be an integer index or an absolute local device path")
        if (kind,source) in used:
            raise ValueError(f"{cam} duplicates another camera source")
        used.add((kind,source))
        sources[cam] = {"label":label, "source":kind, "device_id":source}
    config["cameras"] = sources
    return config


def _fit(cv2, image, width, height):
    """Keep the camera's aspect ratio when the device ignores a size request."""
    h,w = image.shape[:2]
    if (w,h) == (width,height):
        return image
    ratio = min(width/w, height/h)
    rw, rh = max(1, round(w*ratio)), max(1, round(h*ratio))
    image = cv2.resize(image,(rw,rh),interpolation=cv2.INTER_AREA if ratio < 1 else cv2.INTER_LINEAR)
    l,t = (width-rw)//2, (height-rh)//2
    return cv2.copyMakeBorder(image,t,height-rh-t,l,width-rw-l,cv2.BORDER_CONSTANT)


class CameraBank:
    def __init__(self, config):
        self.cv2 = require_opencv()
        self.config = config
        self.sources = config["cameras"]
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.period_s = 1 / config["fps"]
        self.max_age_ms = max(600, 4 * self.period_s * 1000)
        self._frames = {cam:deque(maxlen=min(60, max(8, config["fps"]*2))) for cam in self.sources}
        self._errors = {cam:None for cam in self.sources}
        self._last_error = {cam:(0,None) for cam in self.sources}
        self._threads = [threading.Thread(target=self._reader, args=(cam, spec),
                         name="capture-"+cam, daemon=True) for cam,spec in self.sources.items()]
        for thread in self._threads:
            thread.start()

    def close(self):
        self._stop.set()
        deadline = time.monotonic()+1.5
        for thread in self._threads:
            thread.join(max(0, deadline-time.monotonic()))

    def _set_error(self, cam, reason):
        with self._lock:
            self._errors[cam] = reason
            self._last_error[cam] = (time.monotonic_ns(),reason)

    def _reader(self, cam, spec):
        cv2 = self.cv2
        width, height = self.config["width"], self.config["height"]
        index = 0
        is_file = spec["source"] == "file"
        while not self._stop.is_set():
            cap = None
            try:
                cap = cv2.VideoCapture(spec["device_id"])
                if not cap.isOpened():
                    raise OSError("Camera could not be opened")
                if not is_file:
                    for prop, value in ((cv2.CAP_PROP_FRAME_WIDTH,width), (cv2.CAP_PROP_FRAME_HEIGHT,height),
                                        (cv2.CAP_PROP_FPS, self.config["fps"]), (cv2.CAP_PROP_BUFFERSIZE,1)):
                        cap.set(prop,value)
                while not self._stop.is_set():
                    began = time.monotonic()
                    ok, image = cap.read()
                    stamp = time.monotonic_ns()
                    if not ok or image is None:
                        raise OSError("End of video file" if is_file else "Camera failed to deliver a frame")
                    if is_file:
                        ms = cap.get(cv2.CAP_PROP_POS_MSEC)
                    else:
                        ms = None
                    ok, jpg = cv2.imencode(".jpg", _fit(cv2, image, width, height), [cv2.IMWRITE_JPEG_QUALITY,85])
                    if not ok:
                        raise OSError("Camera JPEG encoding failed")
                    meta = {"camera_id":cam, "label":spec["label"], "index":index, "mime":"image/jpeg",
                            "host_monotonic_ns":stamp, "source_ns": (
                                round(ms*1e6) if ms is not None and math.isfinite(ms) and ms >= 0 else None)}
                    frame = (jpg.tobytes(), meta)
                    with self._lock:
                        self._frames[cam].append(frame)
                        self._errors[cam] = None
                    index += 1
                    self._stop.wait(max(0, began+self.period_s-time.monotonic()))
            except (cv2.error, OSError, ValueError) as exc:
                self._set_error(cam, str(exc))
            finally:
                if cap is not None:
                    cap.release()
            if is_file:
                return
            self._stop.wait(1.0)

    def _status(self, cam, now):
        frames = self._frames[cam]
        meta = frames[-1][1] if frames else {}
        age = (now-meta["host_monotonic_ns"])/1e6 if meta else None
        error = self._errors[cam]
        if error:
            status = "unavailable"
        elif age is None:
            status = "waiting"
        elif age > self.max_age_ms:
            status = "stalled"
        else:
            status = "live"
        return {"id":cam, **self.sources[cam], "status":status, "error":error,
                "age_ms":age,"index":meta.get("index")}

    def statuses(self):
        with self._lock:
            now = time.monotonic_ns()
            return {cam:self._status(cam,now) for cam in self.sources}

    def snapshot(self, cam):
        with self._lock:
            if cam not in self.sources or self._status(cam,time.monotonic_ns())["status"] != "live":
                return None
            jpeg, meta = self._frames[cam][-1]
            return jpeg, meta.copy()

    def since(self, cam, start_ns, after_index):
        with self._lock:
            frames = [frame for frame in self._frames[cam] if frame[1]["host_monotonic_ns"] >= start_ns
                      and frame[1]["index"] > after_index]
            return frames, self._status(cam,time.monotonic_ns()), self._last_error[cam]
