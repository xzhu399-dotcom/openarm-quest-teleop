"""Read-only terminal view of the selected workstation camera and Quest input."""

from contextlib import contextmanager
import os
import shutil
import sys
import textwrap
import threading
import time


def camera_rows(encoded, columns, rows, half_block=True):
    """Fit a color PNG/JPEG in a terminal viewport, preserving the image aspect."""
    import cv2
    import numpy as np

    try:
        frame = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
    except cv2.error as exc:
        raise ValueError("Camera frame could not be decoded") from exc
    if frame is None:
        raise ValueError("Camera frame could not be decoded")
    height, width = frame.shape[:2]
    scale = min(columns / width, 2 * rows / height)
    w = max(1, round(scale * width))
    h = max(2, 2 * int(scale * height / 2))
    pixels = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA).tolist()
    lines = []
    for y in range(0, h, 2):
        run = []
        previous = None
        for upper, lower in zip(pixels[y], pixels[y+1]):
            pair = (upper, lower) if half_block else (lower, lower)
            if pair != previous:
                b,g,r = pair[0]
                bb,bg,br = pair[1]
                run.append(f"\x1b[38;2;{r};{g};{b};48;2;{br};{bg};{bb}m")
                previous = pair
            run.append("\u2580" if half_block else " ")
        lines.append("".join(run) + "\x1b[0m")
    return lines


def hand_report(side, state):
    hand = state["hands"][side]
    if not hand.get("pose"):
        return f"{side.title()} handle: untracked"
    xyz = " ".join(f"{x:+.3f}" for x in hand["pose"]["p"])
    jaw = hand["robot"]["jaw_opening_m"] * 1000
    return f"{side.title()} XYZ {xyz} m | jaw {jaw:.1f} mm | {hand['status']}"


class TerminalMonitor:
    def __init__(self, station, url, stream):
        self.station = station
        self.url = url
        self.stream = stream
        self.stop = threading.Event()
        self.last_frame = None
        self.last_rows = []
        try:
            "\u2580".encode(stream.encoding or "utf-8")
            self.half_block = True
        except UnicodeEncodeError:
            self.half_block = False

    def lines(self, columns, rows):
        columns = max(1, columns-1)
        state = self.station.state()
        rec = state["recording"]
        cam = rec["selected_camera"]
        status = state.get("trial_return", {}).get("status", "idle")
        result = rec.get("last_episode_status") or "none"
        trial = f"REC {rec['elapsed_s']:.1f} s" if rec["active"] else f"Idle | last trial: {result} | return: {status}"
        held = "HOLD " + state["reason"] if state["held"] else "ready"
        info = [f"{trial} | {held} | Ctrl+C to stop",
                hand_report("left", state), hand_report("right", state)]
        if rec.get("error"):
            info.append(str(rec["error"]))
        top = textwrap.wrap(self.url, width=columns, break_on_hyphens=False)
        top += [line[:columns] for line in info]
        frame = self.station.recorder.snapshot(cam)
        if frame is None or rec.get("camera_stalled"):
            reason = rec.get("camera_error") or "waiting for fresh imagery"
            self.last_frame, self.last_rows = None, []
            return top + [f"{rec['camera_label']} | UNAVAILABLE | {reason}"[:columns]]
        encoded, meta = frame
        age_ms = (time.monotonic_ns()-meta["host_monotonic_ns"])/1e6
        info = f"{meta['label']} | frame {meta['index']} | {age_ms:.0f} ms since host capture"
        top.append(info[:columns])
        available = max(1, min(36, rows-len(top)-1))
        key = (cam, meta["index"], columns, available)
        if key != self.last_frame:
            try:
                self.last_rows = camera_rows(encoded, min(columns, 112), available, self.half_block)
            except ValueError:
                self.last_rows = ["UNAVAILABLE | Camera frame could not be decoded"[:columns]]
            self.last_frame = key
        return top + self.last_rows

    def run(self):
        while not self.stop.is_set():
            try:
                cols, rows = shutil.get_terminal_size()
                lines = self.lines(cols, rows)
                self.stream.write("\x1b[H" + "\x1b[K\r\n".join(lines) + "\x1b[K\x1b[J")
                self.stream.flush()
            except (BrokenPipeError, OSError):
                return
            self.stop.wait(.15)


@contextmanager
def terminal_view(station, url, stream=None):
    stream = sys.stdout if stream is None else stream
    if not stream.isatty():
        raise ValueError("--terminal-view needs an interactive terminal. Open a terminal window and rerun.")
    try:
        import cv2  # Validate the optional decoder before serving.
    except ImportError as exc:
        raise ValueError("--terminal-view requires python -m pip install opencv-python") from exc
    if os.name == "nt":
        import ctypes
        import msvcrt
        kernel = ctypes.windll.kernel32
        handle = ctypes.c_void_p(msvcrt.get_osfhandle(stream.fileno()))
        original = ctypes.c_ulong()
        if not kernel.GetConsoleMode(handle, ctypes.byref(original)) or not kernel.SetConsoleMode(handle, original.value | 4):
            raise ValueError("Use Windows Terminal for --terminal-view (ANSI color output required).")
    monitor = TerminalMonitor(station, url, stream)
    worker = threading.Thread(target=monitor.run, name="terminal-preview", daemon=True)
    try:
        stream.write("\x1b[?1049h\x1b[2J\x1b[?25l")
        stream.flush()
        worker.start()
        yield
    finally:
        monitor.stop.set()
        if worker.is_alive():
            worker.join(timeout=2)
        try:
            stream.write("\x1b[0m\x1b[?25h\x1b[?1049l")
            stream.flush()
        finally:
            if os.name == "nt":
                kernel.SetConsoleMode(handle, original.value)
