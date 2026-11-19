"""Plain-text Quest and simulated tool telemetry for a workstation terminal or pipe."""

from contextlib import contextmanager
import queue
import sys
import threading
import time


def format_pose(pose):
    if pose is None:
        return "untracked"
    xyz = ", ".join(f"{v:.3f}" for v in pose["p"])
    xyzw = ", ".join(f"{v:.3f}" for v in pose["q"])
    return f"XYZ m [{xyz}]  XYZW [{xyzw}]"


def report_lines(state):
    rec = state["recording"]
    if rec["active"]:
        recording = f"Recording ACTIVE  {rec['episode']}  {rec['elapsed_s']:.1f} s"
    elif rec["episode"]:
        recording = f"Recording IDLE  last {rec['episode']} ({rec['last_episode_status']})"
    else:
        recording = "Recording IDLE"
    motion = "HOLD " + state["reason"] if state["held"] else "ready"
    if state.get("grip_only"):
        motion = state["reason"] or "side-grip control"
    frame = ascii(state["frame"]) if state["frame"] is not None else "waiting for input"
    result = state["trial_return"]
    destination = "home" if result.get("target") == "home" else "trial"
    lines = [f"{recording} | {motion} | {destination} return {result['status']}",
             f"Frame {frame} | sample {state['seq']} | {state['gripper_mode']} gripper",
             f"Headset  {format_pose(state['head'])}"]
    bench = state.get("gripper_bench")
    if bench:
        def degrees(value):
            return "pending" if value is None else f"{value:.3f} deg"

        lines.append(f"Physical {bench['arm']} J8 | {bench['mode']} | {bench['phase']} | "
                     f"enabled {bench['enabled']} | target {degrees(bench['target_deg'])} | "
                     f"encoder {degrees(bench['measured_deg'])}")
    for side in ("left", "right"):
        hand = state["hands"][side]
        raw_pose = hand["pose"]
        clutch = "engaged" if hand["clutched"] else "released"
        lines.append(f"{side.title()} handle  {format_pose(raw_pose)}")
        if raw_pose is not None:
            lines.append(f"  from initial  {format_pose(hand['delta'])}")
        lines.append(f"  simulated tool target  {format_pose(hand['tool'])}")
        jaw = hand["robot"]["jaw_opening_m"] * 1000
        control = ""
        if raw_pose is not None:
            if state["gripper_mode"] == "stick" and len(hand["axes"]) > 3:
                control = f" | stick Y {hand['axes'][3]:+.2f}"
            elif state["gripper_mode"] == "trigger" and hand["buttons"]:
                control = f" | trigger {hand['buttons'][0]['value']:.2f}"
        lines.append(f"  clutch {clutch} | jaw {jaw:.1f} mm | closure {hand['closure']:.0%}{control}")
    if rec["error"]:
        lines.append(f"Recorder error: {rec['error']}")
    return lines


@contextmanager
def pose_report(station, trial_events, stream=None):
    """Print the latest poses at 2 Hz and every recorder transition in order."""
    stream = sys.stdout if stream is None else stream

    def display():
        previous = None
        next_snapshot = 0.0
        try:
            while True:
                try:
                    event = trial_events.get(timeout=max(0, next_snapshot - time.monotonic()))
                except queue.Empty:
                    lines = report_lines(station.state())
                    if lines != previous:
                        print("\n".join(lines), file=stream, flush=True)
                        previous = lines
                    next_snapshot = time.monotonic() + 0.5
                else:
                    if event is None:
                        break
                    action, episode, status = event
                    print(f"Recording {action}  {episode} ({status})", file=stream, flush=True)
        except (BrokenPipeError, OSError):
            return

    worker = threading.Thread(target=display, name="pose-report", daemon=True)
    worker.start()
    try:
        yield
    finally:
        trial_events.put(None)
        worker.join(timeout=2)
