"""Summarize recorded input timing and OpenArm kinematic tracking for one trial."""

import argparse
import json
import math
import statistics
import struct
import sys
import zlib
from pathlib import Path

from replay import inspect_episode, local_file, rows
from robot_sim import pose_error


def distribution(values):
    """Median, linearly interpolated 95th percentile, and maximum in the caller's units."""
    if not values:
        return None
    ordered = sorted(values)
    k = 0.95 * (len(ordered) - 1)
    lo, hi = ordered[math.floor(k)], ordered[math.ceil(k)]
    return {"count": len(ordered), "median": round(statistics.median(ordered), 6),
            "p95": round(lo + (hi - lo) * (k - math.floor(k)), 6), "max": round(ordered[-1], 6)}


def summarize_samples(records):
    sides = ("left", "right")
    profiles = {side: set() for side in sides}
    frames, intervals = set(), []
    sequence_gaps = 0
    previous = None
    arms = {side: {"tracked_samples": 0, "clutched_samples": 0, "rejection_entries": None,
                   "rejecting": False, "position": [], "angle": [], "jaw": []} for side in sides}
    encoder_readings = {side: {} for side in sides}
    bench_modes = {side: set() for side in sides}
    for raw, action, feedback in records:
        stamp = tuple(raw[key] for key in ("session", "seq", "host_monotonic_ns"))
        if any(tuple(row[key] for key in ("session", "seq", "host_monotonic_ns")) != stamp
               for row in (action, feedback)):
            raise ValueError("Input, action and feedback samples do not match by session, sequence and host time")
        session, seq, ns = stamp
        same_session = previous is not None and previous[0] == session
        if same_session:
            if seq <= previous[1] or ns < previous[2]:
                raise ValueError("Input sequence or host time out of order within session")
            intervals.append((ns - previous[2]) / 1e6)
            sequence_gaps += seq - previous[1] - 1
        previous = stamp
        sample = raw["sample"]
        frames.add(sample["frame"])
        bench = feedback.get("physical_gripper")
        if isinstance(bench, dict) and bench.get("arm") in sides:
            side = bench["arm"]
            if bench.get("mode"):
                bench_modes[side].add(bench["mode"])
            timestamp, angle = bench.get("feedback_host_ns"), bench.get("measured_deg")
            if type(timestamp) is int and type(angle) in (int, float) and math.isfinite(angle):
                encoder_readings[side][timestamp] = angle
        for side, arm in arms.items():
            source = sample["hands"].get(side)
            if source and source.get("profile"):
                profiles[side].add(source["profile"])
            hand = action["state"]["hands"][side]
            arm["tracked_samples"] += bool(hand.get("tracked"))
            arm["clutched_samples"] += bool(hand.get("clutched"))
            result = feedback["hands"][side]
            robot = result.get("robot")
            if not robot:  # The older tool-marker simulator recorded ideal pose feedback.
                continue
            rejected = hand.get("status") == "outside simulated reach"
            if arm["rejection_entries"] is None:
                arm["rejection_entries"] = 0
            if rejected and not (same_session and arm["rejecting"]):
                arm["rejection_entries"] += 1
            arm["rejecting"] = rejected
            if hand.get("tracked"):
                arm["jaw"].append(1000 * robot["jaw_opening_m"])
            if hand.get("clutched") and robot["status"] == "tracking":
                linear, angular = pose_error(hand["tool"], result["tcp"])
                arm["position"].append(1000 * math.hypot(*linear))
                arm["angle"].append(math.degrees(math.hypot(*angular)))

    per_hand = {}
    for side, arm in arms.items():
        aperture = arm["jaw"]
        readbacks = encoder_readings[side].values()
        per_hand[side] = {
            "reported_controller_profiles": sorted(profiles[side]), "tracked_samples": arm["tracked_samples"],
            "clutched_samples": arm["clutched_samples"], "observed_ik_rejection_entries": arm["rejection_entries"],
            "clutched_target_vs_fk_position_error_mm": distribution(arm["position"]),
            "clutched_target_vs_fk_orientation_error_deg": distribution(arm["angle"]),
            "tracked_simulated_jaw_range_mm": [round(min(aperture), 3), round(max(aperture), 3)] if aperture else None,
            "recorded_j8_bench": {"modes": sorted(bench_modes[side]), "distinct_encoder_readbacks": len(readbacks),
                                  "measured_range_deg": [min(readbacks), max(readbacks)] if readbacks else None},
        }
    return {"recorded_input_frames": sorted(frames), "recorded_receive_interval_ms": distribution(intervals),
            "missing_sequence_numbers_between_records": sequence_gaps, "hands": per_hand}


def trial_metrics(directory):
    directory = Path(directory)
    verified = inspect_episode(directory)
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    streams = [rows(local_file(directory, manifest["streams"][name])) for name in ("input", "actions", "feedback")]
    report = {"episode": directory.name, "status": verified["status"], "model": verified["model"],
            "recorded_input_samples": verified["streams"]["input"],
            "recorder_dropped_input_records": verified["dropped_input_records"],
            **summarize_samples(zip(*streams)),
            "physical_j1_j7_and_tcp_feedback": "unavailable"}
    if "robot_io" in manifest["streams"]:
        cycles = verified["streams"]["robot_io"]
        count = sum(bool(row.get("measured_deg")) for row in
                    rows(local_file(directory, manifest["streams"]["robot_io"])))
        report["physical_j1_j7_and_tcp_feedback"] = {
            "hardware_io_cycles": cycles, "cycles_with_encoder_readings": count,
            "missing_io_cycles": verified["missing_robot_io"], "measured_tcp": "unavailable"}
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path, help="episode directory containing manifest.json")
    args = parser.parse_args(argv)
    try:
        print(json.dumps(trial_metrics(args.episode), indent=2, allow_nan=False))
    except (KeyError, OSError, RuntimeError, TypeError, ValueError, struct.error, zlib.error) as exc:
        print(f"Trial metrics failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
