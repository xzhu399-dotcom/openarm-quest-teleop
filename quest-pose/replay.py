"""Verify, inspect and replay a local simulated episode, disconnected from hardware."""

import argparse
import itertools
import json
import math
import struct
import sys
import time
import zlib
from pathlib import Path

from recording import write_manifest
from synthetic_cameras import CAMERAS, HEIGHT, SIGNATURE, WIDTH, still_image
from live_cameras import require_opencv


def rows(path):
    with path.open(encoding="utf-8") as stream:
        for number, line in enumerate(stream, 1):
            try:
                yield json.loads(line)
            except ValueError as exc:
                raise ValueError(f"Invalid JSON at {path.name} line {number}") from exc


def video_frames(path):
    """Read the APNG and verify chunk CRCs, frame sequence, dimensions and decompression."""
    with path.open("rb") as src:
        if src.read(8) != SIGNATURE:
            raise ValueError(f"{path.name}: invalid PNG signature")
        expected_seq = frames = 0
        declared, pending, dimensions = None, False, None
        while True:
            header = src.read(8)
            if len(header) != 8:
                raise ValueError(f"{path.name}: missing PNG end chunk")
            size, tag = struct.unpack(">I4s", header)
            offset = src.tell()
            if size > 8 * 1024 * 1024:
                raise ValueError(f"{path.name}: unreasonable chunk size")
            data = src.read(size)
            crc = src.read(4)
            if len(data) != size or crc != struct.pack(">I", zlib.crc32(tag + data)):
                raise ValueError(f"{path.name}: damaged {tag!r} at byte {offset}")
            if tag == b"IHDR":
                dimensions = struct.unpack(">IIBBBBB", data)
                if dimensions != (WIDTH, HEIGHT, 8, 3, 0, 0, 0):
                    raise ValueError(f"{path.name}: unexpected frame format")
            elif tag == b"acTL":
                declared, _ = struct.unpack(">II", data)
            elif tag == b"fcTL":
                seq, w, h, x, y, *_ = struct.unpack(">IIIIIHHBB", data)
                if pending or seq != expected_seq or (w,h,x,y) != (WIDTH,HEIGHT,0,0):
                    raise ValueError(f"{path.name}: invalid frame control")
                expected_seq += 1
                pending = True
            elif tag in (b"IDAT", b"fdAT"):
                if not pending or (tag == b"IDAT") != (frames == 0):
                    raise ValueError(f"{path.name}: unexpected frame payload")
                if tag == b"fdAT":
                    if data[:4] != struct.pack(">I", expected_seq):
                        raise ValueError(f"{path.name}: invalid sequence")
                    data = data[4:]
                    offset += 4
                    expected_seq += 1
                raster = zlib.decompress(data)
                if len(raster) != (WIDTH+1)*HEIGHT:
                    raise ValueError(f"{path.name}: invalid decoded frame length")
                yield offset, data
                frames += 1
                pending = False
            elif tag == b"IEND":
                if not dimensions or declared != frames or not frames or pending or src.read(1):
                    raise ValueError(f"{path.name}: inconsistent animation length")
                return


def local_file(directory, name):
    path = (directory / name).resolve()
    if path.parent != directory.resolve():
        raise ValueError("Manifest file must be inside the episode directory")
    return path


def mjpeg_records(path, timestamps, info):
    cv2 = require_opencv()
    import numpy as np
    with path.open("rb") as stream:
        for row in timestamps:
            offset, size = stream.tell(), row["size"]
            if type(size) is not int or not 0 < size <= 16*1024*1024 or row["offset"] != offset:
                raise ValueError(f"{path.name}: invalid JPEG frame index")
            jpeg = stream.read(size)
            if (len(jpeg) != size or not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9")
                    or zlib.crc32(jpeg) != row["crc32"]):
                raise ValueError(f"{path.name}: damaged JPEG frame at byte {offset}")
            image = cv2.imdecode(np.frombuffer(jpeg,dtype=np.uint8),cv2.IMREAD_COLOR)
            if image is None or image.shape[:2] != (info["height"],info["width"]):
                raise ValueError(f"{path.name}: invalid JPEG dimensions")
            yield row, (offset,jpeg)
        if stream.read(1):
            raise ValueError(f"{path.name}: video contains unindexed data")


def inspect_episode(directory, at=None, frame_dir=None):
    directory = Path(directory)
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    if manifest.get("schema") != 1 or manifest.get("simulated") is not True:
        raise ValueError("Expected a simulated OpenArm episode (schema 1)")
    if set(manifest["cameras"]) != set(CAMERAS):
        raise ValueError("Expected overview and both wrist cameras")
    if at is not None and (not math.isfinite(at) or at < 0):
        raise ValueError("Choose a finite, nonnegative replay time")
    if frame_dir and Path(frame_dir).resolve() == directory.resolve():
        raise ValueError("Choose a separate directory for extracted frames")
    start = manifest["start_host_ns"]
    when = start + int(1e9*at) if at is not None else None
    summary = {k:manifest.get(k) for k in ("status","outcome","task","simulated","model","duration_s")}
    summary["camera_mode"] = manifest.get("camera_mode","synthetic")
    if "physical_gripper" in manifest:
        summary["physical_gripper"] = manifest["physical_gripper"]
    summary.update(dropped_input_records=manifest.get("dropped_input_records", 0),
                   streams={}, cameras={})
    # A hard process exit retains the initial manifest. Stream contents provide the counts.
    check_counts = manifest.get("end_host_ns") is not None
    snapshot = {"at_s":at, "action":None, "cameras":{}} if when is not None else None
    for name in ("input", "actions", "feedback", "events"):
        count = 0
        for row in rows(local_file(directory, manifest["streams"][name])):
            ns = row["host_monotonic_ns"]
            if not isinstance(ns, int):
                raise ValueError(f"{name}: invalid host timestamp")
            if name == "actions" and when is not None and ns <= when:
                snapshot["action"] = _pose_report(row, start)
            count += 1
        summary["streams"][name] = count
        if check_counts and count != manifest["counts"][name]:
            raise ValueError(f"{name}: manifest count mismatch")
    if len({summary["streams"][k] for k in ("input","actions","feedback")}) != 1:
        raise ValueError("Input, action and feedback records differ in length")
    stills = {}
    for cam, info in manifest["cameras"].items():
        count = skipped = 0
        previous_ns = previous_src = first_ns = selected = None
        timestamps = rows(local_file(directory, info["timestamps"]))
        video = local_file(directory, info["video"])
        live = info["format"] == "MJPEG"
        if live:
            indexed = mjpeg_records(video, timestamps, info)
        elif info["format"] == "APNG":
            indexed = itertools.zip_longest(timestamps,video_frames(video))
        else:
            raise ValueError(f"{cam}: unsupported video format")
        for row, image in indexed:
            if row is None or image is None:
                raise ValueError(f"{cam}: video and frame table differ in length")
            offset, data = image
            ns, src = row["host_monotonic_ns"], row["source_index"]
            if row["frame_index"] != count or row["offset"] != offset or row["size"] != len(data):
                raise ValueError(f"{cam}: frame {count} has an invalid index entry")
            if type(ns) is not int or ns < start or (not live and row["source_ns"] != ns):
                raise ValueError(f"{cam}: invalid capture timestamp")
            if previous_ns is not None and (ns <= previous_ns or src <= previous_src):
                raise ValueError(f"{cam}: timestamps and source indices must increase")
            if previous_src is not None:
                skipped += src - previous_src - 1
            if when is not None and ns <= when:
                selected = (row, data)
            if first_ns is None:
                first_ns = ns
            previous_ns, previous_src = ns, src
            count += 1
        if check_counts and (count != info["frames"] or skipped != info["dropped_frames"]):
            raise ValueError(f"{cam}: manifest frame count mismatch")
        summary["cameras"][cam] = {
            "frames":count,"dropped_frames":skipped,"first_host_ns":first_ns,"last_host_ns":previous_ns,
            "source":info["source"],"capture_issue":info.get("capture_issue"),
        }
        if when is not None:
            snapshot["cameras"][cam] = selected[0] if selected else None
            if selected and frame_dir:
                if live:
                    cv2 = require_opencv()
                    import numpy as np
                    image = cv2.imdecode(np.frombuffer(selected[1],dtype=np.uint8),cv2.IMREAD_COLOR)
                    ok, png = cv2.imencode(".png",image)
                    if not ok:
                        raise ValueError(f"{cam}: failed to export PNG")
                    stills[cam] = png.tobytes()
                else:
                    stills[cam] = still_image(selected[1])
    if stills:
        dest = Path(frame_dir)
        dest.mkdir(parents=True, exist_ok=True)
        if any((dest / (cam+".png")).exists() for cam in stills):
            raise ValueError("Frame export destination already contains a matching PNG")
        for cam, png in stills.items():
            with (dest / (cam+".png")).open("xb") as stream:
                stream.write(png)
    if snapshot:
        summary["snapshot"] = snapshot
    return summary


def _pose_report(row, start):
    state = row["state"]
    hands = {
        side:{key:hand.get(key) for key in ("initial","delta","clutch_delta","tool","closure","clutched")}
        for side, hand in state["hands"].items() if isinstance(hand, dict)
    }
    for side, hand in hands.items():
        robot = state["hands"][side].get("robot")
        if robot:
            hand.update(simulated_joints_rad=robot["joint_angles_rad"], fk_tcp=robot["tcp"],
                        jaw_opening_m=robot["jaw_opening_m"], ik_status=robot["status"])
    return {"elapsed_s":(row["host_monotonic_ns"]-start)/1e9, "seq":row["seq"],
            "held":state.get("held"), "hands":hands}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    parser.add_argument("--at", type=float, metavar="SECONDS",
                        help="inspect the latest action and each camera frame at or before this time")
    parser.add_argument("--frames", type=Path, metavar="DIR", help="export PNG stills for --at")
    parser.add_argument("--play", action="store_true", help="print the recorded tool poses at recorded speed")
    parser.add_argument("--outcome", choices=["success","failure","aborted"], help="label a finished episode")
    args = parser.parse_args(argv)
    if args.frames is not None and args.at is None:
        parser.error("--frames requires --at")
    try:
        summary = inspect_episode(args.episode, args.at, args.frames)
        manifest_path = args.episode / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if args.outcome:
            if manifest["status"] != "saved":
                raise ValueError("Only completed episodes can receive a reviewed outcome")
            manifest["outcome"] = args.outcome
            write_manifest(manifest_path, manifest)
            summary["outcome"] = args.outcome
        print(json.dumps(summary, indent=2))
        if args.play:
            replay_start = time.monotonic()
            for row in rows(local_file(args.episode, manifest["streams"]["actions"])):
                pose = _pose_report(row, manifest["start_host_ns"])
                time.sleep(max(0, replay_start + pose["elapsed_s"] - time.monotonic()))
                print(json.dumps(pose), flush=True)
    except (KeyError, OSError, ValueError, struct.error, zlib.error) as exc:
        print(f"Episode inspection failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
