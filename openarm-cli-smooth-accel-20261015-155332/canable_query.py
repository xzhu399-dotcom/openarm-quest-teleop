"""Bounded classic-CAN diagnostics for DaMiao motors using gs_usb on macOS.

Default: passive listen only. --query sends ONLY parameter-read (0x33, RID 7)
requests to CAN ID 0x7ff. No enable, motion, parameter write, or flash commands.
References: enactic/openarm_can dm_motor_control.cpp and Linux gs_usb.c.
Adapter CAN channel is stopped on exit. Does not change motor configuration.
Use --all-adapters to check each attached adapter separately, in sequence.
"""
import argparse
import json
import struct
import time

import libusb_package
import usb.backend.libusb1
import usb.core
import usb.util


def timing(caps, bitrate):
    _, clock, s1min, s1max, s2min, s2max, sjwmax, bmin, bmax, binc = caps
    candidates = []
    for brp in range(bmin, bmax + 1, binc):
        if clock % (brp * bitrate):
            continue
        quanta = clock // (brp * bitrate)
        for s2 in range(s2min, s2max + 1):
            s1 = quanta - 1 - s2
            if s1min <= s1 <= s1max:
                score = (abs((1 + s1) / quanta - 0.75), -quanta)
                candidates.append((score, (1, s1 - 1, s2, 1, brp)))
    if not candidates:
        raise ValueError(f"No exact bit timing for {bitrate}")
    return min(candidates)[1]


def query_frame(motor_id):
    if not 1 <= motor_id <= 16:
        raise ValueError("This diagnostic is limited to motor IDs 1..16")
    data = struct.pack("<HBBI", motor_id, 0x33, 7, 0)
    return struct.pack("<IIBBBB", 0, 0x7FF, 8, 0, 0, 0) + data


def decode(raw):
    if len(raw) < 20:
        raise ValueError(f"Short gs_usb packet: {len(raw)}")
    echo, cid, dlc, channel, flags, _ = struct.unpack("<IIBBBB", raw[:12])
    if dlc > 8 or flags & 2:
        raise ValueError("Unexpected FD frame in classic-CAN diagnostic")
    return {"echo": echo != 0xFFFFFFFF, "error": bool(cid & 0x20000000),
            "can_id": cid, "channel": channel, "data": raw[12:12 + dlc]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--query", action="store_true")
    parser.add_argument("--all-adapters", action="store_true",
                        help="逐块检测所有 gs_usb CANable，按 USB 设备分别输出结果")
    parser.add_argument("--bitrate", type=int, default=1000000,
                        choices=[125000, 200000, 250000, 500000, 1000000])
    args = parser.parse_args(argv)
    backend = usb.backend.libusb1.get_backend(find_library=libusb_package.find_library)
    devices = list(usb.core.find(find_all=True, backend=backend,
                                idVendor=0x1D50, idProduct=0x606F))
    if not devices:
        raise RuntimeError("未找到 gs_usb CANable；请检查 USB 连接及 BOOT 模式")
    if len(devices) != 1 and not args.all_adapters:
        raise RuntimeError(f"发现 {len(devices)} 块 CANable；逐块只读检测请加 --all-adapters")
    reports = []
    for index, dev in enumerate(devices, start=1):
        result = diagnose(dev, args)
        result["adapter_index"] = index  # Enumeration order is not a left/right identity.
        reports.append(result)
    output = {"adapter_count": len(reports), "adapters": reports} if args.all_adapters else reports[0]
    print(json.dumps(output, indent=2, ensure_ascii=False), flush=True)
    return int(any("failure" in r or "stop_error" in r for r in reports))


def diagnose(dev, args):
    started = False
    results = {"bitrate": args.bitrate, "query": args.query,
               "usb_bus": dev.bus, "usb_address": dev.address,
               "usb_transfers_sent": 0, "tx_echoes": 0,
               "rx_frames": [], "motor_replies": [], "error_frames": []}

    def ctrl(request, payload):
        dev.ctrl_transfer(0x41, request, 0, 0, payload, timeout=1000)

    def receive(seconds, requested_id=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            try:
                raw = bytes(dev.read(0x81, 64, timeout=20))
            except usb.core.USBTimeoutError:
                continue
            frame = decode(raw)
            if frame["echo"]:
                results["tx_echoes"] += 1
                continue
            summary = {"can_id": hex(frame["can_id"]), "data": frame["data"].hex()}
            if frame["error"]:
                results["error_frames"].append(summary)
                if args.query:
                    raise RuntimeError("CAN error received; stopped query to avoid retries")
                continue
            results["rx_frames"].append(summary)
            data = frame["data"]
            # Require an actual received standard CAN frame, not local TX echo.
            if (requested_id is not None and frame["channel"] == 0
                    and not frame["can_id"] & 0xE0000000 and len(data) == 8
                    and data[2:4] == bytes([0x33, 7])
                    and int.from_bytes(data[:2], "little") == requested_id):
                results["motor_replies"].append({"motor_id": requested_id,
                    "reply_can_id": hex(frame["can_id"]),
                    "master_id_parameter": int.from_bytes(data[4:], "little")})
            if len(results["rx_frames"]) + len(results["error_frames"]) >= 256:
                raise RuntimeError("Frame limit reached; stopping bounded diagnostic")

    try:
        try:
            results["usb_serial"] = dev.serial_number
        except (usb.core.USBError, ValueError) as exc:
            results["usb_serial"] = None
            results["serial_read_error"] = str(exc)
        caps = struct.unpack("<10I", bytes(dev.ctrl_transfer(0xC1, 4, 0, 0, 40,
                                                            timeout=1000)))
        if not args.query and not caps[0] & 1:
            raise RuntimeError("Adapter does not support passive listen-only mode")
        settings = timing(caps, args.bitrate)
        results["timing"] = settings
        ctrl(0, struct.pack("<I", 0xBEEF))
        ctrl(2, struct.pack("<II", 0, 0))
        # Discard residual RX/echo packets from any earlier session.
        for _ in range(64):
            try:
                dev.read(0x81, 64, timeout=5)
            except usb.core.USBTimeoutError:
                break
        else:
            raise RuntimeError("Receive buffer did not drain")
        ctrl(1, struct.pack("<5I", *settings))
        mode_flags = 0 if args.query else 1
        if args.query and caps[0] & 8:
            mode_flags |= 8  # one-shot where supported
        ctrl(2, struct.pack("<II", 1, mode_flags))
        started = True
        if args.query:
            for motor_id in range(1, 17):
                # This is the only bulk OUT operation in the program.
                dev.write(0x02, query_frame(motor_id), timeout=500)
                results["usb_transfers_sent"] += 1
                receive(0.08, motor_id)
        else:
            receive(1.0)
    except Exception as exc:
        results["failure"] = str(exc)
    finally:
        if started:
            try:
                ctrl(2, struct.pack("<II", 0, 0))
                results["adapter_stopped"] = True
            except usb.core.USBError as exc:
                results["stop_error"] = str(exc)
        usb.util.dispose_resources(dev)
    results["online_motor_ids"] = sorted({r["motor_id"] for r in results["motor_replies"]})
    return results


if __name__ == "__main__":
    raise SystemExit(main())
