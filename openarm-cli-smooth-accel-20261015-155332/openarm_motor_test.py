"""Mac + CANable gs_usb / classic CAN 1 Mbps: one DaMiao motor per test.

Without --execute this prints a plan and does not import USB libraries or open USB.
Default motion mode: browser keyboard, hold A/D, release to hold reference.
Use --sweep for the older small out-and-back test.
Supports existing MIT (1) and position/velocity (2) modes without switching modes.
Optional --watchdog-ms changes TIMEOUT in RAM and restores it after confirmed
disable. No zeroing or flash writes.
Support the arm mechanically: other joints are not held, and exit disables the
selected motor. Reference speed limiting is NOT a drive-level speed guarantee.
Protocol: https://github.com/enactic/openarm_can
"""
import argparse
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import struct
import sys
import time


@dataclass(frozen=True)
class Limits:
    p: float
    v: float
    t: float


class SessionLog:
    """Keep recent raw frames and controller decisions; persist after shutdown."""
    def __init__(self, motor):
        self.started = datetime.now(timezone.utc).isoformat()
        self.motor = motor
        self.events = deque(maxlen=3000)
        self.total_events = 0

    def record(self, event, **data):
        self.total_events += 1
        self.events.append(dict(t_monotonic=time.monotonic(), event=event, **data))

    def save(self):
        folder = Path(__file__).resolve().parent / 'motor-test-logs'
        folder.mkdir(exist_ok=True)
        name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        path = folder / f'motor-{self.motor}-{name}.json'
        payload = dict(started_utc=self.started, motor=self.motor,
                       total_events=self.total_events, retained_events=len(self.events),
                       events=list(self.events))
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2))
        print(f'诊断日志：{path}')


def log_event(link, event, **data):
    trace = getattr(link, 'trace', None)
    if trace is not None:
        trace.record(event, **data)


def encode_value(value, minimum, maximum, bits):
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f"Command outside encoding range: {value}")
    return round((value - minimum) / (maximum - minimum) * ((1 << bits) - 1))


def mit_frame(q, kp, kd, limits):
    # Zero velocity and torque feedforward; position reference is slowly ramped.
    p = encode_value(q, -limits.p, limits.p, 16)
    v = encode_value(0, -limits.v, limits.v, 12)
    k = encode_value(kp, 0, 500, 12)
    d = encode_value(kd, 0, 5, 12)
    t = encode_value(0, -limits.t, limits.t, 12)
    return bytes([p >> 8, p & 255, v >> 4, ((v & 15) << 4) | (k >> 8),
                  k & 255, d >> 4, ((d & 15) << 4) | (t >> 8), t & 255])


def position_command(args, reference, limits, gain_factor=1.0):
    if args.control_mode == 1:
        return args.motor, mit_frame(reference, args.kp * gain_factor,
                                    args.kd * gain_factor, limits)
    if args.control_mode == 2:
        if not math.isfinite(reference) or not -limits.p < reference < limits.p:
            raise ValueError("Position outside validated mapping range")
        velocity_limit = math.radians(args.speed_deg)
        if not 0 < velocity_limit <= math.radians(2):
            raise ValueError("Invalid position-mode speed limit")
        return args.motor + 0x100, struct.pack("<ff", reference, velocity_limit)
    raise RuntimeError("Unsupported control mode")


def watchdog_ticks(milliseconds):
    # DaMiao CAN Timeout uses 50 microsecond cycles (Seeed Damiao documentation).
    if type(milliseconds) is not int or not 250 <= milliseconds <= 1000:
        raise ValueError("--watchdog-ms 必须为 250–1000 的整数")
    return milliseconds * 20


def parse_state(data, motor, limits):
    if len(data) != 8 or data[0] & 15 != motor:
        raise ValueError("State frame does not match selected motor")
    p = (data[1] << 8) | data[2]
    v = (data[3] << 4) | (data[4] >> 4)
    t = ((data[4] & 15) << 8) | data[5]
    return {"status": data[0] >> 4,
            "raw_hex": data.hex(), "raw_position": p, "raw_velocity": v,
            "q": p * (2 * limits.p) / 65535 - limits.p,
            "v": v * (2 * limits.v) / 4095 - limits.v,
            "torque": t * (2 * limits.t) / 4095 - limits.t,
            "temperature": max(data[6], data[7])}


def smooth_reference(start, delta, elapsed, duration):
    s = min(1.0, max(0.0, elapsed / duration))
    return start + delta * (10 * s**3 - 15 * s**4 + 6 * s**5)


def select_adapter(devices, serial=None):
    """Never fall back to enumeration order when a serial is requested."""
    if serial is None:
        if len(devices) != 1:
            raise RuntimeError(f"只连接一块 CANable；当前发现 {len(devices)} 块")
        return devices[0]
    matches = []
    for device in devices:
        try:
            if device.serial_number == serial:
                matches.append(device)
        except Exception:
            continue
    if len(matches) != 1:
        raise RuntimeError(f"序列号 {serial} 必须唯一匹配；当前匹配 {len(matches)} 块，拒绝选择其他设备")
    return matches[0]


class Link:
    def __init__(self, serial=None):
        # Deferred so preview, import, and offline tests cannot enumerate USB.
        import libusb_package
        import usb.backend.libusb1
        import usb.core
        import usb.util
        from canable_query import timing, decode
        self.usb = usb
        self.trace = None
        self.decode = decode
        self.dev = None
        backend = usb.backend.libusb1.get_backend(find_library=libusb_package.find_library)
        devices = list(usb.core.find(find_all=True, backend=backend,
                                    idVendor=0x1D50, idProduct=0x606F))
        try:
            self.dev = select_adapter(devices, serial)
        finally:
            for device in devices:
                if device is not self.dev:
                    usb.util.dispose_resources(device)
        try:
            caps = struct.unpack("<10I", bytes(self.dev.ctrl_transfer(
                0xC1, 4, 0, 0, 40, timeout=100)))
            self.ctrl(0, struct.pack("<I", 0xBEEF))
            self.ctrl(2, struct.pack("<II", 0, 0))
            self.drain()
            self.ctrl(1, struct.pack("<5I", *timing(caps, 1000000)))
            self.ctrl(2, struct.pack("<II", 1, 0))
        except BaseException:
            self.close()
            raise

    def ctrl(self, request, data):
        self.dev.ctrl_transfer(0x41, request, 0, 0, data, timeout=100)

    def send(self, cid, data):
        if not 0 <= cid <= 0x7FF or len(data) != 8:
            raise ValueError("Only standard 8-byte classic CAN frames are supported")
        log_event(self, 'can_tx_request', can_id=hex(cid), data=data.hex())
        self.dev.write(0x02, struct.pack("<IIBBBB", 0, cid, 8, 0, 0, 0) + data,
                       timeout=50)

    def receive(self, timeout=2):
        try:
            raw = bytes(self.dev.read(0x81, 64, timeout=timeout))
        except self.usb.core.USBTimeoutError:
            return None
        frame = self.decode(raw)
        log_event(self, 'can_rx_or_echo', can_id=hex(frame['can_id']),
                  data=frame['data'].hex(), echo=frame['echo'], error=frame['error'])
        if frame["error"]:
            raise RuntimeError("CAN error frame; stopping")
        if frame["echo"] or frame["channel"] != 0 or frame["can_id"] & 0xE0000000:
            return None
        return frame

    def drain(self):
        for _ in range(128):
            try:
                self.dev.read(0x81, 64, timeout=2)
            except self.usb.core.USBTimeoutError:
                return
        raise RuntimeError("CAN receive buffer is busy; stop other controllers first")

    def param(self, motor, rid, as_float=False):
        self.drain()
        request = struct.pack("<HBBI", motor, 0x33, rid, 0)
        self.send(0x7FF, request)
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            frame = self.receive(10)
            if frame:
                data = frame["data"]
                if len(data) == 8 and data[:4] == request[:4]:
                    return struct.unpack("<f" if as_float else "<I", data[4:])[0]
        raise RuntimeError(f"Motor {motor}, register {rid}: no reply")

    def state(self, motor, reply_id, limits):
        self.drain()
        self.send(0x7FF, struct.pack("<HBBI", motor, 0xCC, 0, 0))
        deadline = time.monotonic() + 0.15
        while time.monotonic() < deadline:
            frame = self.receive(5)
            if frame and frame["can_id"] == reply_id:
                state = parse_state(frame["data"], motor, limits)
                log_event(self, 'motor_state', **state)
                return state
        raise RuntimeError("电机状态反馈超时（150 ms）")

    def set_watchdog(self, motor, ticks):
        """Write ONLY register 9 in RAM; caller must verify disabled state first."""
        if type(ticks) is not int or not 0 <= ticks <= 0xFFFFFFFF:
            raise ValueError("Invalid timeout value")
        self.drain()
        payload = struct.pack("<HBBI", motor, 0x55, 9, ticks)
        self.send(0x7FF, payload)
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            frame = self.receive(5)
            if frame and frame["data"] == payload:
                if self.param(motor, 9) != ticks:
                    raise RuntimeError("TIMEOUT 写入后的读取校验失败")
                return
        raise RuntimeError("TIMEOUT 写入未得到确认")

    def close(self):
        if self.dev is not None:
            try:
                self.ctrl(2, struct.pack("<II", 0, 0))
            except Exception as exc:
                print(f"无法停止适配器：{exc}", file=sys.stderr)
            finally:
                self.usb.util.dispose_resources(self.dev)
                self.dev = None


def check_state(state, origin, target, delta, limits, enabled):
    if state["status"] != (1 if enabled else 0):
        raise RuntimeError(f"Unexpected motor status 0x{state['status']:x}")
    if state["temperature"] >= 60:
        raise RuntimeError("电机温度达到 60°C，停止")
    velocity_tolerance = 2 * limits.v / 4095
    if abs(state["v"]) > math.radians(5) + velocity_tolerance:
        raise RuntimeError(
            f"速度保护：反馈 {math.degrees(state['v']):+.3f}°/s，"
            f"阈值 ±{5 + math.degrees(velocity_tolerance):.3f}°/s "
            f"（5°/s 加量化容差）；位置 {math.degrees(state['q']):.3f}°，"
            f"目标 {math.degrees(target):.3f}°，原始帧 {state.get('raw_hex', 'unknown')}。"
            "已停止；不要直接提高阈值。")
    if abs(state["q"] - origin) > abs(delta) + math.radians(2):
        raise RuntimeError("电机超出本次小范围测试区域，停止")
    if abs(state["q"] - target) > math.radians(3):
        raise RuntimeError("位置跟踪误差超过 3°，停止；不要直接提高增益")


def disable_selected(link, motor, reply_id, limits):
    confirmed = False
    # Best effort only: USB/power faults can prevent these commands arriving.
    for _ in range(3):
        try:
            link.send(motor, b"\xff" * 7 + b"\xfd")
            state = link.state(motor, reply_id, limits)
            log_event(link, 'disable_feedback', status=state['status'])
            if state["status"] == 0:
                confirmed = True
                break
        except Exception:
            pass
    if not confirmed:
        print("未确认电机失能，请立即使用物理急停。", file=sys.stderr)
    else:
        print(f"电机 {motor} 已回复失能状态。")
    return confirmed


def run_sweep(link, args, origin, reply_id, limits):
    motor = args.motor
    delta = math.radians(args.delta_deg)
    target = origin + delta
    duration = 1.875 * abs(args.delta_deg) / args.speed_deg
    phases = [(origin, 0.0, 1.0, True),
              (origin, delta, duration, False),
              (target, 0.0, 0.5, False),
              (target, -delta, duration, False),
              (origin, 0.0, 0.5, False)]
    previous_tick = time.monotonic()
    last_print = 0.0
    for start, displacement, seconds, gain_ramp in phases:
        begun = time.monotonic()
        while True:
            now = time.monotonic()
            if now - previous_tick > 0.15:
                raise RuntimeError("控制循环延迟超过 150 ms，停止")
            previous_tick = now
            elapsed = now - begun
            reference = smooth_reference(start, displacement, elapsed, seconds)
            state = link.state(motor, reply_id, limits)
            log_event(link, 'sweep_reference', reference=reference, origin=origin,
                      measured_q=state['q'], measured_v=state['v'])
            check_state(state, origin, reference, delta, limits, enabled=True)
            if time.monotonic() - now > 0.1:
                raise RuntimeError("反馈延迟超过 100 ms，停止")
            factor = min(1.0, elapsed / seconds) if gain_ramp else 1.0
            link.send(*position_command(args, reference, limits, factor))
            if now - last_print > 0.5:
                print(f"目标 {math.degrees(reference):8.3f}°  "
                      f"反馈 {math.degrees(state['q']):8.3f}°  "
                      f"速度 {math.degrees(state['v']):6.2f}°/s", flush=True)
                last_print = now
            if elapsed >= seconds:
                if not gain_ramp and abs(state["q"] - reference) > math.radians(0.5):
                    raise RuntimeError("未到达小幅运动目标（误差 > 0.5°），测试未通过")
                break
            time.sleep(max(0.0, 0.02 - (time.monotonic() - now)))


def execute(args):
    link = Link()
    trace = SessionLog(args.motor)
    link.trace = trace
    trace.record('session_start', requested_speed_deg=args.speed_deg,
                 range_deg=args.range_deg, keyboard=args.keyboard)
    enable_attempted = False
    completed = False
    reply_id = None
    limits = None
    watchdog_original = None
    watchdog_changed = False
    try:
        motor = args.motor
        reply_id = link.param(motor, 7)
        if not 0 <= reply_id <= 0x7FF or reply_id == 0x7FF:
            raise RuntimeError("Invalid feedback CAN ID")
        if link.param(motor, 8) != motor:
            raise RuntimeError("Motor ID verification failed")
        args.control_mode = link.param(motor, 10)
        trace.record('control_mode', value=args.control_mode)
        if args.control_mode not in (1, 2):
            raise RuntimeError(f"当前模式 {args.control_mode} 暂不支持；支持 MIT=1、位置速度=2")
        print("控制模式：" + ("MIT" if args.control_mode == 1 else
              "位置＋速度（沿用电机内部增益，--kp/--kd 不适用）"))
        limits = Limits(*(link.param(motor, rid, True) for rid in (21, 22, 23)))
        if not all(math.isfinite(v) and v > 0 for v in (limits.p, limits.v, limits.t)):
            raise RuntimeError("Invalid PMAX/VMAX/TMAX; refusing to guess motor model")
        watchdog_original = link.param(motor, 9)
        trace.record('preflight_parameters', pmax=limits.p, vmax=limits.v,
                     tmax=limits.t, watchdog=watchdog_original)
        print(f"反馈 ID=0x{reply_id:x}, 映射参数={limits}, TIMEOUT 原始值={watchdog_original}")
        if watchdog_original == 0 and args.watchdog_ms is None:
            raise RuntimeError("TIMEOUT=0：加 --watchdog-ms 500 可在失能状态下临时设置 500 ms 通信超时")
        state = link.state(motor, reply_id, limits)
        origin = state["q"]
        delta = math.radians(args.range_deg if args.keyboard else args.delta_deg)
        target = origin + delta
        check_state(state, origin, origin, delta, limits, enabled=False)
        if not -limits.p + 0.01 < target < limits.p - 0.01:
            raise RuntimeError("目标超出编码范围；不会进行回零或跨圈运动")
        # Verify a stationary, disabled starting point before enabling.
        time.sleep(0.1)
        state = link.state(motor, reply_id, limits)
        check_state(state, origin, origin, delta, limits, enabled=False)
        if abs(state["q"] - origin) > math.radians(0.2):
            raise RuntimeError("电机仍在移动，请先稳定机械支撑")
        origin = state["q"]
        target = origin + delta
        if not -limits.p + 0.01 < target < limits.p - 0.01:
            raise RuntimeError("更新后的目标超出编码范围")
        if args.keyboard:
            travel = math.radians(args.range_deg)
            if origin - travel <= -limits.p + 0.01 or origin + travel >= limits.p - 0.01:
                raise RuntimeError("键盘测试范围超出编码边界；请减小 --range-deg")
            if args.panel.inputs.sample(time.monotonic())[1]:
                raise RuntimeError("页面已失焦或断开，取消使能")
            print(f"电机 {motor}: 键盘控制，起点 {math.degrees(origin):.3f}°，结束后失能。")
        else:
            print(f"电机 {motor}: {math.degrees(origin):.3f}° → "
                  f"{math.degrees(target):.3f}° → 起始位置；结束后失能。")
        if args.watchdog_ms is not None:
            # State above must be disabled and stationary before this RAM write.
            ticks = watchdog_ticks(args.watchdog_ms)
            if ticks != watchdog_original:
                watchdog_changed = True  # restore even if write/ACK fails halfway
                link.set_watchdog(motor, ticks)
                print(f"临时 TIMEOUT={ticks}（{args.watchdog_ms} ms），未写入 Flash。")
        # Replace the old target while disabled; no absolute-zero target.
        link.send(*position_command(args, origin, limits, 0.0))
        enable_attempted = True  # cleanup also runs if the enable transfer fails
        trace.record('enable_attempt')
        link.send(motor, b"\xff" * 7 + b"\xfc")
        if args.keyboard:
            from openarm_keyboard import jog_loop
            jog_loop(link, args, origin, reply_id, limits)
        else:
            run_sweep(link, args, origin, reply_id, limits)
        completed = True
    except BaseException as exc:
        trace.record('failure', type=type(exc).__name__, detail=str(exc))
        raise
    finally:
        disabled_confirmed = not enable_attempted
        if enable_attempted:
            # Allow cleanup to complete if Ctrl+C/SIGTERM triggered the stop.
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            disabled_confirmed = disable_selected(link, args.motor, reply_id, limits)
            trace.record('disable_result', confirmed=disabled_confirmed)
            if not disabled_confirmed:
                completed = False
        try:
            if watchdog_changed:
                if not enable_attempted:
                    disabled_confirmed = link.state(args.motor, reply_id, limits)["status"] == 0
                if disabled_confirmed:
                    link.set_watchdog(args.motor, watchdog_original)
                    print(f"已恢复原 TIMEOUT={watchdog_original}（未保存到 Flash）。")
                else:
                    print("未确认失能，保留临时通信超时；未恢复 TIMEOUT。", file=sys.stderr)
        except Exception as exc:
            completed = False
            print(f"恢复 TIMEOUT 失败：{exc}；请确认失能并检查参数。", file=sys.stderr)
        finally:
            try:
                link.close()
            finally:
                try:
                    trace.save()
                except Exception as exc:
                    print(f"无法保存诊断日志：{exc}", file=sys.stderr)
    if not completed:
        raise RuntimeError("测试未完成或未确认失能")
    print("键盘会话已结束。" if args.keyboard else "小幅往返测试完成。")


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motor", type=int, choices=range(1, 9), required=True)
    parser.add_argument("--sweep", dest="keyboard", action="store_false",
                        help="使用旧版小幅往返模式；默认是浏览器 A/D 键盘控制")
    parser.add_argument("--range-deg", type=float, default=None,
                        help="键盘相对起点的双向软范围；1–7 默认 ±10°，8 默认 ±1°")
    parser.add_argument("--delta-deg", type=float, default=None,
                        help="相对起点的偏移，可为负；1–7 默认 2°，8 默认 0.5°")
    parser.add_argument("--speed-deg", type=float, default=1.0,
                        help="参考轨迹最大速度 °/s，默认 1，上限 2")
    parser.add_argument("--kp", type=float, default=5.0)
    parser.add_argument("--kd", type=float, default=0.5)
    parser.add_argument("--watchdog-ms", type=int, default=None,
                        help="临时设置 250–1000 ms 通信超时，确认失能后恢复，不存 Flash")
    parser.add_argument("--execute", action="store_true",
                        help="实际连接并运动；不加此项只打印计划")
    return parser


def validate(args):
    if args.range_deg is None:
        args.range_deg = 1.0 if args.motor == 8 else 10.0
    if args.delta_deg is None:
        args.delta_deg = 0.5 if args.motor == 8 else 2.0
    values = [args.delta_deg, args.speed_deg, args.kp, args.kd, args.range_deg]
    if not all(math.isfinite(x) for x in values):
        raise ValueError("参数必须是有限数值")
    cap = 1.0 if args.motor == 8 else 5.0
    if not 0.1 <= abs(args.delta_deg) <= cap:
        raise ValueError(f"偏移绝对值必须在 0.1–{cap}° 之间")
    if not 0.1 <= args.speed_deg <= 2.0:
        raise ValueError("参考速度必须在 0.1–2°/s 之间")
    if not 0 < args.kp <= 5 or not 0 < args.kd <= 0.5:
        raise ValueError("测试增益范围：0 < kp <= 5，0 < kd <= 0.5")
    if not 0.1 <= args.range_deg <= (2.0 if args.motor == 8 else 30.0):
        raise ValueError("键盘范围：1–7 号最多 ±30°，8 号最多 ±2°")
    if args.watchdog_ms is not None:
        watchdog_ticks(args.watchdog_ms)
    return args


def main(argv=None):
    parser = build_parser()
    try:
        args = validate(parser.parse_args(argv))
    except ValueError as exc:
        parser.error(str(exc))
    mode = (f"A/D 键盘控制，软范围 ±{args.range_deg}°" if args.keyboard else
            f"偏移 {args.delta_deg}° 后返回")
    print(f"电机 {args.motor} | {mode} | 参考速度 ≤ {args.speed_deg}°/s | kp={args.kp}, kd={args.kd}")
    if args.watchdog_ms is not None:
        print(f"运行时将临时设置 {args.watchdog_ms} ms 通信超时，确认失能后恢复。")
    if not args.execute:
        print("仅预览：未访问 USB。实际测试需显式添加 --execute。")
        return 0
    print("请确保整条机械臂有机械支撑、运动方向有余量，且没有其他控制程序。\n"
          "本脚本不做重力补偿；其他电机不受控，退出后选中电机也会失能。\n"
          "软件不是硬件限速；USB 断开时失能指令可能无法到达，物理急停须可用。")
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        if args.keyboard:
            from openarm_keyboard import Panel
            with Panel(args) as panel:
                args.panel = panel
                panel.wait_ready()
                execute(args)
        else:
            execute(args)
    except KeyboardInterrupt:
        print("测试已中断。", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"停止：{exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
