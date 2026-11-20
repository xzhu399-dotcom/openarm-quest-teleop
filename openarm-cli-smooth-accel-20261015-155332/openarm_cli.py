"""Serial-bound OpenArm terminal tests. Import/preview never opens USB.

Console can hold both arms; other commands select one arm. No homing, mode changes,
flash writes, gravity compensation or collision planning. Saved poses are taught
encoder positions, not model-space zeroes. Protocol uses the existing tested Link.
"""
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import json
import math
import os
from pathlib import Path
import re
import select
import signal
import struct
import sys
import tempfile
import termios
import time
import tty

from openarm_motor_test import Link, Limits, SessionLog, mit_frame, watchdog_ticks

ROOT = Path(__file__).resolve().parent
CONFIG = ROOT / 'openarm_cli_config.json'
POSE_JOINTS = tuple(range(1, 8))
SPEED_MIN, SPEED_MAX = 0.1, 5.0
HAND_SPEED_MAX = 10.0  # Test-program ceiling, not a certified mechanical limit.
ARM_ACCEL = 2.5  # Reference acceleration, degrees/s²; J8 uses 5 degrees/s².
KEY_LEASE = 0.18
MAX_CYCLE = 0.2


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def speed_limit(motor):
    return HAND_SPEED_MAX if motor == 8 else SPEED_MAX


def motor_speed(motor, speed):
    """Resolve a scalar or per-motor speed; reject missing entries, never guess."""
    if isinstance(speed, dict):
        if motor not in speed:
            raise ValueError(f'J{motor} 缺少速度设置')
        value = speed[motor]
    else:
        value = speed
    if not finite(value) or not SPEED_MIN <= value <= speed_limit(motor):
        raise ValueError(f'J{motor} 测试速度必须为 {SPEED_MIN}–{speed_limit(motor):g}°/s')
    return value


def load_config(path):
    config = json.loads(Path(path).read_text(encoding='utf-8'))
    if config.get('version') != 1:
        raise ValueError('不支持的配置版本')
    serials = []
    for side in ('left', 'right'):
        arm = config['arms'][side]
        serial = arm['serial']
        if not isinstance(serial, str) or not re.fullmatch(r'[A-Za-z0-9_-]+', serial):
            raise ValueError('无效的 CANable 序列号')
        serials.append(serial)
        if not isinstance(arm['poses'], dict) or not isinstance(arm['limits_deg'], dict):
            raise ValueError('姿态/限位配置必须是对象')
        for joint, bounds in arm['limits_deg'].items():
            if (joint not in [str(i) for i in range(1, 9)] or not isinstance(bounds, list)
                    or len(bounds) != 2 or not all(finite(x) for x in bounds)
                    or bounds[0] >= bounds[1]):
                raise ValueError(f'{side} 关节 {joint} 限位无效')
    if len(set(serials)) != 2:
        raise ValueError('左右臂不得使用相同序列号')
    return config


def save_pose(path, side, serial, name, pose, replace=False):
    # Lock/reload before merging, so recording the other arm cannot lose updates.
    path = Path(path).resolve()
    with open(str(path) + '.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        config = load_config(path)
        arm = config['arms'][side]
        if arm['serial'] != serial:
            raise RuntimeError('记录期间设备绑定改变，取消保存')
        if name in arm['poses'] and not replace:
            raise ValueError('姿态已存在；重新记录需加 --replace')
        arm['poses'][name] = pose
        temp_path = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                             prefix='.openarm-pose-', delete=False) as tmp:
                temp_path = tmp.name
                json.dump(config, tmp, ensure_ascii=False, indent=2, allow_nan=False)
                tmp.write('\n')
                tmp.flush()
                os.fsync(tmp.fileno())
            os.replace(temp_path, path)
        finally:
            if temp_path and os.path.exists(temp_path):
                os.unlink(temp_path)


def read_pose(arm, name):
    if name not in arm['poses']:
        raise ValueError(f'尚未记录姿态 {name}；先用 record --name {name}')
    pose = arm['poses'][name]
    if pose.get('serial') != arm['serial'] or pose.get('joints') != list(POSE_JOINTS):
        raise ValueError(f'姿态 {name} 的设备绑定或关节列表不匹配')
    q = pose['positions_rad']
    if set(q) != {str(i) for i in POSE_JOINTS} or not all(finite(x) for x in q.values()):
        raise ValueError(f'姿态 {name} 的角度数据不完整或无效')
    return pose


@contextmanager
def arm_lock(serial):
    folder = ROOT / '.openarm-cli-locks'
    folder.mkdir(exist_ok=True)
    with open(folder / (serial + '.lock'), 'a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('该手臂已有一个 CLI 会话，请先退出')
        yield


@contextmanager
def interrupt_handlers():
    def stop(signum, frame):
        raise KeyboardInterrupt
    signals = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGQUIT, signal.SIGTSTP)
    old = {s: signal.signal(s, stop) for s in signals}
    try:
        yield
    finally:
        for sig, handler in old.items():
            signal.signal(sig, handler)


class Terminal:
    """Bounded cbreak input. Standard terminals do not report key-up events."""
    def __enter__(self):
        if not sys.stdin.isatty():
            raise RuntimeError('运动控制需要交互式终端，不能通过管道或重定向运行')
        self.fd = sys.stdin.fileno()
        self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        termios.tcflush(self.fd, termios.TCIFLUSH)
        return self

    def poll(self):
        if not select.select([self.fd], [], [], 0)[0]:
            return ''
        data = os.read(self.fd, 256)
        if not data:
            raise KeyboardInterrupt
        if len(data) >= 256:
            raise RuntimeError('终端输入积压，停止')
        return data.decode('ascii', errors='ignore').lower()

    def __exit__(self, *unused):
        termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)


class JogInput:
    def __init__(self):
        self.direction = 0
        self.expires = 0.0

    def update(self, keys, now):
        # Stop wins over any buffered movement key in the same read.
        if ' ' in keys or 's' in keys:
            self.direction = 0
            self.expires = now
        else:
            for key in keys:
                if key in 'ad':
                    self.direction = 1 if key == 'a' else -1
                    self.expires = now + KEY_LEASE

    def sample(self, now):
        return self.direction if now < self.expires else 0


def reference_step(q, direction, dt, speed, bounds):
    if not 0 <= dt <= MAX_CYCLE:
        raise RuntimeError('控制循环延迟超过 200 ms')
    return min(bounds[1], max(bounds[0], q + direction * math.radians(speed) * dt))


class JogRamp:
    """Ramp the reference velocity; deadman/stop never adds braking travel.

    Velocity is in degrees/s. A reversal drops the old ramp and starts from
    zero in the requested direction. This shapes commands, not motor dynamics.
    """
    def __init__(self):
        self.velocity = 0.0

    def reset(self):
        self.velocity = 0.0

    def step(self, q, direction, dt, speed, bounds, motor):
        speed = motor_speed(motor, speed)
        if not finite(dt) or not 0 <= dt <= MAX_CYCLE:
            raise RuntimeError('控制循环延迟超过 200 ms')
        if direction == 0:
            self.reset()
            return q
        if direction not in (-1, 1):
            raise ValueError('无效运动方向')
        if self.velocity * direction < 0:
            self.reset()
        # Lowering the selected cap applies immediately, including mid-ramp.
        initial = min(abs(self.velocity), speed)
        acceleration = ARM_ACCEL * (2 if motor == 8 else 1)
        ramp_time = min(dt, (speed - initial) / acceleration)
        final = initial + acceleration * ramp_time
        distance = (initial + final) * .5 * ramp_time + final * (dt - ramp_time)
        self.velocity = direction * final
        requested = q + direction * math.radians(distance)
        target = min(bounds[1], max(bounds[0], requested))
        if target != requested:
            self.reset()
        return target


def pose_reference(start, target, elapsed, speed):
    """Quintic interpolation with bounded reference speed and acceleration."""
    distance = max(abs(target[i] - start[i]) for i in start)
    # Peak normalized quintic acceleration is 10 / sqrt(3).
    duration = max(1.0, 1.875 * distance / math.radians(speed),
                   math.sqrt((10 / math.sqrt(3)) * distance / math.radians(ARM_ACCEL)))
    t = min(1.0, max(0.0, elapsed / duration))
    s = 10 * t**3 - 15 * t**4 + 6 * t**5
    return {i: start[i] + (target[i] - start[i]) * s for i in start}, t == 1.0


def frame_for(motor, metadata, target, speed, factor=1.0):
    limits = metadata['limits']
    if not finite(target) or not -limits.p + .01 < target < limits.p - .01:
        raise ValueError('目标超出电机编码范围')
    speed = motor_speed(motor, speed)
    if metadata['mode'] == 2:
        return motor + 0x100, struct.pack('<ff', target, math.radians(speed))
    if metadata['mode'] == 1:
        return motor, mit_frame(target, 5 * factor, .5 * factor, limits)
    raise RuntimeError('仅支持已有 MIT=1 或位置速度=2 模式，不自动切换')


def check_feedback(motor, meta, state, reference, speed, enabled):
    speed = motor_speed(motor, speed)
    if not all(finite(state[k]) for k in ('q', 'v', 'temperature')):
        raise RuntimeError(f'J{motor} 非有限反馈值')
    if state['status'] != (1 if enabled else 0):
        raise RuntimeError(f'J{motor} 状态异常：0x{state["status"]:x}')
    if state['temperature'] >= 60:
        raise RuntimeError(f'J{motor} 温度达到 60°C')
    tolerance = 2 * meta['limits'].v / 4095
    threshold = math.radians(max(5.0, 2 * speed) if enabled else 3.0) + tolerance
    if abs(state['v']) > threshold:
        raise RuntimeError(f'J{motor} 速度保护：反馈 {math.degrees(state["v"]):.3f}°/s，'
                           f'阈值 {math.degrees(threshold):.3f}°/s；帧 {state.get("raw_hex")}')
    if abs(state['q'] - reference) > math.radians(3):
        raise RuntimeError(f'J{motor} 跟踪误差超过 3°')
    lo, hi = meta['bounds']
    if not lo - math.radians(.1) <= state['q'] <= hi + math.radians(.1):
        raise RuntimeError(f'J{motor} 超出配置限位/编码范围')


class Session:
    def __init__(self, arm, side, joints):
        self.arm, self.side, self.joints = arm, side, tuple(joints)
        self.link = None
        self.meta, self.states, self.refs = {}, {}, {}
        self.feedback_ns = {}
        self.changed, self.attempted = [], []
        self.trace = SessionLog(side + '-' + '-'.join(map(str, joints)))
        self.speed = 1.0

    def __enter__(self):
        try:
            self.link = Link(serial=self.arm['serial'])
            self.link.trace = self.trace
            self.trace.record('arm_binding', side=self.side, serial=self.arm['serial'])
            for motor in self.joints:
                reply = self.link.param(motor, 7)
                if not 0 < reply < 0x7ff or reply in [m['reply'] for m in self.meta.values()]:
                    raise RuntimeError('反馈 CAN ID 无效或重复')
                if self.link.param(motor, 8) != motor:
                    raise RuntimeError(f'J{motor} ID 校验失败')
                mode = self.link.param(motor, 10)
                limits = Limits(*(self.link.param(motor, r, True) for r in (21, 22, 23)))
                if not all(finite(v) and v > 0 for v in vars(limits).values()):
                    raise RuntimeError('无效的反馈映射参数')
                bounds = (-limits.p + .011, limits.p - .011)
                configured = self.arm['limits_deg'].get(str(motor))
                if configured:
                    bounds = (max(bounds[0], math.radians(configured[0])),
                              min(bounds[1], math.radians(configured[1])))
                if bounds[0] >= bounds[1]:
                    raise ValueError('机械限位与编码范围无交集')
                self.meta[motor] = dict(reply=reply, mode=mode, limits=limits, bounds=bounds,
                                        watchdog=self.link.param(motor, 9))
            self.read_states()
            self.refs = {i: self.states[i]['q'] for i in self.joints}
            return self
        except BaseException:
            if self.link:
                self.link.close()
            raise

    def read_states(self, deadline=None):
        for i in self.joints:
            self.states[i] = self.link.state(i, self.meta[i]['reply'], self.meta[i]['limits'])
            self.feedback_ns[i] = time.monotonic_ns()
            if deadline is not None and time.monotonic() > deadline:
                raise RuntimeError('整组反馈延迟超过 200 ms')
        return self.states

    def stationary(self):
        self.read_states()
        initial = {i: self.states[i]['q'] for i in self.joints}
        for i in self.joints:
            check_feedback(i, self.meta[i], self.states[i], initial[i], 1, False)
        time.sleep(.1)
        self.read_states()
        for i in self.joints:
            check_feedback(i, self.meta[i], self.states[i], initial[i], 1, False)
            if abs(initial[i] - self.states[i]['q']) > math.radians(.2):
                raise RuntimeError(f'J{i} 未静止；支撑稳定后重试')
        self.refs = {i: self.states[i]['q'] for i in self.joints}

    def prepare(self, speed, on_preload=None):
        """Prepare while disabled; all arms must finish this before any enable."""
        self.speed = speed
        self.stationary()
        for i in self.joints:
            frame_for(i, self.meta[i], self.refs[i], speed, 0)  # Validate before any writes.
        for i in self.joints:
            if self.meta[i]['watchdog'] != watchdog_ticks(500):
                self.changed.append(i)  # Even a missing ACK may follow a successful write.
                self.link.set_watchdog(i, watchdog_ticks(500))
        self.stationary()
        for i in self.joints:
            stamp = time.monotonic_ns()
            self.link.send(*frame_for(i, self.meta[i], self.refs[i], speed, 0))
            if on_preload is not None:
                on_preload(i, self.refs[i], stamp)

    def enable_prepared(self, guard=None, on_enabled=None):
        begun = time.monotonic()
        for i in self.joints:
            if guard is not None:
                guard()
            if time.monotonic() - begun > MAX_CYCLE:
                raise RuntimeError('多关节使能超时')
            self.attempted.append(i)
            stamp = time.monotonic_ns()
            self.link.send(i, b'\xff' * 7 + b'\xfc')
            if on_enabled is not None:
                on_enabled(i, stamp)
        self.trace.record('enabled_joints', joints=list(self.attempted), speed=self.speed)

    def start(self, speed):
        self.prepare(speed)
        self.enable_prepared()

    def cycle(self, targets, speed, factor):
        begun = time.monotonic()
        self.read_states(deadline=begun + MAX_CYCLE)
        self.send_references(targets, speed, factor, begun)

    def send_references(self, targets, speed, factor, begun, select_target=None, on_sent=None):
        """Send against this cycle's readbacks. A released external clutch may hold the last reference."""
        speeds = {i: motor_speed(i, speed) for i in self.joints}
        if time.monotonic() - begun > MAX_CYCLE:
            raise RuntimeError('整组反馈延迟超过 200 ms')
        for i in self.joints:
            # Compare feedback against the command already sent, not the future one.
            check_feedback(i, self.meta[i], self.states[i], self.refs[i], speeds[i], True)
            lo, hi = self.meta[i]['bounds']
            if not finite(targets[i]) or not lo <= targets[i] <= hi:
                raise RuntimeError(f'J{i} 目标超出配置限位')
        self.trace.record('reference', targets=dict(targets),
                          speed=dict(speed) if isinstance(speed, dict) else speed,
                          motor_speeds=speeds)
        for i in self.joints:
            if time.monotonic() - begun > MAX_CYCLE:
                raise RuntimeError('整组控制延迟超过 200 ms')
            target = targets[i] if select_target is None else select_target(i, targets[i], self.refs[i])
            if target != targets[i] and target != self.refs[i]:
                raise RuntimeError("A command guard may only hold the previous reference.")
            sent_ns = time.monotonic_ns()
            self.link.send(*frame_for(i, self.meta[i], target, speeds[i], factor))
            self.refs[i] = target
            if on_sent is not None:
                on_sent(i, target, sent_ns)

    def verify_pose(self, pose):
        targets = {int(i): q for i, q in pose['positions_rad'].items()}
        if not set(targets).issubset(self.meta):
            raise RuntimeError('姿态包含本次未控制的关节')
        for i in targets:
            sig = pose['motor_parameters'][str(i)]
            meta = self.meta[i]
            if sig['mode'] != meta['mode'] or sig['limits'] != vars(meta['limits']):
                raise RuntimeError(f'J{i} 模式/反馈映射已改变，请重新记录姿态')
            if not meta['bounds'][0] <= targets[i] <= meta['bounds'][1]:
                raise RuntimeError(f'J{i} 记录的角度超出当前限位')
        return targets

    def request_disable(self):
        """Send all stop requests without waiting for feedback."""
        for i in self.attempted:
            try:
                self.link.send(i, b'\xff' * 7 + b'\xfd')
            except Exception:
                pass

    def cleanup(self):
        if self.link is None:
            return True
        ok = True
        confirmed = {}
        # Send stop to EVERY attempted motor before waiting for any reply.
        self.request_disable()
        for i in self.attempted:
            confirmed[i] = False
            for attempt in range(3):
                try:
                    if attempt:
                        self.link.send(i, b'\xff' * 7 + b'\xfd')
                    state = self.link.state(i, self.meta[i]['reply'], self.meta[i]['limits'])
                    if state['status'] == 0:
                        confirmed[i] = True
                        break
                except Exception:
                    continue
            if not confirmed[i]:
                ok = False
                print(f'J{i} 未确认失能，请使用物理急停。', file=sys.stderr)
        for i in self.changed:
            try:
                disabled = confirmed.get(i)
                if disabled is None:
                    disabled = self.link.state(i, self.meta[i]['reply'], self.meta[i]['limits'])['status'] == 0
                if disabled:
                    self.link.set_watchdog(i, self.meta[i]['watchdog'])
                else:
                    ok = False
                    print(f'J{i} 未确认失能，保留临时 TIMEOUT。', file=sys.stderr)
            except Exception as exc:
                ok = False
                print(f'J{i} 恢复 TIMEOUT 失败：{exc}', file=sys.stderr)
        self.trace.record('cleanup', disabled=confirmed, success=ok)
        if self.attempted and all(confirmed.values()):
            print(f'\n{self.side} 本次所有已使能关节均已回复失能状态。')
        return ok

    def __exit__(self, exc_type, exc, tb):
        if exc:
            self.trace.record('failure', detail=str(exc), type=exc_type.__name__)
        signals = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGQUIT, signal.SIGTSTP)
        old = {s: signal.signal(s, signal.SIG_IGN) for s in signals}
        try:
            ok = self.cleanup()
        finally:
            try:
                self.link.close()
                try:
                    self.trace.save()
                except Exception as error:
                    print(f'日志保存失败：{error}', file=sys.stderr)
            finally:
                for sig, handler in old.items():
                    signal.signal(sig, handler)
        if not ok and exc is None:
            raise RuntimeError('退出清理未完全成功，请检查失能/超时参数提示')


def print_states(session):
    for i, state in session.states.items():
        print(f'J{i}: {math.degrees(state["q"]):+.3f}° | '
              f'{math.degrees(state["v"]):+.2f}°/s | {state["temperature"]}°C | '
              f'状态 0x{state["status"]:x} | 模式 {session.meta[i]["mode"]}')


def record(session, path, side, name, replace):
    session.stationary()
    pose = dict(serial=session.arm['serial'], joints=list(POSE_JOINTS),
                recorded_utc=datetime.now(timezone.utc).isoformat(),
                positions_rad={str(i): session.states[i]['q'] for i in POSE_JOINTS},
                motor_parameters={str(i): dict(mode=session.meta[i]['mode'],
                                               limits=vars(session.meta[i]['limits'])) for i in POSE_JOINTS})
    save_pose(path, side, session.arm['serial'], name, pose, replace)
    print_states(session)
    print(f'已保存 {side}/{name} 到 {path}；没有使能、归零或写 Flash。')


def control(session, terminal, args, waypoints):
    pending = terminal.poll()
    if any(k in pending for k in ('q', '\x1b', '\x03', '\x04')):
        return
    session.start(args.speed)
    direction = JogInput()
    ramp = JogRamp()
    speed = args.speed
    begun = previous = last_print = time.monotonic()
    start = dict(session.refs)
    elapsed, waypoint, settled = 0.0, 0, 0.0
    paused = False
    done = False
    print('A/D 按键重复续期；空格停止/暂停；回车继续动作；Q/Esc 退出并失能。')
    if args.command == 'jog':
        print('按 + / - 调整速度，每次 0.1°/s。松键识别最多约 180 ms 加一个控制周期。')
    while True:
        now = time.monotonic()
        dt, previous = now - previous, now
        if dt > MAX_CYCLE:
            raise RuntimeError('控制周期超过 200 ms')
        keys = terminal.poll()
        if any(k in keys for k in ('q', '\x1b', '\x03', '\x04')):
            return
        factor = min(1.0, (now - begun) / 1.0)
        targets = dict(session.refs)
        if args.command == 'jog':
            for key in keys:
                if key in '+=':
                    speed = min(speed_limit(args.joint), round(speed + .1, 2))
                elif key == '-':
                    speed = max(SPEED_MIN, round(speed - .1, 2))
            if factor < 1:
                direction.update(' ', now)
            else:
                if not direction.sample(now):
                    ramp.reset()
                direction.update(keys, now)
            i = args.joint
            bounds = session.meta[i]['bounds']
            if args.range_deg is not None:
                travel = math.radians(args.range_deg)
                bounds = max(bounds[0], start[i] - travel), min(bounds[1], start[i] + travel)
            targets[i] = ramp.step(session.refs[i], direction.sample(now), dt, speed, bounds, i)
        else:
            if ' ' in keys or 's' in keys:
                paused = True
            elif '\r' in keys or '\n' in keys:
                paused = False
            if factor == 1 and not paused and not done:
                elapsed += dt
                targets, reached = pose_reference(start, waypoints[waypoint], elapsed, speed)
                if reached:
                    settled += dt
                    if settled >= 1.0:
                        if any(abs(session.states[i]['q'] - targets[i]) > math.radians(.5)
                               for i in session.joints):
                            raise RuntimeError('姿态到位误差超过 0.5°')
                        print(f'\n已到达姿态：{args.names[waypoint]}')
                        waypoint += 1
                        start, elapsed, settled = dict(targets), 0.0, 0.0
                        if waypoint == len(waypoints):
                            done = True
                            print('动作完成，正在保持；支撑机械臂后按 Q 失能退出。')
        session.cycle(targets, speed, factor)
        if now - last_print >= .5:
            status = '保持' if done else '暂停' if paused else '控制'
            positions = ' '.join(f'J{i}:{math.degrees(session.states[i]["q"]):+.1f}°' for i in session.joints)
            print(f'\r{session.side} | {speed:.1f}°/s | {status} | {positions}      ', end='', flush=True)
            last_print = now
        time.sleep(max(0, (.02 if len(session.joints) == 1 else .05) - (time.monotonic() - now)))


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--arm', choices=('left', 'right', 'both'), required=True)
    p.add_argument('--config', type=Path, default=CONFIG)
    sub = p.add_subparsers(dest='command', required=True)
    sub.add_parser('status', help='只读检查 1–8 号电机')
    sub.add_parser('poses', help='列出已记录姿态，不连接 USB')
    rec = sub.add_parser('record', help='只读记录已支撑、静止且失能的 1–7 号关节')
    rec.add_argument('--name', required=True, help='例如 vertical 或 forearm-raised')
    rec.add_argument('--replace', action='store_true')
    jog = sub.add_parser('jog', help='独立单电机测试（不保持其他电机）；整臂保持请用 console')
    jog.add_argument('--joint', type=int, choices=range(1, 9), required=True)
    jog.add_argument('--range-deg', type=float, help='可选的相对起点双向范围；默认不设小角度窗口')
    play = sub.add_parser('play', help='依次回放本臂已记录姿态，完成后保持')
    play.add_argument('names', nargs='+', help='例如 vertical forearm-raised vertical')
    console = sub.add_parser('console', help='持续控制台：全部关节保持，随时切换手臂/关节/姿态')
    console.add_argument('--hand-speed', type=float, default=10.0,
                         help='两侧 J8 的最高参考速度，0.1–10°/s；默认 10')
    for cmd in (jog, play, console):
        cmd.add_argument('--speed', type=float, default=1.0,
                         help='jog/play 默认 1°/s，console 默认 5；1–7 号/姿态上限 5，J8 上限 10')
        cmd.add_argument('--execute', action='store_true', help='允许实际使能；不加则只预览')
    console.set_defaults(speed=5.0)
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        config = load_config(args.config)
        if args.command == 'console':
            from openarm_console import run_console
            return run_console(args, config)
        if args.arm == 'both':
            raise ValueError('--arm both 目前用于 console；其他命令请选择 left 或 right')
        arm = config['arms'][args.arm]
        print(f'{args.arm} → CANable {arm["serial"]}')
        if args.command == 'poses':
            if not arm['poses']:
                print('尚未记录姿态。')
            for name in arm['poses']:
                pose = read_pose(arm, name)
                angles = {i: round(math.degrees(q), 3) for i, q in pose['positions_rad'].items()}
                print(f'{name}: {angles} °')
            return 0
        motion = args.command in ('jog', 'play')
        poses = []
        if motion:
            motor_speed(args.joint if args.command == 'jog' else 1, args.speed)
            if args.command == 'jog' and args.range_deg is not None:
                if not finite(args.range_deg) or args.range_deg <= 0:
                    raise ValueError('可选相对范围必须是正的有限角度')
            if args.command == 'play':
                poses = [read_pose(arm, name) for name in args.names]
                print('动作路径：当前姿态 → ' + ' → '.join(args.names))
                for name, pose in zip(args.names, poses):
                    print(name, {i: round(math.degrees(q), 2) for i, q in pose['positions_rad'].items()})
            else:
                print(f'单独控制 J{args.joint}；其他电机不受控。')
            print(f'参考速度 {args.speed}°/s；通信超时临时设为 500 ms。')
            if not args.execute:
                print('仅预览，未连接 USB。加 --execute 才会实际使能。')
                return 0
            if not sys.stdin.isatty():
                raise RuntimeError('请在交互式终端执行')
            print('请支撑机械臂并保持急停可用；退出会失能。没有重力补偿或碰撞规划。')
            if not arm['limits_deg']:
                print('尚未配置机械限位；编码范围不代表机械可转范围，请观察线束和关节。')
        if args.command == 'record':
            if not re.fullmatch(r'[a-zA-Z0-9_-]{1,64}', args.name):
                raise ValueError('姿态名仅允许 1–64 个英文字母、数字、下划线或短横线')
            if args.name in arm['poses'] and not args.replace:
                raise ValueError('姿态已存在，重新记录请加 --replace')
        joints = (args.joint,) if args.command == 'jog' else range(1, 9) if args.command == 'status' else POSE_JOINTS
        with arm_lock(arm['serial']), interrupt_handlers(), Session(arm, args.arm, joints) as session:
            if args.command == 'status':
                print_states(session)
            elif args.command == 'record':
                record(session, args.config, args.arm, args.name, args.replace)
            else:
                targets = [session.verify_pose(pose) for pose in poses]
                with Terminal() as terminal:
                    control(session, terminal, args, targets)
        return 0
    except KeyboardInterrupt:
        print('\n已中断。', file=sys.stderr)
        return 130
    except Exception as exc:
        print(f'停止：{exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
