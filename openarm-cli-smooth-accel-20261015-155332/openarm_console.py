"""Persistent terminal console. No hardware access on import.

Every selected arm's eight motors remain in position control. Disk writes are
off the control loop; USB access stays on one thread. No autonomous hardware run.
"""
from contextlib import ExitStack
import copy
from datetime import datetime, timezone
import math
import queue
import signal
import sys
import threading
import time

import openarm_cli as cli


HELP = """
L / R    选择左 / 右臂（仅切换操作对象，其他关节继续保持）
1–8      选择关节；切换后先松开 A/D，再重新按
A / D    正 / 反向；松键超时停止增加目标
+ / -    调速 0.1°/s；选中 J8 时只调整该手部速度，1–7 号与姿态速度独立
空格 / S 取消动作，所有关节保持最后目标
P / 回车 暂停 / 继续姿态回放
Z / X    记录当前臂的竖直 / 小臂抬起姿态；覆盖已有姿态需按 Y
V / F    到已记录的竖直 / 小臂抬起姿态
B        依次执行竖直 → 小臂抬起
H        显示帮助
Q / Esc  结束整个控制台并失能全部参与关节（请先支撑机械臂）
"""


class PoseWriter:
    """A slow disk or config lock must not interrupt motor holding commands."""
    def __init__(self, path):
        self.path = path
        self.thread = None
        self.results = queue.Queue()

    def submit(self, side, serial, name, pose, replace):
        if self.thread and self.thread.is_alive():
            raise ValueError('上次姿态仍在保存，请稍后重试')
        payload = copy.deepcopy(pose)
        def write():
            error = None
            try:
                cli.save_pose(self.path, side, serial, name, payload, replace)
            except Exception as exc:
                error = str(exc)
            self.results.put((side, name, payload, error))
        self.thread = threading.Thread(target=write, daemon=True)
        self.thread.start()

    def poll(self):
        try:
            return self.results.get_nowait()
        except queue.Empty:
            return None

    def finish(self):
        if self.thread:
            self.thread.join(timeout=1)
            if self.thread.is_alive():
                print('\n姿态保存仍未完成；本次未确认保存成功。', file=sys.stderr)


class ConsoleState:
    def __init__(self, sessions, speed, now, hand_speed=None):
        self.sessions = sessions
        self.side = next(iter(sessions))
        self.joint = 1
        self.speed = speed
        self.hand_speeds = {s: cli.motor_speed(8, speed if hand_speed is None else hand_speed)
                            for s in sessions}
        self.targets = {s: dict(session.refs) for s, session in sessions.items()}
        self.direction = cli.JogInput()
        self.ramp = cli.JogRamp()
        self.needs_release = True
        self.release_after = now + cli.KEY_LEASE
        self.last_motion = {s: now for s in sessions}
        self.plan = None
        self.paused = False
        self.pending_save = None

    @property
    def selected_speed(self):
        return self.hand_speeds[self.side] if self.joint == 8 else self.speed

    def speeds_for(self, side):
        return {i: self.hand_speeds[side] if i == 8 else self.speed
                for i in self.sessions[side].joints}

    def stop(self, now):
        self.plan = None
        self.paused = False
        self.direction.update(' ', now)
        self.ramp.reset()

    def switch(self, side, joint, now):
        if side not in self.sessions:
            raise ValueError(f'{side} 未参与本次控制台')
        self.stop(now)
        self.side, self.joint = side, joint
        self.pending_save = None
        self.needs_release = True
        self.release_after = now + cli.KEY_LEASE

    def begin_pose(self, names, now):
        self.stop(now)
        session = self.sessions[self.side]
        goals = []
        for name in names:
            pose = cli.read_pose(session.arm, name)
            target = session.verify_pose(pose)
            # Poses never open/close the gripper; J8 keeps its existing target.
            goal = dict(self.targets[self.side])
            goal.update(target)
            goals.append(goal)
        self.plan = dict(side=self.side, names=names, goals=goals, index=0,
                         start=dict(self.targets[self.side]), elapsed=0.0,
                         settled=0.0, base_speed=self.speed)

    def restart_pose_ramp(self):
        if self.plan:
            self.plan.update(start=dict(self.targets[self.plan['side']]),
                             elapsed=0.0, settled=0.0, base_speed=self.speed)

    def pose_snapshot(self, now):
        side, session = self.side, self.sessions[self.side]
        if self.plan or self.direction.sample(now) or now - self.last_motion[side] < .5:
            raise ValueError('先按空格停止，等待至少 0.5 秒稳定后再记录')
        for i in cli.POSE_JOINTS:
            state, meta = session.states[i], session.meta[i]
            cli.check_feedback(i, meta, state, self.targets[side][i], self.speed, True)
            if abs(state['q'] - self.targets[side][i]) > math.radians(.5):
                raise ValueError(f'J{i} 尚未稳定到目标位置')
            if abs(state['v']) > math.radians(3) + 2 * meta['limits'].v / 4095:
                raise ValueError(f'J{i} 仍在移动')
        return dict(serial=session.arm['serial'], joints=list(cli.POSE_JOINTS),
                    recorded_utc=datetime.now(timezone.utc).isoformat(),
                    positions_rad={str(i): session.states[i]['q'] for i in cli.POSE_JOINTS},
                    motor_parameters={str(i): dict(mode=session.meta[i]['mode'],
                                                   limits=vars(session.meta[i]['limits'])) for i in cli.POSE_JOINTS})

    def handle(self, keys, now, writer):
        if self.pending_save and now > self.pending_save[-1]:
            self.pending_save = None
        if ' ' in keys or 's' in keys:
            self.stop(now)
            self.pending_save = None
            return
        # Selection takes precedence over any buffered direction key.
        new_side, new_joint = self.side, self.joint
        for key in keys:
            if key in 'lr':
                new_side = 'left' if key == 'l' else 'right'
            elif key in '12345678':
                new_joint = int(key)
        if new_side != self.side or new_joint != self.joint:
            self.switch(new_side, new_joint, now)
            return
        previous_speed = self.speed
        for key in keys:
            if key in '+=-':
                value = min(cli.speed_limit(self.joint), max(cli.SPEED_MIN,
                            round(self.selected_speed + (-.1 if key == '-' else .1), 2)))
                if self.joint == 8:
                    self.hand_speeds[self.side] = value
                else:
                    self.speed = value
        if self.speed != previous_speed:
            self.restart_pose_ramp()
        if 'h' in keys:
            print(HELP)
        if 'y' in keys and self.pending_save:
            side, name, pose, _ = self.pending_save
            writer.submit(side, self.sessions[side].arm['serial'], name, pose, True)
            self.pending_save = None
            print(f'\n正在保存 {side}/{name}，全部关节继续保持。')
            return
        if 'z' in keys or 'x' in keys:
            name = 'vertical' if 'z' in keys else 'forearm-raised'
            pose = self.pose_snapshot(now)
            if name in self.sessions[self.side].arm['poses']:
                self.pending_save = (self.side, name, pose, now + 5)
                print(f'\n将覆盖 {self.side}/{name}；5 秒内按 Y 确认，空格取消。')
            else:
                writer.submit(self.side, self.sessions[self.side].arm['serial'], name, pose, False)
                print(f'\n正在保存 {self.side}/{name}，全部关节继续保持。')
            return
        if self.pending_save and keys:
            self.pending_save = None
        if 'v' in keys or 'f' in keys or 'b' in keys:
            names = ['vertical', 'forearm-raised'] if 'b' in keys else ['vertical' if 'v' in keys else 'forearm-raised']
            self.begin_pose(names, now)
            print(f'\n{self.side}：当前 → ' + ' → '.join(names))
            return
        if 'p' in keys:
            self.paused = True
            self.direction.update(' ', now)
            self.ramp.reset()
        elif '\r' in keys or '\n' in keys:
            if self.paused:
                self.restart_pose_ramp()
            self.paused = False
        if self.needs_release:
            if 'a' in keys or 'd' in keys:
                self.release_after = now + cli.KEY_LEASE
            elif now >= self.release_after:
                self.needs_release = False
            return
        if 'a' in keys or 'd' in keys:
            if self.plan or not self.direction.sample(now):
                self.ramp.reset()
            self.plan = None
            self.paused = False
            self.direction.update(keys, now)

    def advance(self, now, dt):
        if not 0 <= dt <= cli.MAX_CYCLE:
            raise RuntimeError('控制台周期超过 200 ms')
        before = {s: dict(q) for s, q in self.targets.items()}
        if self.plan and not self.paused:
            p = self.plan
            # Restart from the held target on resume or a new speed selection.
            if p['base_speed'] != self.speed:
                self.restart_pose_ramp()
            p['elapsed'] += dt
            target, reached = cli.pose_reference(p['start'], p['goals'][p['index']], p['elapsed'], p['base_speed'])
            self.targets[p['side']] = target
            if reached:
                p['settled'] += dt
                if p['settled'] >= 1:
                    session = self.sessions[p['side']]
                    if any(abs(session.states[i]['q'] - target[i]) > math.radians(.5) for i in cli.POSE_JOINTS):
                        raise RuntimeError('姿态到位误差超过 0.5°')
                    print(f'\n已到达 {p["side"]}/{p["names"][p["index"]]}，其余关节继续保持。')
                    p['index'] += 1
                    if p['index'] == len(p['goals']):
                        self.plan = None
                    else:
                        p.update(start=dict(target), elapsed=0.0, settled=0.0)
        elif not self.plan and not self.needs_release:
            i, s = self.joint, self.side
            self.targets[s][i] = self.ramp.step(self.targets[s][i], self.direction.sample(now), dt,
                                               self.selected_speed, self.sessions[s].meta[i]['bounds'], i)
        for side in self.targets:
            if self.targets[side] != before[side]:
                self.last_motion[side] = now


def stop_all(sessions):
    """Stop both arms before any slow confirmation/parameter restoration."""
    signals = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGQUIT, signal.SIGTSTP)
    old = {s: signal.signal(s, signal.SIG_IGN) for s in signals}
    try:
        for session in sessions.values():
            session.request_disable()
    finally:
        for sig, handler in old.items():
            signal.signal(sig, handler)


def prepare_all(sessions, speed, hand_speed=None):
    for side, session in sessions.items():
        for i, meta in session.meta.items():
            if meta['mode'] != 2:
                raise RuntimeError(f'{side} J{i} 当前模式 {meta["mode"]}；全关节保持控制台要求位置速度模式 2，未自动切换')
    for session in sessions.values():
        speeds = speed if hand_speed is None else {i: hand_speed if i == 8 else speed for i in session.joints}
        session.prepare(speeds)
    # Preparation of another arm can take time; refresh every origin while disabled.
    for session in sessions.values():
        session.stationary()
        for i in session.joints:
            value = hand_speed if i == 8 and hand_speed is not None else speed
            session.link.send(*cli.frame_for(i, session.meta[i], session.refs[i], value))
    begun = time.monotonic()
    for session in sessions.values():
        if time.monotonic() - begun > cli.MAX_CYCLE:
            raise RuntimeError('双臂使能总耗时超过 200 ms')
        session.enable_prepared()
    if time.monotonic() - begun > cli.MAX_CYCLE:
        raise RuntimeError('双臂使能总耗时超过 200 ms')


def consume_save(writer, sessions, holding=True):
    result = writer.poll()
    if result:
        side, name, pose, error = result
        if error:
            print(f'\n保存失败：{error}' + ('；控制仍在保持。' if holding else '。'))
        else:
            sessions[side].arm['poses'][name] = pose
            print(f'\n已保存 {side}/{name}。')


def console_loop(sessions, terminal, writer, args):
    pending = terminal.poll()
    if any(k in pending for k in ('q', '\x1b', '\x03', '\x04')):
        return
    print(HELP)
    hand_speed = getattr(args, 'hand_speed', None)
    prepare_all(sessions, args.speed, hand_speed)
    model = ConsoleState(sessions, args.speed, time.monotonic(), hand_speed)
    previous = last_print = time.monotonic()
    reported_ready = False
    while True:
        now = time.monotonic()
        dt, previous = now - previous, now
        keys = terminal.poll()
        if any(k in keys for k in ('q', '\x1b', '\x03', '\x04')):
            return
        consume_save(writer, sessions)
        try:
            model.handle(keys, now, writer)
        except (ValueError, KeyError) as exc:
            model.stop(now)
            print(f'\n操作未执行：{exc}；全部关节继续保持。')
        model.advance(now, dt)
        for side, session in sessions.items():
            if time.monotonic() - now > cli.MAX_CYCLE:
                raise RuntimeError('双臂控制周期超过 200 ms')
            session.cycle(model.targets[side], model.speeds_for(side), 1.0)
        if time.monotonic() - now > cli.MAX_CYCLE:
            raise RuntimeError('双臂控制周期超过 200 ms')
        if not reported_ready:
            print('全部参与关节已反馈使能状态，正在保持；切换选择不会失能。')
            reported_ready = True
        if now - last_print >= .5:
            state = sessions[model.side].states[model.joint]
            mode = '等待松开 A/D' if model.needs_release else '动作暂停' if model.paused else '姿态回放' if model.plan else '手动/保持'
            print(f'\r选中 {model.side} J{model.joint} | {model.selected_speed:.1f}°/s | '
                  f'{math.degrees(state["q"]):+.2f}° | {mode} | '
                  f'{len(sessions) * 8} 个关节持续保持       ', end='', flush=True)
            last_print = now
        time.sleep(max(0, .05 - (time.monotonic() - now)))


def run_console(args, config):
    cli.motor_speed(1, args.speed)
    hand_speed = getattr(args, 'hand_speed', None)
    cli.motor_speed(8, args.speed if hand_speed is None else hand_speed)
    sides = ('left', 'right') if args.arm == 'both' else (args.arm,)
    for side in sides:
        print(f'{side} → {config["arms"][side]["serial"]}，计划让 1–8 号保持当前位置（尚未使能）')
    print(f'初始参考速度：J1–J7/姿态 {args.speed}°/s，J8 {args.speed if hand_speed is None else hand_speed}°/s；'
          '本控制台要求各关节为位置速度模式 2。')
    print('A/D 平滑加速：J1–J7 为 2.5°/s²，J8 为 5°/s²；保留 3° 跟踪误差保护。')
    if not args.execute:
        print(HELP)
        print('仅预览，未连接 USB。加 --execute 才会使能全部参与关节。')
        return 0
    if not sys.stdin.isatty():
        raise RuntimeError('请在交互式终端执行')
    print('请先支撑好机械臂，保持急停可用；通电保持不是机械锁，结束/故障会失能。')
    print('没有重力补偿或碰撞规划；尚未标定的机械范围需人工观察。')
    sessions = {}
    writer = PoseWriter(args.config)
    try:
        with cli.interrupt_handlers(), ExitStack() as stack:
            for side in sides:
                stack.enter_context(cli.arm_lock(config['arms'][side]['serial']))
            for side in sides:
                serial = config['arms'][side]['serial']
                print(f'正在只读检查 {side}（CANable {serial}）的 1–8 号电机……')
                try:
                    sessions[side] = stack.enter_context(cli.Session(config['arms'][side], side, range(1, 9)))
                except Exception as exc:
                    raise RuntimeError(f'{side}（CANable {serial}）初始化失败：{exc}；'
                                       '本次控制台尚未发送任何电机使能指令') from exc
            try:
                with cli.Terminal() as terminal:
                    console_loop(sessions, terminal, writer, args)
            finally:
                stop_all(sessions)
    finally:
        # Disk completion is awaited only AFTER all hardware has been shut down.
        writer.finish()
        consume_save(writer, sessions, holding=False)
    return 0
