"""Local browser key-down/key-up control; imported only for keyboard testing.

No USB access at import. The UI is created only by Panel.__enter__.
"""
import json
import math
import secrets
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


HTML = """<!doctype html><html lang="zh"><meta charset="utf-8">
<title>OpenArm 单电机键盘测试</title>
<style>body{font:18px system-ui;max-width:740px;margin:60px auto;padding:20px;background:#f4f6fa;color:#192735}
button{font:inherit;padding:14px 24px;margin:8px;border-radius:10px;cursor:pointer}
pre{white-space:pre-wrap;background:white;padding:24px;border-radius:12px}kbd{background:white;padding:5px 12px}</style>
<h1>电机 __MOTOR__ · 键盘慢速测试</h1>
<p><kbd>A</kbd> 按住正转　<kbd>D</kbd> 按住反转</p>
<p>松开／空格：停止增加目标角度，保持目标。A+D 同按也停止。</p>
<p>Esc／结束按钮／页面失焦：结束会话并尝试失能，不自动返回起点。</p>
<p>参考速度 __SPEED__°/秒；相对启动位置范围 ±__RANGE__°。</p>
<p>请支撑整条机械臂，确认运动空间，保持物理急停可用。软件限速不能替代硬件保护。</p>
<button id="start">开始控制</button><button id="stop">结束并失能（Esc）</button>
<pre id="status">等待开始。此页面只能控制本次选定的一个电机。</pre>
<script>
const token=__TOKEN__; let armed=false,ended=false,busy=false,pending=false,seq=0;
const keys=new Set(), box=document.getElementById('status');
function direction(){return keys.has('KeyA')===keys.has('KeyD')?0:keys.has('KeyA')?1:-1;}
async function send(){
  if(!armed)return;
  if(busy){pending=true;return;} busy=true;
  const data={seq:++seq,direction:ended?0:direction(),stop:ended};
  try{const r=await fetch('/input',{method:'POST',headers:{'Content-Type':'application/json','X-Control-Token':token},body:JSON.stringify(data)});
    if(!r.ok)throw Error('控制连接已停止'); const s=await r.json(); box.textContent=s.message;
  }catch(e){ended=true;keys.clear();box.textContent='连接中断：请确认电机停止，必要时按物理急停。';}
  finally{busy=false;if(pending){pending=false;send();}}
}
function end(){if(armed){ended=true;keys.clear();send();}}
document.getElementById('start').onclick=()=>{if(ended)return;armed=true;keys.clear();document.getElementById('start').disabled=true;send();};
document.getElementById('stop').onclick=end;
addEventListener('keydown',e=>{if(!armed||ended)return;
  if(['KeyA','KeyD','Space','Escape'].includes(e.code))e.preventDefault();
  if(e.code==='Escape'){end();return;}
  if(e.code==='Space'){keys.clear();send();return;}
  if(['KeyA','KeyD'].includes(e.code)&&!e.repeat){keys.add(e.code);send();}
});
addEventListener('keyup',e=>{if(keys.delete(e.code))send();});
addEventListener('blur',end);addEventListener('pagehide',end);
document.addEventListener('visibilitychange',()=>{if(document.hidden)end();});
setInterval(()=>{if(armed&&!ended)send();},50);
</script></html>"""


class InputState:
    def __init__(self):
        self.lock = threading.Lock()
        self.ready = False
        self.stopped = False
        self.direction = 0
        self.last_seen = 0.0
        self.sequence = -1
        self.message = "正在准备电机；准备期间按键不会移动电机。"

    def update(self, seq, direction, stop, now):
        if type(seq) is not int or seq < 0 or type(direction) is not int or direction not in (-1, 0, 1) or type(stop) is not bool:
            raise ValueError("Invalid key state")
        with self.lock:
            if seq <= self.sequence:
                return
            self.sequence = seq
            self.ready = True
            self.stopped |= stop
            self.direction = 0 if self.stopped else direction
            self.last_seen = now

    def sample(self, now):
        with self.lock:
            if self.ready and now - self.last_seen > 0.3:
                self.stopped = True
                self.message = "页面心跳超时，正在结束并失能。"
            return self.ready, self.stopped, 0 if self.stopped else self.direction


class Panel:
    def __init__(self, args):
        self.inputs = InputState()
        self.token = secrets.token_urlsafe(32)
        self.html = (HTML.replace('__MOTOR__', str(args.motor))
                     .replace('__SPEED__', str(args.speed_deg))
                     .replace('__RANGE__', str(args.range_deg))
                     .replace('__TOKEN__', json.dumps(self.token))).encode()

    def __enter__(self):
        panel = self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *unused):
                pass

            def respond(self, code, body, mime):
                self.send_response(code)
                self.send_header('Content-Type', mime)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Frame-Options', 'DENY')
                self.send_header('Content-Security-Policy', "frame-ancestors 'none'")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path != '/' or self.headers.get('Host') != panel.host:
                    self.respond(404, b'Not found', 'text/plain')
                    return
                self.respond(200, panel.html, 'text/html; charset=utf-8')

            def do_POST(self):
                if (self.path != '/input' or self.headers.get('Host') != panel.host
                        or self.headers.get('Origin') != 'http://' + panel.host
                        or self.headers.get('X-Control-Token') != panel.token):
                    self.respond(403, b'Forbidden', 'text/plain')
                    return
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    if not 0 < length <= 512:
                        raise ValueError('body size')
                    self.connection.settimeout(0.2)
                    data = json.loads(self.rfile.read(length))
                    panel.inputs.update(data['seq'], data['direction'], data['stop'], time.monotonic())
                except (ValueError, KeyError, TypeError, OSError):
                    self.respond(400, b'Invalid input', 'text/plain')
                    return
                with panel.inputs.lock:
                    body = json.dumps({'message': panel.inputs.message}).encode()
                self.respond(200, body, 'application/json; charset=utf-8')
        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.server.daemon_threads = True
        self.host = '127.0.0.1:' + str(self.server.server_port)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        print('键盘控制页面：http://' + self.host)
        webbrowser.open('http://' + self.host)
        return self

    def wait_ready(self):
        while True:
            ready, stopped, _ = self.inputs.sample(time.monotonic())
            if stopped:
                raise RuntimeError('页面会话已结束')
            if ready:
                return
            time.sleep(0.05)

    def __exit__(self, *unused):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1)


def jog_step(reference, direction, dt, speed, origin, travel):
    if dt < 0 or dt > 0.15:
        raise RuntimeError('控制周期异常')
    return min(origin + travel, max(origin - travel, reference + direction * speed * dt))


def jog_loop(link, args, origin, reply_id, limits):
    from openarm_motor_test import check_state, position_command, log_event
    inputs = args.panel.inputs
    travel = math.radians(args.range_deg)
    speed = math.radians(args.speed_deg)
    reference = origin
    begun = previous = time.monotonic()
    while True:
        now = time.monotonic()
        dt = now - previous
        previous = now
        ready, stopped, direction = inputs.sample(now)
        if stopped or not ready:
            return
        # Ramp gains for a second before accepting directional input.
        factor = min(1.0, (now - begun) / 1.0)
        proposed = jog_step(reference, direction if factor == 1 else 0,
                            dt, speed, origin, travel)
        state = link.state(args.motor, reply_id, limits)
        log_event(link, 'keyboard_reference', phase='ready' if factor == 1 else 'startup',
                  direction=direction, dt=dt, reference=proposed, origin=origin,
                  measured_q=state['q'], measured_v=state['v'])
        check_state(state, origin, proposed, travel, limits, enabled=True)
        after_read = time.monotonic()
        if after_read - now > 0.1:
            raise RuntimeError('反馈延迟超过 100 ms')
        _, stopped, current_direction = inputs.sample(after_read)
        if stopped:
            return
        # Release/reverse events arriving during the USB read cancel this step.
        if current_direction != direction:
            proposed = reference
        reference = proposed
        link.send(*position_command(args, reference, limits, factor))
        with inputs.lock:
            inputs.message = (f"电机 {args.motor} | 目标相对角度 {math.degrees(reference-origin):.2f}°\n"
                              f"反馈相对角度 {math.degrees(state['q']-origin):.2f}° | "
                              f"测得速度 {math.degrees(state['v']):.2f}°/s\n"
                              + ('准备中，请暂时松开按键。' if factor < 1 else
                                 'A 正转 / D 反转；松开保持；Esc 结束并失能。'))
        time.sleep(max(0.0, 0.02 - (time.monotonic() - now)))
