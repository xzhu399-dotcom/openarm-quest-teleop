#!/usr/bin/env python3
"""Run the articulated OpenArm simulator locally, or use Quest controllers over HTTPS."""

import argparse
from contextlib import ExitStack
import hmac
from http import cookies
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from ipaddress import IPv4Address
import json
import math
from pathlib import Path
import queue
import secrets
import shutil
import ssl
import subprocess
import sys
import tempfile
import time
from urllib.parse import parse_qs, urlsplit

from local_https import LocalHTTPSServer
from station import Conflict, Station


ROOT = Path(__file__).resolve().parent
ASSETS = {"index.html": "text/html; charset=utf-8", "app.js": "text/javascript",
          "xr.js": "text/javascript", "xr_robot.js": "text/javascript", "scene.js": "text/javascript", "style.css": "text/css",
          "legacy.html": "text/html; charset=utf-8"}


def format_position(value):
    if value is None:
        return "untracked"
    if not isinstance(value, list) or len(value) != 3 or not all(
        type(axis) in (int, float) and math.isfinite(axis) for axis in value
    ):
        raise ValueError("Expected three finite coordinates or null.")
    x, y, z = value
    return f"x={x:+.3f} y={y:+.3f} z={z:+.3f} m"


class PoseHandler(BaseHTTPRequestHandler):
    # Keep the browser's telemetry connection open between POSTs.
    protocol_version = "HTTP/1.1"
    timeout = 5

    def log_message(self, format, *args):
        pass

    def reply(self, code, content=b"", mime="application/json", headers=None):
        self.send_response(code)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("X-Content-Type-Options", "nosniff")
        for name, value in (headers or {}).items():
            self.send_header(name, value)
        self.end_headers()
        if content:
            self.wfile.write(content)

    def problem(self, code, message):
        self.close_connection = True
        self.reply(code, json.dumps({"error": message}).encode(), headers={"Connection": "close"})

    def json(self, data):
        self.reply(200, json.dumps(data, allow_nan=False, separators=(",", ":")).encode())

    def station_json(self, state):
        self.json(dict(state, local_controls=self.client_address[0] == self.server.server_address[0]))

    def authorized(self):
        host = self.headers.get("Host", "")
        if host not in self.server.hosts:
            self.problem(403, "Use the address printed by the receiver.")
            return False
        origin = self.headers.get("Origin")
        if origin and origin != self.server.scheme + "://" + host:
            self.problem(403, "Request origin differs from the receiver.")
            return False
        token = self.server.token
        if not token:
            return True
        if self.command == "GET" and urlsplit(self.path).path == "/":
            link_key = parse_qs(urlsplit(self.path).query).get("key", [""])[0]
            if hmac.compare_digest(link_key.encode(), token.encode()):
                self.reply(303, headers={"Location": "/", "Set-Cookie":
                           f"station_key={token}; Path=/; Secure; HttpOnly; SameSite=Strict"})
                return False
        jar = cookies.SimpleCookie()
        try:
            jar.load(self.headers.get("Cookie", ""))
        except cookies.CookieError:
            pass
        key = jar.get("station_key")
        if key and hmac.compare_digest(key.value.encode(), token.encode()):
            return True
        pairing_origin = getattr(self.server, "http_redirect_origin", None)
        if pairing_origin and self.command == "GET" and self.path == "/":
            self.reply(303, headers={"Location": pairing_origin + "/"})
            return False
        self.problem(403, "Open the complete pairing URL printed in the workstation terminal.")
        return False

    def do_GET(self):
        if not self.authorized():
            return
        path = urlsplit(self.path).path
        try:
            if path == "/state":
                self.station_json(self.server.station.state())
            elif path == "/camera":
                frame = self.server.station.recorder.snapshot()
                if frame is None:
                    self.problem(503, "Selected camera has no fresh frame.")
                    return
                image, meta = frame
                age_ms = (time.monotonic_ns() - meta["host_monotonic_ns"]) / 1e6
                self.reply(200, image, meta.get("mime", "image/png"), {
                    "X-Camera-Id": meta["camera_id"], "X-Camera-Label": meta["label"],
                    "X-Frame-Index": str(meta["index"]), "X-Frame-Age-Ms": f"{age_ms:.2f}",
                })
            else:
                name = path[1:]
                if path == "/":
                    name = "legacy.html" if self.server.legacy else "index.html"
                if name not in ASSETS:
                    self.problem(404, "Unknown page.")
                    return
                self.reply(200, (ROOT / name).read_bytes(), ASSETS[name])
        except (ConnectionError, TimeoutError):
            self.close_connection = True

    def do_POST(self):
        path = urlsplit(self.path).path
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= (512 if path == "/pose" else 16384):
                # Consume modest oversize bodies before closing so the TCP reset on
                # an unread request does not erase the error response on Windows.
                if 0 < size <= 32768:
                    self.rfile.read(size)
                self.problem(413, "Invalid message length.")
                return
            body = self.rfile.read(size)
            if not self.authorized():
                return
            if path not in ("/sample", "/control", "/pose"):
                self.problem(404, "Unknown endpoint.")
                return
            if self.headers.get_content_type() != "application/json":
                self.problem(415, "Send application/json.")
                return
            data = json.loads(body)
            if not isinstance(data, dict):
                raise ValueError("Expected a JSON object.")
            if path == "/pose":
                left, right = (format_position(data[side]) for side in ("left", "right"))
                print(f"L {left:>31}  |  R {right}", flush=True)
                self.reply(204)
                return
            if path == "/sample":
                result = self.server.station.sample(data)
            else:
                local = self.client_address[0] == self.server.server_address[0]
                result = self.server.station.control(data, local=local)
            self.station_json(result)
        except Conflict as error:
            self.problem(409, str(error))
        except PermissionError as error:
            self.problem(403, str(error))
        except (ValueError, KeyError, TypeError, OverflowError, RecursionError) as error:
            self.problem(400, str(error))
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            self.close_connection = True
        except OSError as error:
            self.problem(503, f"Workstation I/O failed: {error}")


def make_tls_context(host, directory, certfile=None, keyfile=None):
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    if certfile is not None:
        context.load_cert_chain(certfile=certfile, keyfile=keyfile)
        return context
    openssl = shutil.which("openssl")
    if not openssl and sys.platform == "win32":
        installed = Path(r"C:\Program Files\Git\usr\bin\openssl.exe")
        openssl = str(installed) if installed.is_file() else None
    if not openssl:
        raise OSError("Install OpenSSL for Quest HTTPS, or use --sim for the mouse.")
    cert, key, config = (Path(directory) / name for name in ("cert.pem", "key.pem", "req.cnf"))
    config.write_text(f"[req]\ndistinguished_name=dn\nx509_extensions=ext\nprompt=no\n"
                      f"[dn]\nCN={host}\n[ext]\nsubjectAltName=IP:{host}\n", encoding="ascii")
    subprocess.run([openssl, "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "1",
                    "-config", str(config), "-keyout", str(key), "-out", str(cert)],
                   check=True, capture_output=True, text=True)
    context.load_cert_chain(certfile=cert, keyfile=key)
    return context


def make_server(host, port, station, legacy=False, tls=None):
    server = (LocalHTTPSServer(host, port, PoseHandler, tls) if tls is not None
              else ThreadingHTTPServer((host, port), PoseHandler))
    try:
        server.station = station
        server.legacy = legacy
        server.token = secrets.token_urlsafe(18) if tls else None
        server.scheme = "https" if tls else "http"
        server.hosts = {f"{host}:{server.server_port}"}
        if IPv4Address(host).is_loopback:
            server.hosts.add(f"localhost:{server.server_port}")
        if server.server_port == (443 if tls else 80):
            server.hosts.add(host)
        return server
    except BaseException:
        server.server_close()
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sim", action="store_true", help="Mouse simulator on loopback HTTP (default port 8000)")
    mode.add_argument("--host", type=IPv4Address, help="Workstation LAN IPv4 address for Quest HTTPS")
    parser.add_argument("--port", type=int, help="Set port, or use 0 to select a free port")
    parser.add_argument("--legacy", action="store_true", help="Serve the original headset-relative XYZ diagnostic page")
    parser.add_argument("--episodes", type=Path, default=Path("episodes"), help="Episode output directory")
    parser.add_argument("--cameras", type=Path, metavar="CONFIG", help="Open the three OpenCV sources in this JSON file")
    parser.add_argument("--gripper-mode", choices=("trigger","stick"), default="trigger",
                        help="Use absolute index trigger position or thumbstick opening rate")
    parser.add_argument("--mode", choices=("simulate", "robot-readonly", "robot-drive"), default="simulate",
                        help="Select simulation, physical encoder preview, or locally armed physical drive")
    parser.add_argument("--robot-calibration", type=Path, metavar="FILE",
                        help="Local calibrated arm configuration for either robot mode")
    output = parser.add_mutually_exclusive_group()
    output.add_argument("--terminal-view", action="store_true",
                        help="Show the selected camera and handle positions in this terminal (requires OpenCV)")
    output.add_argument("--pose-report", action="store_true",
                        help="Print headset and hand poses, simulated targets and jaws, and trial starts/stops")
    parser.add_argument("--gripper-bench", type=Path, metavar="CALIBRATION",
                        help="Compare Quest commands with one physical J8 encoder on the lab Mac")
    parser.add_argument("--execute-gripper", action="store_true",
                        help="Allow supervised J8 movement with --gripper-bench and local operator confirmation")
    parser.add_argument("--certfile", type=Path, help="Lab-trusted TLS certificate chain for the LAN IP (PEM)")
    parser.add_argument("--keyfile", type=Path, help="Private key for --certfile (PEM)")
    parser.add_argument("--pairing-code-file", type=Path, metavar="PATH",
                        help="Serve the LAN simulator with a reusable six-digit PIN stored on this computer")
    parser.add_argument("--pairing-port", type=int, help="HTTPS port for the simulator PIN page (default 443)")
    parser.add_argument("--http-port", type=int, help="HTTP port that redirects to simulator pairing (default 80)")
    args = parser.parse_args()
    host = "127.0.0.1" if args.sim else str(args.host)
    if host == "0.0.0.0":
        parser.error("Use the actual LAN IPv4 address for the certificate and pairing URL.")
    port = args.port if args.port is not None else 8000 if args.sim else 8443
    if not 0 <= port <= 65535:
        parser.error("Port must be in the range 0 through 65535.")
    if bool(args.certfile) != bool(args.keyfile) or (args.sim and args.certfile):
        parser.error("Provide --certfile and --keyfile together with --host.")
    if args.pose_report and args.legacy:
        parser.error("--pose-report needs the standard Quest page. Omit --legacy.")
    if args.execute_gripper and not args.gripper_bench:
        parser.error("--execute-gripper requires --gripper-bench CALIBRATION.")
    if args.gripper_bench and (args.sim or args.legacy or args.terminal_view):
        parser.error("The physical gripper bench requires --host, the standard Quest page, and a text terminal.")
    robot_mode = args.mode != "simulate"
    if robot_mode != bool(args.robot_calibration):
        parser.error("Choose --mode robot-readonly or robot-drive together with --robot-calibration FILE.")
    if robot_mode and (not args.host or args.legacy or args.gripper_bench or args.execute_gripper):
        parser.error("Robot mode requires --host and the standard Quest page, with the full-arm bridge as sole motor owner.")
    if args.pairing_code_file and (args.sim or args.legacy or args.gripper_bench or robot_mode):
        parser.error("--pairing-code-file requires --host and the standard simulated-arm receiver.")
    if not args.pairing_code_file and (args.http_port is not None or args.pairing_port is not None):
        parser.error("--http-port and --pairing-port are options for --pairing-code-file.")
    if robot_mode and sys.platform != "darwin":
        parser.error("Run robot mode on the Mac connected to the CANable adapters. This computer can run --mode simulate.")
    try:
        camera_config = None
        if args.cameras:
            from live_cameras import load_camera_config
            camera_config = load_camera_config(args.cameras)
        with tempfile.TemporaryDirectory(prefix="quest-pose-") as directory, ExitStack() as observers:
            tls = None if args.sim else make_tls_context(host, directory, args.certfile, args.keyfile)
            trial_events = queue.SimpleQueue() if args.pose_report else None
            station = Station(args.episodes, gripper_mode=args.gripper_mode,
                              camera_config=camera_config, trial_events=trial_events)
            try:
                with make_server(host, port, station, args.legacy, tls) as server, ExitStack() as pairing_pages:
                    suffix = f"?key={server.token}" if tls else ""
                    url = f"{server.scheme}://{host}:{server.server_port}/{suffix}"
                    if args.pairing_code_file:
                        from local_https import http_redirect, serving
                        from short_pairing import make_pairing_server
                        pairing = pairing_pages.enter_context(make_pairing_server(
                            url, 443 if args.pairing_port is None else args.pairing_port, tls,
                            code_file=args.pairing_code_file))
                        server.http_redirect_origin = pairing.origin
                        pairing_pages.enter_context(http_redirect(pairing, 80 if args.http_port is None else args.http_port))
                        pairing_pages.enter_context(serving(pairing))
                        print(f"Quest Browser: {pairing.origin}\nSimulator pairing code: {pairing.pairing.code}", flush=True)
                    else:
                        print(f"Open {url}", flush=True)
                    scope = "Simulated arms and jaws"
                    if args.gripper_bench:
                        scope = "Physical J8 execution requested" if args.execute_gripper else "Read-only physical J8 test"
                    if robot_mode:
                        scope = f"Simulated arm display with physical OpenArm ({args.mode})"
                    print(f"{scope} | {args.gripper_mode} gripper | Ctrl+C to stop", flush=True)
                    if robot_mode:
                        print("Camera recording: " + ("configured OpenCV sources" if camera_config
                              else "synthetic views (supply --cameras for lab camera readings)"), flush=True)
                    with ExitStack() as devices:
                        if args.gripper_bench:
                            from gripper_bench import gripper_bench
                            devices.enter_context(gripper_bench(station, args.gripper_bench, args.execute_gripper))
                        if robot_mode:
                            from robot_hardware import robot_hardware
                            devices.enter_context(robot_hardware(station, args.robot_calibration, args.mode == "robot-drive"))
                        if args.terminal_view:
                            from terminal_view import terminal_view
                            devices.enter_context(terminal_view(station, url))
                        if args.pose_report:
                            from pose_report import pose_report
                            # Keep the reporter through recorder.close(), after local hardware prompts.
                            observers.enter_context(pose_report(station, trial_events))
                        server.serve_forever()
            finally:
                station.close()
    except KeyboardInterrupt:
        pass
    except subprocess.CalledProcessError as error:
        parser.exit(1, f"Could not generate the local TLS certificate:\n{error.stderr}\n")
    except (OSError, ValueError, RuntimeError, ImportError) as error:
        parser.exit(1, f"Could not start or run the receiver: {error}\n")


if __name__ == "__main__":
    main()
