"""Pair Quest Browser to a running local receiver over HTTPS."""

import argparse
from collections import deque
from contextlib import ExitStack
import getpass
import hmac
from http import cookies
from http.server import BaseHTTPRequestHandler
from ipaddress import IPv4Address, IPv4Network
import os
from pathlib import Path
import re
import secrets
import subprocess
import sys
import tempfile
import threading
import time

from local_https import LocalHTTPSServer, http_redirect, https_origin
from quest_pose_receiver import make_tls_context


LOCAL_NETWORKS = tuple(IPv4Network(block) for block in
                       ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16", "169.254.0.0/16", "127.0.0.0/8"))
FORM = """<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Pair Quest</title><h1>Pair Quest</h1>
<form action="/" method="post"><label>Six-digit code from the lab computer
<input name="code" type="text" inputmode="numeric" pattern="[0-9]{6}" maxlength="6"
 autocomplete="one-time-code" required></label> <button>Open simulator</button></form></html>"""
UNAVAILABLE = ("This code has expired or is no longer available. "
               "Restart the pairing helper on the lab computer for a new code.")


def parse_pairing_url(url):
    match = re.fullmatch(r"https://([0-9.]+)(?::([0-9]{1,5}))?/\?key=([A-Za-z0-9_-]{1,128})", url)
    if not match:
        raise ValueError("Enter the full HTTPS pairing URL printed by the receiver.")
    address = IPv4Address(match[1])
    port = int(match[2] or "443")
    if not any(address in network for network in LOCAL_NETWORKS) or not 1 <= port <= 65535:
        raise ValueError("The receiver must use a local IPv4 address and a valid TCP port.")
    return str(address), port, match[3]


class PairingCode:
    unavailable = (410, UNAVAILABLE)

    def __init__(self):
        self.code = f"{secrets.randbelow(1_000_000):06d}"
        self.expires_at = time.monotonic() + 15 * 60
        self.remaining = 5
        self.lock = threading.Lock()

    def available(self):
        return self.remaining > 0 and time.monotonic() < self.expires_at

    def redeem(self, value):
        with self.lock:
            if not self.available():
                return False
            if value.isascii() and hmac.compare_digest(self.code, value):
                self.remaining = 0
                return True
            self.remaining -= 1
            return False


def load_simulator_pin(path):
    """Create a per-user PIN once, including when two receiver launches overlap."""
    path = Path(path)
    if not path.exists():
        path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        fd, name = tempfile.mkstemp(dir=path.parent)
        pending = Path(name)
        try:
            with os.fdopen(fd, "w", encoding="ascii") as temp:
                temp.write(f"{secrets.randbelow(1_000_000):06d}\n")
            try:
                # Publish the complete file only if another launcher hasn't created it.
                os.link(pending, path)
            except FileExistsError:
                pass
        finally:
            pending.unlink()
    code = path.read_text(encoding="ascii").rstrip("\n")
    if not re.fullmatch(r"[0-9]{6}", code):
        raise ValueError(f"Expected six digits in the local simulator PIN file: {path}")
    return code


class FixedPairingCode:
    unavailable = (429, "Too many incorrect codes. Try again in five minutes.")

    def __init__(self, path):
        self.code = load_simulator_pin(path)
        self.failures = deque()
        self.lock = threading.Lock()

    def _can_try(self, now):
        while self.failures and self.failures[0] <= now - 300:
            self.failures.popleft()
        return len(self.failures) < 5

    def available(self):
        with self.lock:
            return self._can_try(time.monotonic())

    def redeem(self, value):
        with self.lock:
            now = time.monotonic()
            if not self._can_try(now):
                return False
            if value.isascii() and hmac.compare_digest(self.code, value):
                self.failures.clear()
                return True
            self.failures.append(now)
            return False


class PairingHandler(BaseHTTPRequestHandler):
    timeout = 5

    def log_message(self, *args):
        pass

    def reply(self, status, body="", headers=(), mime="text/plain; charset=utf-8"):
        data = body.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        # Chromium uses the form page's policy when sending Origin on a navigation POST.
        self.send_header("Referrer-Policy", "same-origin")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'none'; base-uri 'none'; "
                         f"frame-ancestors 'none'; form-action 'self' {self.server.receiver_origin}")
        for name, value in headers:
            self.send_header(name, value)
        self.end_headers()
        self.wfile.write(data)

    def redirect(self, newly_paired=False):
        headers = [("Location", self.server.receiver_origin + "/")]
        if newly_paired:
            cookie = f"station_key={self.server.token}; Path=/; Secure; HttpOnly; SameSite=Strict"
            headers.append(("Set-Cookie", cookie))
        self.reply(303, headers=headers)

    def ready(self):
        hosts = self.headers.get_all("Host", [])
        origins = self.headers.get_all("Origin", [])
        if len(hosts) != 1 or hosts[0] not in self.server.hosts:
            self.reply(403, "Use the short address printed by the lab computer.")
            return False
        if (origins or self.command == "POST") and origins != [self.server.origin]:
            self.reply(403, "Submit the code from the pairing page on the lab computer's address.")
            return False
        if self.path != "/":
            self.reply(404, "Open the root of the pairing address.")
            return False
        jar = cookies.SimpleCookie()
        try:
            jar.load(self.headers.get("Cookie", ""))
        except cookies.CookieError:
            pass
        key = jar.get("station_key")
        if key and key.value.isascii() and hmac.compare_digest(key.value, self.server.token):
            self.redirect()
            return False
        if not self.server.pairing.available():
            self.reply(*self.server.pairing.unavailable)
            return False
        return True

    def do_GET(self):
        if self.ready():
            self.reply(200, FORM, mime="text/html; charset=utf-8")

    def do_POST(self):
        lengths = self.headers.get_all("Content-Length", [])
        if (self.headers.get("Transfer-Encoding") or len(lengths) != 1
                or not re.fullmatch(r"[0-9]{1,2}", lengths[0]) or not 1 <= int(lengths[0]) <= 64):
            self.reply(400, "Invalid code submission.")
            return
        # Consume the bounded form body before rejecting an origin, host, or
        # cookie. Closing with an unread TLS record can truncate the reply on Windows.
        body = self.rfile.read(int(lengths[0]))
        if not self.ready():
            return
        if self.headers.get_content_type() != "application/x-www-form-urlencoded":
            self.reply(415, "Use the pairing form.")
            return
        match = re.fullmatch(rb"code=([0-9]{6})", body)
        value = match[1].decode("ascii") if match else ""
        if self.server.pairing.redeem(value):
            self.redirect(newly_paired=True)
        elif self.server.pairing.available():
            retry_form = FORM.replace("<form ", "<p>Incorrect code. Try again.</p><form ")
            self.reply(403, retry_form, mime="text/html; charset=utf-8")
        else:
            self.reply(*self.server.pairing.unavailable)


def make_pairing_server(url, port, tls, *, code_file=None):
    host, receiver_port, token = parse_pairing_url(url)
    if not 0 <= port <= 65535 or port == receiver_port:
        raise ValueError("Choose a valid pairing port different from the receiver's port.")
    server = LocalHTTPSServer(host, port, PairingHandler, tls)
    try:
        server.origin = https_origin(host, server.server_port)
        server.http_redirect_origin = server.origin
        server.receiver_origin = https_origin(host, receiver_port)
        server.hosts = {f"{host}:{server.server_port}"}
        if server.server_port == 443:
            server.hosts.add(host)
        server.token = token
        server.pairing = PairingCode() if code_file is None else FixedPairingCode(code_file)
        return server
    except BaseException:
        server.server_close()
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=443, help="Short address HTTPS port (default 443)")
    parser.add_argument("--http-port", type=int, metavar="PORT",
                        help="Also redirect HTTP visitors to this pairing page (usually port 80)")
    parser.add_argument("--certfile", help="Local TLS certificate chain valid for the receiver IP")
    parser.add_argument("--keyfile", help="Matching local TLS private key")
    args = parser.parse_args(argv)
    if bool(args.certfile) != bool(args.keyfile):
        parser.error("Provide --certfile and --keyfile together.")
    try:
        if sys.stdin.isatty():
            url = getpass.getpass("Paste the receiver's full HTTPS pairing URL and press Enter: ")
        else:
            url = sys.stdin.readline().rstrip("\r\n")
        host, _, _ = parse_pairing_url(url)
        with tempfile.TemporaryDirectory(prefix="quest-pairing-") as directory:
            tls = make_tls_context(host, directory, args.certfile, args.keyfile)
            with make_pairing_server(url, args.port, tls) as server, ExitStack() as redirects:
                if args.http_port is not None:
                    redirects.enter_context(http_redirect(server, args.http_port))
                print(f"Open {server.origin}\nPairing code: {server.pairing.code} (one use, 15 minutes)", flush=True)
                server.serve_forever()
    except KeyboardInterrupt:
        pass
    except (EOFError, ValueError, OSError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Could not start the pairing helper: {error}\n")


if __name__ == "__main__":
    main()
