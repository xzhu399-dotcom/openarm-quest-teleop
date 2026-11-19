"""Direct a workstation's HTTP visitors to its fixed HTTPS address."""

from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import socket
import ssl
import threading


def https_origin(host, port):
    return f"https://{host}" if port == 443 else f"https://{host}:{port}"


class RedirectToHTTPS(BaseHTTPRequestHandler):
    timeout = 5

    def log_message(self, *args):
        pass

    def do_GET(self):
        hosts = self.headers.get_all("Host", [])
        if len(hosts) != 1 or hosts[0] not in self.server.redirect_hosts:
            self.send_response(403)
        else:
            self.send_response(303)
            self.send_header("Location", self.server.http_redirect_origin + "/")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Content-Length", "0")
        self.end_headers()

    do_HEAD = do_GET

    def do_POST(self):
        self.send_response(405)
        self.send_header("Allow", "GET, HEAD")
        self.send_header("Content-Length", "0")
        self.end_headers()


def redirect_hosts(host, port):
    hosts = {f"{host}:{port}"}
    if port == 80:
        hosts.add(host)
    return hosts


class LocalHTTPSServer(ThreadingHTTPServer):
    def __init__(self, host, port, handler, tls):
        super().__init__((host, port), handler)
        self.tls = tls
        self.redirect_hosts = redirect_hosts(host, self.server_port)
        self.http_redirect_origin = None

    def finish_request(self, request, client_address):
        # Classify each accepted connection in its own worker. A browser's idle
        # preconnection must not stall the listener or delay an XR sample.
        request.settimeout(5)
        try:
            first = request.recv(1, socket.MSG_PEEK)
            if first == b"\x16":  # TLS handshake record
                with self.tls.wrap_socket(request, server_side=True) as connection:
                    super().finish_request(connection, client_address)
            elif self.http_redirect_origin and first in (b"G", b"H", b"P"):
                RedirectToHTTPS(request, client_address, self)
        except (ConnectionError, TimeoutError, ssl.SSLError):
            pass


@contextmanager
def serving(server):
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    try:
        yield server
    finally:
        server.shutdown()
        worker.join()


@contextmanager
def http_redirect(pairing, port=80):
    """Run an HTTP-only listener that redirects to the local HTTPS pairing page."""
    if not 0 <= port <= 65535 or port == pairing.server_port:
        raise ValueError("Choose a valid HTTP port different from the HTTPS pairing port.")
    host = pairing.server_address[0]
    with ThreadingHTTPServer((host, port), RedirectToHTTPS) as server:
        server.redirect_hosts = redirect_hosts(host, server.server_port)
        server.http_redirect_origin = pairing.origin
        with serving(server):
            yield server
