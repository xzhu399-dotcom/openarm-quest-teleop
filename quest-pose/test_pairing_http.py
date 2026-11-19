import http.client
import ssl
import tempfile
import unittest
from contextlib import ExitStack
from pathlib import Path

from local_https import http_redirect, serving
from quest_pose_receiver import make_server, make_tls_context
from short_pairing import make_pairing_server


HOST = "127.0.0.1"


class HTTPPairingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        stack = ExitStack()
        cls.addClassCleanup(stack.close)
        temp = stack.enter_context(tempfile.TemporaryDirectory())
        try:
            cls.tls = make_tls_context(HOST, temp)
        except OSError as exc:
            if "Install OpenSSL" in str(exc):
                raise unittest.SkipTest("HTTPS tests require OpenSSL") from exc
            raise
        cls.ca = ssl.create_default_context(cafile=str(Path(temp) / "cert.pem"))
        cls.receiver = stack.enter_context(make_server(HOST, 0, object(), tls=cls.tls))
        cls.key_path = "/?key=" + cls.receiver.token
        url = f"https://{HOST}:{cls.receiver.server_port}{cls.key_path}"
        cls.pairing = stack.enter_context(make_pairing_server(url, 0, cls.tls))
        cls.receiver.http_redirect_origin = cls.pairing.origin
        stack.enter_context(serving(cls.receiver))
        stack.enter_context(serving(cls.pairing))
        cls.http = stack.enter_context(http_redirect(cls.pairing, 0))

    def request(self, server, *, secured=False, method="GET", path="/", headers=None):
        if secured:
            conn = http.client.HTTPSConnection(HOST, server.server_port, context=self.ca, timeout=4)
        else:
            conn = http.client.HTTPConnection(HOST, server.server_port, timeout=4)
        try:
            conn.request(method, path, headers=headers or {})
            res = conn.getresponse()
            return res.status, res.headers, res.read()
        finally:
            conn.close()

    def test_all_http_entry_points_redirect_to_the_same_https_pairing_form(self):
        for server in (self.http, self.pairing, self.receiver):
            for method in ("GET", "HEAD"):
                with self.subTest(port=server.server_port, method=method):
                    status, headers, body = self.request(server, method=method, path=self.key_path)
                    self.assertEqual(status, 303)
                    self.assertEqual(headers["Location"], self.pairing.origin + "/")
                    self.assertIsNone(headers.get("Set-Cookie"))
                    self.assertEqual(body, b"")
        status, headers, body = self.request(self.pairing, secured=True)
        self.assertEqual(status, 200)
        self.assertIn(b"Pair Quest", body)

    def test_https_bookmark_routes_unpaired_browsers_to_the_pairing_form(self):
        status, headers, _ = self.request(self.receiver, secured=True)
        self.assertEqual((status, headers["Location"]), (303, self.pairing.origin + "/"))
        for path in ("/?key=bad", "/state", "/camera"):
            status, headers, _ = self.request(self.receiver, path=path, secured=True)
            self.assertEqual(status, 403)
            self.assertIsNone(headers.get("Location"))
        status, headers, _ = self.request(self.receiver, secured=True, path=self.key_path)
        self.assertEqual((status, headers["Location"]), (303, "/"))
        status, _, content = self.request(self.receiver, secured=True,
                                         headers={"Cookie": "station_key=" + self.receiver.token})
        self.assertEqual(status, 200)
        self.assertIn(b"Robot simulator", content)

    def test_http_refuses_post_and_foreign_host(self):
        for server in (self.http, self.pairing, self.receiver):
            with self.subTest(port=server.server_port):
                status, headers, _ = self.request(server, method="POST", path="/sample")
                self.assertEqual(status, 405)
                self.assertIsNone(headers.get("Location"))
                status, headers, _ = self.request(server, headers={"Host": "elsewhere.invalid"})
                self.assertEqual(status, 403)
                self.assertIsNone(headers.get("Location"))

    def test_standalone_https_receiver_still_requires_its_original_pairing_link(self):
        with make_server(HOST, 0, object(), tls=self.tls) as standalone, serving(standalone):
            status, headers, _ = self.request(standalone, secured=True)
            self.assertEqual(status, 403)
            self.assertIsNone(headers.get("Location"))
