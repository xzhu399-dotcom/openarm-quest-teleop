import http.client
from http.cookies import SimpleCookie
from pathlib import Path
import ssl
import tempfile
import threading
import unittest
from unittest.mock import patch

from quest_pose_receiver import make_tls_context
from short_pairing import make_pairing_server, parse_pairing_url


HOST = "127.0.0.1"
TOKEN = "roundtrip_test_123456789"
RECEIVER = f"https://{HOST}:8443/"
PAIRING_URL = RECEIVER + "?key=" + TOKEN


class PairingURLTests(unittest.TestCase):
    def test_only_local_receiver_pairing_urls_are_accepted(self):
        self.assertEqual(parse_pairing_url(PAIRING_URL), (HOST, 8443, TOKEN))
        for url in (RECEIVER, PAIRING_URL.replace("https:", "http:"),
                    PAIRING_URL.replace(HOST, "8.8.8.8"),
                    PAIRING_URL.replace(HOST, "user@" + HOST),
                    PAIRING_URL.replace("/?", "/state?"), PAIRING_URL + "&key=duplicate"):
            with self.subTest(url=url), self.assertRaises(ValueError):
                parse_pairing_url(url)


class PairingServerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with tempfile.TemporaryDirectory() as temp:
            try:
                cls.server_tls = make_tls_context(HOST, temp)
            except OSError as exc:
                if "Install OpenSSL" in str(exc):
                    raise unittest.SkipTest("HTTPS pairing tests require OpenSSL") from exc
                raise
            cls.client_tls = ssl.create_default_context(cafile=str(Path(temp) / "cert.pem"))

    def setUp(self):
        self.server = make_pairing_server(PAIRING_URL, 0, self.server_tls)
        self.addCleanup(self.server.server_close)
        thread = threading.Thread(target=self.server.serve_forever, args=(0.01,))
        thread.start()
        self.addCleanup(thread.join)
        self.addCleanup(self.server.shutdown)
        self.origin = f"https://{HOST}:{self.server.server_port}"
        self.pairing = self.server.pairing

    def request(self, code=None, **headers):
        conn = http.client.HTTPSConnection(HOST, self.server.server_port, context=self.client_tls, timeout=3)
        try:
            if code is None:
                conn.request("GET", "/", headers=headers)
            else:
                conn.request("POST", "/", "code=" + code, {
                    "Content-Type": "application/x-www-form-urlencoded", "Origin": self.origin, **headers})
            response = conn.getresponse()
            return response.status, response.headers, response.read()
        finally:
            conn.close()

    def test_form_and_local_cookie_redirect(self):
        status, headers, body = self.request()
        self.assertEqual(status, 200)
        self.assertIn(b"<form", body)
        self.assertNotIn(TOKEN.encode(), body)
        self.assertIn("no-store", headers["Cache-Control"])
        self.assertEqual(headers["Referrer-Policy"], "same-origin")
        status, headers, _ = self.request(self.pairing.code)
        self.assertEqual((status, headers["Location"]), (303, RECEIVER))
        cookie = SimpleCookie(headers["Set-Cookie"])["station_key"]
        self.assertEqual(cookie.value, TOKEN)
        self.assertEqual(cookie["path"], "/")
        self.assertTrue(cookie["secure"] and cookie["httponly"])
        self.assertEqual(cookie["samesite"].lower(), "strict")
        self.assertFalse(self.pairing.redeem(self.pairing.code))
        status, headers, _ = self.request(Cookie=f"station_key={TOKEN}")
        self.assertEqual((status, headers["Location"]), (303, RECEIVER))

    def test_wrong_origin_or_host_cannot_redeem_a_code(self):
        for headers in ({"Host": "elsewhere.invalid"}, {"Origin": "https://elsewhere.invalid"}, {"Origin": ""}):
            with self.subTest(headers=headers):
                status, response_headers, _ = self.request(self.pairing.code, **headers)
                self.assertEqual(status, 403)
                self.assertIsNone(response_headers.get("Set-Cookie"))
        self.assertTrue(self.pairing.redeem(self.pairing.code))

    def test_five_wrong_codes_lock_out_later_redemption(self):
        wrong = "000000" if self.pairing.code != "000000" else "111111"
        for _ in range(5):
            self.assertFalse(self.pairing.redeem(wrong))
        self.assertFalse(self.pairing.redeem(self.pairing.code))

    def test_expired_code_cannot_be_redeemed(self):
        with patch("short_pairing.time.monotonic", return_value=self.pairing.expires_at + 1):
            self.assertFalse(self.pairing.redeem(self.pairing.code))
