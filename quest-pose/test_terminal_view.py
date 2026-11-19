import importlib.util
import io
from pathlib import Path
from tempfile import TemporaryDirectory
import time
import unittest

from station import Station
from terminal_view import TerminalMonitor, camera_rows, terminal_view


@unittest.skipUnless(importlib.util.find_spec("cv2"), "optional OpenCV terminal decoder")
class TerminalTests(unittest.TestCase):
    def test_color_half_blocks_keep_rgb_channels(self):
        import cv2
        import numpy as np

        image = np.zeros((2, 4, 3), dtype=np.uint8)
        image[0, :] = [0, 0, 255]
        image[1, :] = [0, 255, 0]
        ok, png = cv2.imencode(".png", image)
        self.assertTrue(ok)
        row, = camera_rows(png.tobytes(), 4, 1)
        self.assertIn("38;2;255;0;0;48;2;0;255;0m", row)
        self.assertEqual(row.count("\u2580"), 4)

    def test_selected_camera_follows_switch_without_opening_an_input_session(self):
        with TemporaryDirectory() as temp:
            station = Station(Path(temp))
            try:
                monitor = TerminalMonitor(station, "http://127.0.0.1:8000/", io.StringIO())
                deadline = time.monotonic() + 3
                while not station.recorder.snapshot() and time.monotonic() < deadline:
                    time.sleep(.01)
                initial = monitor.lines(80, 24)
                self.assertIn("Overview | frame", "\n".join(initial))
                switched = station.control({"action":"view"}, local=True)
                next_image = monitor.lines(80, 24)
                self.assertIn(switched["camera"]["label"] + " | frame", "\n".join(next_image))
                self.assertNotIn("Overview | frame", "\n".join(next_image))
                self.assertNotEqual(initial[-3:], next_image[-3:])
                self.assertIsNone(station.state()["owner"])
                with self.assertRaisesRegex(ValueError, "interactive terminal"):
                    with terminal_view(station, "http://127.0.0.1", io.StringIO()):
                        pass
            finally:
                station.close()
