"""Render the simulated arms into three indexed-color cameras and incremental APNGs."""

import struct
import zlib


CAMERAS = {
    "overview": "Overview",
    "left_wrist": "Left wrist",
    "right_wrist": "Right wrist",
}
WIDTH, HEIGHT, FPS = 320, 180, 8
SIGNATURE = b"\x89PNG\r\n\x1a\n"
PALETTE = bytes.fromhex(
    "101c28 334958 77929e dfecef 3ad4aa f6b95e 70acfe ef8094 172e38"
)
FONT = {
    "0":"75557", "1":"26227", "2":"71747", "3":"71717", "4":"55711",
    "5":"74717", "6":"74757", "7":"71222", "8":"75757", "9":"75717",
    "A":"25755", "C":"34443", "E":"74647", "F":"74644", "G":"34553",
    "H":"55755", "I":"72227", "L":"44447", "M":"57755", "O":"25552",
    "R":"65655", "S":"34216", "T":"72222", "V":"55552", "W":"55775",
    "Y":"55222", ".":"00002", " ":"00000",
}


def chunk(kind, body):
    return (struct.pack(">I", len(body)) + kind + body
            + struct.pack(">I", zlib.crc32(kind + body)))


HEADER = SIGNATURE + chunk(b"IHDR", struct.pack(">IIBBBBB", WIDTH, HEIGHT, 8, 3, 0, 0, 0))
HEADER += chunk(b"PLTE", PALETTE)
END = chunk(b"IEND", b"")


def still_image(compressed):
    return HEADER + chunk(b"IDAT", compressed) + END


def _rect(pixels, x, y, w, h, color):
    x0, x1 = max(0, x), min(WIDTH, x + w)
    if x0 >= x1:
        return
    row = bytes([color]) * (x1 - x0)
    for r in range(max(0, y), min(HEIGHT, y + h)):
        pixels[r * WIDTH + x0:r * WIDTH + x1] = row


def _text(pixels, x, y, message, color, scale=2):
    for char in message.upper():
        for row, bits in enumerate(FONT[char]):
            for col in range(3):
                if int(bits) & (4 >> col):
                    _rect(pixels, x + col*scale, y + row*scale, scale, scale, color)
        x += 4*scale


def _background(camera):
    p = bytearray(WIDTH * HEIGHT)
    _rect(p, 0, 0, WIDTH, 22, 8)
    _rect(p, 0, HEIGHT-22, WIDTH, 22, 8)
    color = {"overview":4, "left_wrist":6, "right_wrist":5}[camera]
    _rect(p, 8, 6, 3, 10, color)
    _text(p, 16, 6, CAMERAS[camera], 3)
    _text(p, 229, 6, "SIM", color)
    _text(p, 8, HEIGHT-16, "FRAME", 2)
    return bytes(p)


BACKGROUNDS = {cam: _background(cam) for cam in CAMERAS}


def _rotate(q, v):
    x, y, z, w = q
    a, b, c = v
    tx, ty, tz = 2*(y*c-z*b), 2*(z*a-x*c), 2*(x*b-y*a)
    return (a+w*tx+y*tz-z*ty, b+w*ty+z*tx-x*tz, c+w*tz+x*ty-y*tx)


def _camera_coordinates(camera, robots):
    if camera == "overview":
        def overview(p):
            x, y, z = p[0], p[1] - .90, p[2] + .15
            return (.82*x - .57*z, -.18*x + .95*y - .26*z, -.54*x - .32*y - .78*z + 2)
        return overview
    robot = robots.get(camera.split("_")[0])
    if not robot:
        return None
    q, p = robot["tcp"]["q"], robot["tcp"]["p"]
    inv = (-q[0], -q[1], -q[2], q[3])
    def wrist(world):
        x, y, z = _rotate(inv, [value-origin for value, origin in zip(world, p)])
        # A virtual camera 20 cm behind and 4 cm above the TCP looks down the fingers.
        return x, y - .04, .20 - z
    return wrist


def _world_segments(robots):
    for i in range(-5, 6):
        k = i * .1
        yield ((k,.70,-.80), (k,.70,.20), 1, 1)
        yield ((-.50,.70,k-.30), (.50,.70,k-.30), 1, 1)
    # Fixed asymmetric calibration targets make wrist translation and roll
    # visible even when the wrist camera moves together with its own fingers.
    for cx in (-.25, .25):
        corners = [(cx-.075,.94,-.90), (cx+.075,.94,-.90),
                   (cx+.075,1.09,-.90), (cx-.075,1.09,-.90)]
        for i in range(4):
            yield (corners[i], corners[(i+1)%4], 2, 2)
        yield (corners[3], (cx,1.09,-.90), 4, 3)
        yield ((cx,1.015,-.90), (cx+.035,1.015,-.90), 7, 2)
        yield ((cx,1.015,-.90), (cx,1.05,-.90), 4, 2)
    for side, robot in robots.items():
        joints = robot["joint_positions_m"]
        color = 6 if side == "left" else 5
        base, palm = robot["base_m"], robot["palm_m"]
        yield ((base[0],.44,base[2]), base, 2, 5)
        chain = [base, *joints, palm]
        for start, end in zip(chain, chain[1:]):
            yield (start, end, color, 4)
        for joint in joints:
            yield (joint, joint, 3, 5)
        roots = robot["finger_roots_m"]
        yield (roots[0], roots[1], color, 3)
        for root, tip in zip(roots, robot["fingers_m"]):
            yield (palm, root, color, 3)
            yield (root, tip, 3, 3)


def _clipped_line(pixels, a, b, color, width):
    """Clip each stroke to the image area between the two label bars."""
    x, y = a
    dx, dy = b[0] - x, b[1] - y
    lo, hi = 0., 1.
    radius = width // 2
    for step, margin in ((-dx, x-radius), (dx, WIDTH-1-radius-x),
                         (-dy, y-23-radius), (dy, HEIGHT-24-radius-y)):
        if step == 0:
            if margin < 0:
                return
        elif step < 0:
            lo = max(lo, margin / step)
        else:
            hi = min(hi, margin / step)
    if lo > hi:
        return
    steps = max(1, round(max(abs(dx), abs(dy)) * (hi-lo)))
    for i in range(steps+1):
        t = lo + (hi-lo)*i/steps
        _rect(pixels, round(x+dx*t)-radius, round(y+dy*t)-radius, width, width, color)


def render(camera, index, elapsed_s, robots):
    pixels = bytearray(BACKGROUNDS[camera])
    coordinates = _camera_coordinates(camera, robots)
    if coordinates:
        strokes = []
        for start, end, color, width in _world_segments(robots):
            a, b = coordinates(start), coordinates(end)
            if min(a[2], b[2]) < .025:
                if max(a[2], b[2]) < .025:
                    continue
                if a[2] > b[2]:
                    a, b = b, a
                t = (.025-a[2]) / (b[2]-a[2])
                a = tuple(x+t*(y-x) for x,y in zip(a,b))
            projected = []
            for x,y,z in (a,b):
                scale = 160 if camera == "overview" else 138/z
                projected.append((WIDTH/2+scale*x, HEIGHT/2-scale*y))
            strokes.append(((a[2]+b[2])/2, projected, color, width))
        for _, points, color, width in sorted(strokes, key=lambda stroke: -stroke[0]):
            _clipped_line(pixels, *points, color, width)
    _text(pixels, 56, HEIGHT-16, f"{index:06d}", 3)
    _text(pixels, 190, HEIGHT-16, f"{elapsed_s:8.2f} S", 3)
    scanlines = b"".join(b"\0" + pixels[i:i+WIDTH] for i in range(0, len(pixels), WIDTH))
    return zlib.compress(scanlines, level=2)


class Animation:
    """Keep a decodable APNG after each frame commit, including during capture."""

    def __init__(self, path):
        self.file = path.open("xb+")
        self.count = self.sequence = 0
        self.file.write(HEADER + self._animation_header() + END)

    def _animation_header(self):
        return chunk(b"acTL", struct.pack(">II", self.count, 0))

    def append(self, compressed):
        f = self.file
        f.seek(-len(END), 2)
        ctl = struct.pack(">IIIIIHHBB", self.sequence, WIDTH, HEIGHT, 0, 0, 1, FPS, 0, 0)
        f.write(chunk(b"fcTL", ctl))
        self.sequence += 1
        if self.count == 0:
            offset = f.tell() + 8
            f.write(chunk(b"IDAT", compressed))
        else:
            offset = f.tell() + 12
            f.write(chunk(b"fdAT", struct.pack(">I", self.sequence) + compressed))
            self.sequence += 1
        f.write(END)
        self.count += 1
        f.seek(len(HEADER))
        f.write(self._animation_header())
        f.flush()
        return offset

    def close(self):
        self.file.close()
