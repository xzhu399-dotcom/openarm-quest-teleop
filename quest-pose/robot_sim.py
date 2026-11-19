"""Nominal OpenArm v1.0 kinematics in a stationary WebXR-coordinate workspace.

The dimensions, axes, joint limits and jaw travel are extracted from Enactic's
expanded v1.urdf, copyright 2026 Enactic, Inc., licensed under Apache-2.0.
Source: https://github.com/enactic/openarm_description/blob/624e3ddadba10304f1fb3bb5df1f8c65977e3b25/assets/robot/openarm_v1.0/urdf/example/v1.urdf
See OPENARM_MODEL_LICENSE.txt. The numerical solver, workspace placement and
ideal contact-point convention belong to this simulator.
"""

import math


MODEL_NAME = "OpenArm v1.0 nominal (624e3dd)"
X, Y, Z = (1, 0, 0), (0, 1, 0), (0, 0, 1)
IDENTITY = (0, 0, 0, 1)
# ROS +X forward, +Y left, +Z up -> XR -Z forward, -X left, +Y up.
ROS_TO_XR = (-0.5, 0.5, 0.5, 0.5)
BODY_ORIGIN = (0, 0.36, -0.08)
HOME = {"right": [0.25, 1.0, -0.45], "left": [-0.25, 1.0, -0.45]}
JOINT_Z_OFFSETS = (0.0625, 0.06, 0.06625, 0.15375, 0.0955, 0.1205, 0.0)
PALM_OFFSET = 0.1001
TCP_FROM_PALM = 0.09
SLIDER_STROKE = 0.044
POSITION_TOL = 0.00005  # metres
ANGLE_TOL = 0.0003      # radians
ORIENTATION_WEIGHT = 0.19  # metres/radian in the IK least-squares problem


def product(a, b):
    x, y, z, w = a
    i, j, k, r = b
    return [w*i + x*r + y*k - z*j, w*j - x*k + y*r + z*i,
            w*k + x*j - y*i + z*r, w*r - x*i - y*j - z*k]


def inverse(q):
    return [-q[0], -q[1], -q[2], q[3]]


def turn(axis, angle):
    s = math.sin(angle / 2)
    return [s * v for v in axis] + [math.cos(angle / 2)]


def rotate(q, v):
    x, y, z, w = q
    a, b, c = v
    tx, ty, tz = 2*(y*c-z*b), 2*(z*a-x*c), 2*(x*b-y*a)
    return [a+w*tx+y*tz-z*ty, b+w*ty+z*tx-x*tz, c+w*tz+x*ty-y*tx]


def translate(p, q, offset):
    return [a+b for a, b in zip(p, rotate(q, offset))]


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def pose_error(goal, tcp):
    delta = product(goal["q"], inverse(tcp["q"]))
    if delta[3] < 0:
        delta = [-v for v in delta]
    sine = math.hypot(*delta[:3])
    factor = 2 * math.atan2(sine, delta[3]) / sine if sine > 1e-12 else 2.0
    linear = [a-b for a, b in zip(goal["p"], tcp["p"])]
    angular = [v * factor for v in delta[:3]]
    return linear, angular


def solve_positive_definite(matrix, rhs):
    """Cholesky solution of a small, positive definite normal equation."""
    n = len(rhs)
    lower = [[0.0]*n for _ in rhs]
    for i in range(n):
        for j in range(i+1):
            value = matrix[i][j] - sum(lower[i][k]*lower[j][k] for k in range(j))
            lower[i][j] = math.sqrt(value) if i == j else value / lower[j][j]
    result = list(rhs)
    for i in range(n):
        result[i] = (result[i] - sum(lower[i][k]*result[k] for k in range(i))) / lower[i][i]
    for i in reversed(range(n)):
        result[i] = (result[i] - sum(lower[k][i]*result[k] for k in range(i+1, n))) / lower[i][i]
    return result


class RobotArm:
    def __init__(self, side):
        self.side = side
        sign = 1 if side == "right" else -1
        self.base = translate(BODY_ORIGIN, ROS_TO_XR, [0, -sign*0.03, 0.697])
        self.mount = product(ROS_TO_XR, turn(X, sign*1.5708))
        self.shoulder = translate(self.base, self.mount, (0, 0, sum(JOINT_Z_OFFSETS[:2])))
        self.j2_rotation = turn(X, sign*1.57079632679)
        first_range = (-1.396263, 3.490659) if sign == 1 else (-3.490659, 1.396263)
        second_range = (-0.17453267320510335, 3.3161253267948965) if sign == 1 else (-3.3161253267948965, 0.17453267320510335)
        self.limits = (first_range, second_range, (-1.570796, 1.570796), (0, 2.443461),
                       (-1.570796, 1.570796), (-0.785398, 0.785398), (-1.570796, 1.570796))
        self.axes = (Z, (-1, 0, 0), Z, Y, Z, X, (0, sign, 0))
        # Native hand advances on +Z and separates fingers on +/-Y. Define a
        # grasp frame with forward -Z and jaw travel +/-X for both controllers.
        self.tool_frame = [math.sqrt(0.5), math.sqrt(0.5), 0, 0]
        self.angles = [sign*0.5, sign*0.6, sign*0.12, 1.2, 0.0, 0.0, 0.0]
        if not self.solve({"p": HOME[side], "q": IDENTITY}):
            raise RuntimeError("The simulated home pose is unreachable")
        self.status = "ready"

    def forward(self, angles):
        """Return the seven joint origins, world rotation axes and achieved TCP."""
        p, q = self.base, self.mount
        points, axes = [], []
        for index, (angle, axis, length) in enumerate(zip(angles, self.axes, JOINT_Z_OFFSETS)):
            p = translate(p, q, (0, 0, length))
            if index == 1:
                q = product(q, self.j2_rotation)
            points.append(p)
            axes.append(rotate(q, axis))
            q = product(q, turn(axis, angle))
        tip = translate(p, q, (0, 0, PALM_OFFSET + TCP_FROM_PALM))
        return points, axes, {"p": tip, "q": product(q, self.tool_frame)}

    def solve(self, goal):
        """Warm-started bounded damped IK. Commit a complete reachable pose atomically."""
        wrist_length = PALM_OFFSET + TCP_FROM_PALM
        wrist = translate(goal["p"], goal["q"], (0, 0, wrist_length))
        if math.dist(wrist, self.shoulder) > sum(JOINT_Z_OFFSETS[2:]) + POSITION_TOL + wrist_length*ANGLE_TOL:
            self.status = "wrist beyond arm reach"
            return False
        angles = list(self.angles)
        damping = 0.012
        for _ in range(46):
            points, axes, tcp = self.forward(angles)
            linear, angular = pose_error(goal, tcp)
            if math.hypot(*linear) < POSITION_TOL and math.hypot(*angular) < ANGLE_TOL:
                if sum(a*b for a, b in zip(tcp["q"], goal["q"])) < 0:
                    tcp["q"] = [-v for v in tcp["q"]]
                self.angles, self.points, self.tcp = angles, points, tcp
                self.status = "tracking"
                return True
            error = linear + [a*ORIENTATION_WEIGHT for a in angular]
            loss = sum(x*x for x in error)
            # Each column maps one revolute joint's increment to a TCP twist.
            jac = []
            for p, axis in zip(points, axes):
                radius = [t-v for t, v in zip(tcp["p"], p)]
                jac.append(cross(axis, radius) + [v*ORIENTATION_WEIGHT for v in axis])
            # At a joint stop, remove outward-pointing columns and redistribute
            # the residual over the remaining joints.
            for _ in range(len(angles) + 1):
                normal = [[sum(col[i]*col[j] for col in jac) + (damping**2 if i == j else 0)
                           for j in range(6)] for i in range(6)]
                correction = solve_positive_definite(normal, error)
                step = [sum(a*b for a, b in zip(col, correction)) for col in jac]
                blocked = [i for i, (q, d, (lo, hi)) in enumerate(zip(angles, step, self.limits))
                           if (q <= lo+1e-10 and d < 0) or (q >= hi-1e-10 and d > 0)]
                if not blocked:
                    break
                for i in blocked:
                    jac[i] = [0.0]*6
            scale = min(1, 0.25 / max(1e-12, max(map(abs, step))))
            improved = False
            for _ in range(5):
                candidate = [min(hi, max(lo, q + scale*d))
                             for (lo, hi), q, d in zip(self.limits, angles, step)]
                _, _, trial = self.forward(candidate)
                dp, dr = pose_error(goal, trial)
                trial_loss = sum(x*x for x in dp) + ORIENTATION_WEIGHT**2 * sum(x*x for x in dr)
                if trial_loss < loss - 1e-14:
                    angles = candidate
                    damping = max(0.005, damping * 0.75)
                    improved = True
                    break
                scale *= 0.5
            if not improved:
                damping *= 3
                if damping > 0.5:
                    break
        self.status = "no local IK solution within joint limits"
        return False

    def snapshot(self, closure):
        p, q = self.tcp["p"], self.tcp["q"]
        half_gap = SLIDER_STROKE * (1-closure)
        fingers = [translate(p, q, (sign*half_gap, 0, 0)) for sign in (-1, 1)]
        roots = [translate(p, q, (sign*(half_gap+0.005), 0, TCP_FROM_PALM-0.015))
                 for sign in (-1, 1)]
        return {"model": MODEL_NAME, "status": self.status, "base_m": self.base,
                "joint_angles_rad": self.angles, "joint_positions_m": self.points, "tcp": self.tcp,
                "palm_m": translate(p, q, (0, 0, TCP_FROM_PALM)), "finger_roots_m": roots,
                "fingers_m": fingers, "jaw_opening_m": 2*half_gap}

    def set_joints(self, angles):
        """Apply a simulated posture already bounded by the model joint limits."""
        if len(angles) != len(self.limits) or any(not lo <= q <= hi for q,(lo,hi) in zip(angles,self.limits)):
            raise ValueError("Simulated joint target outside model limits")
        self.points, _, self.tcp = self.forward(angles)
        self.angles = list(angles)
        self.status = "tracking"
