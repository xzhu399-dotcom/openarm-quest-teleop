"""Trial-start joints and interruptible joint-space returns in simulation."""

import math


class Trial:
    def __init__(self):
        self.start = None
        self.session = None
        self.required = set()
        self.path = None
        self.result = {"status": "idle", "reason": ""}

    def begin(self, hands, session):
        self.path = None
        self.result = {"status": "idle", "reason": ""}
        self.start = {s: list(h.robot.angles) for s, h in hands.items()}
        self.session = session
        self.required = {s for s, h in hands.items() if h.usable}
        return {"joints_rad": self.start, "closure": {s: h.closure for s, h in hands.items()}}

    def observe(self, hands, session):
        if self.start is not None:
            if self.session is None:
                self.session = session
            self.required.update(s for s, h in hands.items() if h.usable)

    @staticmethod
    def _input_problem(hands, required):
        if not required or any(not hands[s].usable for s in required):
            return "Tracking is required for each controller participating in the return."
        if any(h.grip_engaged for h in hands.values()):
            return "Both side grips must remain released for a return."
        return ""

    def finish(self, hands, session, return_requested, held_reason):
        target, self.start = self.start, None
        self.path = None
        if not return_requested:
            self.result = {"status": "stayed", "reason": "Trial ended at the current arm poses and jaw openings."}
            return
        problem = held_reason
        if target is None or self.session != session:
            problem = problem or "The trial-start joints belong to a different input session."
        self._return_to(hands, target, self.required, problem)

    def restore_initial(self, hands, initial_joints, blocked):
        if self.path is not None and self.result.get("target") == "home":
            return
        required = {side for side, hand in hands.items() if hand.usable}
        self._return_to(hands, initial_joints, required, blocked, home=True)

    def _return_to(self, hands, target, required, blocked, home=False):
        self.path = None
        self.result = {"target": "home"} if home else {}
        problem = blocked or self._input_problem(hands, required)
        if problem:
            self.result.update(status="skipped", reason=problem)
            return
        origin = {s: list(h.robot.angles) for s, h in hands.items()}
        distance = max(abs(a-b) for s in hands for a,b in zip(origin[s], target[s]))
        if distance < 1e-12:
            self.result.update(status="completed", reason="Arms are at their initial position." if home
                               else "Arms are at the trial-start joints.")
            return
        # Quintic interpolation stays inside the convex box of per-joint limits;
        # peak speed is 1.875 * distance / duration. Jaws are held independently.
        duration = max(0.6, 1.875 * distance / math.radians(30))
        source = {s: self._source(hands[s]) for s in required}
        self.path = {"origin":origin, "target":target, "duration":duration, "elapsed":0., "source":source}
        self.result.update(status="returning", reason="Returning both arms to their initial position." if home
                           else "Returning simulated joints; jaw openings are held.")

    @staticmethod
    def _source(hand):
        return tuple(hand.raw[k] for k in ("profile", "source", "grip_epoch"))

    def cancel(self, reason):
        if self.path is not None:
            self.path = None
            self.result.update(status="cancelled", reason=reason)

    def advance(self, hands, dt):
        path = self.path
        problem = self._input_problem(hands, path["source"])
        if not problem and any(self._source(hands[s]) != key for s,key in path["source"].items()):
            problem = "A controller changed or its grip was operated during return."
        if problem:
            self.cancel(problem)
            return
        path["elapsed"] = min(path["duration"], path["elapsed"] + dt)
        t = path["elapsed"] / path["duration"]
        blend = min(1., t*t*t * (10 - 15*t + 6*t*t))
        for side, hand in hands.items():
            a, b = path["origin"][side], path["target"][side]
            if a == b:
                continue
            angles = list(b) if t == 1 else [x+(y-x)*blend for x,y in zip(a,b)]
            hand.robot.set_joints(angles)
            hand.tool = hand.robot.tcp
        if t == 1:
            self.path = None
            self.result.update(status="completed", reason="Initial arm position reached. Jaw openings held."
                               if self.result.get("target") == "home" else "Trial-start joints reached. Jaw openings held.")
