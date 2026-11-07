"""nav.posebuf - recent PX4 poses by receive time, so a depth frame uses the pose at its own stamp.
PURE (no ROS).

A depth image arrives tens of ms after it was taken. Rotating its points with the newest attitude
turns the cloud by yaw_rate x latency (60 deg/s x 50 ms = 3 deg, 21 cm at 4 m). The node stores
each PX4 pose with the node-clock time it arrived and looks up the one nearest the image stamp
(same clock: sim time in the lab, system time on the Jetson). yaw_rate() lets the node skip frames
taken while the vehicle turns fast, where any remaining timing error matters most.
"""
from __future__ import annotations

import math
from collections import deque

from .frames import wrap


class PoseBuffer:
    def __init__(self, keep_s: float = 1.0):
        self.keep_s = keep_s
        self._d = deque()                 # (t, value), t increasing

    def add(self, t: float, value) -> None:
        if self._d and t < self._d[-1][0]:
            self._d.clear()               # clock went back (sim restart)
        self._d.append((t, value))
        while self._d and t - self._d[0][0] > self.keep_s:
            self._d.popleft()

    def latest(self):
        return self._d[-1] if self._d else None

    def at(self, t: float, tol_s: float):
        """(t_i, value) of the sample nearest t, or None if none is within tol_s."""
        if not self._d:
            return None
        best = min(self._d, key=lambda s: abs(s[0] - t))
        return best if abs(best[0] - t) <= tol_s else None


def yaw_rate_dps(headings: PoseBuffer, t: float, window_s: float = 0.2) -> float:
    """|d heading / dt| (deg/s) over the samples within window_s / 2 of t (heading in radians,
    the buffer's values); 0 if fewer than two samples."""
    pts = [(ti, h) for ti, h in headings._d if abs(ti - t) <= window_s / 2]
    if len(pts) < 2 or pts[-1][0] <= pts[0][0]:
        return 0.0
    turn = sum(wrap(b[1] - a[1]) for a, b in zip(pts, pts[1:]))
    return abs(math.degrees(turn) / (pts[-1][0] - pts[0][0]))
