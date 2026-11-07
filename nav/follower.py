"""Carrot follower over the planned waypoints (map frame).

The carrot advances along the path at the current speed but never more than lead_m ahead of the
vehicle's projection. Speed drops with the localizer's sigma: > slow_sigma half speed, > stop_sigma
stop and look. At a "stop" waypoint the vehicle settles on it, turns to the most-featured view and
holds while the localizer updates, then turns to the next segment and goes on. While moving it faces
the direction of travel.
"""
import math
from dataclasses import dataclass

import numpy as np

from .frames import wrap


@dataclass
class FollowParams:
    speed_mps: float = 0.4
    lead_m: float = 0.5
    arrive_m: float = 0.15
    settle_max_s: float = 4.0
    look_s: float = 2.0               # hold at the best view at least this long
    look_max_s: float = 5.0           # ...and until sigma <= slow_sigma, at most this long
    face_tol_deg: float = 10.0
    slow_sigma_m: float = 0.10
    stop_sigma_m: float = 0.15


class Follower:
    def __init__(self, wps, scorer, p: FollowParams = FollowParams()):
        self.w, self.scorer, self.p = wps, scorer, p
        self.i = 0                    # current segment: w[i] -> w[i+1]
        self.s = 0.0
        self.mode = "go"
        self.t_mode = None
        self.look_yaw = None
        self.done = False

    def _seg(self):
        a = np.array([self.w[self.i].x, self.w[self.i].y])
        b = np.array([self.w[self.i + 1].x, self.w[self.i + 1].y])
        L = float(np.linalg.norm(b - a))
        return a, b, L, (b - a) / L

    def _set(self, mode, now):
        self.mode, self.t_mode = mode, now

    def step(self, now, dt, pos, yaw, sigma):
        """pos, yaw: the vehicle in the map. -> (carrot xy, target height, target yaw)."""
        p = self.p
        if self.t_mode is None:
            self.t_mode = now
        a, b, L, u = self._seg()
        heading = math.atan2(u[1], u[0])
        if self.mode == "go":
            if sigma > p.stop_sigma_m:
                self.look_yaw = self.scorer.best(pos)[0]
                self._set("look", now)
            else:
                v = p.speed_mps * (0.5 if sigma > p.slow_sigma_m else 1.0)
                proj = float(np.dot(pos - a, u))
                self.s = min(self.s + v * dt, L, max(proj, 0.0) + p.lead_m)
                if self.s >= L:
                    if self.w[self.i + 1].kind == "stop":
                        self._set("settle", now)
                    else:
                        self._next(now)
            return a + u * self.s, self.w[self.i].z, heading
        if self.mode == "settle":
            if np.linalg.norm(pos - b) < p.arrive_m or now - self.t_mode > p.settle_max_s:
                self.look_yaw = self.scorer.best(b)[0]
                self._set("look", now)
            return b, self.w[self.i + 1].z, yaw
        if self.mode == "look":
            hold = a + u * self.s
            if abs(wrap(self.look_yaw - yaw)) > math.radians(p.face_tol_deg):
                self.t_mode = now                    # the look starts once the camera faces the view
            elif now - self.t_mode > p.look_s and (sigma <= p.slow_sigma_m or now - self.t_mode > p.look_max_s):
                if self.s >= L:
                    self._next(now)
                    if self.done:
                        return b, self.w[-1].z, self.look_yaw
                    a, b, L, u = self._seg()
                    self._set("turn", now)
                    return a, self.w[self.i].z, self.look_yaw
                self._set("turn", now)
            return hold, self.w[self.i].z, self.look_yaw
        if self.mode == "turn":
            hold = a + u * self.s
            if abs(wrap(heading - yaw)) < math.radians(p.face_tol_deg):
                self._set("go", now)
            return hold, self.w[self.i].z, heading
        raise ValueError(self.mode)

    def _next(self, now):
        if self.i + 2 >= len(self.w):
            self.done = True
            return
        self.i += 1
        self.s = 0.0
        self._set("go", now)
