"""Carrot follower over the planned waypoints (map frame).

The carrot advances along the path at the current speed but never more than lead_m ahead of the
vehicle's projection. While moving the vehicle faces a point yaw_lead_m further along the path (not
past a stop), so it starts turning before a corner instead of at it.

Sigma (the localizer's, at the vehicle): the speed falls linearly from full at slow_sigma to
min_scale at stop_sigma; above stop_sigma stop and look until it is back to resume_sigma (at most
look_max_s). VIO DEGRADED halves the speed.

At a "stop" waypoint the vehicle settles on it. If sigma is already <= resume_sigma it turns to the
next segment and goes on; otherwise it looks at the most-featured view within +-look_within_deg of
the next segment first. The last window_slow_m before the "window" waypoint and the window segment are
flown at window_speed. If at the window waypoint sigma <= window_sigma and the map correction is in
(blend_ok, set by the mission), it flies straight through; otherwise it stops, faces the window wall
(= the next segment) and waits for that, at most window_wait_s.
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
    look_s: float = 1.0               # hold at the view at least this long once facing it
    look_max_s: float = 5.0
    look_within_deg: float = 90.0     # a look stays within this of the next segment's heading
    face_tol_deg: float = 10.0
    slow_sigma_m: float = 0.10
    min_scale: float = 0.35           # speed factor at stop_sigma
    stop_sigma_m: float = 0.15
    resume_sigma_m: float = 0.08
    window_sigma_m: float = 0.05
    window_wait_s: float = 5.0
    window_speed_mps: float = 0.15
    window_slow_m: float = 0.6
    yaw_lead_m: float = 0.5


class Follower:
    def __init__(self, wps, scorer, p: FollowParams = FollowParams()):
        self.scorer, self.p = scorer, p
        self.replace(wps)

    def replace(self, wps):
        self.w = wps
        self.i = 0                    # current segment: w[i] -> w[i+1]
        self.s = 0.0
        self.mode = "go"
        self.t_mode = None
        self.look_yaw = None
        self.done = False
        self.slow = False             # set by the mission (VIO DEGRADED)
        self.blend_ok = True          # set by the mission: the map correction is in the setpoints

    def _seg(self, i=None):
        i = self.i if i is None else i
        a = np.array([self.w[i].x, self.w[i].y])
        b = np.array([self.w[i + 1].x, self.w[i + 1].y])
        L = float(np.linalg.norm(b - a))
        return a, b, L, (b - a) / L

    def _heading(self, i):
        _, _, _, u = self._seg(i)
        return math.atan2(u[1], u[0])

    def _speed(self, sigma, L):
        p = self.p
        f = 1.0 - (1.0 - p.min_scale) * min(max((sigma - p.slow_sigma_m) / (p.stop_sigma_m - p.slow_sigma_m), 0.0), 1.0)
        v = p.speed_mps * f * (0.5 if self.slow else 1.0)
        if self.w[self.i].kind == "window" or (self.w[self.i + 1].kind == "window" and L - self.s < p.window_slow_m):
            v = min(v, p.window_speed_mps)
        return v

    def _ahead(self, pos, heading):
        """Heading toward the path point yaw_lead_m past the carrot (stops at a stop waypoint)."""
        i, s, left = self.i, self.s, self.p.yaw_lead_m
        while True:
            a, b, L, u = self._seg(i)
            if L - s >= left or self.w[i + 1].kind == "stop" or i + 2 >= len(self.w):
                q = a + u * min(s + left, L)
                break
            left -= L - s
            i, s = i + 1, 0.0
        d = q - pos
        return math.atan2(d[1], d[0]) if np.linalg.norm(d) > 0.15 else heading

    def _set(self, mode, now):
        self.mode, self.t_mode = mode, now

    def _look(self, now, xy, around):
        self.look_yaw = self.scorer.best(xy, around=around, within_deg=self.p.look_within_deg)[0]
        self._set("look", now)

    def step(self, now, dt, pos, yaw, sigma):
        """pos, yaw: the vehicle in the map. -> (carrot xy, target height, target yaw)."""
        p = self.p
        if self.t_mode is None:
            self.t_mode = now
        a, b, L, u = self._seg()
        heading = math.atan2(u[1], u[0])
        last = self.i + 2 >= len(self.w)
        nxt = heading if last else self._heading(self.i + 1)
        hold = a + u * self.s

        if self.mode == "go":
            if sigma > p.stop_sigma_m:
                self._look(now, pos, heading)
                return hold, self.w[self.i].z, yaw
            v = self._speed(sigma, L)
            proj = float(np.dot(pos - a, u))
            self.s = min(self.s + v * dt, L, max(proj, 0.0) + p.lead_m)
            carrot, z, tyaw = a + u * self.s, self.w[self.i].z, self._ahead(pos, heading)   # before _next resets s
            kind = self.w[self.i + 1].kind
            if self.s >= L:
                if kind == "stop":
                    self._set("settle", now)
                elif kind == "window":
                    if not (sigma <= p.window_sigma_m and self.blend_ok):
                        self._set("settle", now)          # stop, face the window wall, wait
                    elif np.linalg.norm(pos - b) < p.arrive_m:
                        self._next(now)                   # localized: straight through
                    # else: the carrot waits at b until the vehicle is on it (the window is ~13 cm wider than the x500 each side)
                else:
                    self._next(now)
            return carrot, z, tyaw

        if self.mode == "settle":
            if np.linalg.norm(pos - b) < p.arrive_m or now - self.t_mode > p.settle_max_s:
                if self.w[self.i + 1].kind == "window":
                    self.look_yaw = nxt
                    self._set("window", now)
                elif last:
                    self.done = True
                elif sigma <= p.resume_sigma_m:
                    self._next(now)
                    self._set("turn", now)
                else:
                    self._look(now, b, nxt)
            return b, self.w[self.i + 1].z, yaw

        if self.mode == "window":
            if abs(wrap(self.look_yaw - yaw)) > math.radians(p.face_tol_deg):
                self.t_mode = now                    # the wait starts once facing the window wall
            elif (sigma <= p.window_sigma_m and self.blend_ok) or now - self.t_mode > p.window_wait_s:
                self._next(now)
                self._set("go", now)
            return b, self.w[self.i + 1].z, self.look_yaw

        if self.mode == "look":
            if abs(wrap(self.look_yaw - yaw)) > math.radians(p.face_tol_deg):
                self.t_mode = now                    # the look starts once the camera faces the view
            elif now - self.t_mode > p.look_s and (sigma <= p.resume_sigma_m or now - self.t_mode > p.look_max_s):
                if self.s >= L:
                    if last:
                        self.done = True
                    else:
                        self._next(now)
                self._set("turn", now)
            return (b if self.s >= L else hold), self.w[self.i].z, self.look_yaw

        if self.mode == "turn":
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
