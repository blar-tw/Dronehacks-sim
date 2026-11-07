"""A kinematic stand-in for PX4 + the vehicle, for end-to-end tests of nav.mission without ROS.

Not a flight model: position follows the setpoint with first-order lag and a speed cap; PX4's
mode logic is reduced to what the mission relies on (Offboard needs a recent setpoint stream,
arm, AUTO_LAND descends and lands, auto-disarm after landing, leaving Offboard on stream loss).
Truth is map-frame ENU; PX4's local frame is NED with its origin at the takeoff point, optionally
with a start offset/yaw error to exercise the aligner.
"""
from __future__ import annotations

import math

import numpy as np

from nav import frames as F
from refnav import core as RC

NAV_POSCTL, NAV_OFFBOARD, NAV_LAND = 2, 14, 18


class FakePX4:
    def __init__(self, start_xy, start_yaw=math.pi / 2, tau=0.35, vmax=1.5, yaw_rate=math.radians(120),
                 odom_err=F.SE2()):
        # odom_err: the aircraft really starts (x, y) off start_xy, and PX4's frame is rotated
        # by yaw from the map's - what the mission does NOT know at takeoff
        self.truth = np.array([start_xy[0] + odom_err.x, start_xy[1] + odom_err.y, 0.0])   # map ENU
        self.yaw = start_yaw                                        # map ENU yaw
        # odom (PX4 local, ENU) origin at the real takeoff spot
        self.T_true = F.SE2(self.truth[0], self.truth[1], odom_err.yaw)
        self.tau, self.vmax, self.yaw_rate = tau, vmax, yaw_rate
        self.armed = False
        self.nav = NAV_POSCTL
        self.landed = True
        self.sp = None
        self.last_stream = -1e9
        self.landed_since = None
        self.collisions = 0
        self.log = []

    # ---- what the mission sees
    def vehicle(self):
        Ti = self.T_true.inverse()
        x, y, z = Ti.apply(*self.truth)
        yaw_odom = F.wrap(self.yaw - self.T_true.yaw)
        n, e, d = F.enu_to_ned(x, y, z)
        hx = F.yaw_to_heading(yaw_odom)
        q = (math.cos(hx / 2), 0.0, 0.0, math.sin(hx / 2))
        return RC.Vehicle(pos=(n, e, d), heading=hx, pos_ok=True, q=q, armed=self.armed,
                          nav_state=self.nav, landed=self.landed)

    # ---- apply the mission's Output, then advance dt
    def apply(self, now, out, dt):
        if out.stream and out.setpoint is not None:
            self.sp = out.setpoint
            self.last_stream = now
        for c in out.commands:
            if c == "offboard" and now - self.last_stream < 0.5:
                self.nav = NAV_OFFBOARD
            elif c == "arm" and self.nav == NAV_OFFBOARD and not self.armed:
                self.armed, self.landed_since = True, None
            elif c == "land":
                self.nav = NAV_LAND
            elif c == "disarm" and self.landed:
                self.armed = False
        if self.nav == NAV_OFFBOARD and now - self.last_stream > 0.5 and self.armed:
            self.nav = NAV_LAND                                    # offboard loss failsafe
            self.log.append((now, "failsafe: offboard stream lost"))
        self._advance(now, dt)

    def _advance(self, now, dt):
        if not self.armed:
            return
        if self.nav == NAV_LAND:
            self.truth[2] = max(0.0, self.truth[2] - 0.5 * dt)
        elif self.nav == NAV_OFFBOARD and self.sp is not None:
            n, e, d, hdg = self.sp
            target = np.array(self.T_true.apply(*F.ned_to_enu(n, e, d)))
            step = (target - self.truth) * min(1.0, dt / self.tau)
            sp = np.linalg.norm(step) / dt
            if sp > self.vmax:
                step *= self.vmax / sp
            self.truth = self.truth + step
            if not math.isnan(hdg):
                want = F.wrap(F.heading_to_yaw(hdg) + self.T_true.yaw)
                dy = F.wrap(want - self.yaw)
                self.yaw = F.wrap(self.yaw + max(-self.yaw_rate * dt, min(self.yaw_rate * dt, dy)))
        self.truth[2] = max(0.0, self.truth[2])
        on_ground = self.truth[2] <= 0.02
        if on_ground and self.nav == NAV_LAND:
            self.landed = True
            if self.landed_since is None:
                self.landed_since = now
            elif now - self.landed_since > 1.0:
                self.armed = False                                  # PX4 auto-disarm
                self.nav = NAV_POSCTL
        elif self.truth[2] > 0.1:
            self.landed = False


def wall_points(course, truth_xyz, yaw, T_map_odom, rng=3.0, fov_deg=72.0, n_rays=80, noise=0.0, seed=0):
    """Fake depth: ray-cast the course faces at the drone's height inside a forward FOV, return the
    hits as odom-frame xy (what nav.landmarks.wall_points would return)."""
    faces = course.faces_at(truth_xyz[2])
    if not faces:
        return np.zeros((0, 2))
    A = np.array([f[0] for f in faces])
    B = np.array([f[1] for f in faces])
    o = np.asarray(truth_xyz[:2])
    hits = []
    for a in np.linspace(-math.radians(fov_deg) / 2, math.radians(fov_deg) / 2, n_rays):
        d = np.array([math.cos(yaw + a), math.sin(yaw + a)])
        e = B - A
        den = d[0] * e[:, 1] - d[1] * e[:, 0]
        with np.errstate(divide="ignore", invalid="ignore"):
            w = A - o
            t = (w[:, 0] * e[:, 1] - w[:, 1] * e[:, 0]) / den
            u = (w[:, 0] * d[1] - w[:, 1] * d[0]) / den
        ok = (np.abs(den) > 1e-9) & (t > 0.05) & (t < rng) & (u >= 0) & (u <= 1)
        if ok.any():
            hits.append(o + d * t[ok].min())
    if not hits:
        return np.zeros((0, 2))
    P = np.array(hits)
    if noise:
        P = P + np.random.default_rng(seed).normal(0, noise, P.shape)
    Ti = T_map_odom.inverse()
    return np.array([Ti.apply(x, y)[:2] for x, y in P])


def clearance(course, xyz, z_band=0.1):
    """Distance from a map point to the nearest obstacle footprint at that height (negative inside)."""
    best = math.inf
    p = np.asarray(xyz[:2])
    for b in course.boxes_at(xyz[2] - z_band, xyz[2] + z_band):
        c, s = math.cos(-b.yaw), math.sin(-b.yaw)
        q = p - (b.cx, b.cy)
        lx, ly = c * q[0] - s * q[1], s * q[0] + c * q[1]
        dx, dy = abs(lx) - b.length / 2, abs(ly) - b.thick / 2
        out = math.hypot(max(dx, 0), max(dy, 0))
        best = min(best, out if out > 0 else max(dx, dy))
    for cy in course.cylinders_at(xyz[2] - z_band, xyz[2] + z_band):
        best = min(best, math.hypot(p[0] - cy.cx, p[1] - cy.cy) - cy.radius)
    return best
