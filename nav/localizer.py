"""Map localization: T_map_odom from depth points matched to the map's wall faces.

wall_points(): one depth frame -> odom-frame xy points of surfaces at the flight height. Floor
points are dropped by height above the floor (floor = the vehicle's odom z at takeoff).

Localizer: state x = (tx, ty, yaw) of T_map_odom with covariance P. predict() grows the vehicle's
uncertainty with distance flown. update() is point-to-line ICP solved as a MAP step against the
prior (Gauss-Newton, re-associating each iteration): residual = signed distance of a point to its
nearest face. A fit is accepted only if its median |residual| <= 6 cm and the step is within 4 sigma
of the prior. Sigmas are reported at the vehicle (yaw error times the lever arm counts).

Hand-placed walls (uncertain_m, see nav/course.py) are not matched against, and points that land
within uncertain_m + uncertain_pad_m of one are dropped before the fit, so they cannot be matched to
a nearby real wall instead.
"""
import math
from dataclasses import dataclass

import numpy as np

from refnav.depth import level_points, level_rotation

from .frames import SE2, rot, wrap


@dataclass
class LocParams:
    stride: int = 4
    border_px: int = 16
    rmin: float = 0.25
    rmax: float = 3.5
    band_m: float = 0.20              # points within +-band of the body height
    floor_clear_m: float = 0.15       # and this far above the floor
    max_points: int = 300
    min_points: int = 40
    match_m: float = 0.25             # association gate point -> face
    meas_sigma_m: float = 0.05
    points_eff: int = 30              # one frame counts as at most this many independent points
    iters: int = 6
    max_median_m: float = 0.06
    gate_sigma: float = 4.0
    init_sigma_xy_m: float = 0.10
    init_sigma_yaw_deg: float = 2.0
    drift_xy_per_m: float = 0.03
    drift_yaw_deg_per_m: float = 0.5
    min_sigma_xy_m: float = 0.02
    min_sigma_yaw_deg: float = 0.5
    uncertain_pad_m: float = 0.10


def wall_points(depth_m, K, R_body_cam, t_body_cam, q_wxyz, pose_odom, floor_z, p: LocParams = LocParams()):
    """(N,2) odom xy of surfaces at the body height. pose_odom = (x, y, z, yaw) odom ENU."""
    pts = level_points(depth_m, K, R_body_cam, t_body_cam, level_rotation(q_wxyz),
                       stride=p.stride, rmin=p.rmin, rmax=p.rmax, border_px=p.border_px)   # (fwd, right, down)
    x, y, z, yaw = pose_odom
    pts = pts[(np.abs(pts[:, 2]) < p.band_m) & (z - pts[:, 2] - floor_z > p.floor_clear_m)]
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + pts[:, 0] * c + pts[:, 1] * s, y + pts[:, 0] * s - pts[:, 1] * c], axis=1)


def _lever(T, odom_xy):
    """J with (d vehicle map xy, d yaw) = J (d tx, d ty, d yaw)."""
    r = rot(T.yaw) @ np.asarray(odom_xy, float)
    return np.array([[1.0, 0.0, -r[1]], [0.0, 1.0, r[0]], [0.0, 0.0, 1.0]])


class Localizer:
    def __init__(self, course, z, p: LocParams = LocParams()):
        self.p = p
        self.A, self.B, self.N = course.faces_at(z)
        self.uncertain = [b for b in course.boxes_at(z, z) if b.uncertain > 0.0]
        self.D = self.B - self.A
        self.L2 = (self.D ** 2).sum(1)
        self.T = SE2()
        self.P = np.eye(3)
        self.vehicle = np.zeros(2)                      # odom xy of the vehicle at the last call
        self.last = "not started"
        self.accepted = 0

    def start(self, map_xy, map_yaw, odom_xy, odom_yaw):
        self.T = SE2.from_poses(map_xy, map_yaw, odom_xy, odom_yaw)
        self.vehicle = np.asarray(odom_xy, float)
        sv = np.diag([self.p.init_sigma_xy_m ** 2] * 2 + [math.radians(self.p.init_sigma_yaw_deg) ** 2])
        Ji = np.linalg.inv(_lever(self.T, self.vehicle))
        self.P = Ji @ sv @ Ji.T

    def vehicle_cov(self):
        J = _lever(self.T, self.vehicle)
        return J @ self.P @ J.T

    def sigma_xy(self):
        return float(math.sqrt(np.linalg.eigvalsh(self.vehicle_cov()[:2, :2])[-1]))

    def predict(self, odom_xy):
        """The vehicle moved to odom_xy: grow its sigmas by the drift per metre."""
        dist = float(np.linalg.norm(np.asarray(odom_xy) - self.vehicle))
        self.vehicle = np.asarray(odom_xy, float)
        if dist <= 0.0:
            return
        C = self.vehicle_cov()
        s = np.sqrt(np.diag(C))
        k = np.array([self.p.drift_xy_per_m] * 2 + [math.radians(self.p.drift_yaw_deg_per_m)]) * dist
        C = C + np.diag((s + k) ** 2 - s ** 2)
        Ji = np.linalg.inv(_lever(self.T, self.vehicle))
        self.P = Ji @ C @ Ji.T

    def _match(self, Pts, x):
        Q = Pts @ rot(x[2]).T + x[:2]
        rel = Q[:, None, :] - self.A[None]
        u = np.einsum("nfk,fk->nf", rel, self.D) / self.L2
        dist = np.linalg.norm(rel - np.clip(u, 0.0, 1.0)[..., None] * self.D, axis=2)
        dist[(u < 0.0) | (u > 1.0)] = np.inf                        # past a face's ends: not on that face
        S = rot(x[2]) @ self.vehicle + x[:2]                       # faces turned away from the camera
        dist[:, ((S - self.A) * self.N).sum(1) <= 0.0] = np.inf
        f = np.argmin(dist, axis=1)
        keep = dist[np.arange(len(Q)), f] < self.p.match_m
        f, q = f[keep], Q[keep]
        n = self.N[f]
        r = ((q - self.A[f]) * n).sum(1)
        v = q - x[:2]
        H = np.stack([n[:, 0], n[:, 1], n[:, 1] * v[:, 0] - n[:, 0] * v[:, 1]], axis=1)
        return H, r

    def update(self, pts_odom):
        """One frame of points (N,2, odom). Returns True if the fit was accepted."""
        p = self.p
        Pts = np.asarray(pts_odom, float).reshape(-1, 2)
        if self.uncertain and len(Pts):
            Q = self.T.apply(Pts)
            keep = np.ones(len(Pts), bool)
            for b in self.uncertain:
                keep &= b.distance(Q) > b.uncertain + p.uncertain_pad_m
            Pts = Pts[keep]
        if len(Pts) > p.max_points:
            Pts = Pts[np.linspace(0, len(Pts) - 1, p.max_points).astype(int)]
        if len(Pts) < p.min_points:
            self.last = f"{len(Pts)} points"
            return False
        x0 = self.T.vec()
        Pi = np.linalg.inv(self.P)
        x = x0.copy()
        for _ in range(p.iters):
            H, r = self._match(Pts, x)
            if len(r) < p.min_points:
                self.last = f"{len(r)} of {len(Pts)} points near a wall"
                return False
            w = min(1.0, p.points_eff / len(r)) / p.meas_sigma_m ** 2
            e = x - x0
            e[2] = wrap(e[2])
            x = x + np.linalg.solve(Pi + w * H.T @ H, -(Pi @ e + w * H.T @ r))
        H, r = self._match(Pts, x)
        med = float(np.median(np.abs(r)))
        d = x - x0
        d[2] = wrap(d[2])
        J = _lever(self.T, self.vehicle)
        dv = J @ d
        m = math.sqrt(float(dv @ np.linalg.solve(J @ self.P @ J.T, dv)))
        if med > p.max_median_m:
            self.last = f"rejected: median {med * 100:.1f} cm"
            return False
        if m > p.gate_sigma:
            self.last = f"rejected: step {np.hypot(*dv[:2]):.2f} m {math.degrees(dv[2]):+.1f} deg = {m:.1f} sigma"
            return False
        w = min(1.0, p.points_eff / len(r)) / p.meas_sigma_m ** 2
        self.T = SE2(*x)
        P = np.linalg.inv(Pi + w * H.T @ H)
        C = J @ P @ J.T                                             # floor the sigmas at the vehicle
        ev, V = np.linalg.eigh(C[:2, :2])
        C[:2, :2] = V @ np.diag(np.maximum(ev, p.min_sigma_xy_m ** 2)) @ V.T
        C[2, 2] = max(C[2, 2], math.radians(p.min_sigma_yaw_deg) ** 2)
        Ji = np.linalg.inv(J)
        self.P = Ji @ C @ Ji.T
        self.accepted += 1
        self.last = f"ok: {len(r)} pts, median {med * 100:.1f} cm, step {np.hypot(*dv[:2]):.2f} m {math.degrees(dv[2]):+.1f} deg"
        return True
