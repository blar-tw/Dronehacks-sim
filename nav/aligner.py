"""nav.aligner - the map <- odom transform (ARCHITECTURE.md 6.1). PURE (numpy only).

State: T_map_odom = (tx, ty, yaw) as nav.frames.SE2 (p_map = R(yaw) p_odom + t) and its 3x3
covariance. Never touches VIO or PX4: the mission converts map targets through T.

update() is an iterated EKF (MAP) step: minimise
    (x - x_prior)' P^-1 (x - x_prior) + sum_i w_i r_i(x)^2
with r_i = n_f . (T(p_i) - a_f), the signed distance of depth point i (moved to the map with the
current estimate) to the line of its nearest wall face f in course.faces_at(z). Association is
redone at every Gauss-Newton iteration (point-to-line ICP). Each residual only constrains the
face normal (and yaw through the lever arm), so unobserved directions keep their prior: one straight
wall corrects the cross-wall position and yaw, never the along-wall position.

Weights w_i = huber(r_i) * cap_f / meas_sigma^2. cap_f = min(1, max_points_eff / n_f) caps the
information one face can add per update to max_points_eff points: the points on a wall share its
map error and the frame's attitude / depth bias, so hundreds of them are not independent.

Process noise is "sigma grows drift_per_m per metre flown", applied at the vehicle: VIO drift
accumulates where the vehicle is, so predict()/init_at_launch() take the vehicle's odom xy and
build the (tx, ty, yaw) covariance with the lever arm from the odom origin.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .frames import SE2, wrap


@dataclass
class AlignerParams:
    init_sigma_xy_m: float = 0.10
    init_sigma_yaw_deg: float = 3.0
    drift_per_m: float = 0.02          # process noise: sigma grows this much per metre flown
    drift_yaw_deg_per_m: float = 0.3
    meas_sigma_m: float = 0.05         # per-point point-to-face noise
    match_max_m: float = 0.25          # point-to-face association gate
    min_points: int = 30
    max_step_xy_m: float = 0.30        # reject an update that moves the vehicle's map position more
    max_step_yaw_deg: float = 8.0
    max_points: int = 300              # subsample per update
    icp_iters: int = 5
    max_points_eff: int = 15           # information cap: at most this many points' worth per face
    huber_k: float = 2.0               # Huber threshold in meas_sigma units
    corner_m: float = 0.05             # drop a point whose 2nd-best face (another line) is within this
                                       # of its best one, or that lies this far past its face's end


@dataclass
class AlignResult:
    accepted: bool
    why: str
    n: int            # matched points used
    dx: float         # proposed change of the reference point's map position (m)
    dy: float
    dyaw: float       # proposed change of yaw (rad)


def _lever(T: SE2, odom_xy) -> np.ndarray:
    """A with (d_pos_vehicle, d_yaw) = A (d_tx, d_ty, d_yaw) for a vehicle at odom_xy."""
    c, s = math.cos(T.yaw), math.sin(T.yaw)
    rx, ry = c * odom_xy[0] - s * odom_xy[1], s * odom_xy[0] + c * odom_xy[1]
    return np.array([[1.0, 0.0, -ry], [0.0, 1.0, rx], [0.0, 0.0, 1.0]])


class Aligner:
    def __init__(self, course, params=AlignerParams()):
        self.course, self.p = course, params
        self.T = SE2()
        self.cov = self._vehicle_cov_to_T(np.diag(self._init_var()), (0.0, 0.0))
        self.t_update = None
        self._faces = {}

    # ------------------------------------------------------------------ state
    def _init_var(self):
        sxy, syaw = self.p.init_sigma_xy_m, math.radians(self.p.init_sigma_yaw_deg)
        return np.array([sxy * sxy, sxy * sxy, syaw * syaw])

    def _vehicle_cov_to_T(self, cov_v, odom_xy):
        Ai = np.linalg.inv(_lever(self.T, odom_xy))
        return Ai @ cov_v @ Ai.T

    def init_at_launch(self, odom_xy, launch_xy, yaw_offset=0.0) -> None:
        """T such that odom_xy (the vehicle at takeoff) maps to launch_xy, rotated by yaw_offset.
        The initial sigmas are about the vehicle, not about the odom origin."""
        c, s = math.cos(yaw_offset), math.sin(yaw_offset)
        ox, oy = odom_xy
        self.T = SE2(launch_xy[0] - (c * ox - s * oy), launch_xy[1] - (s * ox + c * oy), wrap(yaw_offset))
        self.cov = self._vehicle_cov_to_T(np.diag(self._init_var()), odom_xy)

    def predict(self, dist_m, odom_xy=(0.0, 0.0)) -> None:
        """Grow the vehicle's position sigma by drift_per_m and its yaw sigma by drift_yaw_deg_per_m
        for every metre flown (linear in distance, independent of how often this is called)."""
        A = _lever(self.T, odom_xy)
        cv = A @ self.cov @ A.T
        sig = np.sqrt(np.diag(cv))
        k = np.array([self.p.drift_per_m, self.p.drift_per_m, math.radians(self.p.drift_yaw_deg_per_m)])
        cv += np.diag((sig + k * abs(dist_m)) ** 2 - sig ** 2)
        Ai = np.linalg.inv(A)
        self.cov = Ai @ cv @ Ai.T

    def sigma_xy(self) -> float:
        """sqrt of the larger eigenvalue of the xy block (position sigma at the odom origin)."""
        return float(math.sqrt(np.linalg.eigvalsh(self.cov[:2, :2])[-1]))

    def to_map(self, x, y, z, yaw) -> tuple:
        X, Y, Z = self.T.apply(x, y, z)
        return X, Y, Z, self.T.apply_yaw(yaw)

    def to_odom(self, x, y, z, yaw) -> tuple:
        Ti = self.T.inverse()
        X, Y, Z = Ti.apply(x, y, z)
        return X, Y, Z, Ti.apply_yaw(yaw)

    # ------------------------------------------------------------------ matching
    def _faces_at(self, z):
        key = round(z, 2)
        if key not in self._faces:
            f = self.course.faces_at(key)
            if not f:
                self._faces[key] = None
            else:
                A = np.array([a for a, _ in f], float)
                D = np.array([b for _, b in f], float) - A
                L = np.linalg.norm(D, axis=1)
                N = np.stack([D[:, 1], -D[:, 0]], axis=1) / L[:, None]     # outward (CCW boxes)
                same = (np.abs(N @ N.T) > 0.999) & (np.abs(np.einsum("fk,fgk->fg", N, A[None] - A[:, None])) < 1e-3)
                self._faces[key] = (A, D, L, N, same)
        return self._faces[key]

    def _match(self, P, x, faces, sensor):
        """Associate odom points P (n,2) with faces at state x. Returns H (m,3), r (m,), w (m,)."""
        A, D, L, N, same = faces
        c, s = math.cos(x[2]), math.sin(x[2])
        R = np.array([[c, -s], [s, c]])
        Q = P @ R.T + x[:2]
        rel = Q[:, None, :] - A[None]                                     # (n, F, 2)
        u = np.einsum("nfk,fk->nf", rel, D) / (L * L)
        dist = np.linalg.norm(rel - np.clip(u, 0.0, 1.0)[..., None] * D, axis=2)
        if sensor is not None:                                            # cull faces turned away
            S = R @ np.asarray(sensor, float) + x[:2]
            dist[:, ((S - A) * N).sum(1) <= 0.0] = np.inf
        i = np.arange(len(Q))
        best = np.argmin(dist, axis=1)
        d1 = dist[i, best]
        d2 = np.where(same[best], np.inf, dist).min(axis=1)
        ub = u[i, best]
        past = np.maximum(0.0, np.maximum(-ub, ub - 1.0)) * L[best]
        keep = (d1 < self.p.match_max_m) & (d2 - d1 > self.p.corner_m) & (past < self.p.corner_m)
        f, q = best[keep], Q[keep]
        n = N[f]
        r = ((q - A[f]) * n).sum(1)
        v = q - x[:2]                                                     # d(q)/d(yaw) = J v
        H = np.stack([n[:, 0], n[:, 1], -n[:, 0] * v[:, 1] + n[:, 1] * v[:, 0]], axis=1)
        sig, k = self.p.meas_sigma_m, self.p.huber_k
        a = np.abs(r) / sig
        huber = np.where(a <= k, 1.0, k / np.maximum(a, 1e-12))
        cap = np.minimum(1.0, self.p.max_points_eff / np.bincount(f, minlength=len(A))[f])
        return H, r, huber * cap / (sig * sig)

    # ------------------------------------------------------------------ update
    def update(self, points_odom_xy, z, now, sensor_odom_xy=None) -> AlignResult:
        """One scan-match against the map faces at height z. sensor_odom_xy (the vehicle's odom xy,
        optional) culls faces turned away from the camera and is the point the step gate measures;
        without it the points' centroid is used."""
        p = self.p
        P = np.asarray(points_odom_xy, float).reshape(-1, 2)
        if len(P) > p.max_points:
            P = P[np.linspace(0, len(P) - 1, p.max_points).astype(int)]
        faces = self._faces_at(z)
        if faces is None:
            return AlignResult(False, f"no map faces at z={z:.2f} m", 0, 0.0, 0.0, 0.0)
        if len(P) < p.min_points:
            return AlignResult(False, f"only {len(P)} points (min_points {p.min_points})", len(P), 0.0, 0.0, 0.0)

        x0 = np.array([self.T.x, self.T.y, self.T.yaw])
        Pinv = np.linalg.inv(self.cov)
        x = x0.copy()
        for _ in range(p.icp_iters):
            H, r, w = self._match(P, x, faces, sensor_odom_xy)
            e = x - x0
            e[2] = wrap(e[2])
            x = x + np.linalg.solve(Pinv + H.T @ (H * w[:, None]), -(Pinv @ e + H.T @ (w * r)))
        H, r, w = self._match(P, x, faces, sensor_odom_xy)
        n = len(r)

        T_new = SE2(float(x[0]), float(x[1]), wrap(float(x[2])))
        ref = P.mean(axis=0) if sensor_odom_xy is None else sensor_odom_xy
        a, b = self.T.apply(*ref), T_new.apply(*ref)
        dx, dy, dyaw = b[0] - a[0], b[1] - a[1], wrap(T_new.yaw - self.T.yaw)
        step = math.hypot(dx, dy)
        if n < p.min_points:
            return AlignResult(False, f"only {n} of {len(P)} points matched a face (min_points {p.min_points})",
                               n, dx, dy, dyaw)
        if step > p.max_step_xy_m:
            return AlignResult(False, f"step {step:.2f} m > max_step_xy_m {p.max_step_xy_m:.2f}", n, dx, dy, dyaw)
        if abs(math.degrees(dyaw)) > p.max_step_yaw_deg:
            return AlignResult(False, f"yaw step {math.degrees(dyaw):.1f} deg > max_step_yaw_deg "
                                      f"{p.max_step_yaw_deg:.1f}", n, dx, dy, dyaw)
        self.T = T_new
        cov = np.linalg.inv(Pinv + H.T @ (H * w[:, None]))
        self.cov = 0.5 * (cov + cov.T)
        self.t_update = now
        rms = math.sqrt(float(np.mean(r * r)))
        return AlignResult(True, f"ok: {n} points, rms {rms * 100:.1f} cm", n, dx, dy, dyaw)
