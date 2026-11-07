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

Two defences against a bad depth frame (the pipeline can hand update() a frame rotated 3-9 deg
about the sensor and shifted a few cm along the heading when the drone yaws fast; a rigid error
like that is absorbed by the ICP with a small residual, so it is invisible in the rms):
  * step gate: the proposed step (dx, dy, dyaw) at the vehicle is compared with the covariance of
    the correction the prior and this frame allow, P_prior - P_post (Mahalanobis, pseudo-inverse
    so uninformed directions do not count); reject above gate_nsigma, on top of the absolute
    max_step_*. For this to mean anything the posterior is floored at min_sigma_xy_m /
    min_sigma_yaw_deg (at the vehicle): otherwise a few seconds on a wall collapses sigma to
    millimetres and a 0.3 deg attitude jitter would fail the gate. A true jump larger than the
    gate recovers as predict() grows sigma with distance (same as for the absolute gates).
  * rms trust: when the converged point-to-face rms exceeds rms_trust_k * meas_sigma_m (a frame
    that is not a rigid transform of the map: time skew inside the frame, mixed walls), the
    measurement information is scaled by (rms_trust_k * meas_sigma_m / rms)^2 and the MAP step is
    re-solved from the prior with that weight, so the frame counts as the noisier one it is.

Robustness contract: update() never raises and never leaves T or cov non-finite. A singular or
non-finite covariance, a NaN depth point, an empty association or a non-finite solve all end in a
rejected AlignResult with the state untouched (the mission logs the reason and keeps flying on
the prior).
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
    min_sigma_xy_m: float = 0.02       # posterior sigma floor at the vehicle (map + depth bias are
    min_sigma_yaw_deg: float = 0.5     # not white: the per-update cap alone still collapses to mm)
    gate_nsigma: float = 4.0           # reject a step beyond this many prior sigmas (0 = off)
    rms_trust_k: float = 1.5           # scale a frame's information down once rms > k * meas_sigma


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


def _lever_inv(T: SE2, odom_xy) -> np.ndarray:
    """inv(_lever(...)) in closed form (a shear: negate the lever column); no np.linalg.inv at 20 Hz."""
    c, s = math.cos(T.yaw), math.sin(T.yaw)
    rx, ry = c * odom_xy[0] - s * odom_xy[1], s * odom_xy[0] + c * odom_xy[1]
    return np.array([[1.0, 0.0, ry], [0.0, 1.0, -rx], [0.0, 0.0, 1.0]])


class _Faces:
    """Per-face arrays for one face set, built once and shared by every z that yields that set."""
    __slots__ = ("A", "D", "L", "N", "same", "AD", "AN", "invL2", "F")

    def __init__(self, f):
        A = np.array([a for a, _ in f], float)
        D = np.array([b for _, b in f], float) - A
        L = np.linalg.norm(D, axis=1)
        N = np.stack([D[:, 1], -D[:, 0]], axis=1) / L[:, None]       # outward: Box.corners is CCW
        # faces on the same line (parallel or anti-parallel, no normal offset) are not "another
        # line" for the corner test, e.g. the two chicane parts either side of the window
        off = N @ A.T                                                 # off[f, g] = N_f . A_g
        same = (np.abs(N @ N.T) > 0.999) & (np.abs(off - np.diag(off)[:, None]) < 1e-3)
        self.A, self.D, self.L, self.N, self.same = A, D, L, N, same
        self.AD = (A * D).sum(axis=1)                                 # so u = (Q.D - A.D) / L^2
        self.AN = (A * N).sum(axis=1)                                 # so perp = Q.N - A.N
        self.invL2 = 1.0 / (L * L)
        self.F = len(A)


class Aligner:
    def __init__(self, course, params=None):
        self.course, self.p = course, (params if params is not None else AlignerParams())
        self.T = SE2()
        self.cov = self._vehicle_cov_to_T(np.diag(self._init_var()), (0.0, 0.0))
        self.t_update = None
        self._faces = {}          # round(z, 2) -> _Faces | None
        self._faces_by_geom = {}  # face tuple -> _Faces (one build per distinct face set)

    # ------------------------------------------------------------------ state
    def _init_var(self):
        sxy, syaw = self.p.init_sigma_xy_m, math.radians(self.p.init_sigma_yaw_deg)
        return np.array([sxy * sxy, sxy * sxy, syaw * syaw])

    def _vehicle_cov_to_T(self, cov_v, odom_xy):
        Ai = _lever_inv(self.T, odom_xy)
        return Ai @ cov_v @ Ai.T

    def init_at_launch(self, odom_xy, launch_xy, yaw_offset=0.0, odom_z=None, launch_z=0.0) -> None:
        """T such that odom_xy (the vehicle at takeoff) maps to launch_xy, rotated by yaw_offset.
        With odom_z, the z offset makes odom_z map to launch_z (the floor); else z passes through.
        The initial sigmas are about the vehicle, not about the odom origin."""
        c, s = math.cos(yaw_offset), math.sin(yaw_offset)
        ox, oy = odom_xy
        dz = 0.0 if odom_z is None else float(launch_z) - float(odom_z)
        self.T = SE2(launch_xy[0] - (c * ox - s * oy), launch_xy[1] - (s * ox + c * oy), wrap(yaw_offset), dz)
        self.cov = self._vehicle_cov_to_T(np.diag(self._init_var()), odom_xy)

    def predict(self, dist_m, odom_xy=(0.0, 0.0)) -> None:
        """Grow the vehicle's position sigma by drift_per_m and its yaw sigma by drift_yaw_deg_per_m
        for every metre flown (linear in distance, independent of how often this is called)."""
        if not math.isfinite(dist_m):                                     # a NaN odom delta must not poison cov
            return
        A = _lever(self.T, odom_xy)
        cv = A @ self.cov @ A.T
        sig = np.sqrt(np.maximum(np.diag(cv), 0.0))                      # rounding can leave -1e-18 on the diagonal
        k = np.array([self.p.drift_per_m, self.p.drift_per_m, math.radians(self.p.drift_yaw_deg_per_m)])
        cv += np.diag((sig + k * abs(dist_m)) ** 2 - sig ** 2)
        Ai = _lever_inv(self.T, odom_xy)
        cov = Ai @ cv @ Ai.T
        self.cov = 0.5 * (cov + cov.T)

    def sigma_xy(self, odom_xy=None) -> float:
        """sqrt of the larger eigenvalue of the xy block: position sigma at the odom origin, or at
        the vehicle when its odom xy is given (adds the yaw uncertainty times the lever arm)."""
        cov = self.cov if odom_xy is None else self._cov_at(odom_xy)
        return float(math.sqrt(max(0.0, np.linalg.eigvalsh(cov[:2, :2])[-1])))

    def _cov_at(self, odom_xy):
        A = _lever(self.T, odom_xy)
        return A @ self.cov @ A.T

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
                geom = tuple((tuple(map(float, a)), tuple(map(float, b))) for a, b in f)
                if geom not in self._faces_by_geom:     # neighbouring cm of height share one face set
                    self._faces_by_geom[geom] = _Faces(f)
                self._faces[key] = self._faces_by_geom[geom]
        return self._faces[key]

    def _match(self, P, x, faces, sensor, wscale=1.0):
        """Associate odom points P (n,2) with faces at state x. Returns H (m,3), r (m,), w (m,).
        Distance to a segment = hypot(signed distance to its line, overshoot past its ends), so
        everything is (n,F) matmuls and elementwise ops; no (n,F,2) temporaries."""
        fc = faces
        c, s = math.cos(x[2]), math.sin(x[2])
        R = np.array([[c, -s], [s, c]])
        t = x[:2]
        Q = P @ R.T + t                                                   # points in the map (n,2)
        perp = Q @ fc.N.T - fc.AN                                         # n_f . (q - a_f)   (n,F)
        u = (Q @ fc.D.T - fc.AD) * fc.invL2                               # along-face parameter (n,F)
        past = np.maximum(-u, u - 1.0)
        np.maximum(past, 0.0, out=past)
        past *= fc.L                                                      # metres past either end
        dist = np.hypot(perp, past)                                       # == |q - clamp(proj on segment)|
        if sensor is not None:                                            # cull faces turned away
            S = R @ np.asarray(sensor, float) + t
            dist[:, (fc.N @ S - fc.AN) <= 0.0] = np.inf
        i = np.arange(len(Q))
        best = np.argmin(dist, axis=1)
        d1 = dist[i, best]
        dist[fc.same[best]] = np.inf                                      # in place: dist is not needed again
        d2 = dist.min(axis=1)
        keep = (d1 < self.p.match_max_m) & (d2 - d1 > self.p.corner_m) & (past[i, best] < self.p.corner_m)
        f = best[keep]
        n = fc.N[f]
        r = perp[i, best][keep]
        v = Q[keep] - t                                                   # d(q)/d(yaw) = J v
        H = np.empty((len(f), 3))
        H[:, 0] = n[:, 0]
        H[:, 1] = n[:, 1]
        H[:, 2] = n[:, 1] * v[:, 0] - n[:, 0] * v[:, 1]
        sig, k = self.p.meas_sigma_m, self.p.huber_k
        a = np.abs(r) / sig
        huber = np.where(a <= k, 1.0, k / np.maximum(a, 1e-12))
        cap = np.minimum(1.0, self.p.max_points_eff / np.bincount(f, minlength=fc.F)[f])
        return H, r, huber * cap * (wscale / (sig * sig))

    # ------------------------------------------------------------------ update
    def update(self, points_odom_xy, z, now, sensor_odom_xy=None) -> AlignResult:
        """One scan-match against the map faces at height z. sensor_odom_xy (the vehicle's odom xy,
        optional) culls faces turned away from the camera and is the point the step gate measures;
        without it the points' centroid is used."""
        p = self.p
        P = np.asarray(points_odom_xy, float).reshape(-1, 2)
        P = P[np.isfinite(P).all(axis=1)]                 # a NaN/inf depth point must never reach argmin
        if len(P) > p.max_points:
            P = P[np.linspace(0, len(P) - 1, p.max_points).astype(int)]
        faces = self._faces_at(z)
        if faces is None:
            return AlignResult(False, f"no map faces at z={z:.2f} m", 0, 0.0, 0.0, 0.0)
        if len(P) < max(1, p.min_points):
            return AlignResult(False, f"only {len(P)} points (min_points {p.min_points})", len(P), 0.0, 0.0, 0.0)
        if sensor_odom_xy is not None and not np.isfinite(np.asarray(sensor_odom_xy, float)).all():
            return AlignResult(False, "sensor_odom_xy not finite", len(P), 0.0, 0.0, 0.0)

        x0 = np.array([self.T.x, self.T.y, self.T.yaw])
        try:
            Pinv = np.linalg.inv(self.cov)
        except np.linalg.LinAlgError:
            Pinv = None
        if Pinv is None or not np.isfinite(Pinv).all():
            return AlignResult(False, "prior covariance singular or not finite", len(P), 0.0, 0.0, 0.0)

        try:
            x, H, r, w = self._solve(P, x0, Pinv, faces, sensor_odom_xy, 1.0)
            n = len(r)
            rms = math.sqrt(float(np.mean(r * r))) if n else 0.0
            scale = 1.0
            if p.rms_trust_k > 0 and n and rms > p.rms_trust_k * p.meas_sigma_m:
                scale = (p.rms_trust_k * p.meas_sigma_m / rms) ** 2        # treat the frame as sigma = rms/k
                x, H, r, w = self._solve(P, x0, Pinv, faces, sensor_odom_xy, scale)
                n = len(r)
            cov = np.linalg.inv(Pinv + H.T @ (H * w[:, None]))
        except np.linalg.LinAlgError:
            return AlignResult(False, "normal equations singular", 0, 0.0, 0.0, 0.0)
        if not (np.isfinite(x).all() and np.isfinite(cov).all()):
            return AlignResult(False, "non-finite solve", n, 0.0, 0.0, 0.0)

        T_new = SE2(float(x[0]), float(x[1]), wrap(float(x[2])), self.T.z)   # the z anchor is not estimated
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
        if p.gate_nsigma > 0:
            # a rigid error in the frame (rotated about the sensor) converges with a small rms, so
            # the only handle on it is the size of the step. The MAP step d = K * innovation has
            # covariance P_prior - P_post (both at the vehicle), so d' (P_prior - P_post)^+ d is a
            # chi-square(dof) under consistency; directions the frame did not inform (zero
            # eigenvalue, e.g. along one wall) carry no step and are left out of the sum.
            A = _lever(self.T, ref)
            dcv = A @ (self.cov - cov) @ A.T
            lam, V = np.linalg.eigh(0.5 * (dcv + dcv.T))
            ok = lam > 1e-9 * max(float(lam[-1]), 1e-30)
            proj = V.T @ np.array([dx, dy, dyaw])
            m2 = float(np.sum(proj[ok] ** 2 / lam[ok])) if ok.any() else 0.0
            if not (m2 <= p.gate_nsigma ** 2):                             # also catches NaN
                return AlignResult(False, f"step is {math.sqrt(max(m2, 0.0)):.1f} sigma of the allowed "
                                          f"correction (gate_nsigma {p.gate_nsigma:.1f})", n, dx, dy, dyaw)
        # posterior sigma floor at the vehicle (lever of the new T): adding to the diagonal keeps PSD
        An = _lever(T_new, ref)
        cv = An @ cov @ An.T
        floor = np.array([p.min_sigma_xy_m, p.min_sigma_xy_m, math.radians(p.min_sigma_yaw_deg)]) ** 2
        cv += np.diag(np.maximum(0.0, floor - np.diag(cv)))
        Ain = _lever_inv(T_new, ref)
        cov = Ain @ cv @ Ain.T
        self.T = T_new
        self.cov = 0.5 * (cov + cov.T)
        self.t_update = now
        rms = math.sqrt(float(np.mean(r * r))) if n else 0.0
        why = f"ok: {n} points, rms {rms * 100:.1f} cm" + ("" if scale == 1.0 else f", trust {scale:.2f}")
        return AlignResult(True, why, n, dx, dy, dyaw)

    def _solve(self, P, x0, Pinv, faces, sensor, wscale):
        """Gauss-Newton on the MAP objective from the prior mean, re-associating every iteration.
        Returns the converged x and the final association (H, r, w)."""
        x = x0.copy()
        for _ in range(self.p.icp_iters):
            H, r, w = self._match(P, x, faces, sensor, wscale)
            e = x - x0
            e[2] = wrap(e[2])
            HW = H * w[:, None]
            x = x + np.linalg.solve(Pinv + H.T @ HW, -(Pinv @ e + HW.T @ r))
        H, r, w = self._match(P, x, faces, sensor, wscale)
        return x, H, r, w
