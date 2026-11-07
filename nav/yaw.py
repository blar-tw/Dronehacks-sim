"""Perception-aware yaw: which heading gives the camera the most map walls to match.

Wall faces are sampled every 10 cm. A sample counts for a heading if it lies inside the camera's
horizontal FOV and range and nothing else blocks the line of sight. A heading's score is the sum over
faces of min(visible samples, cap): several walls at different angles beat one long wall, since one
wall cannot fix the position along it.
"""
import math

import numpy as np

from .frames import wrap

HFOV_DEG = 72.0          # OAK-D S2 depth, 640x400 at fx 421 -> 74 deg; a little margin
RANGE_M = (0.4, 3.0)


class ViewScorer:
    def __init__(self, course, z, step_m=0.10, cap=8):
        A, B, N = course.faces_at(z)
        self.A, self.B, self.cap = A, B, cap
        S, F, Nn = [], [], []
        for k in range(len(A)):
            n = max(1, int(np.linalg.norm(B[k] - A[k]) / step_m))
            t = (np.arange(n) + 0.5) / n
            S.append(A[k] + t[:, None] * (B[k] - A[k]))
            F.append(np.full(n, k))
            Nn.append(np.repeat(N[k][None], n, 0))
        self.S, self.F, self.N = np.vstack(S), np.concatenate(F), np.vstack(Nn)

    def visible(self, xy):
        """Mask of samples facing xy, in range and not occluded by another face."""
        xy = np.asarray(xy, float)
        d = self.S - xy
        r = np.linalg.norm(d, axis=1)
        ok = (r > RANGE_M[0]) & (r < RANGE_M[1]) & ((d * self.N).sum(1) < 0)
        # segment xy -> sample against every face (except the sample's own)
        P, Rv = xy, d[ok]
        Q, Sv = self.A, self.B - self.A
        cross = lambda a, b: a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]
        den = cross(Rv[:, None, :], Sv[None])
        qp = Q[None] - P
        with np.errstate(divide="ignore", invalid="ignore"):
            t = cross(qp, Sv[None]) / den
            u = cross(qp, Rv[:, None, :]) / den
        hit = (t > 1e-6) & (t < 1 - 1e-3) & (u >= 0) & (u <= 1)
        hit[np.arange(len(Rv)), self.F[ok]] = False
        idx = np.flatnonzero(ok)
        ok[idx[hit.any(1)]] = False
        return ok

    def score(self, xy, yaw, vis=None):
        vis = self.visible(xy) if vis is None else vis
        d = self.S - np.asarray(xy, float)
        bearing = np.abs(np.vectorize(wrap)(np.arctan2(d[:, 1], d[:, 0]) - yaw))
        inview = vis & (bearing < math.radians(HFOV_DEG / 2))
        counts = np.bincount(self.F[inview], minlength=len(self.A))
        return float(np.minimum(counts, self.cap).sum())

    def best(self, xy, around=None, within_deg=180.0, step_deg=10.0):
        """Best ENU yaw at xy; with `around`, only headings within within_deg of it."""
        vis = self.visible(xy)
        best, by = -1.0, around or 0.0
        for k in np.arange(-180.0, 180.0, step_deg):
            yaw = math.radians(k)
            if around is not None and abs(wrap(yaw - around)) > math.radians(within_deg):
                continue
            s = self.score(xy, yaw, vis)
            if s > best:
                best, by = s, yaw
        return by, best
