"""nav.landmarks.wall_points, full frame chain: a synthetic depth image of a known wall, rendered
independently of refnav (Euler-angle attitude, explicit optical -> FLU -> FRD -> NED -> ENU),
must come back as odom xy points on that wall."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nav import landmarks as LM  # noqa: E402
from refnav.depth import quat_to_mat  # noqa: E402

W, H = 640, 400
K = (421.0, 421.0, 320.0, 200.0)
R_BODY_CAM = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])   # FLU <- optical
T_BODY_CAM = np.array([0.1, 0.0, 0.0])
FRD_FLU = np.diag([1.0, -1.0, -1.0])


def euler_ned_frd(roll, pitch, heading):
    """NED <- FRD = Rz(heading) Ry(pitch) Rx(roll), and the same rotation as a (w, x, y, z) quaternion."""
    cr, sr, cp, sp, ch, sh = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(heading), math.sin(heading))
    R = (np.array([[ch, -sh, 0], [sh, ch, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
         @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))
    c = [math.cos(a / 2) for a in (roll, pitch, heading)]
    s = [math.sin(a / 2) for a in (roll, pitch, heading)]
    q = (c[0] * c[1] * c[2] + s[0] * s[1] * s[2], s[0] * c[1] * c[2] - c[0] * s[1] * s[2],
         c[0] * s[1] * c[2] + s[0] * c[1] * s[2], c[0] * c[1] * s[2] - s[0] * s[1] * c[2])
    return R, q


def ned_to_enu_rows(v):
    return np.stack([v[..., 1], v[..., 0], -v[..., 2]], axis=-1)


def render_wall(pos, yaw, roll, pitch, ahead, right_span):
    """Depth image of a vertical wall `ahead` m in front of the body origin (normal = the horizontal
    heading), spanning right_span = (lo, hi) m to the right of the heading line and from the floor
    (z = 0) up to 2 m, plus the floor (z = 0, which the height band must drop). Nothing else returns."""
    R_ned_frd, q = euler_ned_frd(roll, pitch, math.pi / 2 - yaw)
    vs, us = np.mgrid[0:H, 0:W].astype(float)
    ray_opt = np.stack([(us - K[2]) / K[0], (vs - K[3]) / K[1], np.ones_like(us)], axis=-1)   # optical z = 1
    ray = ned_to_enu_rows(ray_opt @ (R_ned_frd @ FRD_FLU @ R_BODY_CAM).T)
    o = np.asarray(pos) + ned_to_enu_rows(R_ned_frd @ FRD_FLU @ T_BODY_CAM)
    fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    rgt = np.array([math.sin(yaw), -math.cos(yaw), 0.0])
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (ahead - (o - pos) @ fwd) / (ray @ fwd)                   # = optical depth (ray has z = 1)
    hit = o + ray * t[..., None]
    lat = (hit - pos) @ rgt
    ok = (t > 0) & (lat >= right_span[0]) & (lat <= right_span[1]) & (hit[..., 2] >= 0) & (hit[..., 2] <= 2.0)
    with np.errstate(divide="ignore", invalid="ignore"):
        t_floor = -o[2] / ray[..., 2]
    depth = np.fmin(np.where(ok, t, np.nan), np.where(t_floor > 0, t_floor, np.nan))
    return depth.astype(np.float32), q, R_ned_frd


@pytest.mark.parametrize("yaw_deg, roll_deg, pitch_deg", [(0, 0, 0), (90, 0, 0), (-30, 0, 0), (-30, 4, -6),
                                                          (150, -3, 5)])
def test_wall_ahead_lands_on_its_odom_line(yaw_deg, roll_deg, pitch_deg):
    pos = np.array([1.3, -0.7, 0.8])
    yaw = math.radians(yaw_deg)
    span = (-0.3, 1.0)                     # asymmetric: a left/right mirror in the chain would show
    d, q, R = render_wall(pos, yaw, math.radians(roll_deg), math.radians(pitch_deg), 1.5, span)
    assert np.allclose(quat_to_mat(*q), R)                         # the test's attitude is PX4's q
    rng = np.random.default_rng(0)
    d[rng.random(d.shape) < 0.2] = np.nan                          # no-return pixels both ways
    d[rng.random(d.shape) < 0.1] = 0.0

    pts = LM.wall_points(d, K, R_BODY_CAM, T_BODY_CAM, q, (pos[0], pos[1], pos[2], yaw), LM.LandmarkParams())

    assert pts is not None and len(pts) > 15 and np.isfinite(pts).all()
    fwd = np.array([math.cos(yaw), math.sin(yaw)])
    rgt = np.array([math.sin(yaw), -math.cos(yaw)])
    rel = pts - pos[:2]
    assert np.abs(rel @ fwd - 1.5).max() < 0.02                     # on the wall line
    lat = rel @ rgt
    assert lat.min() > span[0] - 0.05 and lat.max() < span[1] + 0.05
    assert lat.min() < span[0] + 0.1 and lat.max() > span[1] - 0.1   # the whole span, not mirrored
    cells = np.floor(pts / 0.05).astype(int)
    assert len(np.unique(cells, axis=0)) == len(pts)                # one point per 5 cm cell
