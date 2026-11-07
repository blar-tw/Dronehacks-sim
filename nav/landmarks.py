"""nav.landmarks - wall points at the drone's height from one depth frame. PURE (numpy only).

Frame chain (each step is a rotation of the previous one, nothing is guessed):
  optical (x right, y down, z forward)
    -> base_link FLU        R_body_cam, t_body_cam: TF base_link <- the depth frame (REP-103)
    -> level heading frame  refnav.depth.level_rotation(q): PX4 attitude (FRD -> NED) with its yaw
                            removed; columns (forward, right, down), origin at the body origin
    -> odom ENU             rotate (forward, right) by the ENU yaw of pose_odom, add pose_odom xy:
                            x = x0 + f cos(yaw) + r sin(yaw),  y = y0 + f sin(yaw) - r cos(yaw)
Points within +-band_m of the body height (level frame "down" ~ 0) are kept and voxel-downsampled
in odom xy to one point (the cell mean) per cell_m square.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from refnav.depth import level_points, level_rotation


@dataclass
class LandmarkParams:
    stride: int = 4
    border_px: int = 16
    rmin: float = 0.25           # optical depth limits (m)
    rmax: float = 4.0
    band_m: float = 0.20         # keep points within z_drone +- band (level frame)
    cell_m: float = 0.05         # voxel-downsample to one point per cell (<= 0: off)
    min_valid_frac: float = 0.2  # of the sampled pixels inside the border must return depth


def wall_points(depth_m, K, R_body_cam, t_body_cam, q_att_wxyz, pose_odom, params=LandmarkParams()):
    """Nx2 odom-frame (ENU) xy points of surfaces at the drone's height, or None when fewer than
    min_valid_frac of the sampled pixels returned depth. depth_m: HxW metres (NaN/0 = no return);
    K = (fx, fy, cx, cy); R/t: camera -> base_link (FLU); q_att_wxyz: PX4 vehicle_attitude.q
    (FRD -> NED); pose_odom = (x, y, z, yaw) in odom ENU (yaw = heading_to_yaw(PX4 heading))."""
    p = params
    d = np.asarray(depth_m, np.float32)
    h, w = d.shape
    b, s = int(p.border_px), int(p.stride)
    sampled = d[::s, ::s]
    rows = np.arange(0, h, s)
    cols = np.arange(0, w, s)
    inner = sampled[(rows >= b) & (rows < h - b)][:, (cols >= b) & (cols < w - b)]
    if np.mean(np.isfinite(inner) & (inner > 0.0)) < p.min_valid_frac:
        return None

    pts = level_points(d, K, R_body_cam, t_body_cam, level_rotation(q_att_wxyz),
                       stride=s, rmin=p.rmin, rmax=p.rmax, border_px=b)       # (forward, right, down)
    pts = pts[np.abs(pts[:, 2]) <= p.band_m]
    x0, y0, _, yaw = pose_odom
    c, sn = math.cos(yaw), math.sin(yaw)
    xy = np.stack([x0 + pts[:, 0] * c + pts[:, 1] * sn,
                   y0 + pts[:, 0] * sn - pts[:, 1] * c], axis=1)
    if p.cell_m <= 0 or len(xy) == 0:
        return xy

    cells = np.floor(xy / p.cell_m).astype(np.int64)
    _, inv, counts = np.unique(cells, axis=0, return_inverse=True, return_counts=True)
    inv = inv.ravel()
    out = np.zeros((len(counts), 2))
    np.add.at(out, inv, xy)
    return out / counts[:, None]
