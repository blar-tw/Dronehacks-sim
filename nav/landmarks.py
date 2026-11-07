"""nav.landmarks - wall points at the drone's height from one depth frame. PURE (numpy only).

Frame chain (each step is a rotation of the previous one, nothing is guessed):
  optical (x right, y down, z forward)
    -> base_link FLU        R_body_cam, t_body_cam: TF base_link <- the depth frame (REP-103)
    -> level heading frame  refnav.depth.level_rotation(q): PX4 attitude (FRD -> NED) with its yaw
                            removed; columns (forward, right, down), origin at the body origin
    -> odom ENU             rotate (forward, right) by the ENU yaw of pose_odom, add pose_odom xy:
                            x = x0 + f cos(yaw) + r sin(yaw),  y = y0 + f sin(yaw) - r cos(yaw)
Points within +-band_m of the body height (level frame "down" ~ 0) and at least floor_clear_m above
the floor (odom z = floor_z, the takeoff floor) are kept and voxel-downsampled in odom xy to one point
(the cell mean) per cell_m square. Without the floor cut, on the ground and just after liftoff the floor
is inside the band and comes back as a sheet of "wall" points (local GZ run 2026-11-06: the aligner
moved the map 13 cm on a drone sitting on the launch mark, and the x500 hit the start-corridor wall).

Cost: the whole chain is folded into one 2x3 matrix applied to the band survivors only; the
per-pixel rays of the strided, border-trimmed grid are built once per camera geometry (_rays) and
the image is read through a strided VIEW, so no full-image (256k pixel) pass ever happens.
Also here (pure, so they are unit-testable without ROS): depth_view (zero-copy sensor_msgs/Image
decode), PoseBuffer (PX4 pose history read at the image stamp) and heading_from_quat.
"""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass

import numpy as np

from refnav.depth import level_rotation, quat_to_mat


@dataclass
class LandmarkParams:
    stride: int = 4
    border_px: int = 16          # pixels ignored at every image edge (stereo has no support there)
    border_left_px: int = -1     # per-side override, -1 = border_px. OAK-D stereo: the left ~60-90 px
    border_right_px: int = -1    # of the depth image can never match (disparity search range), so on
    border_top_px: int = -1      # hardware set border_left_px ~ 96 (the lab's simulator has no such band)
    border_bottom_px: int = -1
    rmin: float = 0.25           # optical depth limits (m)
    rmax: float = 4.0
    band_m: float = 0.20         # keep points within z_drone +- band (level frame)
    floor_clear_m: float = 0.10  # ...and at least this far above the floor (odom z = floor_z)
    cell_m: float = 0.05         # voxel-downsample to one point per cell (<= 0: off)
    min_valid_frac: float = 0.2  # of the sampled pixels inside the border must return depth
    # --- used by nav/node.py (pose timing + camera checks); same nav.yaml section ---
    pose_buffer_s: float = 1.0        # PX4 pose history kept for reading the pose at the image stamp
    pose_max_dt_s: float = 0.10       # image stamp this far outside the history: nearest sample + warning
    max_latency_s: float = 0.5        # a frame older than this on arrival is a backlog: dropped
    max_yaw_rate_dps: float = 15.0    # no aligner update while yawing faster: pose timing error dominates
    optical_axis_tol_deg: float = 25.0  # TF check: the optical axis must be this close to body-forward


# ----------------------------------------------------------------------------- image decode
def depth_view(encoding, height, width, step, data, is_bigendian=False):
    """sensor_msgs/Image depth -> HxW array WITHOUT copying the pixels: 32FC1 -> float32 metres,
    16UC1/mono16 -> uint16 millimetres (wall_points scales those after sampling, so the full-image
    mm -> m pass is never paid). The result is a read-only VIEW into `data` (rclpy hands the
    callback an array.array; bytes work too): use it inside the callback, never keep it.
    Raises ValueError on an unsupported encoding or a size that does not match height x step."""
    enc = str(encoding).lower()
    if enc == "32fc1":
        dt, isz = np.dtype(np.float32), 4
    elif enc in ("16uc1", "mono16"):
        dt, isz = np.dtype(np.uint16), 2
    else:
        raise ValueError(f"unsupported depth encoding {encoding!r} (need 32FC1 metres or 16UC1 millimetres)")
    h, w, step = int(height), int(width), int(step)
    try:
        buf = np.frombuffer(data, np.uint8)
    except (TypeError, ValueError, BufferError):
        buf = np.frombuffer(bytes(data), np.uint8)        # a hand-built message with a plain list
    if h <= 0 or w <= 0 or step < w * isz or buf.size != h * step:
        raise ValueError(f"depth image {h}x{w} {encoding}: step {step} B, {buf.size} B of data")
    if step % isz == 0:
        img = buf.view(dt).reshape(h, step // isz)[:, :w]   # row padding is sliced off, still a view
    else:                                                   # odd padding: numpy < 1.23 cannot re-view it
        img = np.ascontiguousarray(buf.reshape(h, step)[:, :w * isz]).view(dt)
    if bool(is_bigendian):                                  # never seen from a camera driver; correct, slow
        img = img.view(dt.newbyteorder(">")).astype(dt)
    return img


# ----------------------------------------------------------------------------- pixel ray table
class _RayTable:
    """The strided, border-trimmed pixel grid of one camera geometry: the row/col slices into the
    full image and the optical ray (x/fx, y/fy) of every sampled pixel, float32, in the C order of
    the sliced view. Built once per (K, shape, stride, borders); a frame then costs one strided view
    plus arithmetic on the sampled pixels."""
    __slots__ = ("key", "rows", "cols", "rx", "ry", "n")

    def __init__(self, key, shape, K, stride, borders):
        fx, fy, cx, cy = (float(c) for c in K)
        h, w = shape
        bl, br, bt, bb = borders
        rows = np.arange(0, h, stride)
        rows = rows[(rows >= bt) & (rows < h - bb)]        # same pixels refnav.level_points samples
        cols = np.arange(0, w, stride)
        cols = cols[(cols >= bl) & (cols < w - br)]
        self.key = key
        self.n = int(rows.size * cols.size)
        if self.n == 0:
            self.rows = self.cols = slice(0, 0)
            self.rx = self.ry = np.zeros(0, np.float32)
            return
        self.rows = slice(int(rows[0]), int(rows[-1]) + 1, stride)
        self.cols = slice(int(cols[0]), int(cols[-1]) + 1, stride)
        u, v = np.meshgrid(cols.astype(np.float32), rows.astype(np.float32))
        self.rx = ((u - np.float32(cx)) / np.float32(fx)).ravel().astype(np.float32)
        self.ry = ((v - np.float32(cy)) / np.float32(fy)).ravel().astype(np.float32)


_RAYS = None     # one entry: a node sees one camera; a different key simply rebuilds (~0.3 ms)


def _rays(K, shape, stride, borders):
    global _RAYS
    key = (tuple(round(float(c), 6) for c in K), tuple(int(c) for c in shape), int(stride), tuple(borders))
    if _RAYS is None or _RAYS.key != key:
        _RAYS = _RayTable(key, shape, K, int(stride), borders)
    return _RAYS


# ----------------------------------------------------------------------------- wall points
def voxel_mean(xy, cell_m):
    """One point (the cell mean) per cell_m square of the (N,2) float array xy. A 1-D integer cell
    key + np.unique + np.bincount: numpy sorts 1-D int64 natively, while np.unique(axis=0) sorts
    (N,2) rows through a structured void view (slow path) and np.add.at is a per-element loop."""
    xy = np.asarray(xy, np.float64).reshape(-1, 2)
    if len(xy) == 0:
        return xy
    ij = np.floor(xy * (1.0 / float(cell_m))).astype(np.int64)
    ij -= ij.min(axis=0)
    key = ij[:, 0] * (int(ij[:, 1].max()) + 1) + ij[:, 1]
    _, inv, counts = np.unique(key, return_inverse=True, return_counts=True)
    inv = inv.ravel()
    n = len(counts)
    out = np.empty((n, 2))
    out[:, 0] = np.bincount(inv, weights=xy[:, 0], minlength=n) / counts
    out[:, 1] = np.bincount(inv, weights=xy[:, 1], minlength=n) / counts
    return out


def wall_points(depth_m, K, R_body_cam, t_body_cam, q_att_wxyz, pose_odom, params=LandmarkParams(),
                floor_z=0.0):
    """Nx2 odom-frame (ENU) xy points of surfaces at the drone's height (float64; an EMPTY (0,2)
    array when the frame is fine but nothing lies at flight height), or None when fewer than
    min_valid_frac of the sampled pixels returned depth (unusable frame). depth_m: HxW metres
    (float; NaN/0 = no return) or uint16 MILLIMETRES straight from a 16UC1 image (0 = no return);
    K = (fx, fy, cx, cy); R/t: camera -> base_link (FLU); q_att_wxyz: PX4 vehicle_attitude.q
    (FRD -> NED); pose_odom = (x, y, z, yaw) in odom ENU (yaw = heading_to_yaw(PX4 heading));
    floor_z = odom z of the floor (0 when PX4's local origin is on the floor; the node passes the
    mission's takeoff odom z once it has one)."""
    p = params
    d = np.asarray(depth_m)
    if d.ndim != 2:
        raise ValueError(f"depth image must be HxW, got shape {d.shape}")
    mm = d.dtype == np.uint16
    if not mm and d.dtype != np.float32:
        d = d.astype(np.float32)
    s = max(1, int(p.stride))
    b = int(p.border_px)
    borders = tuple(b if int(v) < 0 else int(v)
                    for v in (p.border_left_px, p.border_right_px, p.border_top_px, p.border_bottom_px))
    T = _rays(K, d.shape, s, borders)
    if T.n == 0:
        return None
    sub = d[T.rows, T.cols]                                 # strided view: no copy of the image
    if mm:
        valid = sub > 0
        ok = valid & (sub > p.rmin * 1000.0) & (sub < p.rmax * 1000.0)
    else:
        valid = np.isfinite(sub) & (sub > 0.0)
        ok = valid & (sub > p.rmin) & (sub < p.rmax)
    if np.count_nonzero(valid) < p.min_valid_frac * valid.size:
        return None
    z = sub[ok]
    if mm:
        z = z.astype(np.float32) * np.float32(1e-3)         # mm -> m on the sampled pixels only
    if z.size == 0:
        return np.zeros((0, 2))
    sel = ok.ravel()                                        # C order of the view = order of T.rx/T.ry
    x = T.rx[sel] * z                                       # optical x, y at depth z (float32)
    y = T.ry[sel] * z

    R_lb = np.asarray(level_rotation(q_att_wxyz))           # level <- FLU
    R_lc = R_lb @ np.asarray(R_body_cam, float)             # level <- optical
    o_l = R_lb @ np.asarray(t_body_cam, float)              # camera origin in the level frame
    # band filter first, on the 'down' row only (python floats keep the arithmetic in float32)
    down = x * float(R_lc[2, 0]) + y * float(R_lc[2, 1]) + z * float(R_lc[2, 2]) + float(o_l[2])
    # height band, and the floor cut: a point's height above the floor is (z0 - floor_z) - down
    z0 = float(pose_odom[2])
    keep = (np.abs(down) <= p.band_m) & (down <= (z0 - float(floor_z)) - p.floor_clear_m)
    if not keep.any():
        return np.zeros((0, 2))
    x, y, z = x[keep], y[keep], z[keep]
    # odom xy = (x0, y0) + A (forward, right), A = [[c, s], [s, -c]] (ENU yaw of the level frame);
    # folded with R_lc into one 2x3 matrix M so the survivors are transformed once
    x0, y0, _, yaw = pose_odom
    c, sn = math.cos(float(yaw)), math.sin(float(yaw))
    A = np.array([[c, sn], [sn, -c]])
    M = A @ R_lc[:2]
    off = A @ o_l[:2] + np.array([float(x0), float(y0)])
    xy = np.empty((z.size, 2))
    xy[:, 0] = x * float(M[0, 0]) + y * float(M[0, 1]) + z * float(M[0, 2])
    xy[:, 1] = x * float(M[1, 0]) + y * float(M[1, 1]) + z * float(M[1, 2])
    xy += off                                               # the large odom offset is added in float64
    if p.cell_m <= 0:
        return xy
    return voxel_mean(xy, p.cell_m)


# ----------------------------------------------------------------------------- pose history
def heading_from_quat(q_wxyz):
    """PX4 heading (NED, clockwise from north) of a vehicle_attitude quaternion: the same yaw that
    level_rotation removes, so points levelled with q and rotated into odom with this heading see
    no attitude/heading mismatch (PX4's vehicle_local_position.heading IS this value of its q)."""
    R = quat_to_mat(*q_wxyz)
    return math.atan2(R[1, 0], R[0, 0])


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _bracket(ts, t):
    """(i0, i1, alpha, dt_outside) for time t in the sorted stamps ts (non-empty)."""
    if t <= ts[0]:
        return 0, 0, 0.0, ts[0] - t
    if t >= ts[-1]:
        n = len(ts) - 1
        return n, n, 0.0, t - ts[-1]
    i1 = bisect.bisect_right(ts, t)
    i0 = i1 - 1
    span = ts[i1] - ts[i0]
    return i0, i1, ((t - ts[i0]) / span if span > 0.0 else 0.0), 0.0


class PoseBuffer:
    """Short history of PX4 pose samples, stamped on the NODE clock at receipt - how this repo
    relates PX4 data to ROS time (onboard/adapters/px4_odom.py): PX4's own timestamps are on its
    clock (microseconds since boot on the aircraft; only the simulator's happen to equal sim time).
    Position + heading come from vehicle_local_position and the tilt quaternion from
    vehicle_attitude, at different rates and instants (~10 ms apart), so each is kept with its
    own stamps and read at the image stamp separately. Pure: no ROS."""

    def __init__(self, horizon_s=1.0):
        self.horizon = float(horizon_s)
        self.t_pos, self.pos = [], []        # pos: (x, y, z, heading) as PX4 gives them (NED)
        self.t_att, self.att = [], []        # att: (w, x, y, z)

    def __len__(self):
        return min(len(self.t_pos), len(self.t_att))

    def push_pos(self, t, x, y, z, heading):
        self._push(self.t_pos, self.pos, float(t), (float(x), float(y), float(z), float(heading)))

    def push_att(self, t, q_wxyz):
        self._push(self.t_att, self.att, float(t), tuple(float(c) for c in q_wxyz))

    def _push(self, ts, vals, t, v):
        if ts and t < ts[-1]:                # time went backwards (clock jump): the history is void
            del ts[:]
            del vals[:]
        ts.append(t)
        vals.append(v)
        k = bisect.bisect_left(ts, t - self.horizon)
        if k > 0:
            del ts[:k]
            del vals[:k]

    def at(self, t):
        """Pose at time t: ((x, y, z, heading), (w, x, y, z), dt_pos, dt_att). Linear interpolation
        between the bracketing samples (heading on the circle, quaternion normalised lerp with the
        sign made consistent: exact enough for the ~20-50 ms gaps between PX4 samples); outside the
        history the nearest sample, with dt_* = how far t lies outside it (0 when bracketed).
        None when either history is empty."""
        if not self.t_pos or not self.t_att:
            return None
        i0, i1, a, dt_pos = _bracket(self.t_pos, t)
        p0, p1 = self.pos[i0], self.pos[i1]
        pos = (p0[0] + a * (p1[0] - p0[0]), p0[1] + a * (p1[1] - p0[1]), p0[2] + a * (p1[2] - p0[2]),
               _wrap(p0[3] + a * _wrap(p1[3] - p0[3])))
        j0, j1, b, dt_att = _bracket(self.t_att, t)
        q0, q1 = self.att[j0], self.att[j1]
        if sum(u * v for u, v in zip(q0, q1)) < 0.0:
            q1 = tuple(-c for c in q1)
        q = [u + b * (v - u) for u, v in zip(q0, q1)]
        norm = math.sqrt(sum(c * c for c in q)) or 1.0
        return pos, tuple(c / norm for c in q), dt_pos, dt_att

    def yaw_rate(self, t, window_s=0.1):
        """Heading rate (rad/s, PX4 sign) around time t from the heading samples within +-window_s
        (at least the two bracketing t); None with fewer than two distinct stamps."""
        ts = self.t_pos
        if len(ts) < 2:
            return None
        lo = bisect.bisect_left(ts, t - window_s)
        hi = bisect.bisect_right(ts, t + window_s)
        lo = max(0, min(lo, bisect.bisect_right(ts, t) - 1))         # include the sample before t
        hi = min(len(ts), max(hi, bisect.bisect_left(ts, t) + 1))     # and the one after it
        if hi - lo < 2 or ts[hi - 1] - ts[lo] <= 1e-6:
            return None
        return _wrap(self.pos[hi - 1][3] - self.pos[lo][3]) / (ts[hi - 1] - ts[lo])
