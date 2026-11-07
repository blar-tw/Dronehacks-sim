"""nav.landmarks: wall_points over the full frame chain (a synthetic depth image of a known wall,
rendered independently of refnav: Euler-angle attitude, explicit optical -> FLU -> FRD -> NED -> ENU,
must come back as odom xy points on that wall, as 32FC1 and as 16UC1), its equivalence with
refnav.depth.level_points, the voxel downsampling, the empty / unusable returns, the zero-copy
image decode and the PoseBuffer (pose at the image stamp). Pure numpy, no ROS."""
import array
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nav import landmarks as LM  # noqa: E402
from refnav.depth import level_points, level_rotation, quat_to_mat  # noqa: E402

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


def dropout(d, seed=0):
    rng = np.random.default_rng(seed)
    d = d.copy()
    d[rng.random(d.shape) < 0.2] = np.nan                          # no-return pixels both ways
    d[rng.random(d.shape) < 0.1] = 0.0
    return d


def as_16uc1(d_m):
    """A 32FC1 depth image as the uint16 millimetre image a 16UC1 message carries (0 = no return)."""
    mm = np.where(np.isfinite(d_m), np.round(d_m * 1000.0), 0.0)
    return np.clip(mm, 0, 65535).astype(np.uint16)


def check_on_wall(pts, pos, yaw, span, tol_m=0.01):
    assert pts is not None and pts.dtype == np.float64 and pts.shape[1] == 2
    assert len(pts) > 15 and np.isfinite(pts).all()
    fwd = np.array([math.cos(yaw), math.sin(yaw)])
    rgt = np.array([math.sin(yaw), -math.cos(yaw)])
    rel = pts - pos[:2]
    assert np.abs(rel @ fwd - 1.5).max() < tol_m                   # on the wall line
    lat = rel @ rgt
    assert lat.min() > span[0] - 0.05 and lat.max() < span[1] + 0.05
    assert lat.min() < span[0] + 0.1 and lat.max() > span[1] - 0.1   # the whole span, not mirrored
    cells = np.floor(pts / 0.05).astype(int)
    assert len(np.unique(cells, axis=0)) == len(pts)                # one point per 5 cm cell


# ----------------------------------------------------------------------------- full chain
@pytest.mark.parametrize("yaw_deg, roll_deg, pitch_deg", [(0, 0, 0), (90, 0, 0), (-30, 0, 0), (-30, 4, -6),
                                                          (150, -3, 5)])
def test_wall_ahead_lands_on_its_odom_line(yaw_deg, roll_deg, pitch_deg):
    pos = np.array([1.3, -0.7, 0.8])
    yaw = math.radians(yaw_deg)
    span = (-0.3, 1.0)                     # asymmetric: a left/right mirror in the chain would show
    d, q, R = render_wall(pos, yaw, math.radians(roll_deg), math.radians(pitch_deg), 1.5, span)
    assert np.allclose(quat_to_mat(*q), R)                         # the test's attitude is PX4's q
    d = dropout(d)

    pts = LM.wall_points(d, K, R_BODY_CAM, T_BODY_CAM, q, (pos[0], pos[1], pos[2], yaw), LM.LandmarkParams())
    check_on_wall(pts, pos, yaw, span)

    # the same frame as a 16UC1 image (uint16 millimetres, scaled after sampling): same wall
    pts_mm = LM.wall_points(as_16uc1(d), K, R_BODY_CAM, T_BODY_CAM, q, (pos[0], pos[1], pos[2], yaw),
                            LM.LandmarkParams())
    check_on_wall(pts_mm, pos, yaw, span)


def test_16uc1_through_depth_view_matches_32fc1():
    pos = np.array([0.0, 0.0, 0.8])
    d, q, _ = render_wall(pos, 0.0, 0.0, 0.0, 1.5, (-0.5, 0.5))
    d = dropout(d, 1)
    prm = LM.LandmarkParams(cell_m=0.0)                            # raw points: compare one by one
    img_f = LM.depth_view("32FC1", H, W, W * 4, d.tobytes())
    img_u = LM.depth_view("16UC1", H, W, W * 2, as_16uc1(d).tobytes())
    assert img_f.dtype == np.float32 and img_u.dtype == np.uint16
    a = LM.wall_points(img_f, K, R_BODY_CAM, T_BODY_CAM, q, (0.0, 0.0, 0.8, 0.0), prm)
    b = LM.wall_points(img_u, K, R_BODY_CAM, T_BODY_CAM, q, (0.0, 0.0, 0.8, 0.0), prm)
    assert len(a) == len(b) > 100
    assert np.abs(a - b).max() < 2e-3                              # 0.5 mm depth quantisation


@pytest.mark.parametrize("yaw_deg", [0, -30])
def test_matches_refnav_level_points(yaw_deg):
    """The folded chain + ray table reproduces refnav.depth.level_points -> band -> odom rotation
    point for point (level attitude facing the wall: no point sits near the band edge)."""
    pos = np.array([2.0, 3.0, 0.8])
    yaw = math.radians(yaw_deg)
    d, q, _ = render_wall(pos, yaw, 0.0, 0.0, 1.5, (-0.6, 0.8))
    d = dropout(d, 2)
    prm = LM.LandmarkParams(cell_m=0.0)
    mine = LM.wall_points(d, K, R_BODY_CAM, T_BODY_CAM, q, (pos[0], pos[1], pos[2], yaw), prm)
    ref = level_points(d, K, R_BODY_CAM, T_BODY_CAM, level_rotation(q), stride=prm.stride, rmin=prm.rmin,
                       rmax=prm.rmax, border_px=prm.border_px)
    ref = ref[np.abs(ref[:, 2]) <= prm.band_m]
    c, s = math.cos(yaw), math.sin(yaw)
    ref_xy = np.stack([pos[0] + ref[:, 0] * c + ref[:, 1] * s, pos[1] + ref[:, 0] * s - ref[:, 1] * c], axis=1)
    assert len(mine) == len(ref_xy) > 100
    assert np.abs(mine - ref_xy).max() < 1e-4                      # float32 rays vs float64 chain


def test_per_side_border_and_ray_cache():
    pos = np.array([0.0, 0.0, 0.8])
    d, q, _ = render_wall(pos, 0.0, 0.0, 0.0, 1.5, (-2.0, 2.0))
    pose = (0.0, 0.0, 0.8, 0.0)
    base = LM.wall_points(d, K, R_BODY_CAM, T_BODY_CAM, q, pose, LM.LandmarkParams(cell_m=0.0))
    tab = LM._rays(K, (H, W), 4, (16, 16, 16, 16))
    assert LM._rays(K, (H, W), 4, (16, 16, 16, 16)) is tab        # cached, not rebuilt
    assert tab.rows == slice(16, 381, 4) and tab.cols == slice(16, 621, 4) and tab.n == 92 * 152
    # a wide left border (the OAK stereo invalid band) drops only the points seen at the LEFT of the image
    left = LM.wall_points(d, K, R_BODY_CAM, T_BODY_CAM, q, pose, LM.LandmarkParams(cell_m=0.0, border_left_px=96))
    assert 0 < len(left) < len(base)
    tab2 = LM._rays(K, (H, W), 4, (96, 16, 16, 16))
    assert tab2 is not tab and tab2.cols == slice(96, 621, 4)
    # image left = optical -x = body +y (left) = odom +y at yaw 0: only the +y side is cut
    assert left[:, 1].max() < base[:, 1].max() - 0.1
    assert abs(left[:, 1].min() - base[:, 1].min()) < 0.05


# ----------------------------------------------------------------------------- downsampling / returns
def test_voxel_mean_one_point_per_cell():
    rng = np.random.default_rng(3)
    centres = np.array([[0.025, 0.025], [0.125, 0.025], [-0.475, 0.975]])   # three distinct 5 cm cells
    pts = np.concatenate([c + rng.uniform(-0.02, 0.02, (50, 2)) for c in centres])
    out = LM.voxel_mean(pts, 0.05)
    assert out.shape == (3, 2)
    for c in centres:
        k = np.argmin(np.linalg.norm(out - c, axis=1))
        assert np.allclose(out[k], pts[np.all(np.floor(pts / 0.05) == np.floor(c / 0.05), axis=1)].mean(axis=0))
    assert LM.voxel_mean(np.zeros((0, 2)), 0.05).shape == (0, 2)
    one = LM.voxel_mean(np.array([[12.3, -4.56]]), 0.05)
    assert np.allclose(one, [[12.3, -4.56]])


def test_empty_when_nothing_at_flight_height():
    """A usable frame (plenty of depth) with no surface in the band - the wall beyond rmax, the floor
    below the band - returns an EMPTY (0,2) array, not None: the node counts it as 'no walls'."""
    pos = np.array([0.0, 0.0, 0.8])
    d, q, _ = render_wall(pos, 0.0, 0.0, 0.0, 6.0, (-3.0, 3.0))   # wall at 6 m > rmax 4 m; floor in view
    prm = LM.LandmarkParams()
    sampled = d[16:384:4, 16:624:4]
    assert np.mean(np.isfinite(sampled) & (sampled > 0)) > prm.min_valid_frac   # it IS a usable frame
    pts = LM.wall_points(d, K, R_BODY_CAM, T_BODY_CAM, q, (0.0, 0.0, 0.8, 0.0), prm)
    assert isinstance(pts, np.ndarray) and pts.shape == (0, 2)
    pts = LM.wall_points(as_16uc1(d), K, R_BODY_CAM, T_BODY_CAM, q, (0.0, 0.0, 0.8, 0.0), prm)
    assert isinstance(pts, np.ndarray) and pts.shape == (0, 2)


def test_none_when_unusable():
    q = (1.0, 0.0, 0.0, 0.0)
    d = np.full((H, W), np.nan, np.float32)                        # no return anywhere (looking at the net)
    assert LM.wall_points(d, K, R_BODY_CAM, T_BODY_CAM, q, (0.0, 0.0, 0.8, 0.0), LM.LandmarkParams()) is None
    d[::16, ::16] = 2.0                                             # 1/16 of the SAMPLED (stride 4) pixels = 6 % < 20 %
    assert LM.wall_points(d, K, R_BODY_CAM, T_BODY_CAM, q, (0.0, 0.0, 0.8, 0.0), LM.LandmarkParams()) is None
    assert LM.wall_points(np.zeros((H, W), np.uint16), K, R_BODY_CAM, T_BODY_CAM, q, (0.0, 0.0, 0.8, 0.0),
                          LM.LandmarkParams()) is None
    # a border wider than the image samples nothing: unusable, not an exception
    assert LM.wall_points(np.full((H, W), 2.0, np.float32), K, R_BODY_CAM, T_BODY_CAM, q, (0.0, 0.0, 0.8, 0.0),
                          LM.LandmarkParams(border_px=300)) is None


# ----------------------------------------------------------------------------- depth_view
def test_depth_view_32fc1_zero_copy_and_padding():
    img = np.arange(12, dtype=np.float32).reshape(3, 4) * 0.25
    raw = img.tobytes()
    v = LM.depth_view("32FC1", 3, 4, 16, raw)
    assert v.dtype == np.float32 and np.array_equal(v, img)
    assert np.shares_memory(v, np.frombuffer(raw, np.uint8))       # no copy
    assert not v.flags.writeable                                    # bytes -> read-only view
    padded = np.concatenate([img, np.full((3, 2), 9.0, np.float32)], axis=1)   # step = 24 B, 8 B of padding
    vp = LM.depth_view("32fc1", 3, 4, 24, padded.tobytes())
    assert np.array_equal(vp, img)
    # rclpy hands over array.array('B'); a hand-built message may carry a list
    assert np.array_equal(LM.depth_view("32FC1", 3, 4, 16, array.array("B", raw)), img)
    assert np.array_equal(LM.depth_view("32FC1", 3, 4, 16, list(raw)), img)


def test_depth_view_16uc1_and_errors():
    mm = np.array([[0, 1500], [250, 65535]], np.uint16)
    v = LM.depth_view("16UC1", 2, 2, 4, mm.tobytes())
    assert v.dtype == np.uint16 and np.array_equal(v, mm)
    assert np.array_equal(LM.depth_view("mono16", 2, 2, 4, mm.tobytes()), mm)
    with pytest.raises(ValueError):
        LM.depth_view("rgb8", 1, 1, 3, b"\0\0\0")
    with pytest.raises(ValueError):
        LM.depth_view("32FC1", 2, 2, 8, b"\0" * 12)                # 16 B expected
    with pytest.raises(ValueError):
        LM.depth_view("16UC1", 2, 2, 2, b"\0" * 4)                 # step smaller than a row


# ----------------------------------------------------------------------------- PoseBuffer
def test_pose_buffer_interpolates_position_heading_and_tilt():
    pb = LM.PoseBuffer(1.0)
    _, q0 = euler_ned_frd(0.0, 0.0, 0.0)
    _, q1 = euler_ned_frd(math.radians(4.0), 0.0, math.radians(20.0))
    pb.push_pos(10.00, 0.0, 0.0, -1.0, 0.0)
    pb.push_pos(10.10, 0.5, -0.2, -1.2, math.radians(6.0))          # 0.5 m/s, 60 deg/s
    pb.push_att(10.02, q0)
    pb.push_att(10.06, q1)
    pos, q, dt_pos, dt_att = pb.at(10.05)
    assert dt_pos == 0.0 and dt_att == 0.0
    assert np.allclose(pos, (0.25, -0.1, -1.1, math.radians(3.0)))
    assert abs(math.degrees(LM.heading_from_quat(q)) - 15.0) < 0.1    # 3/4 of the way 10.02 -> 10.06 (nlerp)
    assert abs(np.linalg.norm(q) - 1.0) < 1e-12
    # the yaw rate over the samples around the stamp
    assert abs(math.degrees(pb.yaw_rate(10.05)) - 60.0) < 1e-6
    # heading interpolates across the +-pi seam
    pb2 = LM.PoseBuffer(1.0)
    pb2.push_pos(0.0, 0, 0, 0, math.pi - 0.1)
    pb2.push_pos(1.0, 0, 0, 0, -math.pi + 0.1)
    pb2.push_att(0.5, q0)
    assert abs(abs(pb2.at(0.5)[0][3]) - math.pi) < 1e-9
    assert abs(math.degrees(pb2.yaw_rate(0.5)) - math.degrees(0.2)) < 1e-6


def test_pose_buffer_outside_history_and_trimming():
    pb = LM.PoseBuffer(0.5)
    assert pb.at(1.0) is None
    _, q = euler_ned_frd(0.0, 0.0, 0.3)
    for i in range(20):                                             # 20 samples at 50 Hz = 0.38 s
        pb.push_pos(100.0 + 0.02 * i, float(i), 0.0, 0.0, 0.0)
        pb.push_att(100.0 + 0.02 * i, q)
    assert len(pb) == 20
    pos, qq, dt_pos, dt_att = pb.at(100.5)                          # 0.12 s after the newest sample
    assert pos[0] == 19.0 and abs(dt_pos - 0.12) < 1e-9 and abs(dt_att - 0.12) < 1e-9
    pos, _, dt_pos, _ = pb.at(99.9)                                 # before the oldest
    assert pos[0] == 0.0 and abs(dt_pos - 0.1) < 1e-9
    assert abs(LM.heading_from_quat(qq) - 0.3) < 1e-12
    assert pb.yaw_rate(100.2) == 0.0
    for i in range(20, 60):                                         # another 0.8 s: the horizon trims
        pb.push_pos(100.0 + 0.02 * i, float(i), 0.0, 0.0, 0.0)
    assert pb.t_pos[0] >= pb.t_pos[-1] - 0.5 - 1e-9 and len(pb.t_pos) <= 27
    pb.push_pos(50.0, 0.0, 0.0, 0.0, 0.0)                           # clock jumped back: history restarts
    assert pb.t_pos == [50.0]
    assert pb.yaw_rate(50.0) is None


def test_heading_from_quat_is_px4_heading():
    for h in (0.0, 0.7, -2.5, 3.0):
        _, q = euler_ned_frd(math.radians(5.0), math.radians(-3.0), h)
        assert abs(LM.heading_from_quat(q) - h) < 1e-9


@pytest.mark.parametrize("z", [0.0, 0.05, 0.15])
def test_floor_is_not_a_wall_near_the_ground(z):
    """On the ground and just after liftoff the floor is inside the +-band_m height band; it must not
    come back as wall points (local GZ 2026-11-06: floor points matched m1_maze_divider and the aligner
    moved the map 13 cm on a drone sitting still on the launch mark)."""
    pos = np.array([1.3, -0.7, z])
    yaw = math.radians(90)
    d, q, _ = render_wall(pos, yaw, 0.0, 0.0, 1.5, (-0.5, 0.5))

    pts = LM.wall_points(d, K, R_BODY_CAM, T_BODY_CAM, q, (pos[0], pos[1], pos[2], yaw), LM.LandmarkParams())

    assert pts is not None and len(pts) > 5
    fwd = np.array([math.cos(yaw), math.sin(yaw)])
    assert np.abs((pts - pos[:2]) @ fwd - 1.5).max() < 0.05          # only the wall, no floor


def test_floor_cut_follows_the_takeoff_floor_when_px4_origin_is_offset():
    """PX4's local z origin is where its EKF started, not necessarily the floor: with the floor at
    odom z = -0.4 and the drone 0.05 m above it, the floor must still be cut (floor_z = -0.4)."""
    floor_z = -0.4
    pos_floor = np.array([1.3, -0.7, 0.05])                  # rendered relative to the floor
    yaw = math.radians(90)
    d, q, _ = render_wall(pos_floor, yaw, 0.0, 0.0, 1.5, (-0.5, 0.5))
    pose = (pos_floor[0], pos_floor[1], pos_floor[2] + floor_z, yaw)    # what PX4 reports

    pts = LM.wall_points(d, K, R_BODY_CAM, T_BODY_CAM, q, pose, LM.LandmarkParams(), floor_z=floor_z)

    assert pts is not None and len(pts) > 5
    fwd = np.array([math.cos(yaw), math.sin(yaw)])
    assert np.abs((pts - pos_floor[:2]) @ fwd - 1.5).max() < 0.05
