"""nav.aligner: recovers a known map <- odom error from wall points, and moves only observable
directions (one straight wall: cross-wall position + yaw, never along the wall)."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nav import course as C  # noqa: E402
from nav.aligner import Aligner, AlignerParams  # noqa: E402
from nav.frames import SE2  # noqa: E402

Z = 0.8


def visible_face_points(faces, viewer, radius, step=0.025, noise=0.01, seed=0):
    """Map-frame points every `step` m on the faces turned toward `viewer`, within `radius` of it."""
    rng = np.random.default_rng(seed)
    out = []
    for a, b in faces:
        a, b = np.asarray(a), np.asarray(b)
        d = b - a
        n = np.array([d[1], -d[0]]) / np.linalg.norm(d)
        if (viewer - a) @ n <= 0:
            continue
        s = np.linspace(0, 1, max(2, int(np.linalg.norm(d) / step)))
        p = a + s[:, None] * d
        out.append(p[np.linalg.norm(p - viewer, axis=1) < radius])
    p = np.concatenate(out)
    return p + rng.normal(0, noise, p.shape)


def to_odom(T, pts):
    Ti = T.inverse()
    return np.array([Ti.apply(x, y)[:2] for x, y in pts])


def test_recovers_known_offset_near_launch():
    course = C.load(C.default_map_path())
    al = Aligner(course, AlignerParams())
    al.init_at_launch((0.0, 0.0), course.launch)
    T_true = SE2(course.launch[0] + 0.15, course.launch[1] + 0.10, math.radians(3.0))
    viewer = np.array(T_true.apply(0.0, 0.0)[:2])                   # the vehicle, at the odom origin
    pts = to_odom(T_true, visible_face_points(course.faces_at(Z), viewer, 2.0))

    for k in range(3):
        r = al.update(pts, Z, now=float(k), sensor_odom_xy=(0.0, 0.0))
        assert r.accepted, r.why
    assert math.hypot(al.T.x - T_true.x, al.T.y - T_true.y) < 0.02
    assert abs(math.degrees(al.T.yaw - T_true.yaw)) < 0.3
    assert al.sigma_xy() < 0.05


def test_one_straight_wall_moves_only_cross_wall_and_yaw():
    wall = C.Box("long", 0.0, 1.0, 4.0, 0.1, 0.0, 1.7, 0.0)      # along x, south face at y = 0.95
    course = C.Course(1.7, (-3.0, 3.0), (-3.0, 3.0), (0.0, 3.0), (0.0, 0.0), (0.0, 0.0, 0.5), boxes=[wall])
    al = Aligner(course, AlignerParams())                          # T = identity, diagonal prior
    T_true = SE2(0.10, 0.08, math.radians(2.0))                    # 10 cm along, 8 cm across, 2 deg
    xs = np.linspace(-1.5, 1.5, 120)
    pts = to_odom(T_true, np.stack([xs, np.full_like(xs, 0.95)], axis=1))
    sig_along0 = math.sqrt(al.cov[0, 0])

    for k in range(2):
        r = al.update(pts, Z, now=float(k), sensor_odom_xy=(0.0, 0.0))
        assert r.accepted, r.why
    assert abs(al.T.x) < 1e-3                                       # along the wall: unobserved, unchanged
    assert abs(al.T.y - T_true.y) < 0.01
    assert abs(math.degrees(al.T.yaw - T_true.yaw)) < 0.2
    assert math.sqrt(al.cov[0, 0]) > 0.95 * sig_along0              # and its sigma stays the prior's
    assert math.sqrt(al.cov[1, 1]) < 0.3 * sig_along0


def floor_sheet(rng, n=600):
    """Odom points of the floor ahead of a drone at the odom origin facing +y, as wall_points returned
    them without the floor cut: spread over range 0.35-2.5 m and the camera's width."""
    f = rng.uniform(0.35, 2.5, n)
    return np.stack([rng.uniform(-0.9, 0.9, n) * f, f], axis=1)


def test_run1_floor_points_never_move_the_map():
    """Lab run1 (2026-11-06): on the launch mark at z 0.04 the floor came in as "wall" points; the
    aligner accepted 4 updates (first: 45 points, rms 14.4 cm) and moved the map 0.40 m, 5.8 deg,
    and the drone flew into the corridor wall. This floor sheet reproduces that first update
    (45 points, rms 14.3 cm, accepted before the fit gate)."""
    course = C.load(C.default_map_path())
    rng = np.random.default_rng(1)
    for _ in range(50):
        al = Aligner(course, AlignerParams())
        al.init_at_launch((0.0, 0.0), course.launch)
        for k in range(4):
            al.update(floor_sheet(rng), 0.04, now=float(k), sensor_odom_xy=(0.0, 0.0))
        assert math.hypot(al.T.x - course.launch[0], al.T.y - course.launch[1]) < 0.01
        assert abs(math.degrees(al.T.yaw)) < 0.2


def test_noisy_real_walls_still_align():
    """The fit gate must still pass real walls with stereo depth noise (8 cm per point)."""
    course = C.load(C.default_map_path())
    al = Aligner(course, AlignerParams())
    al.init_at_launch((0.0, 0.0), course.launch)
    T_true = SE2(course.launch[0] + 0.12, course.launch[1] - 0.06, math.radians(2.0))
    viewer = np.array(T_true.apply(0.0, 0.0)[:2])
    pts = to_odom(T_true, visible_face_points(course.faces_at(Z), viewer, 3.0, noise=0.08, seed=3))
    accepted = [al.update(pts, Z, now=float(k), sensor_odom_xy=(0.0, 0.0)).accepted for k in range(3)]
    assert any(accepted)
    assert math.hypot(al.T.x - T_true.x, al.T.y - T_true.y) < 0.05


def test_uncertainty_has_a_floor():
    """Many updates on the same walls must not shrink the vehicle's sigma below the floor, or the
    step gate would lock the estimate."""
    course = C.load(C.default_map_path())
    p = AlignerParams()
    al = Aligner(course, p)
    al.init_at_launch((0.0, 0.0), course.launch)
    pts = to_odom(al.T, visible_face_points(course.faces_at(Z), np.array(course.launch), 3.0, noise=0.01))
    for k in range(50):
        al.update(pts, Z, now=float(k), sensor_odom_xy=(0.0, 0.0))
    assert math.sqrt(np.linalg.eigvalsh(al.cov[:2, :2])[0]) >= p.min_sigma_xy_m - 1e-9
    assert math.sqrt(al.cov[2, 2]) >= math.radians(p.min_sigma_yaw_deg) - 1e-9
