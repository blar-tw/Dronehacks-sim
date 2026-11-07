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
