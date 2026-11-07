"""nav.aligner: recovers a known map <- odom error from wall points, and moves only observable
directions (one straight wall: cross-wall position + yaw, never along the wall).

Pure numpy, synthetic courses built from nav.course.Box; nothing here depends on timing."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nav import config as CFG  # noqa: E402
from nav import course as C  # noqa: E402
from nav.aligner import Aligner, AlignerParams  # noqa: E402
from nav.frames import SE2  # noqa: E402

Z = 0.8
YAWED = 1.2217                                                      # the chicane walls' yaw in course_map.yaml


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


def rotate(pts, ang):
    c, s = math.cos(ang), math.sin(ang)
    return pts @ np.array([[c, -s], [s, c]]).T


def make_course(boxes):
    return C.Course(1.7, (-3.0, 3.0), (-3.0, 3.0), (0.0, 3.0), (0.0, 0.0), (0.0, 0.0, 0.5), boxes=list(boxes))


def single_wall():
    """One wall along x, south face at y = 0.95, seen from the origin; 120 noise-free points."""
    wall = C.Box("long", 0.0, 1.0, 4.0, 0.1, 0.0, 1.7, 0.0)
    xs = np.linspace(-1.5, 1.5, 120)
    return make_course([wall]), np.stack([xs, np.full_like(xs, 0.95)], axis=1)


def corridor():
    """A corridor along y (inner faces x = +-0.599) closed by a wall ahead (inner face y = 2.549):
    all three dof observable from the origin."""
    west = C.Box("west", -0.65, 1.0, 3.1, 0.1016, 0.0, 1.7, math.pi / 2)
    east = C.Box("east", 0.65, 1.0, 3.1, 0.1016, 0.0, 1.7, math.pi / 2)
    front = C.Box("front", 0.0, 2.6, 1.4, 0.1016, 0.0, 1.7, 0.0)
    course = make_course([west, east, front])
    pts_map = visible_face_points(course.faces_at(Z), np.zeros(2), 3.0, noise=0.005)
    return course, pts_map


def reference_match(al, P, x, fc, sensor):
    """The original (n, F, 2) einsum implementation of Aligner._match, kept as the oracle."""
    p = al.p
    A, D, L, N = fc.A, fc.D, fc.L, fc.N
    same = (np.abs(N @ N.T) > 0.999) & (np.abs(np.einsum("fk,fgk->fg", N, A[None] - A[:, None])) < 1e-3)
    c, s = math.cos(x[2]), math.sin(x[2])
    R = np.array([[c, -s], [s, c]])
    Q = P @ R.T + x[:2]
    rel = Q[:, None, :] - A[None]
    u = np.einsum("nfk,fk->nf", rel, D) / (L * L)
    dist = np.linalg.norm(rel - np.clip(u, 0.0, 1.0)[..., None] * D, axis=2)
    if sensor is not None:
        S = R @ np.asarray(sensor, float) + x[:2]
        dist[:, ((S - A) * N).sum(1) <= 0.0] = np.inf
    i = np.arange(len(Q))
    best = np.argmin(dist, axis=1)
    d1 = dist[i, best]
    d2 = np.where(same[best], np.inf, dist).min(axis=1)
    ub = u[i, best]
    past = np.maximum(0.0, np.maximum(-ub, ub - 1.0)) * L[best]
    keep = (d1 < p.match_max_m) & (d2 - d1 > p.corner_m) & (past < p.corner_m)
    f, q = best[keep], Q[keep]
    n = N[f]
    r = ((q - A[f]) * n).sum(1)
    v = q - x[:2]
    H = np.stack([n[:, 0], n[:, 1], -n[:, 0] * v[:, 1] + n[:, 1] * v[:, 0]], axis=1)
    sig, k = p.meas_sigma_m, p.huber_k
    a = np.abs(r) / sig
    huber = np.where(a <= k, 1.0, k / np.maximum(a, 1e-12))
    cap = np.minimum(1.0, p.max_points_eff / np.bincount(f, minlength=len(A))[f])
    return same, H, r, huber * cap / (sig * sig)


# ---------------------------------------------------------------------------- existing behaviour
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
    course, pts_map = single_wall()
    al = Aligner(course, AlignerParams())                          # T = identity, diagonal prior
    T_true = SE2(0.10, 0.08, math.radians(2.0))                    # 10 cm along, 8 cm across, 2 deg
    pts = to_odom(T_true, pts_map)
    sig_along0 = math.sqrt(al.cov[0, 0])

    for k in range(2):
        r = al.update(pts, Z, now=float(k), sensor_odom_xy=(0.0, 0.0))
        assert r.accepted, r.why
    assert abs(al.T.x) < 1e-3                                       # along the wall: unobserved, unchanged
    assert abs(al.T.y - T_true.y) < 0.01
    assert abs(math.degrees(al.T.yaw - T_true.yaw)) < 0.2
    assert math.sqrt(al.cov[0, 0]) > 0.95 * sig_along0              # and its sigma stays the prior's
    assert math.sqrt(al.cov[1, 1]) < 0.3 * sig_along0


# ---------------------------------------------------------------------------- geometry
def test_face_normals_point_outward_for_a_yawed_box():
    """Independent of the test helper: every outward normal points away from the box centre."""
    box = C.Box("c", 0.3, -0.2, 1.0, 0.1016, 0.0, 1.7, YAWED)
    al = Aligner(make_course([box]))
    fc = al._faces_at(Z)
    mid = fc.A + 0.5 * fc.D
    assert fc.F == 4
    assert np.all(((mid - np.array([box.cx, box.cy])) * fc.N).sum(1) > 0.04)
    assert np.allclose(np.linalg.norm(fc.N, axis=1), 1.0)
    assert np.allclose((fc.N * fc.D).sum(1), 0.0)                   # normal is perpendicular to the face


def test_recovers_offset_against_yawed_walls():
    """Two walls at the chicane yaw forming an L around the origin: the association, residual sign
    and yaw Jacobian are exercised with normals that are not axis aligned."""
    th = YAWED
    n1 = np.array([-math.sin(th), math.cos(th)])                     # long-face normal of a box at yaw th
    n2 = np.array([-math.cos(th), -math.sin(th)])                    # long-face normal of a box at yaw th+90
    w1 = C.Box("w1", -n1[0], -n1[1], 2.0, 0.1016, 0.0, 1.7, th)      # inner face 0.95 m from the origin
    w2 = C.Box("w2", -n2[0], -n2[1], 2.0, 0.1016, 0.0, 1.7, th + math.pi / 2)
    course = make_course([w1, w2])
    pts_map = visible_face_points(course.faces_at(Z), np.zeros(2), 2.5, noise=0.005)
    assert len(pts_map) > 120
    T_true = SE2(0.08, -0.06, math.radians(2.0))
    pts = to_odom(T_true, pts_map)
    al = Aligner(course)
    for k in range(3):
        r = al.update(pts, Z, now=float(k), sensor_odom_xy=(0.0, 0.0))
        assert r.accepted, r.why
    assert math.hypot(al.T.x - T_true.x, al.T.y - T_true.y) < 0.02
    assert abs(math.degrees(al.T.yaw - T_true.yaw)) < 0.3
    assert 0.02 - 1e-9 <= al.sigma_xy((0.0, 0.0)) < 0.05           # posterior floored at min_sigma_xy_m


def test_match_equals_einsum_reference():
    """The matmul/hypot association is the old (n, F, 2) implementation to rounding, on the real
    map (52 faces at cruise height), with and without face culling."""
    course = C.load(C.default_map_path())
    al = Aligner(course)
    al.init_at_launch((0.0, 0.0), course.launch)
    T_true = SE2(course.launch[0] + 0.12, course.launch[1] - 0.08, math.radians(2.0))
    viewer = np.array(T_true.apply(0.0, 0.0)[:2])
    P = to_odom(T_true, visible_face_points(course.faces_at(Z), viewer, 3.0, noise=0.02, seed=3))
    fc = al._faces_at(Z)
    assert fc.F == 52
    x = np.array([al.T.x + 0.03, al.T.y - 0.02, math.radians(1.0)])
    for sensor in ((0.0, 0.0), None):
        same_ref, H_ref, r_ref, w_ref = reference_match(al, P, x, fc, sensor)
        H, r, w = al._match(P, x, fc, sensor)
        assert np.array_equal(fc.same, same_ref)
        assert H.shape == H_ref.shape and len(r) > 30
        assert np.allclose(H, H_ref, atol=1e-9) and np.allclose(r, r_ref, atol=1e-9)
        assert np.allclose(w, w_ref, rtol=1e-9)


def test_faces_built_once_per_face_set():
    """Synthetic course: the real map's tube half-width changes with z, so there the per-cm face
    sets differ by millimetres and each cm is (correctly) its own build."""
    course, _ = single_wall()
    al = Aligner(course)
    assert al._faces_at(0.80) is al._faces_at(0.81)                # neighbouring cm share one build
    assert len(al._faces_by_geom) == 1 and len(al._faces) == 2
    assert al._faces_at(5.0) is None                                 # above every wall
    assert not al.update(np.zeros((40, 2)), 5.0, 0.0).accepted


# ---------------------------------------------------------------------------- observability
def test_single_wall_never_moves_along_wall_over_many_updates():
    """20 update/predict cycles on one wall: the along-wall mean stays at the prior and its sigma
    grows exactly by the process noise (the floor and the measurement never touch it)."""
    course, pts_map = single_wall()
    al = Aligner(course)
    pts = to_odom(SE2(0.10, 0.08, math.radians(2.0)), pts_map)
    for k in range(20):
        r = al.update(pts, Z, now=float(k), sensor_odom_xy=(0.0, 0.0))
        assert r.accepted, r.why
        al.predict(0.05, odom_xy=(0.0, 0.0))
    assert abs(al.T.x) < 1e-6
    assert abs(math.sqrt(al.cov[0, 0]) - (0.10 + 20 * 0.05 * 0.02)) < 1e-6
    assert abs(al.T.y - 0.08) < 0.005


def test_corridor_recovers_start_error_within_three_updates():
    course, pts_map = corridor()
    T_true = SE2(0.07, 0.07, math.radians(2.0))                     # 0.1 m, 2 deg
    pts = to_odom(T_true, pts_map)
    al = Aligner(course)
    for k in range(3):
        r = al.update(pts, Z, now=float(k), sensor_odom_xy=(0.0, 0.0))
        assert r.accepted, r.why
    assert math.hypot(al.T.x - T_true.x, al.T.y - T_true.y) < 0.02
    assert abs(math.degrees(al.T.yaw - T_true.yaw)) < 0.3


# ---------------------------------------------------------------------------- robustness
def test_outliers_beyond_match_max_have_no_effect():
    """A 'boulder' ring 1 m from any face: not associated, so T is bit-for-bit the clean result."""
    course, pts_map = single_wall()
    T_true = SE2(0.10, 0.08, math.radians(2.0))
    clean = to_odom(T_true, pts_map)
    ang = np.linspace(0, 2 * math.pi, 40, endpoint=False)
    ring = np.stack([0.5 + 0.23 * np.cos(ang), -0.3 + 0.23 * np.sin(ang)], axis=1)
    dirty = np.concatenate([clean, to_odom(T_true, ring)])
    a, b = Aligner(course), Aligner(course)
    ra = a.update(clean, Z, 0.0, sensor_odom_xy=(0.0, 0.0))
    rb = b.update(dirty, Z, 0.0, sensor_odom_xy=(0.0, 0.0))
    assert ra.accepted and rb.accepted
    assert rb.n == ra.n == len(clean)
    assert np.allclose([a.T.x, a.T.y, a.T.yaw], [b.T.x, b.T.y, b.T.yaw], atol=1e-12)
    assert np.allclose(a.cov, b.cov, atol=1e-12)


def test_nan_and_inf_points_are_ignored():
    course, pts_map = single_wall()
    clean = to_odom(SE2(0.10, 0.08, math.radians(2.0)), pts_map)
    bad = np.array([[np.nan, np.nan], [np.inf, 0.0], [0.0, np.nan]])
    a, b = Aligner(course), Aligner(course)
    ra = a.update(clean, Z, 0.0, sensor_odom_xy=(0.0, 0.0))
    rb = b.update(np.concatenate([clean, bad]), Z, 0.0, sensor_odom_xy=(0.0, 0.0))
    assert ra.accepted and rb.accepted and rb.n == ra.n
    assert np.allclose([a.T.x, a.T.y, a.T.yaw], [b.T.x, b.T.y, b.T.yaw], atol=1e-12)
    assert np.isfinite(b.cov).all()


def test_empty_match_is_rejected_and_state_untouched():
    course, _ = single_wall()
    al = Aligner(course)
    T0, cov0 = al.T, al.cov.copy()
    far = np.stack([np.linspace(-1, 1, 60), np.full(60, -1.5)], axis=1)   # 2.45 m from the face
    r = al.update(far, Z, 0.0, sensor_odom_xy=(0.0, 0.0))
    assert not r.accepted and r.n == 0 and "matched" in r.why
    assert al.T == T0 and np.array_equal(al.cov, cov0)
    r = al.update(np.zeros((0, 2)), Z, 0.0, sensor_odom_xy=(0.0, 0.0))
    assert not r.accepted and al.T == T0


def test_singular_or_nan_prior_is_rejected_not_raised():
    course, pts_map = single_wall()
    pts = to_odom(SE2(0.0, 0.05, 0.0), pts_map)
    al = Aligner(course)
    al.cov = np.zeros((3, 3))
    r = al.update(pts, Z, 0.0, sensor_odom_xy=(0.0, 0.0))
    assert not r.accepted and "singular" in r.why and al.T == SE2()
    al.cov = np.full((3, 3), np.nan)
    r = al.update(pts, Z, 0.0, sensor_odom_xy=(0.0, 0.0))
    assert not r.accepted and al.T == SE2()
    al.predict(float("nan"), odom_xy=(0.0, 0.0))                    # ignored, does not poison cov
    assert np.isnan(al.cov).all()                                    # (still the NaN we planted)
    r = al.update(pts, Z, 0.0, sensor_odom_xy=(float("nan"), 0.0))
    assert not r.accepted


def test_absolute_step_gate_rejects_a_large_jump():
    course, pts_map = single_wall()
    al = Aligner(course, AlignerParams(gate_nsigma=0.0))           # only the absolute gates
    pts = to_odom(SE2(0.0, 0.20, 0.0), pts_map)                     # 20 cm across: associates, but
    al.p.max_step_xy_m = 0.15                                       # more than the gate allows
    r = al.update(pts, Z, 0.0, sensor_odom_xy=(0.0, 0.0))
    assert not r.accepted and "max_step_xy_m" in r.why and al.T == SE2()
    assert abs(r.dy - 0.20) < 0.02                                  # the proposal is still reported


def test_rigid_rotation_of_a_frame_is_rejected_by_the_prior_gate():
    """A frame rotated 4 deg about the sensor (the depth time-alignment fault) fits the walls with a
    small rms and passes the 8 deg absolute gate, yet it is 8 prior sigmas once yaw has converged:
    rejected, state untouched, and the next good frame is accepted again."""
    course, pts_map = corridor()
    pts = to_odom(SE2(0.0, 0.0, 0.0), pts_map)
    al = Aligner(course)
    for k in range(3):
        assert al.update(pts, Z, float(k), sensor_odom_xy=(0.0, 0.0)).accepted
    T0 = al.T
    sig_yaw = math.sqrt(al.cov[2, 2])
    assert abs(math.degrees(sig_yaw) - 0.5) < 1e-6                 # at the yaw floor, odom origin = sensor
    r = al.update(rotate(pts, math.radians(4.0)), Z, 3.0, sensor_odom_xy=(0.0, 0.0))
    assert not r.accepted and "gate_nsigma" in r.why, r.why
    assert abs(math.degrees(r.dyaw)) > 1.5                          # the frame did pull toward 4 deg
    assert abs(math.degrees(r.dyaw)) < 8.0                          # ...and the absolute gate alone would pass it
    assert al.T == T0
    r = al.update(pts, Z, 4.0, sensor_odom_xy=(0.0, 0.0))
    assert r.accepted, r.why
    assert abs(math.degrees(al.T.yaw)) < 0.2


def test_distorted_frame_is_trusted_less():
    """Side walls squeezed toward each other by 12 cm each: no rigid transform fits, rms after
    convergence is above rms_trust_k * meas_sigma, the frame is accepted with reduced trust and
    the symmetric error does not move the estimate."""
    course, pts_map = corridor()
    squeezed = pts_map.copy()
    side = np.abs(squeezed[:, 0]) > 0.5
    squeezed[side, 0] -= 0.12 * np.sign(squeezed[side, 0])
    al = Aligner(course)
    r = al.update(squeezed, Z, 0.0, sensor_odom_xy=(0.0, 0.0))
    assert r.accepted and "trust" in r.why, r.why
    assert math.hypot(al.T.x, al.T.y) < 0.01 and abs(math.degrees(al.T.yaw)) < 0.2
    al2 = Aligner(course, AlignerParams(rms_trust_k=0.0))
    r2 = al2.update(squeezed, Z, 0.0, sensor_odom_xy=(0.0, 0.0))
    assert r2.accepted and "trust" not in r2.why


# ---------------------------------------------------------------------------- state / API
def test_predict_is_linear_in_distance_and_acts_at_the_vehicle():
    a, b = Aligner(make_course([])), Aligner(make_course([]))
    before = a.sigma_xy((2.0, 0.0))
    a.predict(1.0, odom_xy=(2.0, 0.0))
    b.predict(0.5, odom_xy=(2.0, 0.0))
    b.predict(0.5, odom_xy=(2.0, 0.0))
    assert np.allclose(a.cov, b.cov, atol=1e-12)                    # call rate does not matter
    assert abs((a.sigma_xy((2.0, 0.0)) - before) - 0.02) < 1e-9     # drift_per_m, at the vehicle
    assert np.allclose(a.cov, a.cov.T)
    c = Aligner(make_course([]))
    assert abs(c.sigma_xy() - 0.10) < 1e-9
    assert abs(c.sigma_xy((5.0, 0.0)) - math.hypot(0.10, 5.0 * math.radians(3.0))) < 1e-9   # yaw x lever


def test_init_at_launch_z_anchor():
    al = Aligner(make_course([]))
    al.init_at_launch((1.0, 2.0), (3.0, 4.0))
    assert al.T.z == 0.0 and np.allclose(al.to_map(1.0, 2.0, 0.7, 0.0)[:3], (3.0, 4.0, 0.7))
    al.init_at_launch((1.0, 2.0), (3.0, 4.0), yaw_offset=math.radians(10.0), odom_z=0.3, launch_z=0.0)
    X, Y, Zm, yaw = al.to_map(1.0, 2.0, 0.3, 0.0)
    assert np.allclose((X, Y, Zm), (3.0, 4.0, 0.0)) and abs(math.degrees(yaw) - 10.0) < 1e-9
    x, y, z, _ = al.to_odom(3.0, 4.0, 0.0, 0.0)
    assert np.allclose((x, y, z), (1.0, 2.0, 0.3))
    course, pts_map = single_wall()                                 # an update keeps the z anchor
    al = Aligner(course)
    al.init_at_launch((0.0, 0.0), (0.0, 0.0), odom_z=0.3)
    r = al.update(to_odom(SE2(0.0, 0.05, 0.0), pts_map), Z, 0.0, sensor_odom_xy=(0.0, 0.0))
    assert r.accepted and al.T.z == -0.3


def test_params_fill_from_yaml_keys():
    p = CFG.fill(AlignerParams, {"min_sigma_xy_m": 0.03, "gate_nsigma": 5, "rms_trust_k": 2,
                                 "min_sigma_yaw_deg": 1})
    assert p.min_sigma_xy_m == 0.03 and p.gate_nsigma == 5.0 and isinstance(p.gate_nsigma, float)
    assert p.rms_trust_k == 2.0 and p.min_sigma_yaw_deg == 1.0 and p.match_max_m == 0.25
