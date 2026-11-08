"""nav: course + frames (S1), A* (S2), map localization (S4) on the real course map."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nav import course as COURSE  # noqa: E402
from nav import planner  # noqa: E402
from nav.frames import (SE2, enu_to_ned, heading_to_yaw, ned_to_enu, odom_after_reset, wrap,  # noqa: E402
                        yaw_to_heading)
from nav.localizer import Localizer, LocParams, wall_points  # noqa: E402
from nav.mission import Mission  # noqa: E402
from nav.yaw import ViewScorer  # noqa: E402

C = COURSE.load()
Z = 0.8


def test_course_loads_walls_tube_and_lines():
    names = {b.name for b in C.boxes}
    assert {"m1_maze_divider", "barrier_1", "tube_west", "tube_east"} <= names
    assert {"first_corridor", "maze_gate_a", "tunnel_south_end"} <= set(C.lines)
    assert C.clearance([C.launch], 0.55, 1.1)[0] > 0.33           # the launch mark is free at flight height


def test_transforms_round_trip():
    assert np.allclose(enu_to_ned(*ned_to_enu(1.0, 2.0, -3.0)), (1.0, 2.0, -3.0))
    for h in (0.0, 1.0, -2.5):
        assert math.isclose(yaw_to_heading(heading_to_yaw(h)), h, abs_tol=1e-9)
    T = SE2.from_poses([-2.5, 0.9], math.pi / 2, [0.3, -0.2], 0.1)
    assert np.allclose(T.apply([0.3, -0.2]), [-2.5, 0.9])
    P = np.random.default_rng(0).normal(size=(5, 2))
    assert np.allclose(T.inverse_apply(T.apply(P)), P)


def _cross(p, q, a, b):
    s = lambda u, v, w: (v[0] - u[0]) * (w[1] - u[1]) - (v[1] - u[1]) * (w[0] - u[0])
    return (s(a, b, p) > 0) != (s(a, b, q) > 0) and (s(p, q, a) > 0) != (s(p, q, b) > 0)


def test_path_crosses_the_checkpoints_in_order_and_stays_clear():
    wps = planner.plan(C)
    P = np.array([[w.x, w.y] for w in wps])
    order = []
    for k in range(len(P) - 1):
        for name, line in [*C.lines.items(), ("window", C.window)]:
            if name != "chicane_window" and _cross(P[k], P[k + 1], line.a, line.b):
                order.append(name)
    assert order == ["first_corridor", "maze_gate_a", "tunnel_south_end", "window"], order
    assert np.linalg.norm(P[-1] - C.pad_center) < 0.05
    dense = np.vstack([a + np.linspace(0, 1, 30)[:, None] * (b - a) for a, b in zip(P, P[1:])])
    assert C.clearance(dense[5:], 0.55, 1.1).min() > 0.30             # past the launch mark


def test_return_leg_is_over_the_walls_and_under_the_net_to_the_launch_mark():
    p = planner.PlanParams()
    r = planner.plan_return(C, p)
    assert np.allclose([r[0].x, r[0].y], C.pad_center) and np.allclose([r[-1].x, r[-1].y], C.launch)
    top_net = C.doc["net"]["z"][1]
    assert all(C.wall_top + 0.4 < w.z < top_net - 0.4 for w in r)


def _scan(xy, yaw, scorer):
    """Wall samples the camera at xy/yaw would see (the map itself, no noise)."""
    vis = scorer.visible(xy)
    d = scorer.S - xy
    bearing = np.abs((np.arctan2(d[:, 1], d[:, 0]) - yaw + math.pi) % (2 * math.pi) - math.pi)
    return scorer.S[vis & (bearing < math.radians(36))]


def test_injected_offset_converges():
    scorer = ViewScorer(C, Z, step_m=0.03)
    loc = Localizer(C, Z)
    truth_xy, truth_yaw = np.array([-1.37, 1.8]), -math.pi / 2 + 0.2     # 2nd corridor, facing the barrier
    # odom: the vehicle thinks it is 0.12 m east, 0.08 m south and 3 deg off of where it is
    T_true = SE2(0.12, -0.08, math.radians(3.0))
    odom_xy = T_true.inverse_apply(truth_xy)
    odom_yaw = truth_yaw - T_true.yaw
    loc.start(truth_xy, truth_yaw, odom_xy, odom_yaw)                   # start believing odom = map
    loc.T = SE2(0.0, 0.0, 0.0)
    pts_map = _scan(truth_xy, truth_yaw, scorer)
    pts_odom = T_true.inverse_apply(pts_map)
    for _ in range(10):
        loc.update(pts_odom)
    err = loc.T.apply(odom_xy) - truth_xy
    assert np.linalg.norm(err) < 0.03, (err, loc.last)
    assert abs(math.degrees(loc.T.yaw - T_true.yaw)) < 1.0


def test_floor_points_do_not_move_the_map():
    # a level camera 0.35 m above an empty floor, depth = floor only
    W, H, f = 160, 100, 106.0
    K = (f, f, W / 2, H / 2)
    R_bc = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    v = (np.arange(H)[:, None] - K[3]) / f * np.ones((1, W))
    depth = np.where(v > 0.01, 0.35 / np.maximum(v, 1e-6), np.nan).astype(np.float32)
    pts = wall_points(depth, K, R_bc, np.zeros(3), (1.0, 0.0, 0.0, 0.0), (-2.5, 1.0, 0.35, math.pi / 2), floor_z=0.0)
    loc = Localizer(C, Z)
    loc.start(C.launch, math.pi / 2, (0.0, 0.0), 0.0)
    before = loc.T.vec()
    assert not loc.update(pts)
    assert np.allclose(loc.T.vec(), before)


def _offset_fit(p):
    """test_injected_offset_converges' scene: the right fit moves the vehicle 0.157 m and turns 3 deg."""
    scorer = ViewScorer(C, Z, step_m=0.03)
    loc = Localizer(C, Z, p)
    truth_xy, truth_yaw = np.array([-1.37, 1.8]), -math.pi / 2 + 0.2
    T_true = SE2(0.12, -0.08, math.radians(3.0))
    odom_xy = T_true.inverse_apply(truth_xy)
    loc.start(truth_xy, truth_yaw, odom_xy, truth_yaw - T_true.yaw)
    loc.T = SE2(0.0, 0.0, 0.0)
    return loc, T_true.inverse_apply(_scan(truth_xy, truth_yaw, scorer))


def test_step_cap_rejects_a_fit_that_moves_the_vehicle_too_far():
    loc, pts_odom = _offset_fit(LocParams(max_step_xy_m=0.10))      # the fit needs 0.157 m
    for _ in range(5):
        assert not loc.update(pts_odom)
    assert loc.capped == 5 and loc.accepted == 0, loc.last
    assert "over the cap" in loc.last
    assert np.allclose(loc.T.vec(), 0.0)


def test_step_cap_rejects_a_fit_that_turns_the_map_too_far():
    loc, pts_odom = _offset_fit(LocParams(max_step_yaw_deg=2.0))    # the fit needs 3 deg
    assert not loc.update(pts_odom)
    assert loc.capped == 1 and "over the cap" in loc.last
    assert np.allclose(loc.T.vec(), 0.0)


def test_default_cap_takes_the_same_fit():
    loc, pts_odom = _offset_fit(LocParams())
    assert loc.update(pts_odom), loc.last
    assert loc.capped == 0


def test_odom_reset_keeps_the_map_pose_and_sigma():
    loc = Localizer(C, Z)
    old, yaw_old = np.array([0.3, -0.2]), 0.4
    loc.start(C.launch, math.pi / 2, old, yaw_old)
    T0 = SE2(*loc.T.vec())
    cov0 = loc.vehicle_cov()
    new, d_yaw = np.array([0.9, 0.5]), 0.3                         # a 0.92 m jump: far over the step cap
    loc.odom_reset(old, new, d_yaw)
    assert np.allclose(loc.T.apply(new), C.launch)
    assert abs(wrap(yaw_old + d_yaw + loc.T.yaw - math.pi / 2)) < 1e-9
    assert np.allclose(loc.vehicle_cov(), cov0)
    q_old = np.array([1.5, 2.0])                                    # any fixed point stays put in the map
    assert np.allclose(loc.T.apply(odom_after_reset(q_old, old, new, d_yaw)), T0.apply(q_old))


def test_mission_setpoints_follow_an_odom_reset():
    m = Mission(C, planner.plan(C), Localizer(C, Z), ViewScorer(C, Z))
    m.loc.start(C.launch, math.pi / 2, (0.4, 0.9), math.pi / 2)
    m.Tc = SE2(0.1, -0.2, 0.3)                                      # mid-blend: not equal to loc.T
    m.sp, m.hold_sp = [0.5, 1.0, 0.6, 0.2], [0.4, 0.9, 0.6, 0.1]
    Tc0, sp0 = SE2(*m.Tc.vec()), list(m.sp)
    old, new, d_yaw = (0.4, 0.9), (1.4, 0.4), -0.25
    m.on_odom_reset(old, new, d_yaw)
    assert np.allclose(m.Tc.apply(m.sp[:2]), Tc0.apply(sp0[:2]))   # same map target
    assert abs(wrap(m.sp[3] + m.Tc.yaw - (sp0[3] + Tc0.yaw))) < 1e-9
    assert m.sp[2] == sp0[2]
    assert np.allclose(m.hold_sp[:2], new)                          # the hover point was the vehicle
    assert np.allclose(m.loc.T.apply(new), C.launch)

