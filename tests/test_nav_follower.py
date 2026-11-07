"""nav.follower: carrot speed, corners, and progress that can't skip to a parallel corridor."""
import math

import numpy as np

from nav.follower import Follower, FollowerParams
from nav.frames import wrap

DT = 0.05  # 20 Hz
TAU = 0.4  # first-order vehicle: position and yaw follow the setpoint with this time constant


def fly(f, pts, T=40.0, yaw0=0.0):
    """Fly a simulated first-order vehicle along pts; returns [(t, pos, yaw, setpoint)]."""
    pos, yaw = np.array(pts[0], float), yaw0
    f.set_path(pts, 0.0, tuple(pos), yaw)
    alpha = 1 - math.exp(-DT / TAU)
    log = []
    for i in range(int(T / DT)):
        t = (i + 1) * DT
        sp = f.step(t, tuple(pos), yaw)
        log.append((t, pos.copy(), yaw, sp))
        if sp.done:
            break
        pos += (np.array([sp.x, sp.y, sp.z]) - pos) * alpha
        yaw = wrap(yaw + wrap(sp.yaw - yaw) * alpha)
    return log


def test_straight_line_reaches_speed_respects_accel_and_finishes():
    p = FollowerParams()
    log = fly(Follower(p), [(0, 0, 0.8), (4, 0, 0.8)])
    sps = [sp for *_, sp in log]
    v = np.array([sp.speed_mps for sp in sps])
    assert sps[-1].done and sps[-1].carrot_m == sps[-1].length_m == 4.0
    assert v.max() <= p.speed_mps + 1e-9 and v.max() > p.speed_mps - 0.01
    assert np.all(np.abs(np.diff(v)) <= p.accel_mps2 * DT + 1e-9)          # both up and down
    assert all(sp.carrot_m - sp.progress_m <= p.lead_max_m + 1e-9 for sp in sps)
    assert np.linalg.norm(log[-1][1] - (4, 0, 0.8)) <= p.arrive_m


def test_l_corner_slows_the_carrot():
    p = FollowerParams(corner_stop_deg=0)                                 # stop corners off
    log = fly(Follower(p), [(0, 0, 0.8), (3, 0, 0.8), (3, 3, 0.8)])
    sps = [sp for *_, sp in log]
    # approaching the corner, and until the vehicle itself has rounded it
    near = [sp.speed_mps for sp in sps if 2.9 <= sp.carrot_m and sp.progress_m < 3.0]
    assert near and max(near) <= p.corner_speed_mps + 0.01
    assert max(sp.speed_mps for sp in sps if sp.carrot_m < 2.0) > p.speed_mps - 0.01
    assert not any(sp.holding for sp in sps)
    assert sps[-1].done


def test_l_corner_stops_and_holds_then_completes():
    p = FollowerParams()                                                  # 90 deg > corner_stop_deg 45
    log = fly(Follower(p), [(0, 0, 0.8), (3, 0, 0.8), (3, 3, 0.8)])
    holding = [(t, pos, sp) for t, pos, _, sp in log if sp.holding]
    assert holding and all(sp.carrot_m == 3.0 and sp.speed_mps == 0.0 for *_, sp in holding)
    arrived = [t for t, pos, _ in holding if np.linalg.norm(pos - (3, 0, 0.8)) <= p.arrive_m]
    assert abs((arrived[-1] - arrived[0]) - p.corner_hold_s) <= 2 * DT    # still for ~corner_hold_s
    assert log[-1][3].done


def test_parallel_corridor_never_steals_progress():
    # U-turn: two legs 0.6 m apart. A vehicle pushed 0.35 m toward the second leg is closer to
    # it than to its own, but progress stays on the first leg (windowed search).
    p = FollowerParams()
    pts = [(0, 0, 0.8), (3, 0, 0.8), (3, 0.6, 0.8), (0, 0.6, 0.8)]
    f = Follower(p)
    f.set_path(pts, 0.0, (0, 0, 0.8), 0.0)
    for i, x in enumerate(np.arange(0.0, 2.5, 0.02)):
        sp = f.step((i + 1) * DT, (x, 0.35, 0.8), 0.0)
        assert abs(sp.progress_m - x) < 1e-6
    # and a normal flight round the U: progress never jumps
    s = [sp.progress_m for *_, sp in fly(Follower(p), pts)]
    assert np.all(np.diff(s) <= 0.1) and np.all(np.diff(s) >= -0.05)


# ----------------------------------------------------------------- review additions (nav-review)
L_CORNER = [(0, 0, 0.8), (3, 0, 0.8), (3, 3, 0.8)]


def run_ideal(p, pts, dt, T=60.0, offset=(0.0, 0.0, 0.0)):
    """Vehicle that sits exactly on the previous setpoint (plus a constant offset) and holds its yaw.
    Steps every dt; stops at done, or after T. Returns [(t, pos, yaw, setpoint)]."""
    f = Follower(p)
    off = np.array(offset, float)
    pos, yaw = np.array(pts[0], float) + off, 0.0
    f.set_path(pts, 0.0, tuple(pos), yaw)
    log = []
    for i in range(int(round(T / dt))):
        t = (i + 1) * dt
        sp = f.step(t, tuple(pos), yaw)
        log.append((t, pos.copy(), yaw, sp))
        if sp.done:
            break
        pos, yaw = np.array([sp.x, sp.y, sp.z]) + off, sp.yaw
    return log


def test_brake_profile_lands_exactly_on_the_stop_for_any_dt():
    # d = v (v + a dt) / 2a is what speeds v, v - a dt, ..., 0 cover, so the carrot must land exactly
    # on the stop (no overshoot), the speed must never go negative, and each step may drop the speed
    # by at most a dt, except the last one, which is clipped to the remaining distance (<= 9/8 a dt).
    p = FollowerParams()
    for L in (3.7137, 2.01, 0.9):
        for dt in (0.05, 0.1, 0.2, 0.5):                       # 0.5 is clamped to DT_MAX_S inside
            log = run_ideal(p, [(0, 0, 0.8), (L, 0, 0.8)], dt)
            sps = [sp for *_, sp in log]
            s = np.array([sp.carrot_m for sp in sps])
            v = np.array([sp.speed_mps for sp in sps])
            Lp = sps[-1].length_m                                   # == L up to one ulp of the norm
            assert sps[-1].done and s[-1] == Lp and abs(Lp - L) < 1e-12, (L, dt, s[-1])
            assert np.all(np.diff(s) >= 0) and np.all(s <= Lp) and np.all(v >= 0), (L, dt)
            eff = min(dt, 0.2)
            assert np.all(np.diff(v) >= -(9 / 8) * p.accel_mps2 * eff - 1e-9), (L, dt)
            assert np.all(np.diff(v) <= p.accel_mps2 * eff + 1e-9), (L, dt)


def test_repeated_points_and_a_single_point_path_give_finite_setpoints():
    # a replan from a vehicle sitting on the gate entry can hand the follower two equal points in a
    # row; a zero-length segment made the unit tangent NaN and every setpoint NaN
    p = FollowerParams()
    pts = [(0, 0, 0.8), (0, 0, 0.8), (2, 0, 0.8), (2, 0, 0.8), (2, 2, 0.8)]
    f = Follower(p)
    f.set_path(pts, 0.0, (0, 0, 0.8), 0.0)
    assert len(f.P) == 3 and f.L == 4.0 and np.all(np.isfinite(f.U))
    log = fly(Follower(p), pts)
    assert all(np.all(np.isfinite([sp.x, sp.y, sp.z, sp.yaw, sp.speed_mps])) for *_, sp in log)
    assert log[-1][3].done
    # a single point is a zero-length path: setpoint = that point, yaw kept, done once the vehicle is there
    f = Follower(p)
    f.set_path([(1.0, 2.0, 0.8)], 0.0, (1.0, 2.0, 0.8), 0.3)
    sp = f.step(DT, (1.0, 2.0, 0.8), 0.3)
    assert (sp.x, sp.y, sp.z) == (1.0, 2.0, 0.8) and abs(sp.yaw - 0.3) < 1e-9
    assert sp.done and sp.length_m == 0.0 and sp.speed_mps == 0.0 and not sp.holding
    sp = f.step(2 * DT, (1.5, 2.0, 0.8), 0.3)                  # vehicle displaced: not done, no NaN
    assert not sp.done and np.all(np.isfinite([sp.x, sp.y, sp.z, sp.yaw, sp.cross_track_m]))


def test_stop_every_vertex_holds_at_every_vertex_including_straight_ones():
    # strict stop-and-go (team decision 5): the collinear vertex at s = 2 (a gate entry point in the
    # planner's output looks like this) becomes a stop too; default behaviour is unchanged
    pts = [(0, 0, 0.8), (2, 0, 0.8), (4, 0, 0.8), (4, 2, 0.8)]
    base = fly(Follower(FollowerParams()), pts)
    assert {sp.carrot_m for *_, sp in base if sp.holding} == {4.0}
    p = FollowerParams(stop_every_vertex=True)
    log = fly(Follower(p), pts)
    assert {sp.carrot_m for *_, sp in log if sp.holding} == {2.0, 4.0}
    assert all(sp.speed_mps == 0.0 for *_, sp in log if sp.holding)
    for s_stop, vertex in ((2.0, pts[1]), (4.0, pts[2])):
        still = [t for t, pos, _, sp in log
                 if sp.holding and sp.carrot_m == s_stop and np.linalg.norm(pos - np.array(vertex, float)) <= p.arrive_m]
        assert still and abs((still[-1] - still[0]) - p.corner_hold_s) <= 2 * DT, s_stop
    assert log[-1][3].done and base[-1][3].done
    assert log[-1][0] > base[-1][0] + p.corner_hold_s             # the price of the extra stop


def test_corner_arrive_timeout_releases_a_vehicle_that_cannot_settle():
    # vehicle tracks perfectly but 0.3 m to the side (> arrive_m): it can never "arrive" at the corner.
    # Default: the hold starts after corner_arrive_timeout_s anyway and the carrot moves on.
    p = FollowerParams()
    log = run_ideal(p, L_CORNER, DT, T=20.0, offset=(0.0, 0.3, 0.0))
    hold = [t for t, *_, sp in log if sp.holding]
    assert hold and all(sp.carrot_m == 3.0 for *_, sp in log if sp.holding)
    assert abs((hold[-1] - hold[0]) - (p.corner_arrive_timeout_s + p.corner_hold_s)) <= 2 * DT
    assert log[-1][3].carrot_m > 3.5 and not log[-1][3].holding and not log[-1][3].done
    # 0 = the old behaviour: wait for arrival forever (the mission's segment timeout has to free it)
    log0 = run_ideal(FollowerParams(corner_arrive_timeout_s=0.0), L_CORNER, DT, T=20.0, offset=(0.0, 0.3, 0.0))
    assert log0[-1][3].holding and log0[-1][3].carrot_m == 3.0
    # a vehicle that does arrive is released by arrival, long before the timeout
    log1 = run_ideal(p, L_CORNER, DT, T=20.0)
    hold1 = [t for t, *_, sp in log1 if sp.holding]
    assert hold1 and (hold1[-1] - hold1[0]) < p.corner_hold_s + 0.5


def test_corner_limit_zigzag_is_the_largest_bend_and_same_direction_turns_add_up():
    p = FollowerParams()
    c60, s60 = math.cos(math.radians(60)), math.sin(math.radians(60))
    p0, p1 = (0.0, 0.0, 0.8), (0.3, 0.0, 0.8)
    p2 = (p1[0] + 0.2 * c60, p1[1] + 0.2 * s60, 0.8)                 # 60 deg left at s = 0.3
    zig = [p0, p1, p2, (p2[0] + 1.0, p2[1], 0.8)]                    # then 60 deg right at s = 0.5
    same = [p0, p1, p2, (p2[0] + math.cos(math.radians(120)), p2[1] + math.sin(math.radians(120)), 0.8)]
    single = [p0, p1, p2]

    def limit(pts):
        f = Follower(p)
        f.set_path(pts, 0.0, p0, 0.0)                                # both vertices within lookahead_m
        return f._corner_limit()

    v60 = p.speed_mps - (p.speed_mps - p.corner_speed_mps) * (60 / 90)
    assert abs(limit(zig) - v60) < 1e-6 and abs(limit(single) - v60) < 1e-6
    assert abs(limit(same) - p.corner_speed_mps) < 1e-6              # 60 + 60 = 120 deg >= 90: corner speed


def test_face_path_turns_toward_the_next_leg_while_parked_and_turn_first_gates_the_release():
    p = FollowerParams()
    log = fly(Follower(p), L_CORNER)
    idx = [i for i, (*_, sp) in enumerate(log) if sp.holding]
    yaws = np.array([log[i][3].yaw for i in idx])
    assert np.all(np.diff(yaws) >= -1e-12) and yaws[-1] > math.radians(45)   # turning while stopped
    assert np.all(np.diff(yaws) <= math.radians(p.yaw_rate_dps) * DT + 1e-9)
    moving = [i for i in range(idx[-1] + 1, len(log)) if log[i][3].speed_mps > 0]
    assert moving
    yaw_at_release = log[moving[0]][2]                              # the vehicle yaw passed to step
    assert abs(wrap(yaw_at_release - math.pi / 2)) <= math.radians(p.yaw_slow_deg) + 1e-9
    assert all(log[i][3].carrot_m == 3.0 for i in range(idx[-1] + 1, moving[0]))  # waited, did not creep


def test_vertical_first_segment_keeps_the_vehicle_yaw_and_is_not_a_stop_corner():
    p = FollowerParams()
    pts = [(0, 0, 0.5), (0, 0, 1.0), (2, 0, 1.0)]
    f = Follower(p)
    f.set_path(pts, 0.0, pts[0], 0.7)
    assert f.H[0] == 0.7 and abs(f.H[1]) < 1e-12 and abs(f.turn[0] + 0.7) < 1e-12
    assert len(f.stops) == 0 and abs(f.bend[0] - math.pi / 2) < 1e-9   # climb slows, does not stop
    log = fly(Follower(p), pts, yaw0=0.7)
    assert log[-1][3].done and abs(log[-1][2]) < math.radians(10)


def test_new_params_load_via_config_fill():
    from nav.config import fill
    q = fill(FollowerParams, {"stop_every_vertex": 1, "corner_arrive_timeout_s": 2})
    assert q.stop_every_vertex is True and q.corner_arrive_timeout_s == 2.0
    assert isinstance(q.corner_arrive_timeout_s, float) and not FollowerParams().stop_every_vertex
