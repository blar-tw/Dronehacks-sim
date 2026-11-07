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
