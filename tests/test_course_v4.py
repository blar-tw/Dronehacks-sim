"""v4 course (hand-edited map): walls, path, localization with hand-placed walls, whole mission."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nav import course as COURSE  # noqa: E402
from nav import planner  # noqa: E402
from nav.frames import SE2, wrap  # noqa: E402
from nav.localizer import Localizer  # noqa: E402
from nav.mission import NAV_AUTO_LAND, NAV_OFFBOARD, Mission, MissionParams, Vehicle  # noqa: E402
from nav.yaw import ViewScorer  # noqa: E402

V4 = COURSE.load("course_map_v4.yaml")
V3 = COURSE.load()
PP = planner.PlanParams()
BAND = PP.band_z


def _cross(p, q, a, b):
    s = lambda u, v, w: (v[0] - u[0]) * (w[1] - u[1]) - (v[1] - u[1]) * (w[0] - u[0])  # noqa: E731
    return (s(a, b, p) > 0) != (s(a, b, q) > 0) and (s(p, q, a) > 0) != (s(p, q, b) > 0)


def _dense(P):
    return np.vstack([a + np.linspace(0, 1, 30)[:, None] * (b - a) for a, b in zip(P, P[1:])])


def test_v4_walls():
    names = {b.name for b in V4.boxes}
    assert not names & {"m1_maze_divider", "m2_maze_divider", "m3_maze_divider"}
    assert {"d1_maze_diagonal", "d2_tunnel_diagonal", "barrier_1", "c2a_chicane_wall#0"} <= names
    unsure = {b.name for b in V4.boxes if b.uncertain > 0}
    assert unsure == {"d1_maze_diagonal", "d2_tunnel_diagonal", "d3_north_wall"}
    # the localizer's faces leave the hand-placed walls out: 3 removed, 3 hand-placed, 4 faces each
    assert len(V4.faces_at(0.8)[0]) == len(V3.faces_at(0.8)[0]) - 3 * 4
    # d1 ends where the plan drew them
    d1 = next(b for b in V4.boxes if b.name == "d1_maze_diagonal")
    ends = d1.corners().mean(0)
    assert np.allclose(ends, [-1.2985, 1.395], atol=1e-3)


def test_v4_path_order_clearance_and_no_wiggles():
    wps = planner.plan(V4)
    P = np.array([[w.x, w.y] for w in wps])
    order = []
    for k in range(len(P) - 1):
        for name, line in [*V4.lines.items(), ("window", V4.window)]:
            if name != "chicane_window" and _cross(P[k], P[k + 1], line.a, line.b):
                order.append(name)
    assert order == ["first_corridor", "maze_gate_a", "tunnel_south_end", "window"], order
    assert np.linalg.norm(P[-1] - V4.pad_center) < 0.05
    D = _dense(P)[5:]
    assert V4.clearance(D, *BAND).min() > 0.30                       # with the 0.15 m margins
    real = [b for b in V4.boxes if b.uncertain > 0]
    for b in real:
        assert b.distance(D).min() > 0.45, b.name                    # >= 0.30 + 0.15 from the drawn wall
    stops = [w.label for w in wps if w.kind != "pass"]
    assert stops == ["cp1_after", "window_before", "pad"], stops   # no stop after Gate A or at the tunnel exit
    assert len(wps) <= 16


def test_v3_plan_is_unchanged():
    wps = planner.plan(V3)
    assert len(wps) == 24
    assert [w.label for w in wps if w.kind != "pass"] == ["cp1_after", "window_before", "pad"]   # noimu-opt: no tunnel-exit stop
    assert np.allclose([wps[11].x, wps[11].y], [-0.10, 2.58], atol=0.01)      # gate_a_before at the line centre


def test_points_on_a_hand_placed_wall_are_dropped():
    from nav.localizer import LocParams
    d1 = next(b for b in V4.boxes if b.name == "d1_maze_diagonal")
    c, s = math.cos(d1.yaw), math.sin(d1.yaw)
    t = np.linspace(-d1.sx / 2, d1.sx / 2, 200)[:, None]
    face = np.array([d1.cx, d1.cy]) + t * [c, s] + 0.05 * np.array([s, -c])     # its south-east face
    xy = np.array([-2.2, 1.6])
    off = Localizer(V4, 0.8, LocParams(uncertain_pad_m=-10.0))                   # filter off
    off.start(xy, 0.6, xy, 0.6)
    off.update(face + [0.12, -0.08])                                             # the real wall is a bit off
    assert off.last.endswith("points near a wall") and not off.last.startswith("0 ")   # some match the barrier
    on = Localizer(V4, 0.8)
    on.start(xy, 0.6, xy, 0.6)
    on.update(face + [0.12, -0.08])
    assert on.last == "0 points", on.last                                        # all dropped before the fit


class _Loc:
    def __init__(self):
        self.T, self.accepted, self.last = SE2(), 0, ""

    def start(self, map_xy, map_yaw, odom_xy, odom_yaw):
        self.T = SE2.from_poses(map_xy, map_yaw, odom_xy, odom_yaw)

    def predict(self, odom_xy):
        pass

    def sigma_xy(self):
        return 0.03


def test_whole_v4_mission_with_a_simple_vehicle():
    m = Mission(V4, planner.plan(V4), _Loc(), ViewScorer(V4, sum(BAND) / 2), ret=planner.plan_return(V4, PP),
                p=MissionParams(launch_offset_m=0.2))
    pos, vel, yaw = np.zeros(3), np.zeros(3), 0.0
    v = Vehicle(pos=tuple(pos), yaw=yaw, pos_ok=True, armed=False, nav_state=4)
    m.on_health(0.0, True)
    t, dt, worst = 0.0, 0.05, math.inf
    while t < 400 and m.phase != "done":
        m.guard.on_frame(t)
        out = m.step(t, v)
        if "arm" in out.commands:
            v.armed, v.nav_state = True, NAV_OFFBOARD
        if "land" in out.commands:
            v.nav_state = NAV_AUTO_LAND
        if v.nav_state == NAV_AUTO_LAND:
            pos[2] -= 0.5 * dt
            if pos[2] <= 0:
                pos[2], v.armed, v.nav_state = 0.0, False, 4
        elif out.stream and out.setpoint is not None and v.armed:
            vel += np.clip(1.0 * (np.array(out.setpoint[:3]) - pos) - vel, -3 * dt, 3 * dt)
            pos += vel * dt
            yaw = wrap(yaw + max(-1.5 * dt, min(1.5 * dt, wrap(out.setpoint[3] - yaw))))
            if m.phase == "follow" and m.leg == "out":
                worst = min(worst, float(V4.clearance(m.loc.T.apply(pos[:2]), *BAND)[0]))
        v.pos, v.yaw = tuple(pos), yaw
        t += dt
    assert m.result == "complete with return", m.status()
    assert worst > 0.25, worst
