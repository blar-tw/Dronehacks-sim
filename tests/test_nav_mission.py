"""nav.mission with fake planner / follower / aligner and the kinematic FakePX4 (tests/px4_fake.py).

The real modules flying the real course is tests/test_nav_e2e.py; here the mission's own logic is
pinned on small synthetic courses: arming flow + frames, gate crossing direction, the return leg,
tunnel mode, the failure ladder.
"""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from nav import frames as F
from nav.course import Checkpoint, Course
from nav.mission import Mission, MissionParams, crossing
from px4_fake import FakePX4

DT = 0.05
LEAD = 0.35


# ---------------------------------------------------------------------------- fakes
class FakePlanner:
    """Straight lines, no walls: start -> entry -> exit for a gate, start -> goal for a point."""

    def __init__(self, course, fail=None, paths=None):
        self.course, self.fail, self.paths, self.calls = course, dict(fail or {}), dict(paths or {}), []

    def through_gate(self, start_xy, name, z):
        self.calls.append(("gate", name, tuple(start_xy), z))
        if name in self.fail:
            return SimpleNamespace(ok=False, pts=[], why=self.fail[name], cross_index=None)
        if name in self.paths:
            pts, ci = self.paths[name]
            return SimpleNamespace(ok=True, pts=[tuple(start_xy) + (z,)] + [p + (z,) for p in pts], why="",
                                   cross_index=ci)
        a, b = (np.array(p) for p in self.course.checkpoint(name).line)
        d = (b - a) / np.linalg.norm(b - a)
        n = np.array([-d[1], d[0]])
        sgn = -1.0 if np.dot(np.array(start_xy) - a, n) > 0 else 1.0
        mid = (a + b) / 2
        pts = [tuple(start_xy), tuple(mid - sgn * LEAD * n), tuple(mid + sgn * LEAD * n)]
        return SimpleNamespace(ok=True, pts=[(x, y, z) for x, y in pts], why="", cross_index=1)

    def to_point(self, start_xy, goal_xy, z):
        self.calls.append(("point", tuple(goal_xy), tuple(start_xy), z))
        return SimpleNamespace(ok=True, pts=[tuple(start_xy) + (z,), tuple(goal_xy) + (z,)], why="", cross_index=None)


class FakeFollower:
    """Carrot along the polyline at speed * scale, never more than `lead` ahead of the vehicle."""

    def __init__(self, arrive=0.15, lead=0.6):
        self.arrive, self.lead, self.calls, self.scales, self.held = arrive, lead, [], [], False

    def set_path(self, pts, now, pos_xyz, yaw, speed_mps=None):
        self.pts = np.array(pts, float)
        self.cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(self.pts[:, :2], axis=0), axis=1))])
        self.L, self.s, self.t, self.speed, self.held = self.cum[-1], 0.0, now, speed_mps, False
        self.prog = 0.0
        self.calls.append(("set_path", now))

    def _point(self, s):
        i = min(max(int(np.searchsorted(self.cum, s, side="right")) - 1, 0), len(self.pts) - 2)
        f = (s - self.cum[i]) / max(self.cum[i + 1] - self.cum[i], 1e-9)
        p = self.pts[i] + f * (self.pts[i + 1] - self.pts[i])
        d = self.pts[i + 1] - self.pts[i]
        return p, math.atan2(d[1], d[0])

    def _progress(self, pos):
        ss = np.arange(max(0.0, self.prog - 0.3), min(self.L, self.prog + 1.0) + 1e-9, 0.02)
        d = [np.linalg.norm(self._point(s)[0][:2] - pos[:2]) for s in ss]
        self.prog = float(ss[int(np.argmin(d))])          # searched only near the last progress
        return self.prog

    def step(self, now, pos_xyz, yaw, speed_scale=1.0):
        dt, self.t = now - self.t, now
        self.scales.append((now, speed_scale))
        prog = self._progress(np.asarray(pos_xyz))
        if not self.held:
            self.s = min(self.L, self.s + self.speed * speed_scale * dt, prog + self.lead)
        p, yaw_sp = self._point(self.s)
        done = self.s >= self.L - 1e-9 and np.linalg.norm(np.asarray(pos_xyz) - self.pts[-1]) < self.arrive
        return SimpleNamespace(x=p[0], y=p[1], z=p[2], yaw=yaw_sp, done=bool(done), progress_m=prog, length_m=self.L)

    def hold(self):
        self.held = True
        self.calls.append(("hold",))

    def back_off(self, dist_m):
        self.s, self.held = max(0.0, self.s - dist_m), True
        self.calls.append(("back_off", dist_m))


class FakeAligner:
    def __init__(self):
        self.T, self.inits, self.updates = F.SE2(), [], []

    def init_at_launch(self, odom_xy, launch_xy, yaw_offset=0.0):
        self.inits.append((tuple(odom_xy), tuple(launch_xy), yaw_offset))
        c, s = math.cos(yaw_offset), math.sin(yaw_offset)
        self.T = F.SE2(launch_xy[0] - (c * odom_xy[0] - s * odom_xy[1]), launch_xy[1] - (s * odom_xy[0] + c * odom_xy[1]),
                       yaw_offset)

    def predict(self, dist_m, odom_xy=(0.0, 0.0)):
        pass

    def update(self, pts, z, now, sensor_odom_xy=None):
        self.updates.append(now)
        return SimpleNamespace(accepted=True, why="ok", n=len(pts), dx=0.0, dy=0.0, dyaw=0.0)

    def sigma_xy(self):
        return 0.1

    def to_map(self, x, y, z, yaw):
        return (*self.T.apply(x, y, z), self.T.apply_yaw(yaw))

    def to_odom(self, x, y, z, yaw):
        Ti = self.T.inverse()
        return (*Ti.apply(x, y, z), Ti.apply_yaw(yaw))


def course(checkpoints, launch=(0.0, 0.0), tube=None):
    return Course(wall_top=1.7272, net_x=(-6.0, 6.0), net_y=(-6.0, 6.0), net_z=(0.0, 3.0), launch=launch,
                  pad=(0.0, 0.0, 0.5), checkpoints=checkpoints, tube=tube)


def fly(c, params, planner=None, t_max=200.0, vio=lambda t, m, px4: "OK", health=lambda t: True,
        points=lambda t, m, px4: True, on_tick=None, T_odom=None):
    px4 = FakePX4(c.launch)
    if T_odom is not None:
        px4.T_true = T_odom                      # PX4's local frame: origin/heading unknown to the mission
    fol, al = FakeFollower(), FakeAligner()
    m = Mission(c, planner or FakePlanner(c), fol, al, params)
    log, now = [], 1.0
    while now < t_max and m.result is None:
        st = vio(now, m, px4)
        h = health(now)
        if h is not None:
            m.on_health(now, h and st != "WITHHELD")           # ev_bridge: healthy false <=> WITHHELD
        m.on_vio_state(now, st, 0.0)
        if points(now, m, px4):
            m.on_points(now, np.array([[1.0, 0.0], [1.0, 0.5]]), px4.truth[2])
        if on_tick:
            on_tick(now, m, px4)
        nav_before, armed_before = px4.nav, px4.armed
        out = m.step(now, px4.vehicle())
        px4.apply(now, out, DT)
        log.append(SimpleNamespace(t=now, out=out, truth=tuple(px4.truth), nav=nav_before, armed=armed_before,
                                   phase=m.phase, leg=m.leg, tunnel=m.tunnel_active))
        now += DT
    events = [e for r in log for e in r.out.events]
    return SimpleNamespace(m=m, px4=px4, fol=fol, al=al, log=log, events=events,
                           planner=m.planner)


def CP(i, name, line=None, circle=None):
    return Checkpoint(i, name, 1.0, line=line, circle=circle)


# ---------------------------------------------------------------------------- tests
def test_arming_flow_health_gate_frames_and_yield():
    """No control request before health has been good for health_quiet_s; setpoints stream
    stream_lead_s before Offboard/arm; the aligner is anchored at the launch mark from the TAKEOFF
    odom position (PX4's frame is shifted and rotated); NED setpoints are odom ENU swapped; depth is
    not needed to arm; PX4 leaving Offboard is never fought."""
    c = course([CP(1, "g1", line=((-1.5, 1.5), (1.5, 1.5))), CP(2, "pad", circle=(0.0, 3.0, 0.5))],
               launch=(-0.5, 0.2))
    T_odom = F.SE2(-1.7, 0.9, 0.3)              # map <- odom: PX4's origin elsewhere, heading off by 0.3 rad
    p = MissionParams(return_leg=False, speed_mps=1.0, depth_stale_hover_s=1.0)

    def leave_offboard(t, m, px4):
        if any(q["name"] == "g1" for q in m.passed) and px4.nav == 14:
            px4.nav = 2                          # a pilot takes over
    r = fly(c, p, health=lambda t: None if t < 3.0 else True, points=lambda t, m, px4: t > 9.5,
            on_tick=leave_offboard, T_odom=T_odom)

    first_stream = next(x.t for x in r.log if x.out.stream)
    first_cmd = next(x.t for x in r.log if x.out.commands)
    assert first_stream >= 3.0 + p.health_quiet_s - 1e-6
    assert first_cmd - first_stream >= p.stream_lead_s - 1e-6
    assert r.log[[x.t for x in r.log].index(first_cmd)].out.commands == ["offboard", "arm"]
    takeoff = next(x for x in r.log if x.phase == "climb")
    assert takeoff.t < 8.0                       # armed and climbing before any depth arrived

    (odom_xy, launch_xy, yaw_off), = r.al.inits
    true_odom = T_odom.inverse().apply(c.launch[0], c.launch[1])
    assert odom_xy == pytest.approx(true_odom[:2], abs=1e-6)
    assert launch_xy == c.launch and yaw_off == pytest.approx(0.3)

    # the first climb setpoint: NED (n, e, d) = (odom y, odom x, -z) of the takeoff spot
    n, e, d, hdg = next(x.out.setpoint for x in r.log if x.phase == "climb")
    assert (n, e) == pytest.approx((true_odom[1], true_odom[0]), abs=1e-6) and d <= 0.0
    assert hdg == pytest.approx(F.yaw_to_heading(math.pi / 2 - 0.3), abs=1e-6)

    assert any("hold: no depth points" in e for e in r.events) and any(e.startswith("resume") for e in r.events)
    assert [q["name"] for q in r.m.passed] == ["g1"]
    assert r.m.result.startswith("yielded: PX4 left Offboard"), r.m.result
    i = next(i for i, x in enumerate(r.log) if x.phase == "done")
    assert r.log[i].nav == 2 and not (r.log[i].out.stream or r.log[i].out.commands)


def test_gate_crossing_direction():
    a, b = (-1.0, 0.0), (1.0, 0.0)               # left normal of a->b points north (+y)
    assert crossing((0, -0.1), (0, 0.1), a, b) == 1
    assert crossing((0, 0.1), (0, -0.1), a, b) == -1
    assert crossing((2, -0.1), (2, 0.1), a, b) == 0          # beside the opening
    assert crossing((0, 0.1), (0, 0.3), a, b) == 0

    # the path crosses north, comes back south (wrong way: not counted), then north again
    c = course([CP(1, "g1", line=((-1.5, 1.5), (1.5, 1.5))), CP(2, "pad", circle=(0.0, 3.0, 0.5))])
    pl = FakePlanner(c, paths={"g1": ([(0.0, 2.2), (0.6, 1.0), (1.2, 2.0)], 2)})
    r = fly(c, MissionParams(return_leg=False, speed_mps=1.0), planner=pl)
    assert r.m.result == "complete", (r.m.result, r.events[-10:])
    crossed = [e for e in r.events if "g1 crossed" in e or "crossed at" in e]
    assert len(crossed) == 3 and "WRONG direction" in crossed[1], crossed
    assert [q["name"] for q in r.m.passed] == ["g1", "pad"]


@pytest.mark.parametrize("return_leg", [True, False])
def test_return_leg_rearms_and_completes(return_leg):
    """Gates in id order (the chicane id has two lines: `chicane` fails in the planner -> fallback),
    pad landing through PX4 AUTO_LAND (no streaming after it), disarm, re-arm by itself, climb over
    the walls, land at the launch mark -> 'complete'. Main-run z stays below wall_top - 0.3."""
    cps = [CP(3, "pad", circle=(3.0, 4.0, 0.5)),
           CP(2, "alpha", line=((1.5, 2.5), (1.5, 3.5))), CP(2, "beta", line=((1.5, 1.5), (1.5, 2.5))),
           CP(1, "g1", line=((-1.0, 1.5), (1.0, 1.5)))]
    c = course(cps)
    pl = FakePlanner(c, fail={"alpha": "gap 0.31 m closed at (1.50, 3.00)"})
    p = MissionParams(chicane="alpha", chicane_fallback="beta", return_leg=return_leg, speed_mps=1.0)
    r = fly(c, p, planner=pl)
    m = r.m

    assert m.result == "complete", (m.result, r.events[-10:])
    assert [s.name for s in m.segments] == ["g1", "beta", "pad"] + (["launch_mark"] if return_leg else [])
    assert [q["name"] for q in m.passed] == ["g1", "beta", "pad"] + (["launch_mark"] if return_leg else [])
    assert any("trying beta" in e and "gap 0.31" in e for e in r.events)
    assert len(r.al.inits) == 1                  # anchored once, at the first takeoff

    arms = [x.t for x in r.log if "arm" in x.out.commands]
    flows = 1 + sum(1 for t0, t1 in zip(arms, arms[1:]) if t1 - t0 > 5.0)
    assert flows == (2 if return_leg else 1)
    # PX4 owns the landing: nothing streams once AUTO_LAND has been seen
    assert not any(x.out.stream for x in r.log if x.nav == 18 and x.phase == "land")
    main_z = [x.truth[2] for x in r.log if x.leg == "main"]
    assert max(main_z) <= c.wall_top - 0.3 + 0.05
    if return_leg:
        ret_z = [x.truth[2] for x in r.log if x.leg == "return"]
        assert max(ret_z) > c.wall_top
        assert math.hypot(r.px4.truth[0] - c.launch[0], r.px4.truth[1] - c.launch[1]) < 0.3
    assert not r.px4.armed


def test_tunnel_mode_is_geometric_and_commits():
    """The gate on the tube bounding box (any name) is the tunnel: inside it the mission keeps
    flying at tunnel speed with no depth and VIO WITHHELD, and does not update the aligner. Outside
    it WITHHELD holds the carrot."""
    tube = {"x": [-0.5, 0.5], "y": [1.0, 3.0], "z": [0.0, 1.8]}
    c = course([CP(1, "zz", line=((-0.5, 3.0), (0.5, 3.0))), CP(2, "tunnel_like_name", line=((-1, 4.5), (1, 4.5))),
                CP(3, "pad", circle=(0.0, 5.5, 0.5))], tube=tube)
    p = MissionParams(return_leg=False, speed_mps=1.0, tunnel_speed_mps=0.3, depth_stale_hover_s=1.0)

    def in_tube(px4):
        return -0.5 <= px4.truth[0] <= 0.5 and 1.0 <= px4.truth[1] <= 3.0

    first = {}

    def vio(t, m, px4):
        for key, cond in (("out", m.phase == "fly" and px4.truth[1] > 0.2), ("in", px4.truth[1] > 1.5)):
            if cond and key not in first:
                first[key] = t
        # withheld 1.5 s before the tube and 2 s inside it (< withheld_land_s)
        return "WITHHELD" if any(0 <= t - first[k] < d for k, d in (("out", 1.5), ("in", 2.0)) if k in first) else "OK"
    r = fly(c, p, vio=vio, points=lambda t, m, px4: not in_tube(px4))

    assert [s.tunnel for s in r.m.segments] == [True, False, False]
    assert r.m.result == "complete", (r.m.result, r.events[-10:])
    assert any(e.startswith("tunnel enter") for e in r.events) and any(e.startswith("tunnel exit") for e in r.events)
    t_in = [x.t for x in r.log if x.tunnel]
    assert t_in[-1] - t_in[0] > 2 * p.depth_stale_hover_s    # long without depth, never held
    scales = dict(r.fol.scales)
    assert all(scales[t] == pytest.approx(0.3) for t in t_in if t in scales)
    assert not [t for t in r.al.updates if t_in[0] < t <= t_in[-1]]   # (points of the entry tick came first)
    held = [s for t, s in r.fol.scales if first["out"] + 0.1 <= t < first["out"] + 1.4]
    assert held and max(held) == 0.0                          # outside the tube WITHHELD holds
    assert first["in"] > t_in[0] and any("hold: VIO WITHHELD" in e for e in r.events)


def test_timeout_ladder_then_land_and_z_clamp():
    """A blocked segment: timeout -> hold hover_s -> back off back_off_m -> replan -> timeout -> land
    where it is ('aborted'). A cruise height above the walls is clamped (rule 7.7)."""
    c = course([CP(1, "g1", line=((-1.0, 1.5), (1.0, 1.5))), CP(2, "pad", circle=(0.0, 3.0, 0.5))])
    p = MissionParams(return_leg=False, speed_mps=1.0, segment_timeout_s=4.0, cruise_z_m=1.6)

    def wall(t, m, px4):
        px4.truth[1] = min(px4.truth[1], 1.0)    # something the fake planner did not know about
    r = fly(c, p, on_tick=wall)

    assert r.m.result.startswith("aborted: g1 timed out"), r.m.result
    phases = [x.phase for x in r.log]
    order = [phases.index(ph) for ph in ("fly", "hover", "back_off", "land")]
    assert order == sorted(order)
    assert ("back_off", p.back_off_m) in r.fol.calls
    assert [k for k, name, *_ in r.planner.calls] == ["gate", "gate"]
    cap = c.wall_top - p.wall_margin_m
    assert any("rule 7.7" in e for e in r.events)
    assert max(-x.out.setpoint[2] for x in r.log if x.out.setpoint) <= cap + 1e-9
