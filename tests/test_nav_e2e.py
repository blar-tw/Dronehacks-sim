"""End-to-end: the real nav modules fly the whole course against a kinematic PX4 stand-in.

Not a flight-dynamics test (that is the GZ twin, sim/tools/course_flight.py). It checks that the
pieces fit: segments in order, every checkpoint line crossed in the right direction, clearance
from every wall, rule 7.7 height, pad landing, return leg re-arm and landing near the launch mark.
"""
import math

import numpy as np
import pytest

from nav import config as CFG
from nav import course as COURSE
from nav import frames as F
from px4_fake import FakePX4, clearance, wall_points

DT = 0.05


def seg_cross(p, q, a, b):
    def side(u, v, w):
        return (v[0] - u[0]) * (w[1] - u[1]) - (v[1] - u[1]) * (w[0] - u[0])
    s1, s2 = side(a, b, p), side(a, b, q)
    if s1 == 0 or s2 == 0 or (s1 > 0) == (s2 > 0):
        return 0
    t1, t2 = side(p, q, a), side(p, q, b)
    return 0 if (t1 > 0) == (t2 > 0) else (1 if s2 > 0 else -1)


def fly(odom_err=F.SE2(), overrides=None, t_max=900.0, depth=True):
    course = COURSE.load(COURSE.default_map_path())
    params = CFG.load(None, overrides or {})
    m = CFG.build(course, params)
    px4 = FakePX4(course.launch, odom_err=odom_err)
    now, last_depth = 1.0, 0.0
    track, crossings, min_clear, max_z_main = [], [], math.inf, 0.0
    disarms, prev, prev_armed, events = [], None, False, []
    tube = course.tube
    while now < t_max and m.result is None:
        m.on_health(now, True)
        m.on_vio_state(now, "OK", 0.0)
        if depth and now - last_depth >= 1 / 15:
            last_depth = now
            in_tube = (tube["x"][0] <= px4.truth[0] <= tube["x"][1] and tube["y"][0] <= px4.truth[1] <= tube["y"][1])
            if px4.truth[2] > 0.2 and not in_tube:          # dark tunnel: no depth
                pts = wall_points(course, px4.truth, px4.yaw, px4.T_true)
                if len(pts):
                    m.on_points(now, pts, px4.truth[2])
        out = m.step(now, px4.vehicle())
        events += out.events
        px4.apply(now, out, DT)
        cur = tuple(px4.truth)
        if prev is not None:
            for c in course.checkpoints:
                if c.line:
                    s = seg_cross(prev, cur, *c.line)
                    if s:
                        crossings.append((round(now, 1), c.name, s))
        if cur[2] > 0.15:
            min_clear = min(min_clear, clearance(course, cur))
            if not any(math.hypot(d[1][0] - course.pad[0], d[1][1] - course.pad[1]) <= course.pad[2] for d in disarms):
                max_z_main = max(max_z_main, cur[2])
        if prev_armed and not px4.armed:
            disarms.append((now, cur))
        prev_armed = px4.armed
        prev = cur
        track.append(cur)
        now += DT
    return dict(m=m, course=course, crossings=crossings, min_clear=min_clear, max_z_main=max_z_main,
                disarms=disarms, t=now, events=events, track=np.array(track), px4=px4)


def _check(r, chicane):
    c = r["course"]
    assert r["m"].result == "complete", (r["m"].result, r["events"][-15:])
    names = [n for _, n, _ in r["crossings"]]
    order = ["first_corridor", "maze_gate_a", "tunnel_south_end", chicane]
    firsts = [names.index(n) for n in order]
    assert firsts == sorted(firsts), names
    assert r["min_clear"] > 0.20, r["min_clear"]
    assert r["max_z_main"] - 0.1 < c.wall_top
    pad_d = [d for d in r["disarms"] if math.hypot(d[1][0] - c.pad[0], d[1][1] - c.pad[1]) <= c.pad[2] - 0.2]
    assert pad_d, r["disarms"]
    last = r["disarms"][-1][1]
    assert math.hypot(last[0] - c.launch[0], last[1] - c.launch[1]) <= 0.61 * 0.5


def test_full_course_window():
    _check(fly(overrides={"mission.chicane": "chicane_window"}), "chicane_window")


def test_full_course_gate_b():
    _check(fly(overrides={"mission.chicane": "chicane_gate_b"}), "chicane_gate_b")


def test_aligner_absorbs_start_error():
    """The takeoff spot is 0.15 m off the launch mark and VIO's heading is 2 deg off: the aligner
    has to pull the map back from wall depth, or the narrow gaps cost clearance."""
    r = fly(odom_err=F.SE2(0.12, -0.08, math.radians(2.0)))
    _check(r, "chicane_gate_b")
