"""nav.planner: A* + string pulling through gates on the inflated grid (nav/SPEC.md planner.py)."""
import math
import os
import time

import numpy as np
import pytest

from nav import course as COURSE
from nav.course import Box, Checkpoint, Course
from nav.planner import Planner

Z = 0.8


def synth(boxes=(), checkpoints=()):
    return Course(wall_top=1.5, net_x=(-2.5, 2.5), net_y=(-2.5, 2.5), net_z=(0.0, 3.0),
                  launch=(0.0, -2.0), pad=(0.0, 2.0, 0.5), boxes=list(boxes),
                  checkpoints=list(checkpoints))


def wall(name, x0, y0, x1, y1, thick=0.1):
    return Box(name, (x0 + x1) / 2, (y0 + y1) / 2, math.hypot(x1 - x0, y1 - y0), thick, 0.0, 1.5,
               math.atan2(y1 - y0, x1 - x0))


def side(a, b, p):
    return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])


def crosses(p, q, a, b):
    return side(a, b, p) * side(a, b, q) < 0 and side(p, q, a) * side(p, q, b) < 0


def point_rect_dist(px, py, box):
    c, s = math.cos(box.yaw), math.sin(box.yaw)
    u = (px - box.cx) * c + (py - box.cy) * s
    v = -(px - box.cx) * s + (py - box.cy) * c
    return np.hypot(np.maximum(np.abs(u) - box.length / 2, 0), np.maximum(np.abs(v) - box.thick / 2, 0))


def min_clearance(path, boxes):
    """Smallest distance from any polyline edge (sampled every 5 mm) to any box footprint."""
    best = (math.inf, None)
    for p, q in zip(path.pts[:-1], path.pts[1:]):
        n = max(2, int(math.dist(p[:2], q[:2]) / 0.005) + 1)
        t = np.linspace(0, 1, n)
        x, y = p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])
        for b in boxes:
            d = float(point_rect_dist(x, y, b).min())
            if d < best[0]:
                best = (d, b.name)
    return best


def check_gate(path, start, cp):
    """The gate line is crossed exactly once, on the cross_index edge, away from the start's side."""
    a, b = cp.line
    edges = list(zip(path.pts[:-1], path.pts[1:]))
    hits = [k for k, (p, q) in enumerate(edges) if crosses(p, q, a, b)]
    assert hits == [path.cross_index], (cp.name, hits, path.cross_index)
    p, q = edges[path.cross_index]
    assert side(a, b, start) * side(a, b, p) > 0 and side(a, b, q) * side(a, b, start) < 0


# ------------------------------------------------------------------ synthetic courses
def test_synthetic_wall_detour_and_gate_from_either_side():
    # wall along y=0 with a 1 m opening at x in [-0.5, 0.5]; the gate line spans the opening
    c = synth([wall("w_west", -2.5, 0, -0.5, 0), wall("w_east", 0.5, 0, 2.5, 0)],
              [Checkpoint(1, "gap", 1.0, line=((-0.5, 0.0), (0.5, 0.0)))])
    pl = Planner(c)
    straight = pl.to_point((-1.5, -1.5), (1.5, -1.0), Z)
    assert straight.ok and len(straight.pts) == 2 and straight.inflate_m == pytest.approx(0.33)

    around = pl.to_point((-1.5, -1.5), (1.5, 1.5), Z)
    assert around.ok and len(around.pts) >= 3 and around.gate is None
    assert min_clearance(around, c.boxes)[0] >= 0.25

    for start in ((-1.5, -1.5), (1.5, 1.5)):
        p = pl.through_gate(start, "gap", Z)
        assert p.ok and p.gate == "gap" and p.pts[0][:2] == start
        check_gate(p, start, c.checkpoint("gap"))
        e, x = p.pts[p.cross_index], p.pts[p.cross_index + 1]
        assert abs(e[0]) < 0.05 and abs(x[0]) < 0.05 and abs(e[1]) == pytest.approx(0.35, abs=0.03)
        assert min_clearance(p, c.boxes)[0] >= 0.25


def test_synthetic_fallback_window_and_failure_messages():
    # 0.60 m opening: closed at full inflation (0.33/side), open once inflation drops below ~0.27
    c = synth([wall("w_west", -2.5, 0, -0.3, 0), wall("w_east", 0.3, 0, 2.5, 0)],
              [Checkpoint(1, "tight", 1.0, line=((-0.3, 0.0), (0.3, 0.0))),
               Checkpoint(2, "win", 1.0, line=((-2.5, 1.5), (2.5, 1.5)), zmin=0.3, zmax=1.2)])
    pl = Planner(c)
    p = pl.through_gate((0.0, -1.5), "tight", Z)
    assert p.ok and 0.15 <= p.inflate_m < 0.30 and "inflation reduced" in p.notes
    assert min_clearance(p, c.boxes)[0] >= p.inflate_m - 0.05

    # window: z must be inside [zmin + 0.1, zmax - 0.1]
    assert pl.through_gate((0.0, 1.0), "win", Z).ok
    for z in (0.35, 1.15):
        bad = pl.through_gate((0.0, 1.0), "win", z)
        assert not bad.ok and "'win'" in bad.why and "0.30..1.20" in bad.why

    # 0.20 m opening never opens (needs > 2 x 0.15): why names the gate and gives the widths
    c2 = synth([wall("w_west", -2.5, 0, -0.1, 0), wall("w_east", 0.1, 0, 2.5, 0)],
               [Checkpoint(1, "slot", 1.0, line=((-0.1, 0.0), (0.1, 0.0))),
                Checkpoint(2, "far", 1.0, line=((-2.5, 1.5), (2.5, 1.5)))])
    pl2 = Planner(c2)
    f = pl2.through_gate((0.0, -1.5), "slot", Z)
    assert not f.ok and "gate 'slot'" in f.why and "open width at this height 0.2" in f.why
    # gate itself wide open, but the approach is closed by the slot: why names gate and gap
    f2 = pl2.through_gate((0.0, -1.5), "far", Z)
    assert not f2.ok and "gate 'far'" in f2.why and "narrowest gap" in f2.why
    assert "w_west" in f2.why and "w_east" in f2.why


# ---------------------------------------------------------------------- real map
def real_route(planner, course, chicane):
    """launch -> gates in id order (chicane = which id-4 gate) -> pad centre, chained."""
    segs, start = [], course.launch
    for name in ("first_corridor", "maze_gate_a", "tunnel_south_end", chicane, None):
        t = time.perf_counter()
        p = planner.through_gate(start, name, Z) if name else planner.to_point(start, course.pad[:2], Z)
        segs.append((name or "pad", start, p, time.perf_counter() - t))
        start = p.pts[-1][:2] if p.ok else start
    return segs


@pytest.mark.parametrize("chicane", ["chicane_window", "chicane_gate_b"])
def test_real_map_full_route(chicane):
    course = COURSE.load(COURSE.default_map_path())
    pl = Planner(course)
    segs = real_route(pl, course, chicane)
    boxes = course.boxes_at(Z - pl.p.z_band_m, Z + pl.p.z_band_m)
    for name, start, p, dt in segs:
        assert p.ok, (name, p.why)
        assert dt < 1.0, (name, dt)
        assert p.pts[0][:2] == tuple(start)
        assert all(z == Z for _, _, z in p.pts)
        d, wall_name = min_clearance(p, boxes)
        assert d >= 0.2, (name, d, wall_name)
        if name != "pad":
            check_gate(p, start, course.checkpoint(name))
    assert math.dist(segs[-1][2].pts[-1][:2], course.pad[:2]) < 1e-9
    if os.environ.get("NAV_PLANNER_PLOT"):
        plot_route(course, pl, segs, os.environ["NAV_PLANNER_PLOT"].replace(".png", f"_{chicane}.png"))


def test_real_map_return_leg_over_the_walls():
    course = COURSE.load(COURSE.default_map_path())
    p = Planner(course).to_point(course.pad[:2], course.launch, course.wall_top + 0.5)
    assert p.ok and len(p.pts) == 2          # only the net matters above the walls
    assert math.dist(p.pts[-1][:2], course.launch) < 1e-9


def plot_route(course, planner, segs, out):     # debug only: NAV_PLANNER_PLOT=/path/route.png
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    g = planner.grid(Z, planner.inflate_m())
    fig, ax = plt.subplots(figsize=(8, 8))
    ext = (g.x0, g.x0 + g.occ.shape[1] * g.res, g.y0, g.y0 + g.occ.shape[0] * g.res)
    ax.imshow(g.occ, origin="lower", extent=ext, cmap="Greys", alpha=0.35)
    for b in course.boxes_at(Z - 0.25, Z + 0.25):
        k = np.vstack([b.corners(), b.corners()[:1]])
        ax.fill(k[:, 0], k[:, 1], color="tab:brown")
    for cp in course.checkpoints:
        if cp.line:
            (x0, y0), (x1, y1) = cp.line
            ax.plot([x0, x1], [y0, y1], "b--", lw=1)
    for name, _, p, _ in segs:
        P = np.array(p.pts)
        ax.plot(P[:, 0], P[:, 1], "-o", ms=3, label=f"{name} {p.length_m:.2f} m")
    ax.set_aspect("equal")
    ax.legend(loc="lower right", fontsize=8)
    fig.savefig(out, dpi=110)
    plt.close(fig)


def test_fallback_never_inflates_below_the_propeller_radius():
    """min_inflate_m 0.15 is below the 0.20 m propeller radius: the fallback stops at the radius plus
    half a cell diagonal, so a 0.45 m opening (open at 0.15-0.22 m) stays closed."""
    c = synth([wall("w_west", -2.5, 0, -0.225, 0), wall("w_east", 0.225, 0, 2.5, 0)],
              [Checkpoint(1, "slit", 1.0, line=((-0.225, 0.0), (0.225, 0.0))),
               Checkpoint(2, "far", 1.0, line=((-2.5, 1.5), (2.5, 1.5)))])
    pl = Planner(c)
    p = pl.through_gate((0.0, -1.5), "slit", Z)
    assert not p.ok and p.inflate_m >= pl.p.drone_span_m / 2
