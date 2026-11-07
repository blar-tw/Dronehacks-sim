"""nav.planner: A* + string pulling through gates on the inflated grid (nav/SPEC.md planner.py)."""
import math
import os
import time

import numpy as np
import pytest

from nav import course as COURSE
from nav.course import Box, Checkpoint, Course, Grid
from nav.planner import Planner, PlannerParams

Z = 0.8
YAW_CHICANE = 1.2217            # the real course's chicane walls (course_map.yaml)


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


def min_clearance(path, boxes, cylinders=()):
    """Smallest distance from any polyline edge (sampled every 5 mm) to any box footprint / cylinder."""
    best = (math.inf, None)
    for p, q in zip(path.pts[:-1], path.pts[1:]):
        n = max(2, int(math.dist(p[:2], q[:2]) / 0.005) + 1)
        t = np.linspace(0, 1, n)
        x, y = p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])
        for b in boxes:
            d = float(point_rect_dist(x, y, b).min())
            if d < best[0]:
                best = (d, b.name)
        for cy in cylinders:
            d = float((np.hypot(x - cy.cx, y - cy.cy) - cy.radius).min())
            if d < best[0]:
                best = (d, cy.name)
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
    assert min_clearance(around, c.boxes)[0] >= around.inflate_m - 1e-6

    for start in ((-1.5, -1.5), (1.5, 1.5)):
        p = pl.through_gate(start, "gap", Z)
        assert p.ok and p.gate == "gap" and p.pts[0][:2] == start
        check_gate(p, start, c.checkpoint("gap"))
        e, x = p.pts[p.cross_index], p.pts[p.cross_index + 1]
        assert abs(e[0]) < 0.05 and abs(x[0]) < 0.05 and abs(e[1]) == pytest.approx(0.35, abs=0.03)
        assert min_clearance(p, c.boxes)[0] >= p.inflate_m - 1e-6


def test_synthetic_fallback_window_and_failure_messages():
    # 0.66 m opening: closed at full inflation (0.33 + half a cell per side), open once inflation
    # drops below 0.28 (cell centres at +-0.025 must clear 0.33 - inflation - 0.025)
    c = synth([wall("w_west", -2.5, 0, -0.33, 0), wall("w_east", 0.33, 0, 2.5, 0)],
              [Checkpoint(1, "tight", 1.0, line=((-0.33, 0.0), (0.33, 0.0))),
               Checkpoint(2, "win", 1.0, line=((-2.5, 1.5), (2.5, 1.5)), zmin=0.3, zmax=1.2)])
    pl = Planner(c)
    p = pl.through_gate((0.0, -1.5), "tight", Z)
    assert p.ok and 0.25 <= p.inflate_m < 0.30 and "inflation reduced" in p.notes
    assert min_clearance(p, c.boxes)[0] >= p.inflate_m - 1e-6

    # window: z must be inside [zmin + 0.1, zmax - 0.1]
    assert pl.through_gate((0.0, 1.0), "win", Z).ok
    for z in (0.35, 1.15):
        bad = pl.through_gate((0.0, 1.0), "win", z)
        assert not bad.ok and "'win'" in bad.why and "0.30..1.20" in bad.why

    # 0.20 m opening never opens (needs > 2 x 0.25): why names the gate and gives the widths
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


def test_fallback_never_plans_below_half_the_drone_span():
    """Bug fixed: min_inflate_m 0.15 < 0.20 (propeller radius) planned the tips into the wall, and
    with the half-cell loss that was 0.115 m of real clearance. Floor = max(min_inflate_m, span/2)."""
    assert PlannerParams().min_inflate_m >= 0.40 / 2 + 0.05
    # 0.54 m slot: cell centres at +-0.025 open only once inflation + 0.025 < 0.245, i.e. at 0.21
    c = synth([wall("w_west", -2.5, 0, -0.27, 0), wall("w_east", 0.27, 0, 2.5, 0)],
              [Checkpoint(1, "slot", 1.0, line=((-0.27, 0.0), (0.27, 0.0)))])
    f = Planner(c).through_gate((0.0, -1.5), "slot", Z)
    assert not f.ok and f.inflate_m == pytest.approx(0.25) and "tried 0.33..0.25" in f.why
    # asking for less than the propeller radius is clamped to drone_span_m / 2
    pl = Planner(c, PlannerParams(min_inflate_m=0.05))
    assert min(pl._schedule()) >= 0.20 - 1e-9
    p = pl.through_gate((0.0, -1.5), "slot", Z)
    assert p.ok and 0.20 <= p.inflate_m <= 0.23 and "inflation reduced" in p.notes, p.why
    check_gate(p, (0.0, -1.5), c.checkpoint("slot"))
    assert min_clearance(p, c.boxes)[0] >= p.inflate_m - 1e-6


@pytest.mark.parametrize("yaw", [0.0, YAW_CHICANE])
def test_free_cells_keep_inflate_m_from_straight_and_yawed_walls(yaw):
    """Bug fixed: cells were marked by their centre only, so a path could hug a wall up to res/2
    (res/2 (|cos|+|sin|) for a yawed wall) closer than Path.inflate_m claims. Lines parallel to the
    wall at offsets around inflate_m are either kept straight or detoured, never closer than it."""
    u = np.array([math.cos(yaw), math.sin(yaw)])
    n = np.array([-u[1], u[0]])
    # the wall's far face sits at 0.09 from the origin: grown by 0.33 that is 0.42, just under the
    # cell centre at 0.425, which the old grid left free although its near edge is 0.31 from the face
    c = synth([Box("w", 0.04 * n[0], 0.04 * n[1], 1.0, 0.1, 0.0, 1.5, yaw)])
    pl = Planner(c)
    for off in (0.31, 0.315, 0.32, 0.325, 0.33, 0.36, 0.42):     # old grid: 0.315..0.325 flew straight at < 0.33
        start, goal = tuple(-1.8 * u + (0.09 + off) * n), tuple(1.8 * u + (0.09 + off) * n)
        p = pl.to_point(start, goal, Z)
        assert p.ok and p.inflate_m == pytest.approx(0.33), (yaw, off, p.why)
        d, who = min_clearance(p, c.boxes)
        assert d >= p.inflate_m - 1e-6, (yaw, off, d, who, p.pts)
        if off >= 0.42:
            assert len(p.pts) == 2, (yaw, off, p.pts)       # far enough: the straight line is kept


def test_seg_free_is_an_exact_cell_traversal():
    """Bug fixed: _seg_free sampled points every res/2, so an edge clipping an occupied cell's
    corner over a chord shorter than that was reported free. Now every cell the segment passes
    through is tested; a cell only touched at a corner is not (the half-cell growth covers it)."""
    occ = np.zeros((20, 20), dtype=bool)
    occ[10, 10] = True                                      # cell x, y in [0.50, 0.55)
    occ[11, 11] = True
    g = Grid(0.0, 0.0, 0.05, occ)
    pl = Planner(synth())
    # chord through the top-right corner of (10, 10): from (0.5354, 0.55) to (0.55, 0.5468), 15 mm long
    assert not pl._seg_free(g, (0.30, 0.601), (0.60, 0.536))
    assert not pl._seg_free(g, (0.60, 0.536), (0.30, 0.601))
    # the same line two cells higher runs through free cells only ((13, j), (12, 10), (12, 11))
    assert pl._seg_free(g, (0.30, 0.701), (0.60, 0.636))
    # x + y = 1.1 passes exactly through the shared corner (0.55, 0.55) of the two occupied cells:
    # it runs through the free cells (11, 10) and (10, 11) only, so it is free
    assert pl._seg_free(g, (0.51, 0.59), (0.59, 0.51))
    # axis-parallel edges and a zero-length edge
    assert pl._seg_free(g, (0.30, 0.70), (0.60, 0.70))
    assert not pl._seg_free(g, (0.52, 0.30), (0.52, 0.70))
    assert pl._seg_free(g, (0.30, 0.30), (0.30, 0.30)) and not pl._seg_free(g, (0.52, 0.52), (0.52, 0.52))
    # anything leaving the grid is blocked
    assert not pl._seg_free(g, (0.30, 0.30), (1.20, 0.30))
    # through_gate / to_point never return an edge that enters an occupied cell (exact re-check)
    c = synth([wall("w_west", -2.5, 0, -0.5, 0), wall("w_east", 0.5, 0, 2.5, 0)])
    p = Planner(c).to_point((-1.5, -1.5), (1.5, 1.5), Z)
    gr = Planner(c).grid(Z, p.inflate_m)
    assert all(Planner(c)._seg_free(gr, a[:2], b[:2]) for a, b in zip(p.pts[:-1], p.pts[1:]))


def test_tube_walls_use_the_interior_of_the_oval_in_the_band():
    """Bug fixed: the tube's bounding box is its OUTER shell; the walls were placed with their inner
    face on the outer oval (3 cm optimistic). Narrowest half-width in the band is at the band's end
    farthest from the oval's centre height."""
    c = synth()
    c.tube = {"x": [-1.0, 1.0], "y": [-1.0, 1.0], "z": [0.0, 2.0]}
    walls = {b.name: b for b in c.tube_boxes(0.55, 1.05)}
    assert sorted(walls) == ["tube_east", "tube_west"]
    outer = math.sqrt(1.0 - ((0.55 - 1.0) / 1.0) ** 2)         # at z = 0.55, the band's far end
    inner = outer - c.tube_wall_m
    w = walls["tube_west"]
    assert w.yaw == pytest.approx(math.pi / 2) and w.length == pytest.approx(2.0)
    assert w.thick == pytest.approx(c.tube_wall_m) and w.zmin == 0.0 and w.zmax == 2.0
    assert max(k[0] for k in w.corners()) == pytest.approx(-inner, abs=1e-6)   # inner face
    assert min(k[0] for k in walls["tube_east"].corners()) == pytest.approx(inner, abs=1e-6)
    assert c.tube_boxes(1.9, 2.4)[0].name == "tube_solid"       # band reaches the roof
    assert c.tube_boxes(2.1, 2.6) == [] and c.tube_boxes(-0.5, -0.1) == []
    # the real tube at cruise: 1.27 m of interior, a corridor the drone fits through at full inflation
    course = COURSE.load(COURSE.default_map_path())
    pl = Planner(course)
    tw = {b.name: b for b in course.tube_boxes(Z - pl.p.z_band_m, Z + pl.p.z_band_m)}
    interior = tw["tube_east"].cx - tw["tube_west"].cx - course.tube_wall_m
    assert 1.25 < interior < 1.30
    assert interior - 2 * (pl.inflate_m() + pl.p.res / 2) > 0.5


def test_window_in_a_yawed_wall_is_crossed_perpendicular_to_the_wall():
    """A window opening in a wall at the chicane yaw: the entry -> exit edge must be the wall's normal
    through the opening centre, 0.35 m either side, away from the side the start is on."""
    yaw = YAW_CHICANE
    u = np.array([math.cos(yaw), math.sin(yaw)])
    n = np.array([-u[1], u[0]])
    cen, w = np.array([-0.4, -0.3]), 0.9144
    parts = [Box("c#0", *(cen - (w / 2 + 0.3) * u), 0.6, 0.1, 0.0, 1.5, yaw),
             Box("c#1", *(cen + (w / 2 + 0.3) * u), 0.6, 0.1, 0.0, 1.5, yaw)]
    a, b = cen - w / 2 * u, cen + w / 2 * u
    c = synth(parts, [Checkpoint(1, "win", 1.0, line=(tuple(a), tuple(b)), zmin=0.3, zmax=1.2)])
    pl = Planner(c)
    for start in (tuple(cen + 1.5 * n + 0.3 * u), tuple(cen - 1.5 * n - 0.4 * u)):
        p = pl.through_gate(start, "win", Z)
        assert p.ok and p.inflate_m == pytest.approx(0.33), p.why
        check_gate(p, start, c.checkpoint("win"))
        e = np.array(p.pts[p.cross_index][:2])
        x = np.array(p.pts[p.cross_index + 1][:2])
        assert abs(np.dot(x - e, u)) < 1e-6                             # perpendicular to the wall
        assert np.linalg.norm(e - cen) == pytest.approx(0.35, abs=0.03)
        assert np.linalg.norm(x - cen) == pytest.approx(0.35, abs=0.03)
        assert np.dot(x - e, n) * np.dot(np.array(start) - cen, n) < 0  # away from the start's side
        assert min_clearance(p, c.boxes)[0] >= p.inflate_m - 1e-6
        assert "gate free width" in p.notes


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


@pytest.mark.parametrize("chicane", ["chicane_window", "chicane_gate_b"])
def test_real_map_every_edge_keeps_inflate_m_from_every_obstacle(chicane):
    """Path.inflate_m is a true lower bound: every polyline edge of every segment, gates in id order
    at z = 0.8, stays >= inflate_m from every box and cylinder in the height band, with no fallback."""
    course = COURSE.load(COURSE.default_map_path())
    pl = Planner(course)
    z0, z1 = Z - pl.p.z_band_m, Z + pl.p.z_band_m
    boxes, cyls = course.boxes_at(z0, z1), course.cylinders_at(z0, z1)
    assert {b.name for b in boxes} >= {"tube_west", "tube_east", "c2a_chicane_wall#0", "c2a_chicane_wall#1"}
    assert not any("cradle" in b.name for b in boxes)           # 0.28 m high: under the band
    assert not any(b.name.startswith("c2a_chicane_wall#2") or b.name.startswith("c2a_chicane_wall#3")
                   for b in boxes)                              # window sill / lintel: outside the band
    for name, start, p, _ in real_route(pl, course, chicane):
        assert p.ok, (name, p.why)
        assert p.inflate_m == pytest.approx(pl.inflate_m()), (name, p.notes)
        assert "snapped" not in p.notes, (name, p.notes)
        d, who = min_clearance(p, boxes, cyls)
        assert d >= p.inflate_m - 1e-6, (name, d, who, p.pts)
        for a, b in zip(p.pts[:-1], p.pts[1:]):
            assert course.in_net(*a[:2], margin=p.inflate_m - 1e-6) and course.in_net(*b[:2], margin=p.inflate_m - 1e-6)


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
