"""A* over the course map: launch mark -> CP1 -> Gate A -> tunnel -> window -> pad.

Goals come from the map geometry, not hand-placed waypoints: each checkpoint line (and the tube's
open ends) gives a before/after point on its normal, oriented by the direction the course arrives
from. Between goals, A* runs on a 5 cm grid of the obstacle clearance at the flight heights; cells
closer than `inflate_m` to an obstacle are blocked and cells closer than `prefer_m` cost more, so
the path stays centred in corridors. The cell path is simplified (Douglas-Peucker) to waypoints.

Per-course options come from the map's `planner:` section (v3 has none, so it plans as before):
- shortcut: drop cell-path points where the straight line between their neighbours stays at least
  as far from obstacles as the path it replaces (or prefer_m): no wiggles in open areas.
- gate_a_free: cross the Gate A line where the path is shortest (gate_end_m from its ends), not at
  its centre (v4: M3 is gone, nothing pins the crossing).
Any line's before/after point closer than inflate_m to an obstacle is pulled back toward the line
until it is clear (v4: the d3 wall is ~0.5 m past CP1, so cp1_after sits almost on the line).

Waypoint kinds: "stop" = stop, look if needed and turn before going on (sharp corners, CP1, pad);
"window" = the window approach (slow; stops to localize only if not localized yet); "pass" = fly through.
"""
import heapq
import math
from dataclasses import dataclass, fields, replace

import numpy as np


@dataclass
class PlanParams:
    res_m: float = 0.05
    inflate_m: float = 0.33           # x500 tip radius ~0.32 m; Sky Beta is smaller
    prefer_m: float = 0.55            # clearance below this costs extra (corridors are ~1.07 m wide)
    prefer_gain: float = 3.0
    band_z: tuple = (0.55, 1.10)      # heights the airframe sweeps at height_m (map z, floor = 0): feet ~0.6 m
                                      # clear the 0.53 m boulders; the window opening is 0.30-1.22 m
    gate_m: float = 0.40              # before/after distance for checkpoint lines and the tube
    window_m: float = 0.60            # longer straight approach through the window
    simplify_m: float = 0.04
    corner_stop_deg: float = 60.0
    height_m: float = 0.6             # flight height above the takeoff point, every segment
    return_height_m: float = 2.3      # return leg: over the 1.73 m walls, under the 3.05 m net
    shortcut: bool = False
    gate_a_free: bool = False
    gate_end_m: float = 0.30          # gate_a_free: keep the crossing this far from the line's ends


@dataclass
class Waypoint:
    x: float
    y: float
    z: float                          # height above the takeoff point
    kind: str = "pass"
    label: str = ""


class Grid:
    def __init__(self, course, p: PlanParams):
        (x0, x1), (y0, y1) = course.net
        self.x0, self.y0, self.res = x0, y0, p.res_m
        self.nx, self.ny = int(round((x1 - x0) / p.res_m)), int(round((y1 - y0) / p.res_m))
        jj, ii = np.meshgrid(np.arange(self.nx), np.arange(self.ny))
        centers = np.stack([x0 + (jj + 0.5) * p.res_m, y0 + (ii + 0.5) * p.res_m], -1).reshape(-1, 2)
        self.clear = course.clearance(centers, *p.band_z).reshape(self.ny, self.nx)

    def cell(self, xy):
        return (min(max(int((xy[1] - self.y0) / self.res), 0), self.ny - 1),
                min(max(int((xy[0] - self.x0) / self.res), 0), self.nx - 1))

    def center(self, c):
        return np.array([self.x0 + (c[1] + 0.5) * self.res, self.y0 + (c[0] + 0.5) * self.res])


_MOVES = [(di, dj, math.hypot(di, dj)) for di in (-1, 0, 1) for dj in (-1, 0, 1) if di or dj]


def astar(grid, p: PlanParams, a, b):
    """Cell path from xy a to xy b (both cells are allowed even inside the inflation)."""
    start, goal = grid.cell(a), grid.cell(b)
    cl = grid.clear
    pen = 1.0 + p.prefer_gain * np.clip((p.prefer_m - cl) / p.prefer_m, 0.0, None)
    blocked = cl < p.inflate_m
    g = {start: 0.0}
    came = {}
    h = lambda c: math.hypot(c[0] - goal[0], c[1] - goal[1])
    heap = [(h(start), start)]
    while heap:
        _, c = heapq.heappop(heap)
        if c == goal:
            path = [c]
            while c in came:
                c = came[c]
                path.append(c)
            return path[::-1]
        for di, dj, step in _MOVES:
            n = (c[0] + di, c[1] + dj)
            if not (0 <= n[0] < grid.ny and 0 <= n[1] < grid.nx) or (blocked[n] and n != goal):
                continue
            ng = g[c] + step * pen[n]
            if ng < g.get(n, math.inf):
                g[n], came[n] = ng, c
                heapq.heappush(heap, (ng + h(n), n))
    raise RuntimeError(f"no path from {a} to {b}")


def simplify(P, tol):
    """Douglas-Peucker on an (N,2) polyline; keeps both ends."""
    if len(P) < 3:
        return P
    a, b = P[0], P[-1]
    d = b - a
    L = np.linalg.norm(d)
    dist = (np.abs(d[0] * (P[:, 1] - a[1]) - d[1] * (P[:, 0] - a[0])) / L if L > 1e-9
            else np.linalg.norm(P - a, axis=1))
    k = int(np.argmax(dist))
    if dist[k] <= tol:
        return np.array([a, b])
    return np.vstack([simplify(P[:k + 1], tol)[:-1], simplify(P[k:], tol)])


def _through(line_a, line_b, prev, d):
    """Before/after points on the normal of segment a-b through its centre, ordered from prev."""
    c = (line_a + line_b) / 2
    t = line_b - line_a
    n = np.array([t[1], -t[0]]) / np.linalg.norm(t)
    if np.dot(c - prev, n) < 0:
        n = -n
    return c - d * n, c + d * n


def _cross(line_a, line_b, prev, nxt, d, end_m):
    """Before/after points through the point of segment a-b (end_m from its ends) that makes
    prev -> point -> nxt shortest, ordered from prev."""
    t = line_b - line_a
    L = float(np.linalg.norm(t))
    ss = np.linspace(end_m, L - end_m, 200)
    pts = line_a + np.outer(ss / L, t)
    c = pts[np.argmin(np.linalg.norm(pts - prev, axis=1) + np.linalg.norm(pts - nxt, axis=1))]
    n = np.array([t[1], -t[0]]) / L
    if np.dot(c - prev, n) < 0:
        n = -n
    return c - d * n, c + d * n


def _shortcut(course, p, P):
    """String-pull an (N,2) polyline: from each kept point jump to the farthest later point whose
    straight segment keeps clearance >= min(prefer_m, the clearance of the path it replaces)."""
    clear = lambda Q: course.clearance(Q, *p.band_z)  # noqa: E731
    cP = clear(P)
    out, i = [P[0]], 0
    while i < len(P) - 1:
        j = len(P) - 1
        while j > i + 1:
            seg = P[i] + np.linspace(0, 1, max(2, int(np.linalg.norm(P[j] - P[i]) / 0.02)))[:, None] * (P[j] - P[i])
            if clear(seg).min() >= min(p.prefer_m, cP[i:j + 1].min()) - 1e-9:
                break
            j -= 1
        out.append(P[j])
        i = j
    return np.array(out)


def _fit(course, p, before, after):
    """Pull before/after (on one normal through a line) toward their midpoint until each has
    clearance >= inflate_m; error if even the line itself is blocked."""
    mid = (before + after) / 2
    res = []
    for q in (before, after):
        for k in np.linspace(1.0, 0.0, 41):
            pt = mid + k * (q - mid)
            if course.clearance([pt], *p.band_z)[0] >= p.inflate_m + 0.01:
                res.append(pt)
                break
        else:
            raise RuntimeError(f"no clear point between {mid} and {q}")
    return res


def goals(course, p: PlanParams):
    """[(xy, kind, label)] from the launch mark to the pad."""
    out = [(course.launch, "pass", "launch")]
    cp1 = course.lines["first_corridor"]
    b, a = _fit(course, p, *_through(cp1.a, cp1.b, out[-1][0], p.gate_m))
    out += [(b, "pass", "cp1_before"), (a, "stop", "cp1_after")]
    ga = course.lines["maze_gate_a"]
    x0, x1, y0, y1 = course.tube
    north, south = np.array([(x0 + x1) / 2, y1 + p.gate_m]), np.array([(x0 + x1) / 2, y0 - p.gate_m])
    if np.linalg.norm(ga.center - north) > np.linalg.norm(ga.center - south):
        north, south = south, north
    if p.gate_a_free:
        b, a = _fit(course, p, *_cross(ga.a, ga.b, out[-1][0], north, p.gate_m, p.gate_end_m))
    else:
        b, a = _fit(course, p, *_through(ga.a, ga.b, out[-1][0], p.gate_m))
    out += [(b, "pass", "gate_a_before"), (a, "pass", "gate_a_after")]
    out += [(north, "pass", "tube_before"), (south, "pass", "tube_after")]   # re-align on the move after the exit
    w = course.window
    b, a = _through(w.a, w.b, out[-1][0], p.window_m)
    out += [(b, "window", "window_before"), (a, "pass", "window_after")]
    out.append((course.pad_center, "stop", "pad"))
    return out


def course_params(course, p: PlanParams):
    """p with the map's `planner:` options applied."""
    opts = course.doc.get("planner") or {}
    known = {f.name for f in fields(PlanParams)}
    bad = set(opts) - known
    if bad:
        raise ValueError(f"unknown planner options in the map: {sorted(bad)}")
    return replace(p, **opts)


def plan(course, p: PlanParams = PlanParams()):
    p = course_params(course, p)
    grid = Grid(course, p)
    G = goals(course, p)
    wps = [Waypoint(*G[0][0], p.height_m, G[0][1], G[0][2])]
    for (a, _, _), (b, kind, label) in zip(G, G[1:]):
        cells = astar(grid, p, a, b)
        P = np.array([grid.center(c) for c in cells])
        P[0], P[-1] = a, b
        if p.shortcut:
            P = _shortcut(course, p, P)
        for q in simplify(P, p.simplify_m)[1:-1]:
            wps.append(Waypoint(q[0], q[1], p.height_m))
        wps.append(Waypoint(b[0], b[1], p.height_m, kind, label))
    for i in range(1, len(wps) - 1):                     # sharp corners are stops too
        u = np.array([wps[i].x - wps[i - 1].x, wps[i].y - wps[i - 1].y])
        v = np.array([wps[i + 1].x - wps[i].x, wps[i + 1].y - wps[i].y])
        turn = math.degrees(abs(math.atan2(u[0] * v[1] - u[1] * v[0], u @ v)))
        if turn > p.corner_stop_deg and wps[i].kind == "pass":
            wps[i].kind = "stop"
    return wps


def plan_return(course, p: PlanParams = PlanParams()):
    """Return leg (rule 7.10): from the pad, climb to return_height_m, fly straight over the barrier to
    the launch mark, land there. Above the walls nothing is in the way, so no A*."""
    pad, launch = course.pad_center, course.launch
    return [Waypoint(pad[0], pad[1], p.return_height_m, "stop", "return_climb"),
            Waypoint(launch[0], launch[1], p.return_height_m, "stop", "launch")]
