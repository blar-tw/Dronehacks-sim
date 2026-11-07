"""A* over the course map: launch mark -> CP1 -> Gate A -> tunnel -> window -> pad.

Goals come from the map geometry, not hand-placed waypoints: each checkpoint line (and the tube's
open ends) gives a before/after point on its normal, oriented by the direction the course arrives
from. Between goals, A* runs on a 5 cm grid of the obstacle clearance at the flight heights; cells
closer than `inflate_m` to an obstacle are blocked and cells closer than `prefer_m` cost more, so
the path stays centred in corridors. The cell path is simplified (Douglas-Peucker) to waypoints.

Waypoint kinds: "stop" = stop, look and turn before going on (sharp corners, tunnel entry, window
approach); "pass" = fly through.
"""
import heapq
import math
from dataclasses import dataclass

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


def goals(course, p: PlanParams):
    """[(xy, kind, label)] from the launch mark to the pad."""
    out = [(course.launch, "pass", "launch")]
    cp1 = course.lines["first_corridor"]
    b, a = _through(cp1.a, cp1.b, out[-1][0], p.gate_m)
    out += [(b, "pass", "cp1_before"), (a, "stop", "cp1_after")]
    ga = course.lines["maze_gate_a"]
    b, a = _through(ga.a, ga.b, out[-1][0], p.gate_m)
    out += [(b, "pass", "gate_a_before"), (a, "pass", "gate_a_after")]
    x0, x1, y0, y1 = course.tube
    north, south = np.array([(x0 + x1) / 2, y1 + p.gate_m]), np.array([(x0 + x1) / 2, y0 - p.gate_m])
    if np.linalg.norm(out[-1][0] - north) > np.linalg.norm(out[-1][0] - south):
        north, south = south, north
    out += [(north, "stop", "tube_before"), (south, "pass", "tube_after")]
    w = course.window
    b, a = _through(w.a, w.b, out[-1][0], p.window_m)
    out += [(b, "stop", "window_before"), (a, "pass", "window_after")]
    out.append((course.pad_center, "stop", "pad"))
    return out


def plan(course, p: PlanParams = PlanParams()):
    grid = Grid(course, p)
    G = goals(course, p)
    wps = [Waypoint(*G[0][0], p.height_m, G[0][1], G[0][2])]
    for (a, _, _), (b, kind, label) in zip(G, G[1:]):
        cells = astar(grid, p, a, b)
        P = np.array([grid.center(c) for c in cells])
        P[0], P[-1] = a, b
        for q in simplify(P, p.simplify_m)[1:-1]:
            wps.append(Waypoint(q[0], q[1], p.height_m))
        wps.append(Waypoint(b[0], b[1], p.height_m, kind, label))
    for i in range(1, len(wps) - 1):                     # sharp corners are stops too
        u = np.array([wps[i].x - wps[i - 1].x, wps[i].y - wps[i - 1].y])
        v = np.array([wps[i + 1].x - wps[i].x, wps[i + 1].y - wps[i].y])
        turn = math.degrees(abs(math.atan2(u[0] * v[1] - u[1] * v[0], u @ v)))
        if turn > p.corner_stop_deg:
            wps[i].kind = "stop"
    return wps
