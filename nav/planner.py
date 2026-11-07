"""nav.planner - 2D A* through the course's gates on an inflated occupancy grid. PURE: numpy only.

One altitude per segment (ARCHITECTURE.md "Planner"): obstacles overlapping the band
[z - z_band_m, z + z_band_m] are grown by half the drone span + tilt + localization margins
(nav.course.occupancy, which also adds the half-cell the centre-marking can miss, so every point of
every FREE cell is at least inflate_m from every real obstacle), A* runs 8-connected on that grid,
and the cell path is string-pulled into a short polyline whose every edge is checked free on the
grid (exact cell traversal, not point sampling). A gate (checkpoint line) is crossed on one straight
entry -> exit edge through the centre of the free part of the line, away from the side the start is
on. No course coordinates here: all geometry comes from the Course object.
"""
from __future__ import annotations

import heapq
import math
from collections import OrderedDict
from dataclasses import dataclass

import numpy as np

from nav.course import Course, Grid, occupancy

_SQ2 = math.sqrt(2.0)


@dataclass
class PlannerParams:
    res: float = 0.05
    drone_span_m: float = 0.40     # tip-to-tip; half of it is inflated
    tilt_margin_m: float = 0.05
    loc_margin_m: float = 0.08     # localization error allowance
    z_band_m: float = 0.25         # obstacles within z +- this count at a segment altitude
    gate_lead_m: float = 0.35      # entry/exit points this far before/after a gate line
    min_inflate_m: float = 0.25    # if no path, retry with inflation shrinking down to this (= span/2 +
                                   # tilt: only the localization allowance is traded away); the schedule
                                   # never goes below drone_span_m / 2, the propeller tips
    inflate_step_m: float = 0.02   # (addition) fallback step
    window_z_margin_m: float = 0.10  # (addition) z must be inside the window opening by this
    snap_max_m: float = 0.30       # (addition) occupied start/goal: snap to a free cell this close


@dataclass
class Path:
    pts: list            # [(x, y, z)] map frame, first = start, polyline after shortcutting
    ok: bool
    why: str = ""        # on failure: which gate / which gap closed, with numbers
    gate: str | None = None          # checkpoint name this path crosses (None for a plain goal)
    cross_index: int | None = None   # pts[cross_index] -> pts[cross_index+1] crosses the gate line
    inflate_m: float = 0.0           # inflation actually used: every edge keeps >= this from every obstacle
    notes: str = ""      # (addition) on success: snaps, reduced inflation, gate free width

    @property
    def length_m(self) -> float:
        p = np.asarray(self.pts, dtype=float)
        return float(np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1).sum()) if len(p) > 1 else 0.0


class _Fail(Exception):
    def __init__(self, msg: str, **data):
        super().__init__(msg)
        self.msg, self.data = msg, data


class Planner:
    def __init__(self, course: Course, params: PlannerParams | None = None):
        self.course = course
        self.p = params if params is not None else PlannerParams()
        self._grids: OrderedDict = OrderedDict()

    # ------------------------------------------------------------------ public
    def inflate_m(self) -> float:
        p = self.p
        return p.drone_span_m / 2 + p.tilt_margin_m + p.loc_margin_m

    def grid(self, z: float, inflate: float) -> Grid:
        """Inflated occupancy at the band around z, cached per (band, inflate)."""
        z0, z1 = z - self.p.z_band_m, z + self.p.z_band_m
        key = (round(z0, 4), round(z1, 4), round(inflate, 4))
        if key not in self._grids:
            self._grids[key] = occupancy(self.course, z0, z1, inflate, self.p.res)
            if len(self._grids) > 64:
                self._grids.popitem(last=False)
        return self._grids[key]

    def through_gate(self, start_xy, gate_name: str, z: float) -> Path:
        cp = self.course.checkpoint(gate_name)
        m = self.p.window_z_margin_m
        lo = cp.zmin + m if cp.zmin > 0 else -math.inf
        hi = cp.zmax - m if math.isfinite(cp.zmax) else math.inf
        if not lo <= z <= hi:
            return Path([], False, f"gate '{gate_name}': z={z:.2f} m is outside the opening "
                        f"{cp.zmin:.2f}..{cp.zmax:.2f} m (needs {lo:.2f}..{hi:.2f} with {m:.2f} m margin)",
                        gate=gate_name)
        start = (float(start_xy[0]), float(start_xy[1]))
        a, b = np.asarray(cp.line[0], float), np.asarray(cp.line[1], float)
        d = b - a
        L = float(np.hypot(*d))
        left = np.array([-d[1], d[0]]) / L
        side = d[0] * (start[1] - a[1]) - d[1] * (start[0] - a[0])   # > 0: start is left of a->b
        n = -left if side > 0 else left                    # away from the start's side

        def attempt(g: Grid):
            centre, width = self._free_run(g, a, b)
            if centre is None:
                raise _Fail(f"the {L:.2f} m gate line has no free cell", kind="line")
            entry = tuple(centre - n * self.p.gate_lead_m)
            exit_ = tuple(centre + n * self.p.gate_lead_m)
            if not self._seg_free(g, entry, exit_):
                raise _Fail(f"gate free width {width:.2f} m, but the entry->exit edge "
                            f"({entry[0]:.2f}, {entry[1]:.2f})->({exit_[0]:.2f}, {exit_[1]:.2f}) is blocked",
                            kind="lead")
            poly, notes = self._leg(g, start, entry)
            notes.append(f"gate free width {width:.2f} m at ({centre[0]:.2f}, {centre[1]:.2f})")
            return poly + [exit_], len(poly) - 1, notes

        return self._with_fallback(attempt, z, f"gate '{gate_name}'", gate_name, (a, b))

    def to_point(self, start_xy, goal_xy, z: float) -> Path:
        start = (float(start_xy[0]), float(start_xy[1]))
        goal = (float(goal_xy[0]), float(goal_xy[1]))

        def attempt(g: Grid):
            poly, notes = self._leg(g, start, goal)
            return poly, None, notes

        return self._with_fallback(attempt, z, f"goal ({goal[0]:.2f}, {goal[1]:.2f})", None, None)

    # ---------------------------------------------------------------- internals
    def _schedule(self) -> list:
        p = self.p
        top, step = self.inflate_m(), p.inflate_step_m
        lo = max(p.min_inflate_m, p.drone_span_m / 2)   # never plan the propeller tips into a wall
        out = [top]
        while out[-1] - step >= lo - 1e-9:
            out.append(round(out[-1] - step, 6))
        return out

    def _with_fallback(self, attempt, z, what, gate, line) -> Path:
        tried = []
        for r in self._schedule():
            g = self.grid(z, r)
            try:
                pts, ci, notes = attempt(g)
            except _Fail as f:
                tried.append((r, f, g))
                continue
            if tried:
                # loud: the mission should log this and slow down (only the localization allowance is left)
                notes.insert(0, f"inflation reduced to {r:.2f} m from {tried[0][0]:.2f} m "
                                f"({tried[0][1].msg}): margin is thin, fly slowly")
            return Path([(float(x), float(y), float(z)) for x, y in pts], True, "", gate, ci, r,
                        "; ".join(notes))
        r, f, g = tried[-1]
        why = f"{what} at z={z:.2f}: {f.msg} at inflation {r:.2f} m"
        if f.data.get("kind") == "line":
            w0 = self._open_width(z, *line)
            why += (f" (open width at this height {w0:.2f} m, needs > {2 * (r + self.p.res / 2):.2f} m"
                    f" = 2 x (inflation + half a cell))")
        elif f.data.get("kind") == "astar":
            why += self._gap(z, r, g, f.data["s"], f.data["g"])
        why += f" (tried {self.inflate_m():.2f}..{r:.2f} m)"
        return Path([], False, why, gate, None, r)

    def _leg(self, g: Grid, start, goal):
        """A* + string pull from start to goal on grid g -> (polyline [(x, y)], notes)."""
        notes = []
        s, start_snapped = self._snap(g, start, "start", notes)
        t, goal_snapped = self._snap(g, goal, "goal", notes)
        cells = _astar(g.occ, s, t)
        if cells is None:
            raise _Fail(f"no path from ({start[0]:.2f}, {start[1]:.2f}) to ({goal[0]:.2f}, {goal[1]:.2f})",
                        kind="astar", s=s, g=t)
        end = g.center(*t) if goal_snapped else goal
        nodes = [start] + [g.center(i, j) for i, j in cells] + [end]
        poly = self._pull(g, nodes)
        for k in range(1 if start_snapped else 0, len(poly) - 1):
            assert self._seg_free(g, poly[k], poly[k + 1]), "string pulling left a blocked edge"
        return poly, notes

    def _snap(self, g: Grid, p, label, notes):
        i, j = g.cell(*p)
        if g.inside(i, j) and not g.occ[i, j]:
            return (i, j), False
        res, rmax = self.p.res, self.p.snap_max_m
        k = int(math.ceil(rmax / res)) + 1
        H, W = g.occ.shape
        i0, i1, j0, j1 = max(0, i - k), min(H, i + k + 1), max(0, j - k), min(W, j + k + 1)
        ys = g.y0 + (np.arange(i0, i1) + 0.5) * res
        xs = g.x0 + (np.arange(j0, j1) + 0.5) * res
        D = np.hypot(xs[None, :] - p[0], ys[:, None] - p[1])
        D[g.occ[i0:i1, j0:j1] | (D > rmax)] = np.inf
        if not np.isfinite(D.min()):
            raise _Fail(f"{label} ({p[0]:.2f}, {p[1]:.2f}) has no free cell within {rmax:.2f} m",
                        kind="snap")
        a, b = np.unravel_index(int(np.argmin(D)), D.shape)
        c = g.center(i0 + a, j0 + b)
        notes.append(f"{label} ({p[0]:.2f}, {p[1]:.2f}) is in an inflated obstacle: snapped "
                     f"{D[a, b]:.2f} m to ({c[0]:.2f}, {c[1]:.2f})")
        return (i0 + a, j0 + b), True

    def _seg_free(self, g: Grid, a, b) -> bool:
        """True if every cell the segment a->b passes through is free. Exact traversal: the segment is
        cut at every grid line it crosses and the cell of each piece's midpoint is tested, so a corner
        clip shorter than a sample step cannot slip through (point sampling could miss it). A cell the
        segment only touches at a corner (zero-length piece) does not count: with the half-cell growth
        in occupancy its corner is still >= inflate from the obstacle."""
        H, W = g.occ.shape
        r = g.res
        ax, ay = (a[0] - g.x0) / r, (a[1] - g.y0) / r          # cell units
        bx, by = (b[0] - g.x0) / r, (b[1] - g.y0) / r
        if min(ax, bx) < 0 or min(ay, by) < 0 or max(ax, bx) >= W or max(ay, by) >= H:
            return False
        dx, dy = bx - ax, by - ay
        parts = [np.array([0.0, 1.0])]
        if dx != 0.0:                                          # vertical grid lines crossed
            k = np.arange(math.ceil(min(ax, bx)), math.floor(max(ax, bx)) + 1)
            parts.append((k - ax) / dx)
        if dy != 0.0:                                          # horizontal grid lines crossed
            k = np.arange(math.ceil(min(ay, by)), math.floor(max(ay, by)) + 1)
            parts.append((k - ay) / dy)
        t = np.sort(np.concatenate(parts))
        mid = ((t[:-1] + t[1:]) / 2)[np.diff(t) > 1e-9]       # drop zero-length pieces (corner touches)
        j = (ax + mid * dx).astype(int)                        # floor: everything is >= 0 here
        i = (ay + mid * dy).astype(int)
        return not g.occ[i, j].any()

    def _pull(self, g: Grid, nodes):
        """Greedy line-of-sight shortcut, then drop any vertex its neighbours can see past."""
        out, k = [nodes[0]], 0
        while k < len(nodes) - 1:
            j = k + 1
            while j + 1 < len(nodes) and self._seg_free(g, nodes[k], nodes[j + 1]):
                j += 1
            out.append(nodes[j])
            k = j
        changed = True
        while changed:
            changed = False
            for m in range(1, len(out) - 1):
                if self._seg_free(g, out[m - 1], out[m + 1]):
                    del out[m]
                    changed = True
                    break
        kept = [out[0]]
        for p in out[1:]:                                      # no zero-length edges (goal == last cell centre)
            if math.dist(p, kept[-1]) > 1e-9:
                kept.append(p)
        if len(kept) == 1:                                     # start == goal: keep one edge for the follower
            kept.append(out[-1])
        return kept

    def _line_samples(self, a, b):
        L = float(np.hypot(*(b - a)))
        t = np.linspace(0.0, 1.0, max(2, int(math.ceil(L / (self.p.res / 2))) + 1))
        return a[None, :] + t[:, None] * (b - a)[None, :], L / (len(t) - 1)

    def _free_run(self, g: Grid, a, b):
        """Centre of the longest free run of samples along the gate line, and its width (span
        between the blocked samples on either side, so within one sample step)."""
        pts, step = self._line_samples(a, b)
        H, W = g.occ.shape
        j = np.floor((pts[:, 0] - g.x0) / g.res).astype(int)   # vectorised cell lookup (was per-sample)
        i = np.floor((pts[:, 1] - g.y0) / g.res).astype(int)
        ok = (i >= 0) & (j >= 0) & (i < H) & (j < W)
        free = np.zeros(len(pts), dtype=bool)
        free[ok] = ~g.occ[i[ok], j[ok]]
        best, cur = (0, -1), None
        for k, f in enumerate(free.tolist()):
            if f and cur is None:
                cur = k
            if (not f or k == len(free) - 1) and cur is not None:
                end = k if f else k - 1
                if end - cur > best[1] - best[0]:
                    best = (cur, end)
                cur = None
        if best[1] < best[0]:
            return None, 0.0
        return pts[(best[0] + best[1]) // 2], min((best[1] - best[0] + 2) * step, (len(pts) - 1) * step)

    # ------------------------------------------------------ failure diagnostics
    def _dists(self, z, X, Y):
        """Per-obstacle distance (in the inflation metric: grown rectangles / circles / net).
        X, Y broadcast against each other (1-D samples, or (1, W) x (H, 1) for a whole grid)."""
        z0, z1 = z - self.p.z_band_m, z + self.p.z_band_m
        names, out = [], []
        for bx in self.course.boxes_at(z0, z1):
            c, s = math.cos(bx.yaw), math.sin(bx.yaw)
            u = (X - bx.cx) * c + (Y - bx.cy) * s
            v = -(X - bx.cx) * s + (Y - bx.cy) * c
            names.append(bx.name)
            out.append(np.maximum(np.abs(u) - bx.length / 2, np.abs(v) - bx.thick / 2))
        for cy in self.course.cylinders_at(z0, z1):
            names.append(cy.name)
            out.append(np.hypot(X - cy.cx, Y - cy.cy) - cy.radius)
        (nx0, nx1), (ny0, ny1) = self.course.net_x, self.course.net_y
        names.append("net")
        out.append(np.minimum(np.minimum(X - nx0, nx1 - X), np.minimum(Y - ny0, ny1 - Y)))
        return names, np.stack(np.broadcast_arrays(*out))

    def _open_width(self, z, a, b) -> float:
        pts, step = self._line_samples(a, b)
        _, D = self._dists(z, pts[:, 0], pts[:, 1])
        free = D.min(axis=0) > 0
        best = run = 0
        for f in free:
            run = run + 1 if f else 0
            best = max(best, run)
        return min(best + 1, len(pts) - 1) * step if best else 0.0

    def _gap(self, z, r, g: Grid, s, t) -> str:
        """The narrowest gap on the widest route from cell s to cell t (maximin path)."""
        H, W = g.occ.shape
        X = (g.x0 + (np.arange(W) + 0.5) * g.res)[None, :]    # broadcast instead of two full meshgrids
        Y = (g.y0 + (np.arange(H) + 0.5) * g.res)[:, None]
        names, D = self._dists(z, X, Y)
        b, (i, j) = _widest(D.min(axis=0), s, t)
        x, y = g.center(i, j)
        k1, k2 = np.argsort(D[:, i, j])[:2]
        return (f": narrowest gap on the best route is ~{2 * b:.2f} m wide at ({x:.2f}, {y:.2f}) "
                f"between '{names[k1]}' and '{names[k2]}' ({b:.2f} m/side, needs > {r:.2f} m/side)")


# ---------------------------------------------------------------------- search
def _moves(W2):
    return [(-W2, 1.0, 0, 0), (W2, 1.0, 0, 0), (-1, 1.0, 0, 0), (1, 1.0, 0, 0),
            (-W2 - 1, _SQ2, -W2, -1), (-W2 + 1, _SQ2, -W2, 1),
            (W2 - 1, _SQ2, W2, -1), (W2 + 1, _SQ2, W2, 1)]


def _astar(occ: np.ndarray, s, t):
    """8-connected A* (no corner cutting) on occ; s, t = (i, j). Returns the cell list or None."""
    H, W = occ.shape
    W2 = W + 2
    free = (~np.pad(occ, 1, constant_values=True)).ravel().tolist()
    si, ti = (s[0] + 1) * W2 + s[1] + 1, (t[0] + 1) * W2 + t[1] + 1
    N = len(free)
    # octile heuristic (x 1.001 tie-break) for every padded cell once, in numpy: the inner loop then does
    # one list lookup per relaxation instead of divmod + abs + max/min
    dr = np.abs(np.arange(H + 2) - (t[0] + 1))[:, None]
    dc = np.abs(np.arange(W2) - (t[1] + 1))[None, :]
    hval = (1.001 * (np.maximum(dr, dc) + (_SQ2 - 1) * np.minimum(dr, dc))).ravel().tolist()
    gcost, parent, closed = [math.inf] * N, [-1] * N, bytearray(N)
    gcost[si] = 0.0
    heap = [(0.0, 0.0, si)]
    moves = _moves(W2)
    heappop, heappush = heapq.heappop, heapq.heappush
    while heap:
        _, gu, u = heappop(heap)
        if u == ti:
            out = []
            while u != -1:
                r, c = divmod(u, W2)
                out.append((r - 1, c - 1))
                u = parent[u]
            return out[::-1]
        if closed[u]:
            continue
        closed[u] = 1
        for dv, cost, o1, o2 in moves:
            v = u + dv
            if closed[v] or not free[v] or (o1 and not (free[u + o1] and free[u + o2])):
                continue
            ng = gu + cost
            if ng < gcost[v]:
                gcost[v], parent[v] = ng, u
                heappush(heap, (ng + hval[v], ng, v))
    return None


def _widest(clr: np.ndarray, s, t):
    """Maximin path on the clearance map: (bottleneck clearance, bottleneck cell)."""
    H, W = clr.shape
    W2 = W + 2
    C = np.pad(clr, 1, constant_values=-np.inf).ravel().tolist()
    si, ti = (s[0] + 1) * W2 + s[1] + 1, (t[0] + 1) * W2 + t[1] + 1
    N = len(C)
    best, parent = [-math.inf] * N, [-1] * N
    best[si] = math.inf
    heap = [(-math.inf, si)]
    moves = _moves(W2)
    while heap:
        nb, u = heapq.heappop(heap)
        if -nb < best[u]:
            continue
        if u == ti:
            break
        for dv, _, _, _ in moves:
            v = u + dv
            b = min(-nb, C[v])
            if b > best[v]:
                best[v], parent[v] = b, u
                heapq.heappush(heap, (-b, v))
    u, worst, cell = ti, math.inf, (t[0], t[1])
    while u != -1:
        if u not in (si, ti) and C[u] < worst:
            r, c = divmod(u, W2)
            worst, cell = C[u], (r - 1, c - 1)
        u = parent[u]
    return worst, cell
