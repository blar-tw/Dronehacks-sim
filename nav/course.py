"""nav.course - course geometry from course_map.yaml. PURE: numpy + yaml, no ROS.

course_map.yaml is the only source of course geometry (ARCHITECTURE.md). Nothing in nav/ may
hold a course coordinate; everything comes from here.

Map frame: ENU, metres, origin at the floor centre (+x east, +y north, +z up).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

XY = Tuple[float, float]


@dataclass(frozen=True)
class Box:
    """A solid box: footprint rectangle (length along yaw, thickness across) and z range."""
    name: str
    cx: float
    cy: float
    length: float
    thick: float
    zmin: float
    zmax: float
    yaw: float = 0.0

    def covers_z(self, z0: float, z1: float) -> bool:
        return self.zmax > z0 and self.zmin < z1

    def corners(self, grow: float = 0.0) -> np.ndarray:
        """4x2 footprint corners (counter-clockwise), grown by `grow` on every side."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hl, ht = self.length / 2 + grow, self.thick / 2 + grow
        local = np.array([[hl, ht], [-hl, ht], [-hl, -ht], [hl, -ht]])
        R = np.array([[c, -s], [s, c]])
        return local @ R.T + np.array([self.cx, self.cy])

    def faces(self) -> List[Tuple[XY, XY]]:
        """The 4 footprint edges as segments (what a depth camera sees at a fixed height)."""
        k = self.corners()
        return [(tuple(k[i]), tuple(k[(i + 1) % 4])) for i in range(4)]


@dataclass(frozen=True)
class Cylinder:
    name: str
    cx: float
    cy: float
    radius: float
    zmax: float
    zmin: float = 0.0

    def covers_z(self, z0: float, z1: float) -> bool:
        return self.zmax > z0 and self.zmin < z1


@dataclass(frozen=True)
class Checkpoint:
    """A scoring line (or the landing circle). `line` is a crossing segment, `circle` (cx, cy, r)."""
    id: int
    name: str
    points: float
    line: Optional[Tuple[XY, XY]] = None
    circle: Optional[Tuple[float, float, float]] = None
    bonus: float = 0.0
    zmin: float = 0.0           # opening bottom (window); 0 = floor
    zmax: float = math.inf      # opening top (window); inf = open


@dataclass
class Course:
    wall_top: float
    net_x: XY
    net_y: XY
    net_z: XY
    launch: XY
    pad: Tuple[float, float, float]                  # cx, cy, outer ring radius
    boxes: List[Box] = field(default_factory=list)
    cylinders: List[Cylinder] = field(default_factory=list)
    checkpoints: List[Checkpoint] = field(default_factory=list)
    tube: Optional[dict] = None                      # {x: [..], y: [..], z: [..]} bounding box
    tube_wall_m: float = 0.03                        # shell thickness assumed for the tube

    # ------------------------------------------------------------------ queries
    def boxes_at(self, z0: float, z1: float) -> List[Box]:
        """Every solid box that overlaps the height band [z0, z1], tube walls included."""
        out = [b for b in self.boxes if b.covers_z(z0, z1)]
        out += self.tube_boxes(z0, z1)
        return out

    def cylinders_at(self, z0: float, z1: float) -> List[Cylinder]:
        return [c for c in self.cylinders if c.covers_z(z0, z1)]

    def tube_boxes(self, z0: float, z1: float) -> List[Box]:
        """The oval tube seen in the band [z0, z1]: two side walls along its axis (y), placed at the
        narrowest interior half-width found in the band. Outside the tube's height it is solid."""
        if not self.tube:
            return []
        (x0, x1), (y0, y1), (tz0, tz1) = self.tube["x"], self.tube["y"], self.tube["z"]
        if z1 <= tz0 or z0 >= tz1:
            return []
        cx, a = (x0 + x1) / 2, (x1 - x0) / 2
        cz, b = (tz0 + tz1) / 2, (tz1 - tz0) / 2
        half = a
        for z in (max(z0, tz0), min(z1, tz1)):
            u = (z - cz) / b
            half = min(half, a * math.sqrt(max(0.0, 1.0 - u * u)))
        ly, cy = y1 - y0, (y0 + y1) / 2
        if half <= self.tube_wall_m:                       # band runs into the roof/floor: solid
            return [Box("tube_solid", cx, cy, ly, 2 * a, tz0, tz1, math.pi / 2)]
        t = self.tube_wall_m
        return [Box("tube_west", cx - half - t / 2, cy, ly, t, tz0, tz1, math.pi / 2),
                Box("tube_east", cx + half + t / 2, cy, ly, t, tz0, tz1, math.pi / 2)]

    def faces_at(self, z: float) -> List[Tuple[XY, XY]]:
        """Wall faces (segments) visible at height z: what the aligner matches depth against.
        The net is not a face (depth sees through it)."""
        out = []
        for b in self.boxes_at(z - 0.01, z + 0.01):
            out += b.faces()
        return out

    def checkpoint(self, name: str) -> Checkpoint:
        for c in self.checkpoints:
            if c.name == name:
                return c
        raise KeyError(f"no checkpoint {name!r} in the course map")

    def in_net(self, x: float, y: float, margin: float = 0.0) -> bool:
        return (self.net_x[0] + margin <= x <= self.net_x[1] - margin
                and self.net_y[0] + margin <= y <= self.net_y[1] - margin)


def _box(name: str, d: dict) -> Box:
    cx, cy, cz = d["center"]
    L, T, H = d["size"]
    return Box(name, float(cx), float(cy), float(L), float(T), float(cz - H / 2), float(cz + H / 2),
               float(d.get("yaw", 0.0)))


def load(path) -> Course:
    import yaml
    doc = yaml.safe_load(Path(path).read_text())
    net = doc["net"]
    pad = doc["landing_pad"]
    c = Course(wall_top=float(doc["wall_top"]), net_x=tuple(net["x"]), net_y=tuple(net["y"]),
               net_z=tuple(net["z"]), launch=tuple(doc["launch_mark"]),
               pad=(float(pad["center"][0]), float(pad["center"][1]), float(max(pad["rings_radius"]))),
               tube=doc.get("tube"))
    for w in doc.get("walls", []):
        if "parts" in w:
            for i, p in enumerate(w["parts"]):
                c.boxes.append(_box(f"{w['name']}#{i}", p))
        else:
            c.boxes.append(_box(w["name"], w))
    for m in doc.get("other_meshes", []):
        (x0, y0, z0), (x1, y1, z1) = m["min"], m["max"]
        c.boxes.append(Box(m["name"], (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0, z0, z1, 0.0))
    for b in doc.get("boulders", []):
        c.cylinders.append(Cylinder(b["name"], float(b["center"][0]), float(b["center"][1]),
                                    float(b["radius"]), float(b["height"])))
    win = doc.get("window")
    for cp in doc.get("checkpoints", []):
        line = tuple(tuple(map(float, p)) for p in cp["line"]) if "line" in cp else None
        circ = None
        if "circle" in cp:
            circ = (float(cp["circle"]["center"][0]), float(cp["circle"]["center"][1]),
                    float(cp["circle"]["radius"]))
        zmin, zmax = 0.0, math.inf
        if win and "window" in cp["name"]:
            zmin, zmax = float(win["bottom"]), float(win["top"])
        c.checkpoints.append(Checkpoint(int(cp["id"]), cp["name"], float(cp["points"]), line, circ,
                                        float(cp.get("bonus", 0.0)), zmin, zmax))
    return c


def default_map_path() -> Path:
    """workspace/course/course_map.yaml (ships with the program), else the team folder's copy."""
    here = Path(__file__).resolve().parent.parent
    for p in (here / "course" / "course_map.yaml", here.parent / "course" / "course_map.yaml"):
        if p.exists():
            return p
    raise FileNotFoundError("course_map.yaml not found (give map_file:=...)")


# ---------------------------------------------------------------------- grid
@dataclass
class Grid:
    """2D occupancy at one height band. occ[i, j]: row i = y, column j = x."""
    x0: float
    y0: float
    res: float
    occ: np.ndarray

    def cell(self, x: float, y: float) -> Tuple[int, int]:
        return int(math.floor((y - self.y0) / self.res)), int(math.floor((x - self.x0) / self.res))

    def center(self, i: int, j: int) -> XY:
        return self.x0 + (j + 0.5) * self.res, self.y0 + (i + 0.5) * self.res

    def inside(self, i: int, j: int) -> bool:
        return 0 <= i < self.occ.shape[0] and 0 <= j < self.occ.shape[1]

    def free(self, x: float, y: float) -> bool:
        i, j = self.cell(x, y)
        return self.inside(i, j) and not self.occ[i, j]


def _fill_polygon(occ, x0, y0, res, poly):
    """Mark cells whose centres lie inside a convex polygon (Nx2, CCW or CW)."""
    lo, hi = poly.min(axis=0), poly.max(axis=0)
    j0, j1 = max(0, int((lo[0] - x0) / res)), min(occ.shape[1] - 1, int((hi[0] - x0) / res) + 1)
    i0, i1 = max(0, int((lo[1] - y0) / res)), min(occ.shape[0] - 1, int((hi[1] - y0) / res) + 1)
    if j1 < j0 or i1 < i0:
        return
    xs = x0 + (np.arange(j0, j1 + 1) + 0.5) * res
    ys = y0 + (np.arange(i0, i1 + 1) + 0.5) * res
    X, Y = np.meshgrid(xs, ys)
    inside = np.ones_like(X, dtype=bool)
    n = len(poly)
    sign = None
    for k in range(n):
        ax, ay = poly[k]
        bx, by = poly[(k + 1) % n]
        cross = (bx - ax) * (Y - ay) - (by - ay) * (X - ax)
        if sign is None:
            area = sum((poly[m][0] * poly[(m + 1) % n][1] - poly[(m + 1) % n][0] * poly[m][1]) for m in range(n))
            sign = 1.0 if area > 0 else -1.0
        inside &= sign * cross >= 0
    occ[i0:i1 + 1, j0:j1 + 1] |= inside


def occupancy(course: Course, z0: float, z1: float, inflate: float, res: float = 0.05,
              net_margin: Optional[float] = None) -> Grid:
    """Obstacles overlapping the height band [z0, z1], each grown by `inflate` metres, plus the
    net (grown by net_margin, default `inflate`). Cells outside the net are occupied."""
    nx0, nx1 = course.net_x
    ny0, ny1 = course.net_y
    W = int(math.ceil((nx1 - nx0) / res))
    H = int(math.ceil((ny1 - ny0) / res))
    occ = np.zeros((H, W), dtype=bool)
    g = Grid(nx0, ny0, res, occ)
    for b in course.boxes_at(z0, z1):
        _fill_polygon(occ, nx0, ny0, res, b.corners(grow=inflate))
    xs = nx0 + (np.arange(W) + 0.5) * res
    ys = ny0 + (np.arange(H) + 0.5) * res
    X, Y = np.meshgrid(xs, ys)
    for c in course.cylinders_at(z0, z1):
        occ |= (X - c.cx) ** 2 + (Y - c.cy) ** 2 <= (c.radius + inflate) ** 2
    m = inflate if net_margin is None else net_margin
    occ |= (X < nx0 + m) | (X > nx1 - m) | (Y < ny0 + m) | (Y > ny1 - m)
    return g
