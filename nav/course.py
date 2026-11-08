"""The course from course_map.yaml (map frame = course ENU, metres).

Obstacles are vertical prisms: oriented boxes (walls and their parts), boulders (cylinders) and the
tube's two side walls (the tube is open at its north and south ends). The net bounds the field.

A wall with `uncertain_m` was placed by hand, not measured: clearance() (planning) treats it as that
much thicker, and faces_at() (localization, view scoring) leaves it out, so a wall that is a bit off
never pulls the map. A relative path in load() is looked up in the course/ folder.
"""
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

DEFAULT_MAP = Path(__file__).resolve().parent.parent / "course" / "course_map.yaml"
TUBE_WALL_M = 0.06          # modelled thickness of each tube side wall


@dataclass
class Box:
    name: str
    cx: float
    cy: float
    sx: float                # length along yaw
    sy: float                # thickness
    yaw: float
    z0: float
    z1: float
    uncertain: float = 0.0   # metres: hand-placed, keep this much further away, do not localize on it

    def corners(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        u, v = np.array([c, s]) * self.sx / 2, np.array([-s, c]) * self.sy / 2
        o = np.array([self.cx, self.cy])
        return np.array([o - u - v, o + u - v, o + u + v, o - u + v])      # counter-clockwise

    def distance(self, P):
        """Distance from points P (N,2) to the box footprint (0 inside)."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        d = P - np.array([self.cx, self.cy])
        lx, ly = d[:, 0] * c + d[:, 1] * s, -d[:, 0] * s + d[:, 1] * c
        qx, qy = np.maximum(np.abs(lx) - self.sx / 2, 0), np.maximum(np.abs(ly) - self.sy / 2, 0)
        return np.hypot(qx, qy)


@dataclass
class Line:
    name: str
    a: np.ndarray
    b: np.ndarray

    @property
    def center(self):
        return (self.a + self.b) / 2


class Course:
    def __init__(self, doc):
        self.doc = doc
        self.wall_top = float(doc["wall_top"])
        self.net = (tuple(doc["net"]["x"]), tuple(doc["net"]["y"]))
        self.launch = np.array(doc["launch_mark"], float)
        self.pad_center = np.array(doc["landing_pad"]["center"], float)
        self.pad_radius = float(doc["landing_pad"]["rings_radius"][0])
        self.boxes = []
        for w in doc["walls"]:
            for i, p in enumerate(w.get("parts", [w])):
                (cx, cy, cz), (sx, sy, sz) = p["center"], p["size"]
                self.boxes.append(Box(f"{w['name']}#{i}" if "parts" in w else w["name"],
                                      cx, cy, sx, sy, p["yaw"], cz - sz / 2, cz + sz / 2,
                                      float(w.get("uncertain_m", 0.0))))
        t = doc["tube"]
        (x0, x1), (y0, y1), (z0, z1) = t["x"], t["y"], t["z"]
        self.tube = (x0, x1, y0, y1)
        for name, x in (("tube_west", x0 + TUBE_WALL_M / 2), ("tube_east", x1 - TUBE_WALL_M / 2)):
            self.boxes.append(Box(name, x, (y0 + y1) / 2, y1 - y0, TUBE_WALL_M, math.pi / 2, z0, z1))
        self.boulders = [(np.array(b["center"], float), float(b["radius"]), float(b["height"]))
                         for b in doc["boulders"]]
        w = doc["window"]
        self.window = Line("window", np.array(w["ends"][0], float), np.array(w["ends"][1], float))
        self.window_z = (float(w["bottom"]), float(w["top"]))
        self.lines = {c["name"]: Line(c["name"], np.array(c["line"][0], float), np.array(c["line"][1], float))
                      for c in doc["checkpoints"] if "line" in c}

    def boxes_at(self, z0, z1):
        return [b for b in self.boxes if b.z0 < z1 and b.z1 > z0]

    def clearance(self, P, z0, z1):
        """Distance from points P (N,2) to the nearest obstacle overlapping heights z0..z1, or to the
        net, whichever is closer."""
        P = np.asarray(P, float).reshape(-1, 2)
        (nx0, nx1), (ny0, ny1) = self.net
        d = np.minimum.reduce([P[:, 0] - nx0, nx1 - P[:, 0], P[:, 1] - ny0, ny1 - P[:, 1]])
        for b in self.boxes_at(z0, z1):
            d = np.minimum(d, np.maximum(b.distance(P) - b.uncertain, 0.0))
        for c, r, h in self.boulders:
            if h > z0:
                d = np.minimum(d, np.maximum(np.linalg.norm(P - c, axis=1) - r, 0.0))
        return d

    def faces_at(self, z):
        """Wall faces at height z as (A, B, N): segment ends (F,2) and outward normals (F,2)."""
        A, B = [], []
        for b in self.boxes_at(z, z):
            if b.uncertain > 0.0:
                continue
            c = b.corners()
            for k in range(4):
                A.append(c[k])
                B.append(c[(k + 1) % 4])
        A, B = np.array(A), np.array(B)
        D = B - A
        N = np.stack([D[:, 1], -D[:, 0]], axis=1) / np.linalg.norm(D, axis=1)[:, None]
        return A, B, N


def load(path=DEFAULT_MAP):
    path = Path(path)
    if not path.is_absolute() and not path.exists():
        path = DEFAULT_MAP.parent / path
    with open(path) as f:
        return Course(yaml.safe_load(f))
