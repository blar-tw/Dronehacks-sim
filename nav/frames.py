"""nav.frames - the three frames nav/ uses, and the conversions between them. PURE.

  map    course frame from course_map.yaml: ENU, origin at the floor centre.
  odom   PX4's local frame expressed in ENU (x east, y north, z up), origin at PX4's local origin
         (the takeoff point). This is the "VIO frame" of ARCHITECTURE.md: smooth, but drifts.
  PX4    vehicle_local_position / trajectory_setpoint: NED, heading clockwise from north.

Map corrections never go into PX4 (ARCHITECTURE.md "Key rule"): the aligner keeps
T_map_odom, and every map target is converted to odom, then to NED, right before it is sent.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def ned_to_enu(x: float, y: float, z: float) -> Tuple[float, float, float]:
    return y, x, -z


def enu_to_ned(x: float, y: float, z: float) -> Tuple[float, float, float]:
    return y, x, -z


def heading_to_yaw(heading_ned: float) -> float:
    """PX4 heading (from north, clockwise) -> ENU yaw (from east, counter-clockwise)."""
    return wrap(math.pi / 2 - heading_ned)


def yaw_to_heading(yaw_enu: float) -> float:
    return wrap(math.pi / 2 - yaw_enu)


@dataclass(frozen=True)
class SE2:
    """p_map = R(yaw) p_odom + (x, y); z passes through with offset z."""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    z: float = 0.0

    def apply(self, px: float, py: float, pz: float = 0.0) -> Tuple[float, float, float]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c * px - s * py, self.y + s * px + c * py, pz + self.z

    def inverse(self) -> "SE2":
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return SE2(-(c * self.x + s * self.y), -(-s * self.x + c * self.y), -self.yaw, -self.z)

    def apply_yaw(self, yaw: float) -> float:
        return wrap(yaw + self.yaw)

    def compose(self, other: "SE2") -> "SE2":
        """self * other: first other, then self."""
        x, y, z = self.apply(other.x, other.y, other.z)
        return SE2(x, y, wrap(self.yaw + other.yaw), z)
