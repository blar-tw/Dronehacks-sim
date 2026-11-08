"""Frames. map = course ENU (course_map.yaml: x east, y north, z up, origin floor centre).
odom = PX4 local position turned ENU (x = east, y = north, z = up). T_map_odom is a 2D rigid
transform (x, y, yaw): p_map = R(yaw) p_odom + t. Heights are not transformed."""
import math

import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def ned_to_enu(n, e, d):
    return e, n, -d


def enu_to_ned(x, y, z):
    return y, x, -z


def heading_to_yaw(heading):
    """PX4 heading (rad, from north, clockwise) -> ENU yaw (from east, counter-clockwise)."""
    return wrap(math.pi / 2 - heading)


def yaw_to_heading(yaw):
    return wrap(math.pi / 2 - yaw)


def rot(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


class SE2:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x, self.y, self.yaw = float(x), float(y), wrap(float(yaw))

    @classmethod
    def from_poses(cls, map_xy, map_yaw, odom_xy, odom_yaw):
        """The transform that puts the odom pose (odom_xy, odom_yaw) at (map_xy, map_yaw)."""
        yaw = wrap(map_yaw - odom_yaw)
        t = np.asarray(map_xy, float) - rot(yaw) @ np.asarray(odom_xy, float)
        return cls(t[0], t[1], yaw)

    def vec(self):
        return np.array([self.x, self.y, self.yaw])

    def apply(self, xy):
        """odom -> map for (2,) or (N,2)."""
        p = np.asarray(xy, float)
        return p @ rot(self.yaw).T + np.array([self.x, self.y])

    def inverse_apply(self, xy):
        """map -> odom."""
        p = np.asarray(xy, float) - np.array([self.x, self.y])
        return p @ rot(self.yaw)
