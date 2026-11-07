"""nav.follower - carrot follower on a map-frame polyline, stepped at 20 Hz. PURE.

The path is a polyline of (x, y, z) map-frame points; the carrot moves along it by 3D arclength.
Each step:
  * vehicle progress s_veh = projection of the vehicle onto the path, searched only in
    [s_prev - 0.2, s_prev + progress_window_m] so a parallel corridor can't make it skip ahead;
  * carrot speed ramps at accel_mps2 toward min(speed * speed_scale, corner limit), where the
    corner limit comes from the largest turn within lookahead_m ahead of the carrot (and between
    the vehicle and the carrot): 0 deg -> speed_mps, >= 90 deg -> corner_speed_mps, linear between;
    it also brakes to stop exactly at the path end;
  * the carrot never leads s_veh by more than lead_max_m (it waits), stops at once on
    speed_scale 0, and does not advance while the vehicle faces more than yaw_slow_deg away from
    the path tangent at the carrot (turn first);
  * yaw setpoint = path tangent at the carrot, rate-limited by yaw_rate_dps;
  * stop corners (xy heading change > corner_stop_deg): the carrot brakes to stop exactly on the
    vertex; once the vehicle is within arrive_m of it, it holds corner_hold_s (so the map aligner
    can correct while the vehicle is still; yaw already turns toward the next leg), then goes on.
    If the vehicle has not arrived corner_arrive_timeout_s after the carrot parked, the hold
    starts anyway (a vehicle that cannot settle to arrive_m must not stall the mission).
  * stop_every_vertex (team decision 5, "stop-and-go"): every interior vertex is a stop corner,
    turn angle or not; the planner's gate entry point is a vertex, so the vehicle is still there.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .frames import wrap

BACK_WINDOW_M = 0.2   # progress may slip back this far per step
DT_MAX_S = 0.2


@dataclass
class FollowerParams:
    speed_mps: float = 0.5
    accel_mps2: float = 0.5
    yaw_rate_dps: float = 60.0
    lead_max_m: float = 0.6        # carrot never more than this ahead of the vehicle's progress
    lookahead_m: float = 0.6       # corner look-ahead distance for speed planning
    corner_speed_mps: float = 0.25 # speed at a 90 deg corner (scales with turn angle)
    progress_window_m: float = 0.8 # progress search only in [s_prev - 0.2, s_prev + window]
    arrive_m: float = 0.15
    yaw_slow_deg: float = 30.0     # carrot stops advancing while |yaw error| exceeds this
    face_path: bool = True         # yaw follows the path tangent (forward camera sees ahead)
    corner_stop_deg: float = 45.0  # vertex turning more than this is a stop corner (0 or >= 180: off)
    corner_hold_s: float = 1.0     # still time at a stop corner once the vehicle has arrived
    corner_arrive_timeout_s: float = 4.0  # parked this long without arrival: hold anyway (0 = wait forever)
    stop_every_vertex: bool = False       # strict stop-and-go: hold corner_hold_s at every interior vertex


@dataclass
class Setpoint:
    x: float
    y: float
    z: float
    yaw: float             # map frame, yaw ENU
    progress_m: float      # vehicle progress along the path
    carrot_m: float        # carrot arclength
    length_m: float
    cross_track_m: float
    speed_mps: float       # carrot speed this step
    done: bool             # carrot at the end AND vehicle within arrive_m of the end
    holding: bool = False  # carrot stopped at a stop corner (waiting for arrival or corner_hold_s)


class Follower:
    def __init__(self, params: FollowerParams = FollowerParams()):
        self.p = params
        self.P = None

    # ------------------------------------------------------------------ path
    def set_path(self, pts, now, pos_xyz, yaw, speed_mps=None) -> None:
        """New path; the carrot starts at the vehicle's projection on it (whole path), speed 0."""
        P = np.asarray(pts, dtype=float)
        # drop repeated consecutive points (planner start == gate entry after a replan): a zero-length
        # segment would make U NaN and every setpoint NaN; a single point becomes a zero-length path
        keep = np.concatenate([[True], np.linalg.norm(P[1:] - P[:-1], axis=1) > 1e-6])
        P = P[keep]
        if len(P) < 2:
            P = np.vstack([P, P])
        D = P[1:] - P[:-1]
        lens = np.linalg.norm(D, axis=1)
        self.P, self.U = P, D / np.where(lens > 0, lens, 1.0)[:, None]   # the only 0 len is the padded point
        self.S = np.concatenate([[0.0], np.cumsum(lens)])
        self.L = float(self.S[-1])
        # xy heading per segment (a vertical segment keeps the previous heading), unwrapped so that
        # differences are signed turns
        H, h = [], yaw
        for dx, dy, _ in D:
            if math.hypot(dx, dy) > 1e-6:
                h = math.atan2(dy, dx)
            H.append(h if not H else H[-1] + wrap(h - H[-1]))
        self.H = np.array(H)
        # interior vertices: arclength, signed xy turn, bend (also counts climbs/descents)
        self.Sv = self.S[1:-1]
        self.turn = self.H[1:] - self.H[:-1]
        ang3d = np.arccos(np.clip(np.sum(self.U[1:] * self.U[:-1], axis=1), -1.0, 1.0))
        self.bend = np.maximum(np.abs(self.turn), ang3d)
        cs = self.p.corner_stop_deg
        if self.p.stop_every_vertex:
            self.stops = self.Sv                      # stop-and-go: every vertex, including gate entries
        else:
            self.stops = self.Sv[np.abs(self.turn) > math.radians(cs)] if 0 < cs < 180 else self.Sv[:0]

        self.v_path = self.p.speed_mps if speed_mps is None else float(speed_mps)
        self.s_veh = self._project(np.asarray(pos_xyz, dtype=float), -math.inf, math.inf)
        self.s_cmd = self.s_veh
        self.v = 0.0
        self.yaw_sp = yaw
        self.yaw_hold = yaw            # face_path False: keep the yaw the path started with
        self.held = False
        self.t = now
        self._next_stop()

    def hold(self) -> None:
        """Freeze the carrot (mission hover) until resume() or the next set_path."""
        self.held = True
        self.v = 0.0

    def resume(self) -> None:
        self.held = False

    def back_off(self, dist_m) -> None:
        """Move the carrot back along the path by dist_m (not past the start) and hold there."""
        self.s_cmd = max(0.0, self.s_cmd - dist_m)
        self._next_stop()
        self.hold()

    def _next_stop(self):
        self.k_stop = int(np.searchsorted(self.stops, self.s_cmd, side="right"))
        self.t_corner = None   # when the hold began (vehicle arrived, or arrival timed out)
        self.t_parked = None   # when the carrot reached the stop vertex

    # ------------------------------------------------------------------ step
    def step(self, now, pos_xyz, yaw, speed_scale=1.0) -> Setpoint:
        p = self.p
        dt = min(max(now - self.t, 0.0), DT_MAX_S)
        self.t = now
        pos = np.asarray(pos_xyz, dtype=float)
        self.s_veh = self._project(pos, self.s_veh - BACK_WINDOW_M, self.s_veh + p.progress_window_m)

        # stop corner: park the carrot on the vertex until the vehicle has been there corner_hold_s
        stop_s = self.stops[self.k_stop] if self.k_stop < len(self.stops) else self.L
        holding = self.s_cmd >= stop_s and self.k_stop < len(self.stops)
        if holding:
            if self.t_parked is None:
                self.t_parked = now
            if self.t_corner is None:
                arrived = np.linalg.norm(pos - self._point(stop_s)) <= p.arrive_m
                # a vehicle that never settles to arrive_m (position-hold error, aligner jump) would
                # otherwise park here until the mission's 60 s timeout and its back-off/replan ladder
                timed_out = 0 < p.corner_arrive_timeout_s <= now - self.t_parked
                if arrived or timed_out:
                    self.t_corner = now
            if self.t_corner is not None and now - self.t_corner >= p.corner_hold_s:
                self.k_stop += 1
                self.t_corner = self.t_parked = None
                holding = False
                stop_s = self.stops[self.k_stop] if self.k_stop < len(self.stops) else self.L

        goal = self._yaw_goal(self.s_cmd)
        turn_first = abs(wrap(goal - yaw)) > math.radians(p.yaw_slow_deg)
        if self.held or speed_scale <= 0 or turn_first or holding:
            self.v = 0.0
        elif dt > 0:
            a = p.accel_mps2
            vc = self._corner_limit()
            target = min(self.v_path * min(speed_scale, 1.0), vc)
            v = min(self.v + a * dt, target) if self.v < target else max(self.v - a * dt, target)
            # brake to stop exactly at the next stop corner / the end: d = v(v + a dt) / 2a is the distance
            # covered by the speeds v, v - a dt, ... down to 0, so solving for v makes the speed drop a*dt
            # per step and the last step land on the stop point, for any dt; v >= 0 whenever d >= 0. The
            # max() guards the 1-ulp case where S0 + (S1 - S0) projected the vehicle a hair past the end.
            d_stop = max(stop_s - self.s_cmd, 0.0)
            v = min(v, 0.5 * (-a * dt + math.sqrt((a * dt) ** 2 + 8 * a * d_stop)))
            s_new = max(self.s_cmd, min(self.s_cmd + v * dt, self.s_veh + p.lead_max_m, stop_s))
            self.v = (s_new - self.s_cmd) / dt
            self.s_cmd = s_new

        goal = self._yaw_goal(self.s_cmd)
        step = math.radians(p.yaw_rate_dps) * dt
        self.yaw_sp = wrap(self.yaw_sp + max(-step, min(step, wrap(goal - self.yaw_sp))))

        x, y, z = self._point(self.s_cmd)
        done = self.s_cmd >= self.L and float(np.linalg.norm(pos - self.P[-1])) <= p.arrive_m
        cross = float(np.linalg.norm(pos - self._point(self.s_veh)))
        return Setpoint(x, y, z, self.yaw_sp, self.s_veh, self.s_cmd, self.L, cross, self.v, done, holding)

    # ------------------------------------------------------------------ geometry
    def _seg(self, s):
        return int(np.clip(np.searchsorted(self.S, s, side="right") - 1, 0, len(self.U) - 1))

    def _point(self, s):
        i = self._seg(s)
        return tuple(float(c) for c in self.P[i] + self.U[i] * (s - self.S[i]))

    def _yaw_goal(self, s):
        return wrap(float(self.H[self._seg(s)])) if self.p.face_path else self.yaw_hold

    def _project(self, pos, lo, hi):
        """Arclength of the closest path point to pos, searched only within [lo, hi]."""
        lo, hi = max(lo, 0.0), min(hi, self.L)
        A, S0, S1 = self.P[:-1], self.S[:-1], self.S[1:]
        t = np.sum((pos - A) * self.U, axis=1)
        t = np.clip(t, np.maximum(lo - S0, 0.0), np.minimum(hi - S0, S1 - S0))
        d = np.linalg.norm(pos - (A + self.U * t[:, None]), axis=1)
        d[(S1 < lo) | (S0 > hi)] = math.inf
        i = int(np.argmin(d))
        return float(S0[i] + t[i])

    def _corner_limit(self):
        """Speed for the largest turn between min(vehicle, carrot) and lookahead_m past the carrot."""
        lo, hi = min(self.s_veh, self.s_cmd), self.s_cmd + self.p.lookahead_m
        m = (self.Sv >= lo) & (self.Sv <= hi)
        if not m.any():
            return self.v_path
        theta = max(float(np.max(np.abs(np.cumsum(self.turn[m])))), float(np.max(self.bend[m])))
        vc = min(self.p.corner_speed_mps, self.v_path)
        return self.v_path - (self.v_path - vc) * min(theta / (math.pi / 2), 1.0)
