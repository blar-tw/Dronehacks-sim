"""Mission state machine: wait -> arm -> climb -> look -> follow -> descend -> land, then the return
leg (arm -> climb -> look -> follow -> descend -> land again) if it fits in the time left -> done.

Map corrections only move targets: every setpoint is a map target turned into odom. PX4 sees only
VIO. The follower decides with the localizer's T_map_odom (the best estimate); setpoints use Tc, a
copy that slides toward it at most blend_mps / blend_dps at the vehicle, so a map fix never makes
the setpoint jump.

Descend: offboard, fast down to slow_h above the ground, then slow; at touch_h PX4 Land takes over
for the last few cm (it only has to detect the touchdown). Height above the ground is the down
rangefinder minus its reading on the ground at arm time, or the odom height without a rangefinder.

The guard is the last layer: while it holds, the airborne phases stop advancing and the setpoint
is the pose where the hold began.
"""
import math
from dataclasses import dataclass

import numpy as np

from .follower import Follower, FollowParams
from .frames import SE2, after_odom_reset, odom_after_reset, rot, wrap
from .guard import Guard

NAV_OFFBOARD, NAV_AUTO_LAND = 14, 18


@dataclass
class MissionParams:
    launch_yaw_enu: float = math.pi / 2     # the launch mark faces up the first corridor (course +y)
    launch_offset_m: float = 0.0            # start this far ahead of the launch mark along launch_yaw_enu
    health_quiet_s: float = 2.0
    climb_rate_mps: float = 0.5
    climb_tol_m: float = 0.10
    yaw_rate_dps: float = 30.0              # camera-only VIO: slow turns (lab 2026-11-08: VIO lost in the maze at 60)
    cmd_resend_s: float = 1.0
    stream_lead_s: float = 0.5
    takeoff_scan_deg: float = 0.0           # turn in place after the climb (lab 2026-11-07: 360 at 30 deg/s -> VIO lost, PX4 failsafe)
    scan_rate_dps: float = 30.0
    time_limit_s: float = 180.0             # rule: 3 min from the first takeoff
    rearm_s: float = 5.0                    # return estimate: disarmed on the pad -> climbing
    return_margin_s: float = 10.0
    blend_mps: float = 0.05                 # map corrections reach the setpoint at most this fast
    blend_dps: float = 3.0
    blend_ok_m: float = 0.03                # the window waits until the correction is in
    descend_fast_mps: float = 0.4
    descend_slow_mps: float = 0.12
    slow_h_m: float = 0.30
    touch_h_m: float = 0.05
    descend_lag_m: float = 0.25             # the z setpoint stays at most this far below the vehicle
    descend_max_s: float = 20.0
    land_s: float = 3.0                     # return estimate: PX4 Land from touch_h to disarmed


@dataclass
class Vehicle:
    pos: tuple = None        # odom ENU (x, y, z)
    yaw: float = None        # odom ENU yaw
    pos_ok: bool = False
    armed: bool = None
    nav_state: int = None
    range_m: float = None    # down rangefinder, None when missing or stale


@dataclass
class Out:
    phase: str
    setpoint: tuple = None   # odom ENU (x, y, z, yaw)
    stream: bool = False
    commands: tuple = ()
    event: str = ""


class Mission:
    def __init__(self, course, wps, loc, scorer, ret=None, p: MissionParams = MissionParams(),
                 fp: FollowParams = FollowParams(), guard: Guard = None):
        self.course, self.wps, self.ret, self.loc, self.scorer, self.p = course, wps, ret, loc, scorer, p
        self.follower = Follower(wps, scorer, fp)
        self.guard = guard or Guard()
        self.leg = "out"                     # out | return
        self.phase = "wait"
        self.t_phase = self.t_cmd = self.t_last = None
        self.t_start = None                  # first takeoff
        self.healthy_since = None
        self.home = None                     # odom (x, y, z, yaw) at the arm request of this leg
        self.sp = None                       # odom (x, y, z, yaw)
        self.hold_sp = None
        self.offboard_seen = self.auto_land_seen = False
        self.scan_left = 0.0
        self.result = None
        self.Tc = SE2()                      # T_map_odom the setpoints use (slides toward loc.T)
        self.blend_err = (0.0, 0.0)          # (m at the vehicle, rad) still to slide in
        self.ground_rng = None               # rangefinder reading on the ground at arm time

    def on_health(self, now, ok):
        self.healthy_since = (self.healthy_since or now) if ok else None

    def floor_z(self):
        return self.home[2]

    def on_odom_reset(self, old_xy, new_xy, d_yaw):
        """PX4 reset its local frame (see frames.odom_after_reset). The map estimate, the setpoint
        transform and the odom setpoints follow it, so nothing moves in the map and no map correction
        is needed (a jump bigger than the localizer's step cap could never be corrected)."""
        self.loc.odom_reset(old_xy, new_xy, d_yaw)
        self.Tc = after_odom_reset(self.Tc, old_xy, new_xy, d_yaw)
        for sp in (self.sp, self.hold_sp):
            if sp is not None:
                sp[0], sp[1] = (float(c) for c in odom_after_reset(sp[:2], old_xy, new_xy, d_yaw))
                sp[3] = wrap(sp[3] + d_yaw)

    def _set(self, phase, now, why=""):
        self.phase, self.t_phase = phase, now
        return f"-> {phase}" + (f": {why}" if why else "")

    def _resend(self, now):
        if self.t_cmd is None or now - self.t_cmd >= self.p.cmd_resend_s:
            self.t_cmd = now
            return True
        return False

    def _yaw_toward(self, target, dt):
        step = math.radians(self.p.yaw_rate_dps) * dt
        self.sp[3] = wrap(self.sp[3] + max(-step, min(step, wrap(target - self.sp[3]))))

    def _blend(self, dt, odom_xy):
        """Slide Tc toward loc.T, limited at the vehicle: its map position moves at most blend_mps,
        the yaw at most blend_dps."""
        q = np.asarray(odom_xy, float)
        a, b = self.Tc.apply(q), self.loc.T.apply(q)
        d, dyaw = b - a, wrap(self.loc.T.yaw - self.Tc.yaw)
        dist = float(np.linalg.norm(d))
        f = 1.0
        if dist > 1e-9:
            f = min(f, self.p.blend_mps * dt / dist)
        if abs(dyaw) > 1e-9:
            f = min(f, math.radians(self.p.blend_dps) * dt / abs(dyaw))
        yaw = self.Tc.yaw + f * dyaw
        t = a + f * d - rot(yaw) @ q
        self.Tc = SE2(t[0], t[1], yaw)
        self.blend_err = ((1.0 - f) * dist, (1.0 - f) * dyaw)

    def height(self, v):
        """-> (height above the ground, source)."""
        if v.range_m is not None and self.ground_rng is not None:
            return v.range_m - self.ground_rng, "range"
        return v.pos[2] - self.floor_z(), "odom"

    def _descend_s(self, h):
        p = self.p
        return max(h - p.slow_h_m, 0.0) / p.descend_fast_mps + min(h, p.slow_h_m) / p.descend_slow_mps + p.land_s

    def map_pose(self, v):
        xy = self.loc.T.apply(np.array(v.pos[:2]))
        return xy, wrap(v.yaw + self.loc.T.yaw)

    def return_estimate_s(self):
        """Disarmed on the pad -> landed on the launch mark."""
        p, fp = self.p, self.follower.p
        a, b = self.ret[0], self.ret[-1]
        d = math.hypot(b.x - a.x, b.y - a.y)
        return p.rearm_s + a.z / p.climb_rate_mps + d / fp.speed_mps + self._descend_s(a.z) + p.return_margin_s

    def _return_or_done(self, now):
        if self.ret is None:
            self.result = "complete"
            return self._set("done", now, "landed and disarmed")
        left = self.p.time_limit_s - (now - self.t_start)
        need = self.return_estimate_s()
        if need > left:
            self.result = "complete (no return)"
            return self._set("done", now, f"on the pad; return needs {need:.0f} s, {left:.0f} s left")
        self.leg = "return"
        self.wps = self.ret
        self.follower.replace(self.ret)
        self.home = None
        self.offboard_seen = self.auto_land_seen = False
        self.t_cmd = None
        return self._set("wait", now, f"return leg: needs {need:.0f} s, {left:.0f} s left")

    def step(self, now, v: Vehicle) -> Out:
        p = self.p
        dt = 0.0 if self.t_last is None else min(now - self.t_last, 0.25)
        self.t_last = now
        if self.t_phase is None:
            self.t_phase = now
        ev = ""
        cmds = []
        if self.phase == "done":
            return Out("done")
        airborne = self.phase in ("climb", "look", "follow", "descend")
        if airborne and self.offboard_seen and v.nav_state != NAV_OFFBOARD:
            self.result = f"yielded: PX4 left Offboard (nav_state {v.nav_state})"
            return Out("done", event=self._set("done", now, self.result))
        if airborne or self.phase == "land":
            self.loc.predict(v.pos[:2])
            self._blend(dt, v.pos[:2])

        if airborne:
            held = self.hold_sp is not None
            if self.guard.check(now, v.pos, self.hold_sp or self.sp, behind=self.phase != "climb"):
                if not held:
                    self.hold_sp = [*v.pos, v.yaw]
                    ev = f"hover: {self.guard.why}"
                return Out(self.phase, tuple(self.hold_sp), True, (), ev)
            if held:
                self.sp, self.hold_sp = list(self.hold_sp), None
                ev = "resume"

        if self.phase == "wait":
            if (v.armed is False and v.pos_ok and self.healthy_since is not None
                    and now - self.healthy_since >= p.health_quiet_s):
                self.home = (*v.pos, v.yaw)
                self.sp = list(self.home)
                if self.leg == "out":
                    start = np.asarray(self.course.launch) + p.launch_offset_m * np.array(
                        [math.cos(p.launch_yaw_enu), math.sin(p.launch_yaw_enu)])
                    self.loc.start(start, p.launch_yaw_enu, v.pos[:2], v.yaw)
                    self.Tc = SE2(*self.loc.T.vec())
                self.ground_rng = v.range_m
                ev = self._set("arm", now, "no rangefinder, descend on odom height" if v.range_m is None
                               else f"rangefinder on the ground {v.range_m:.3f} m")
        elif self.phase == "arm":
            if v.armed and v.nav_state == NAV_OFFBOARD:
                self.offboard_seen = True
                self.t_start = self.t_start if self.t_start is not None else now
                self.guard.start(now)
                ev = self._set("climb", now)
            elif now - self.t_phase >= p.stream_lead_s and self._resend(now):
                cmds = ["offboard", "arm"]
        elif self.phase == "climb":
            target = self.home[2] + self.wps[0].z
            self.sp[2] = min(target, self.sp[2] + p.climb_rate_mps * dt)
            if self.sp[2] >= target and abs(v.pos[2] - target) < p.climb_tol_m:
                self.scan_left = math.radians(p.takeoff_scan_deg)
                ev = self._set("look", now, f"scan {p.takeoff_scan_deg:.0f} deg")
        elif self.phase == "look":
            step = min(self.scan_left, math.radians(p.scan_rate_dps) * dt)
            self.sp[3] = wrap(self.sp[3] + step)
            self.scan_left -= step
            if self.scan_left <= 0.0 and abs(wrap(self.sp[3] - v.yaw)) < math.radians(self.follower.p.face_tol_deg):
                ev = self._set("follow", now, f"sigma {self.loc.sigma_xy():.3f} m, {self.loc.accepted} fits")
        elif self.phase == "follow":
            xy, yaw = self.map_pose(v)
            # above the walls (return leg) the map has nothing to see: fly on VIO, ignore map sigma
            sigma = 0.0 if self.leg == "return" else self.loc.sigma_xy()
            self.follower.slow = self.guard.slow
            self.follower.blend_ok = self.blend_err[0] <= p.blend_ok_m
            carrot, z, tyaw = self.follower.step(now, dt, xy, yaw, sigma)
            c = self.Tc.inverse_apply(carrot)
            self.sp[0], self.sp[1], self.sp[2] = c[0], c[1], self.home[2] + z
            self._yaw_toward(tyaw - self.Tc.yaw, dt)
            if self.follower.done:
                h, src = self.height(v)
                ev = self._set("descend", now, ("at the pad" if self.leg == "out" else "at the launch mark")
                               + f", {h:.2f} m up ({src})")
        elif self.phase == "descend":
            h, src = self.height(v)
            rate = p.descend_fast_mps if h > p.slow_h_m else p.descend_slow_mps
            self.sp[2] = max(self.sp[2] - rate * dt, v.pos[2] - p.descend_lag_m)
            if h <= p.touch_h_m or now - self.t_phase > p.descend_max_s:
                why = f"{h:.3f} m up ({src})" if h <= p.touch_h_m else f"descend timeout, {h:.2f} m up ({src})"
                ev = self._set("land", now, why)
        elif self.phase == "land":
            if v.nav_state == NAV_AUTO_LAND:
                self.auto_land_seen = True
            if not self.auto_land_seen and self._resend(now):
                cmds = ["land"]
            if self.auto_land_seen and v.armed is False:
                if self.leg == "out":
                    ev = self._return_or_done(now)
                else:
                    self.result = "complete with return"
                    ev = self._set("done", now, "landed on the launch mark")

        stream = self.phase in ("arm", "climb", "look", "follow", "descend") or (self.phase == "land" and not self.auto_land_seen)
        return Out(self.phase, tuple(self.sp) if self.sp else None, stream, tuple(cmds), ev)

    def status(self):
        f = self.follower
        return {"phase": self.phase, "leg": self.leg, "result": self.result, "follow_mode": f.mode,
                "segment": f.i, "segments": len(self.wps) - 1,
                "target": self.wps[min(f.i + 1, len(self.wps) - 1)].label,
                "hover": self.guard.why if self.hold_sp is not None else "", "vio": self.guard.vio,
                "sigma_xy": round(self.loc.sigma_xy(), 3), "T_map_odom": [round(c, 3) for c in self.loc.T.vec()],
                "Tc": [round(c, 3) for c in self.Tc.vec()], "blend_left_m": round(self.blend_err[0], 3),
                "fits": self.loc.accepted, "capped": self.loc.capped, "last_fit": self.loc.last}
