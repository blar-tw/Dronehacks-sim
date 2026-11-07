"""Mission state machine: wait -> arm -> climb -> look -> follow -> land, then the return leg
(arm -> climb -> look -> follow -> land again) if it fits in the time left -> done.

Map corrections only move targets: every setpoint is a map target turned into odom with the
localizer's current T_map_odom. PX4 sees only VIO.

The guard is the last layer: while it holds, the airborne phases stop advancing and the setpoint
is the pose where the hold began.
"""
import math
from dataclasses import dataclass

import numpy as np

from .follower import Follower, FollowParams
from .frames import wrap
from .guard import Guard

NAV_OFFBOARD, NAV_AUTO_LAND = 14, 18


@dataclass
class MissionParams:
    launch_yaw_enu: float = math.pi / 2     # the launch mark faces up the first corridor (course +y)
    health_quiet_s: float = 2.0
    climb_rate_mps: float = 0.5
    climb_tol_m: float = 0.10
    yaw_rate_dps: float = 60.0
    cmd_resend_s: float = 1.0
    stream_lead_s: float = 0.5
    takeoff_scan_deg: float = 0.0           # turn in place after the climb (lab 2026-11-07: 360 at 30 deg/s -> VIO lost, PX4 failsafe)
    scan_rate_dps: float = 30.0
    time_limit_s: float = 180.0             # rule: 3 min from the first takeoff
    rearm_s: float = 5.0                    # return estimate: disarmed on the pad -> climbing
    land_rate_mps: float = 0.5              # return estimate: PX4 land descent
    return_margin_s: float = 10.0


@dataclass
class Vehicle:
    pos: tuple = None        # odom ENU (x, y, z)
    yaw: float = None        # odom ENU yaw
    pos_ok: bool = False
    armed: bool = None
    nav_state: int = None


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

    def on_health(self, now, ok):
        self.healthy_since = (self.healthy_since or now) if ok else None

    def floor_z(self):
        return self.home[2]

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

    def map_pose(self, v):
        xy = self.loc.T.apply(np.array(v.pos[:2]))
        return xy, wrap(v.yaw + self.loc.T.yaw)

    def return_estimate_s(self):
        """Disarmed on the pad -> landed on the launch mark."""
        p, fp = self.p, self.follower.p
        a, b = self.ret[0], self.ret[-1]
        d = math.hypot(b.x - a.x, b.y - a.y)
        return p.rearm_s + a.z / p.climb_rate_mps + d / fp.speed_mps + a.z / p.land_rate_mps + p.return_margin_s

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
        airborne = self.phase in ("climb", "look", "follow")
        if airborne and self.offboard_seen and v.nav_state != NAV_OFFBOARD:
            self.result = f"yielded: PX4 left Offboard (nav_state {v.nav_state})"
            return Out("done", event=self._set("done", now, self.result))
        if airborne or self.phase == "land":
            self.loc.predict(v.pos[:2])

        if airborne:
            held = self.hold_sp is not None
            if self.guard.check(now, v.pos, self.hold_sp or self.sp):
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
                    self.loc.start(self.course.launch, p.launch_yaw_enu, v.pos[:2], v.yaw)
                ev = self._set("arm", now)
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
            carrot, z, tyaw = self.follower.step(now, dt, xy, yaw, sigma)
            c = self.loc.T.inverse_apply(carrot)
            self.sp[0], self.sp[1], self.sp[2] = c[0], c[1], self.home[2] + z
            self._yaw_toward(tyaw - self.loc.T.yaw, dt)
            if self.follower.done:
                ev = self._set("land", now, "at the pad" if self.leg == "out" else "at the launch mark")
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

        stream = self.phase in ("arm", "climb", "look", "follow") or (self.phase == "land" and not self.auto_land_seen)
        return Out(self.phase, tuple(self.sp) if self.sp else None, stream, tuple(cmds), ev)

    def status(self):
        f = self.follower
        return {"phase": self.phase, "leg": self.leg, "result": self.result, "follow_mode": f.mode,
                "segment": f.i, "segments": len(self.wps) - 1,
                "target": self.wps[min(f.i + 1, len(self.wps) - 1)].label,
                "hover": self.guard.why if self.hold_sp is not None else "", "vio": self.guard.vio,
                "sigma_xy": round(self.loc.sigma_xy(), 3), "T_map_odom": [round(c, 3) for c in self.loc.T.vec()],
                "fits": self.loc.accepted, "last_fit": self.loc.last}
