"""Mission state machine: wait -> arm -> climb -> look -> follow -> land -> done.

Map corrections only move targets: every setpoint is a map target turned into odom with the
localizer's current T_map_odom. PX4 sees only VIO.
"""
import math
from dataclasses import dataclass

import numpy as np

from .follower import Follower, FollowParams
from .frames import wrap

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
    def __init__(self, course, wps, loc, scorer, p: MissionParams = MissionParams(), fp: FollowParams = FollowParams()):
        self.course, self.wps, self.loc, self.scorer, self.p = course, wps, loc, scorer, p
        self.follower = Follower(wps, scorer, fp)
        self.phase = "wait"
        self.t_phase = self.t_cmd = self.t_last = None
        self.healthy_since = None
        self.home = None                     # odom (x, y, z, yaw) at the arm request
        self.sp = None                       # odom (x, y, z, yaw)
        self.offboard_seen = self.auto_land_seen = False
        self.look_yaw = None
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

        if self.phase == "wait":
            if (v.armed is False and v.pos_ok and self.healthy_since is not None
                    and now - self.healthy_since >= p.health_quiet_s):
                self.home = (*v.pos, v.yaw)
                self.sp = list(self.home)
                self.loc.start(self.course.launch, p.launch_yaw_enu, v.pos[:2], v.yaw)
                ev = self._set("arm", now)
        elif self.phase == "arm":
            if v.armed and v.nav_state == NAV_OFFBOARD:
                self.offboard_seen = True
                ev = self._set("climb", now)
            elif now - self.t_phase >= p.stream_lead_s and self._resend(now):
                cmds = ["offboard", "arm"]
        elif self.phase == "climb":
            target = self.home[2] + self.wps[0].z
            self.sp[2] = min(target, self.sp[2] + p.climb_rate_mps * dt)
            if self.sp[2] >= target and abs(v.pos[2] - target) < p.climb_tol_m:
                # localize facing the launch heading: turning in place next to the launch walls is
                # where the x500's 0.39 m prop tips reach barrier_1 (local GZ 2026-11-07)
                self.look_yaw = self.map_pose(v)[1]
                ev = self._set("look", now, "localize after takeoff")
        elif self.phase == "look":
            _, yaw = self.map_pose(v)
            self._yaw_toward(self.look_yaw - self.loc.T.yaw, dt)
            fp = self.follower.p
            if (abs(wrap(self.look_yaw - yaw)) < math.radians(fp.face_tol_deg) and now - self.t_phase > fp.look_s
                    and (self.loc.sigma_xy() <= fp.slow_sigma_m or now - self.t_phase > fp.look_max_s)):
                ev = self._set("follow", now, f"sigma {self.loc.sigma_xy():.3f} m, {self.loc.accepted} fits")
        elif self.phase == "follow":
            xy, yaw = self.map_pose(v)
            carrot, z, tyaw = self.follower.step(now, dt, xy, yaw, self.loc.sigma_xy())
            c = self.loc.T.inverse_apply(carrot)
            self.sp[0], self.sp[1], self.sp[2] = c[0], c[1], self.home[2] + z
            self._yaw_toward(tyaw - self.loc.T.yaw, dt)
            if self.follower.done:
                ev = self._set("land", now, "at the pad")
        elif self.phase == "land":
            if v.nav_state == NAV_AUTO_LAND:
                self.auto_land_seen = True
            if not self.auto_land_seen and self._resend(now):
                cmds = ["land"]
            if self.auto_land_seen and v.armed is False:
                self.result = "complete"
                ev = self._set("done", now, "landed and disarmed")

        stream = self.phase in ("arm", "climb", "look", "follow") or (self.phase == "land" and not self.auto_land_seen)
        return Out(self.phase, tuple(self.sp) if self.sp else None, stream, tuple(cmds), ev)

    def status(self):
        f = self.follower
        return {"phase": self.phase, "result": self.result, "follow_mode": f.mode, "segment": f.i,
                "segments": len(self.wps) - 1, "target": self.wps[min(f.i + 1, len(self.wps) - 1)].label,
                "sigma_xy": round(self.loc.sigma_xy(), 3), "T_map_odom": [round(c, 3) for c in self.loc.T.vec()],
                "fits": self.loc.accepted, "last_fit": self.loc.last}
