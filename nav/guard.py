"""Last layer before PX4: hover in place while something is unstable, go on once it has been fine
for resume_s.

Hover on: VIO WITHHELD (ev_bridge state), no camera frames for frames_timeout_s, or the vehicle
behind its setpoint by more than behind_xy_m / behind_z_m for behind_s (not while climbing off the
ground: the vehicle lags the climb ramp while it spools up). VIO DEGRADED only slows the follower.
Map sigma (stop and look) is the follower's job.
"""
import math
from dataclasses import dataclass


@dataclass
class GuardParams:
    behind_xy_m: float = 1.0
    behind_z_m: float = 0.4
    behind_s: float = 2.0
    frames_timeout_s: float = 10.0
    resume_s: float = 1.0


class Guard:
    def __init__(self, p: GuardParams = GuardParams()):
        self.p = p
        self.vio = "OK"
        self.t_frame = None
        self.behind_since = None
        self.holding = False
        self.why = ""
        self.fine_since = None

    def on_vio(self, state):
        self.vio = state

    def on_frame(self, now):
        self.t_frame = now

    def start(self, now):
        """Airborne from now: the frame timeout counts from here."""
        self.t_frame = now
        self.behind_since = None

    @property
    def slow(self):
        return self.vio == "DEGRADED"

    def _problem(self, now, pos, sp, behind):
        p = self.p
        if self.vio == "WITHHELD":
            return "VIO withheld"
        if now - self.t_frame > p.frames_timeout_s:
            return f"no camera frames for {now - self.t_frame:.0f} s"
        if behind and (math.hypot(pos[0] - sp[0], pos[1] - sp[1]) > p.behind_xy_m or abs(pos[2] - sp[2]) > p.behind_z_m):
            self.behind_since = self.behind_since if self.behind_since is not None else now
            if now - self.behind_since > p.behind_s:
                return "behind the setpoint"
        else:
            self.behind_since = None
        return ""

    def check(self, now, pos, sp, behind=True):
        """pos: vehicle odom (x, y, z); sp: the setpoint being flown; behind: check the setpoint lag.
        -> True while hovering."""
        why = self._problem(now, pos, sp, behind)
        if why:
            self.holding, self.why, self.fine_since = True, why, None
        elif self.holding:
            self.fine_since = self.fine_since if self.fine_since is not None else now
            if now - self.fine_since >= self.p.resume_s:
                self.holding = False
        return self.holding
