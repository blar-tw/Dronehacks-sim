"""Reject impossible VIO kinematics and require a quiet recovery interval."""
from __future__ import annotations

import math
from collections import deque
from typing import NamedTuple


class VioHealth:
    def __init__(self, *, max_speed_mps=5.0, jump_m=1.0, window_s=10.0, max_jumps=1, quiet_s=5.0):
        self.max_speed = float(max_speed_mps)
        self.jump_m = float(jump_m)
        self.window_s = float(window_s)
        self.max_jumps = int(max_jumps)
        self.quiet_s = float(quiet_s)
        self._last = None               # (t, pos) of the last FINITE, advancing sample
        self._events = deque()          # times of insane samples
        self._last_bad_t = None
        self.fresh = False              # the latest sample was itself a jump/speed/time violation
        self.reason = "no samples"
        self.n_bad = 0

    def update(self, t, pos):
        """-> (ok, reason). t in seconds (measurement time), pos a 3-tuple in any consistent frame."""
        bad = None
        finite = math.isfinite(t) and all(math.isfinite(float(c)) for c in pos)
        if not finite:
            bad = "non-finite sample"
        elif self._last is not None:
            t0, p0 = self._last
            dt = t - t0
            d = math.dist(p0, pos)
            if dt <= 0:
                bad = "measurement time did not advance" + (f" (moved {d:.2f} m)" if d > 0 else "")
            elif d > self.jump_m:
                bad = f"jump {d:.2f} m"
            elif d / dt > self.max_speed:
                bad = f"speed {d / dt:.1f} m/s"
        if finite and (self._last is None or t > self._last[0]):
            self._last = (t, tuple(float(c) for c in pos))
        tb = t if math.isfinite(t) else (self._last[0] if self._last else 0.0)
        self.fresh = bad is not None
        if bad:
            self.n_bad += 1
            self._last_bad_t = tb
            self._events.append(tb)
        while self._events and tb - self._events[0] > self.window_s:
            self._events.popleft()
        if len(self._events) > self.max_jumps:
            self.reason = f"{len(self._events)} insane samples in {self.window_s:.0f} s (last: {bad or 'earlier'})"
            return False, self.reason
        if self._last_bad_t is not None and tb - self._last_bad_t < self.quiet_s:
            self.reason = f"settling after '{bad or 'insane sample'}' ({tb - self._last_bad_t:.1f}/{self.quiet_s:.0f} s quiet)"
            return False, self.reason
        self.reason = "ok"
        return True, self.reason


class Decision(NamedTuple):
    send: bool          # publish this sample to PX4
    state: str          # OK | DEGRADED | WITHHELD
    pos_var: float      # m^2 added to each position variance
    vel_var: float      # (m/s)^2 added to each velocity variance
    reset: bool         # bump reset_counter on this sample
    forced: bool        # an insane sample sent because the withhold cap ran out


class VioLoss:
    """Keep PX4 aided through weak or lost visual tracking, so its no-aiding
    timeout (EKF2_NOAID_TOUT, 5 s by default) never runs out. See decision-1.md.

    Weak tracking (fewer than min_features in a frame): keep sending, with the
    position and velocity sigma growing with the time since the last visually
    tracked frame, up to a cap. Insane samples (VioHealth) are withheld for at
    most withhold_max_s, then sent anyway at the capped sigma with one reset,
    PROVIDED the sample itself is kinematically consistent with its predecessor
    (only VioHealth's settle/window timer objects). A sample that is itself a
    fresh jump or over-speed restarts the withhold clock and is never sent: VIO
    producing impossible kinematics every frame is not aiding, and the mission
    (WITHHELD > 3 s -> land) acts before PX4's 5 s timeout.

    No feature count for features_timeout_s (topic absent on this cuVSLAM
    build, or not heard yet) means UNKNOWN, not weak: samples go out unchanged
    (PX4 then applies its EKF2_EVP/EVV_NOISE floors), never DEGRADED forever."""

    def __init__(self, *, min_features=30, pos_sigma_rate=0.25, pos_sigma_max=0.5,
                 vel_sigma_rate=0.25, vel_sigma_max=0.5, withhold_max_s=2.0, features_timeout_s=1.0):
        vals = (pos_sigma_rate, pos_sigma_max, vel_sigma_rate, vel_sigma_max, withhold_max_s, features_timeout_s)
        if not all(math.isfinite(v) and v > 0 for v in vals) or not min_features >= 0:
            raise ValueError("VioLoss sigma rates/caps, withhold_max_s and features_timeout_s must be positive finite")
        self.min_features = min_features
        self.pos_rate, self.pos_max = pos_sigma_rate, pos_sigma_max
        self.vel_rate, self.vel_max = vel_sigma_rate, vel_sigma_max
        self.withhold_max_s, self.features_timeout_s = withhold_max_s, features_timeout_s
        self.weak = False               # last heard frame had < min_features
        self.last_features = None       # time any feature count was heard (None: never)
        self.first_features = None      # lost_s baseline before the first visually tracked frame
        self.last_visual = None         # time of the last frame with enough features
        self.withheld_since = None
        self.forced = False

    def features(self, t, n):
        """Feature count of the latest cuVSLAM frame, received at time t (s).
        n < 0 (count unavailable) counts as weak, as cuVSLAM's visually_tracked does."""
        self.last_features = t
        if self.first_features is None:
            self.first_features = t
        self.weak = n < self.min_features
        if not self.weak:
            self.last_visual = t

    def known(self, t):
        """A feature count was heard recently enough to trust `weak`."""
        return self.last_features is not None and t - self.last_features <= self.features_timeout_s

    def lost_s(self, t):
        """Seconds since the last visually tracked frame; 0 while nothing is known to be lost."""
        if not self.known(t):
            return 0.0
        since = self.last_visual if self.last_visual is not None else self.first_features
        return max(0.0, t - since)

    def decide(self, t, sane, fresh=False):
        """fresh: this very sample is a jump/speed/time violation (VioHealth.fresh), not just settling."""
        if not sane:
            if self.withheld_since is None or fresh:        # a fresh violation starts a new episode: new cap, new reset
                self.withheld_since, self.forced = t, False
            if fresh or t - self.withheld_since < self.withhold_max_s:
                return Decision(False, "WITHHELD", 0.0, 0.0, False, False)
            reset = not self.forced
            self.forced = True
            return Decision(True, "DEGRADED", self.pos_max ** 2, self.vel_max ** 2, reset, True)
        self.withheld_since = None
        self.forced = False
        if not self.weak or not self.known(t):
            return Decision(True, "OK", 0.0, 0.0, False, False)
        lost = self.lost_s(t)
        return Decision(True, "DEGRADED", min(self.pos_max, self.pos_rate * lost) ** 2,
                        min(self.vel_max, self.vel_rate * lost) ** 2, False, False)


def inflate_xyz(cov, var):
    """Row-major 6x6 covariance with var added to the x, y, z diagonal.
    A non-finite or negative diagonal (cuVSLAM cov_bad) is treated as 0 first, as
    frames._diag would, so the inflation survives instead of becoming NaN -> 0."""
    out = list(cov)
    for i in (0, 7, 14):
        c = float(out[i])
        out[i] = (c if math.isfinite(c) and c >= 0.0 else 0.0) + var
    return out
