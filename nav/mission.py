"""nav.mission - the mission state machine on the refnav pattern. PURE: no ROS, no clock reads, no files.

Time arrives with every call (one clock). The planner, follower and aligner are passed in, so tests
can use fakes. Course geometry (gates, tube, pad, launch mark, wall top) comes only from the Course.

Flow (phases):
  wait     PX4 position + heading valid, explicit disarmed report, estimator healthy for health_quiet_s
  arm      stream a hold setpoint, after stream_lead_s request Offboard + arm (resent every cmd_resend_s)
  climb    vertical altitude change at the current xy (takeoff, and before a segment at another z)
  fly      follow the planned path of the current segment
  hover    failure ladder: segment timed out, carrot frozen for hover_s
  back_off failure ladder: carrot moved back back_off_m, then replan (replans times), then land
  descend  over the landing target, down to land_descend_z_m
  land     PX4 AUTO_LAND; streaming stops once nav_state 18 is seen; disarm when landed
  ground   disarmed on the pad, ground_wait_s before the return leg re-arms (wait -> arm -> ...)
  done     result: "complete" | "aborted: <why>" | "refused: <why>" | "yielded: <why>"

Segments: course checkpoints in id order, one per id (the `chicane` param picks among several,
`chicane_fallback` is tried if the planner finds no path), the 'line' ones are gates, the 'circle'
one is the landing target, then (return_leg) the launch mark at wall_top + return_clearance_m.
The tunnel segment is the gate whose line lies on the course's tube bounding box (geometry, not a
name): inside the tube it flies at tunnel speed, ignores depth staleness and VIO WITHHELD, and the
aligner is not updated until the gate is crossed.

Safety behaviour kept from refnav.core: health gate before arming; PX4 leaving Offboard while
airborne is never fought ('yielded'); PX4 owns every landing (no streaming after AUTO_LAND);
unhealthy/silent estimator lands. Rule 7.7: in the main run the commanded z never exceeds
wall_top - wall_margin_m (clamped and logged).

Added after lab run1 (2026-11-06, the drone hit the first corridor's wall and the mission kept
flying for 6 s): the vehicle staying more than track_z_m (height; not while climbing) or track_xy_m
(horizontal) from the commanded setpoint for track_lost_s lands it where it is.

Heights: map z is height above the main-leg takeoff spot (odom z minus the takeoff odom z), because
PX4's z origin is the EKF start point, not the floor. depth_lost_land_s watches the camera (any
frame, usable or not); depth_stale_hover_s watches wall points.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from refnav.core import NAV_AUTO_LAND, NAV_OFFBOARD, Health, Output, Vehicle  # noqa: F401 (Vehicle: API type)

from . import frames as F

AIRBORNE = ("climb", "fly", "hover", "back_off", "descend")


@dataclass
class MissionParams:
    cruise_z_m: float = 0.80
    return_clearance_m: float = 0.50   # return altitude = wall_top + this (rule 7.10 exception)
    chicane: str = "chicane_window"    # name of the checkpoint to use where an id has several
    chicane_fallback: str = "chicane_gate_b"   # tried if planning through `chicane` fails ("" = none)
    return_leg: bool = True
    speed_mps: float = 0.5
    tunnel_speed_mps: float = 0.3
    segment_timeout_s: float = 60.0
    hover_s: float = 2.0               # failure ladder: hold this long, then back off
    back_off_m: float = 0.4
    back_off_s: float = 2.0            # time given to the back-off before replanning
    replans: int = 1
    land_descend_z_m: float = 0.30     # descend to this over the target, then PX4 land
    ground_wait_s: float = 2.0         # on the pad after disarm before the return takeoff
    vio_degraded_scale: float = 0.5
    depth_stale_hover_s: float = 0.0   # > 0: hold when no wall points this long (outside the tunnel, below
                                       # wall top). 0 = off: the forward camera facing the net sees no walls,
                                       # and holding there never brings them back into view
    depth_lost_land_s: float = 10.0
    launch_yaw_deg: float = 90.0       # map yaw the aircraft faces on the launch mark (placement convention);
                                       # nan = trust PX4's heading as the map heading
    wall_margin_m: float = 0.30        # rule 7.7: main-run z <= wall_top - this
    tube_tol_m: float = 0.05           # a gate line within this of a tube bbox face is the tunnel gate
    tunnel_margin_m: float = 0.30      # tunnel mode starts this far outside the tube's footprint
    climb_rate_mps: float = 0.5
    descend_rate_mps: float = 0.3
    climb_tol_m: float = 0.15
    disarm_delay_s: float = 0.5        # landed this long and still armed: send a (checked) disarm
    # arming / health, as refnav.core.Params
    health_quiet_s: float = 3.0
    health_stale_s: float = 1.0
    health_bad_land_s: float = 1.0
    withheld_land_s: float = 3.0       # unhealthy while vio_state is WITHHELD (ev_bridge withholds <= 2 s)
    stream_lead_s: float = 1.0
    wait_s: float = 300.0
    arm_wait_s: float = 30.0
    cmd_resend_s: float = 1.0
    land_timeout_s: float = 60.0
    track_z_m: float = 0.40            # tracking lost: |z - setpoint z| above this (not in climb) ...
    track_xy_m: float = 1.00           # ... or horizontal distance to the setpoint above this ...
    track_lost_s: float = 2.0          # ... for this long: land where it is (0 = off)


@dataclass
class Segment:
    kind: str                       # "gate" | "pad" | "return"
    name: str                       # checkpoint name, or "launch_mark"
    z: float                        # map z flown on this segment
    cp: object = None               # the Checkpoint (gate line / landing circle)
    target: tuple | None = None     # pad / return: map xy goal
    tunnel: bool = False
    options: list = field(default_factory=list)   # gate names still to try if planning fails


def on_tube(course, line, tol) -> bool:
    """The gate line lies on a face of the course's tube bounding box (within tol)."""
    if not course.tube or line is None:
        return False
    (x0, x1), (y0, y1) = course.tube["x"], course.tube["y"]
    (ax, ay), (bx, by) = line
    within_x = min(ax, bx) >= x0 - tol and max(ax, bx) <= x1 + tol
    within_y = min(ay, by) >= y0 - tol and max(ay, by) <= y1 + tol
    on_y = any(abs(ay - y) <= tol and abs(by - y) <= tol for y in (y0, y1))
    on_x = any(abs(ax - x) <= tol and abs(bx - x) <= tol for x in (x0, x1))
    return (on_y and within_x) or (on_x and within_y)


def build_segments(course, p: MissionParams):
    """-> ([Segment], [note]). Checkpoints in id order, one per id; gates, the landing circle, return."""
    by_id = {}
    for c in course.checkpoints:
        by_id.setdefault(c.id, []).append(c)
    segs, notes = [], []
    z_cap = course.wall_top - p.wall_margin_m
    z = p.cruise_z_m
    if z > z_cap:
        notes.append(f"rule 7.7: cruise_z_m {z:.2f} m above wall_top - {p.wall_margin_m:.2f} = {z_cap:.2f} m; clamped")
        z = z_cap
    for cid in sorted(by_id):
        group = by_id[cid]
        circle = next((c for c in group if c.circle is not None), None)
        if circle is not None:
            segs.append(Segment("pad", circle.name, z, cp=circle, target=(circle.circle[0], circle.circle[1])))
            continue
        names = [c.name for c in group]
        order = [n for n in (p.chicane, p.chicane_fallback) if n and n in names] if len(group) > 1 else []
        if len(group) > 1 and not order:
            notes.append(f"checkpoint id {cid}: neither {p.chicane!r} nor {p.chicane_fallback!r} in {names}; using {names[0]}")
        name = order[0] if order else names[0]
        cp = course.checkpoint(name)
        segs.append(Segment("gate", name, z, cp=cp, tunnel=on_tube(course, cp.line, p.tube_tol_m), options=order[1:]))
    if p.return_leg:
        segs.append(Segment("return", "launch_mark", course.wall_top + p.return_clearance_m,
                            target=tuple(course.launch)))
    return segs, notes


def crossing(p0, p1, a, b) -> int:
    """+1 / -1 if the track p0 -> p1 crosses the segment a-b (sign = side of the left normal of a->b
    it ends on), else 0. The line itself counts as the + side."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    s0 = dx * (p0[1] - a[1]) - dy * (p0[0] - a[0])
    s1 = dx * (p1[1] - a[1]) - dy * (p1[0] - a[0])
    if (s0 < 0) == (s1 < 0):
        return 0
    f = s0 / (s0 - s1)
    ix, iy = p0[0] + f * (p1[0] - p0[0]), p0[1] + f * (p1[1] - p0[1])
    u = ((ix - a[0]) * dx + (iy - a[1]) * dy) / (dx * dx + dy * dy)
    if not 0.0 <= u <= 1.0:
        return 0
    return 1 if s1 >= 0 else -1


def _side(p, a, b) -> float:
    return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])


class Mission:
    def __init__(self, course, planner, follower, aligner, params: MissionParams | None = None):
        self.course, self.planner, self.follower, self.aligner = course, planner, follower, aligner
        self.p = p = params or MissionParams()
        self.health = Health(p.health_quiet_s, p.health_stale_s)
        self.segments, notes = build_segments(course, p)
        self._pending = list(notes)
        self.phase, self.why, self.result = "wait", "", None
        self.leg = "main"                     # "main" (to the pad) | "return"
        self.seg_i = 0
        self.t_phase = self.t_last = self.t_cmd = self.t_wait0 = None
        self.t_takeoff = self.t_airborne = self.seg_t0 = None
        self.offboard_seen = False
        self.sp_ned = None                    # arm phase: hold setpoint straight from PX4 (n, e, d, heading)
        self.sp_map = None                    # flight: [x, y, z, yaw] map frame
        self.climb_z = None
        self.cross_sign = 0
        self.crossed = False
        self.replans_used = 0
        self.tunnel_active = self.tunnel_done = False
        self.t_tunnel = None
        self.pos_map = self.pos_odom = self.prev_map = self.prev_odom = None
        self.aligner_ready = False
        self.takeoff_odom = None
        self.passed = []                      # [{"id", "name", "t", "elapsed_s"}]
        self.land_kind = self.land_reason = None
        self.land_auto_seen = False
        self.landed_since = None
        self.land_request = self.stop_request = None
        self.pts_t = None                     # last frame with wall points
        self.frame_t = None                   # last depth frame of any kind (the camera is alive)
        self.z0 = 0.0                         # odom z of the main-leg takeoff spot (map z = odom z - z0)
        self.track_bad_since = None
        self.vio_state, self.vio_lost_s = "UNKNOWN", 0.0
        self.last_align = None
        self.speed_scale = None
        self.hold_why = ""
        self.z_cap = course.wall_top - p.wall_margin_m
        self.clamping = False
        self.counts = {"plans": 0, "align_accepted": 0, "align_rejected": 0, "align_skipped_tunnel": 0,
                       "point_frames": 0, "z_clamps": 0}

    # ------------------------------------------------------------------ inputs
    def on_health(self, t, ok):
        self.health.on_msg(t, ok)

    def on_vio_state(self, t, state, lost_s):
        state = str(state).upper()
        if state != self.vio_state:
            self._pending.append(f"vio_state {self.vio_state} -> {state} (lost {float(lost_s):.1f} s)")
        self.vio_state, self.vio_lost_s = state, float(lost_s)

    def on_depth_frame(self, t):
        """A depth frame arrived, usable or not: the camera is alive."""
        self.frame_t = t

    def on_points(self, t, points_odom_xy, z):
        """Wall points of one depth frame, odom xy, taken at odom height z."""
        self.frame_t = t
        pts = np.asarray(points_odom_xy, float).reshape(-1, 2)
        if len(pts) == 0:
            return
        self.pts_t = t
        z = z - self.z0 + self.aligner.T.z                      # map height of the frame
        self.counts["point_frames"] += 1
        if not self.aligner_ready or self.pos_odom is None:
            return
        if self.tunnel_active:
            self.counts["align_skipped_tunnel"] += 1
            return
        r = self.aligner.update(pts, z, t, sensor_odom_xy=(self.pos_odom[0], self.pos_odom[1]))
        self.counts["align_accepted" if r.accepted else "align_rejected"] += 1
        if self.last_align is None or r.accepted != self.last_align.accepted:
            self._pending.append(f"aligner {'accepted' if r.accepted else 'rejected'}: {r.why} (n {r.n}, "
                                 f"d ({r.dx:+.3f}, {r.dy:+.3f}) m, {math.degrees(r.dyaw):+.2f} deg)")
        self.last_align = r

    def request_land(self, why="requested"):
        if self.land_request is None:
            self.land_request = str(why) or "requested"

    # ------------------------------------------------------------------ helpers
    def _event(self, out, text):
        out.events.append(text)

    def _set(self, phase, now, out, why=""):
        out.events.append(f"{self.phase} -> {phase}" + (f": {why}" if why else ""))
        self.phase, self.t_phase = phase, now
        if why:
            self.why = why

    def _end(self, now, out, result):
        if self.leg == "return" and result != "complete":
            kind, _, why = result.partition(": ")
            result = f"{kind}: return leg: {why}"
        self.result = result
        self._set("done", now, out, result)

    def _resend(self, now):
        if self.t_cmd is None or now - self.t_cmd >= self.p.cmd_resend_s:
            self.t_cmd = now
            return True
        return False

    def _land(self, now, out, why, kind="abort"):
        """kind: "pad" | "return" (planned landings) | "abort" (land where it is)."""
        self.land_kind, self.land_reason = kind, why
        self.land_auto_seen, self.landed_since = False, None
        if kind == "abort" and self.pos_map is not None:
            self.sp_map = [self.pos_map[0], self.pos_map[1], self.sp_map[2], self.sp_map[3]]
        if self.tunnel_active:
            self.tunnel_active = False
            self._event(out, "tunnel exit (landing)")
        self._set("land", now, out, why)

    def _seg(self):
        return self.segments[self.seg_i]

    def _elapsed(self, now):
        return now - self.t_takeoff if self.t_takeoff is not None else 0.0

    def _depth_matters(self):
        return (self.phase in ("climb", "fly", "hover", "back_off") and not self.tunnel_active
                and self._seg().z < self.course.wall_top)

    def _depth_age(self, now):
        """Time since the last wall points (or since takeoff)."""
        ref = self.t_airborne if self.pts_t is None else max(self.pts_t, self.t_airborne)
        return now - ref

    def _camera_age(self, now):
        """Time since the last depth frame of any kind (or since takeoff)."""
        ref = self.t_airborne if self.frame_t is None else max(self.frame_t, self.t_airborne)
        return now - ref

    def _to_map(self, x, y, z, yaw):
        X, Y, Z, YAW = self.aligner.to_map(x, y, z, yaw)
        return X, Y, Z - self.z0, YAW

    def _to_odom(self, x, y, z, yaw):
        return self.aligner.to_odom(x, y, z + self.z0, yaw)

    def _tracking_error(self, now):
        """'' or why the vehicle has been too far from its setpoint for track_lost_s."""
        p = self.p
        if p.track_lost_s <= 0 or self.sp_map is None or self.pos_map is None:
            return ""
        dz = self.pos_map[2] - self.sp_map[2]
        dxy = math.hypot(self.pos_map[0] - self.sp_map[0], self.pos_map[1] - self.sp_map[1])
        bad = dxy > p.track_xy_m or (self.phase != "climb" and abs(dz) > p.track_z_m)
        if not bad:
            self.track_bad_since = None
            return ""
        if self.track_bad_since is None:
            self.track_bad_since = now
        if now - self.track_bad_since < p.track_lost_s:
            return ""
        return (f"tracking lost for {now - self.track_bad_since:.1f} s: {dxy:.2f} m from the setpoint, "
                f"height {self.pos_map[2]:.2f} m vs {self.sp_map[2]:.2f} m (a hit or a stall)")

    def _in_tube(self, pm):
        (x0, x1), (y0, y1) = self.course.tube["x"], self.course.tube["y"]
        m = self.p.tunnel_margin_m
        return x0 - m <= pm[0] <= x1 + m and y0 - m <= pm[1] <= y1 + m

    # ------------------------------------------------------------------ step
    def step(self, now, v) -> Output:
        p = self.p
        out = Output(self.phase)
        out.events += self._pending
        self._pending = []
        dt = 0.0 if self.t_last is None else min(max(now - self.t_last, 0.0), 0.25)
        self.t_last = now
        if self.t_phase is None:
            self.t_phase = now
        if self.t_wait0 is None:
            self.t_wait0 = now
        if self.phase == "done":
            return out

        # ---- before any control request
        if self.phase in ("wait", "arm") and self.t_cmd is None:
            if v.armed is True:
                self._end(now, out, "yielded: vehicle armed before this mission requested control")
            elif now - self.t_wait0 > p.wait_s:
                self._end(now, out, f"refused: {self.why or 'not ready'} after {p.wait_s:.0f} s")

        # ---- airborne supervision: never fight PX4 leaving Offboard
        if self.phase in AIRBORNE or (self.phase == "land" and not self.land_auto_seen):
            allowed = (NAV_OFFBOARD, NAV_AUTO_LAND) if self.phase == "land" else (NAV_OFFBOARD,)
            if self.offboard_seen and v.nav_state is not None and v.nav_state not in allowed:
                self._end(now, out, f"yielded: PX4 left Offboard (nav_state {v.nav_state}): a pilot or a failsafe has it")
        if self.land_request is not None and self.phase != "done":
            why = f"landing requested: {self.land_request}"
            self.stop_request, self.land_request = why, None
            if self.phase == "wait":
                self._end(now, out, f"refused: {why} before arming")
            elif self.phase == "arm":
                if v.armed:
                    out.commands.append("disarm")
                self._end(now, out, f"refused: {why} while arming")
            elif self.phase in AIRBORNE:
                self._land(now, out, why)
            elif self.phase == "ground":
                self._end(now, out, f"aborted: {why} (on the pad, return leg not flown)")

        # ---- pose (odom ENU from PX4 NED, map via the aligner)
        if v.pos is not None and v.heading is not None:
            x, y, z = F.ned_to_enu(*v.pos)
            self.pos_odom = (x, y, z, F.heading_to_yaw(v.heading))
            if self.aligner_ready:
                self.pos_map = self._to_map(*self.pos_odom)

        if self.phase in AIRBORNE:
            bad = self.health.bad_for(now)
            limit = p.withheld_land_s if self.vio_state == "WITHHELD" else p.health_bad_land_s
            if bad > limit:
                self._land(now, out, f"estimator unhealthy or silent for {bad:.1f} s (limit {limit:.1f} s)")
            elif not v.pos_ok or v.pos is None:
                self._land(now, out, "PX4 position or heading invalid")
            elif self._depth_matters() and self._camera_age(now) > p.depth_lost_land_s:
                self._land(now, out, f"no depth frames for {self._camera_age(now):.1f} s")
            elif self._tracking_error(now):
                self._land(now, out, self._tracking_error(now))
            else:
                if self.prev_odom is not None:
                    self.aligner.predict(math.hypot(self.pos_odom[0] - self.prev_odom[0],
                                                    self.pos_odom[1] - self.prev_odom[1]),
                                         odom_xy=(self.pos_odom[0], self.pos_odom[1]))

        ph = self.phase
        if ph == "wait":
            self._wait(now, out, v)
        elif ph == "arm":
            self._arm(now, out, v)
        elif ph == "climb":
            self._climb(now, dt, out)
        elif ph in ("fly", "hover", "back_off"):
            self._fly(now, out)
        elif ph == "descend":
            self._descend(now, dt, out)
        elif ph == "ground":
            if now - self.t_phase >= p.ground_wait_s:
                self.leg, self.t_cmd, self.t_wait0, self.sp_ned, self.sp_map = "return", None, now, None, None
                self._set("wait", now, out, "return leg: re-arming on the pad")
        if self.phase == "land":
            self._landing(now, out, v)

        self.prev_map, self.prev_odom = self.pos_map, self.pos_odom
        out.phase = self.phase
        out.stream = self.phase in ("arm",) + AIRBORNE or (self.phase == "land" and not self.land_auto_seen)
        out.setpoint = self._setpoint_ned(out) if out.stream else None
        return out

    # ------------------------------------------------------------------ phases
    def _wait(self, now, out, v):
        ok, why = self.health.arm_ok(now)
        if v.armed is not False:
            self.why = "waiting for an explicit disarmed PX4 report"
        elif not v.pos_ok or v.pos is None or v.heading is None:
            self.why = "waiting for PX4 position and heading"
        elif not ok:
            self.why = why
        else:
            n, e, d = v.pos
            self.sp_ned = [n, e, d, v.heading]
            self._set("arm", now, out, f"{self.leg} leg: streaming, then Offboard + arm")

    def _arm(self, now, out, v):
        p = self.p
        ok, why = self.health.arm_ok(now)
        if not ok:
            if self.t_cmd is None and v.armed is False:
                self.sp_ned = None
                self._set("wait", now, out, f"{why}; requalifying before any control request")
            else:
                if v.armed:
                    out.commands.append("disarm")
                self._end(now, out, f"refused: {why} before arming")
        elif now - self.t_phase > p.arm_wait_s:
            if v.armed:
                out.commands.append("disarm")
            self._end(now, out, f"refused: not armed in Offboard after {p.arm_wait_s:.0f} s")
        elif v.armed and v.nav_state == NAV_OFFBOARD:
            self.offboard_seen = True
            self._takeoff(now, out)
        elif now - self.t_phase >= p.stream_lead_s and self._resend(now):
            out.commands += ["offboard", "arm"]

    def _takeoff(self, now, out):
        ox, oy, oz, oyaw = self.pos_odom
        if not self.aligner_ready:             # main run: anchor the map at the launch mark, on the ground
            self.takeoff_odom, self.t_takeoff = (ox, oy, oz), now
            self.z0 = oz
            yaw_off = 0.0 if math.isnan(self.p.launch_yaw_deg) else F.wrap(math.radians(self.p.launch_yaw_deg) - oyaw)
            self.aligner.init_at_launch((ox, oy), tuple(self.course.launch), yaw_offset=yaw_off)
            self.aligner_ready = True
            self._event(out, f"takeoff: odom ({ox:.2f}, {oy:.2f}, {oz:.2f}) heading {math.degrees(oyaw):.1f} deg -> "
                             f"launch mark ({self.course.launch[0]:.2f}, {self.course.launch[1]:.2f}), "
                             f"yaw offset {math.degrees(yaw_off):+.1f} deg")
        self.pos_map = self._to_map(ox, oy, oz, oyaw)
        self.sp_map = list(self.pos_map)
        self.track_bad_since = None
        self.t_airborne = now
        self._begin_segment(now, out)

    def _begin_segment(self, now, out):
        seg = self._seg()
        self.crossed, self.replans_used, self.seg_t0 = False, 0, now
        self.tunnel_active = self.tunnel_done = False
        if abs(seg.z - self.sp_map[2]) > self.p.climb_tol_m:
            self.climb_z = seg.z
            self._set("climb", now, out, f"to {seg.z:.2f} m at ({self.sp_map[0]:.2f}, {self.sp_map[1]:.2f}) "
                                         f"before {seg.name}")
        else:
            self._plan(now, out)

    def _climb(self, now, dt, out):
        z, target = self.sp_map[2], self.climb_z
        step = (self.p.climb_rate_mps if target > z else self.p.descend_rate_mps) * dt
        self.sp_map[2] = target if abs(target - z) <= step else z + math.copysign(step, target - z)
        if self.sp_map[2] == target and abs(self.pos_map[2] - target) < self.p.climb_tol_m:
            self._plan(now, out)

    def _plan(self, now, out):
        seg = self._seg()
        start = (self.pos_map[0], self.pos_map[1])
        while True:
            self.counts["plans"] += 1
            if seg.kind == "gate":
                path = self.planner.through_gate(start, seg.name, seg.z)
            else:
                path = self.planner.to_point(start, seg.target, seg.z)
            if path.ok:
                break
            if not seg.options:
                self._land(now, out, f"planner: no path to {seg.name}: {path.why}")
                return
            alt = seg.options.pop(0)
            self._event(out, f"planner: no path through {seg.name} ({path.why}); trying {alt}")
            seg.name, seg.cp = alt, self.course.checkpoint(alt)
            seg.tunnel = on_tube(self.course, seg.cp.line, self.p.tube_tol_m)
        if seg.kind == "gate":
            a, b = seg.cp.line
            pts = path.pts
            ci = path.cross_index
            q0, q1 = (pts[ci], pts[ci + 1]) if ci is not None else (pts[0], pts[-1])
            self.cross_sign = 1 if _side(q1, a, b) - _side(q0, a, b) > 0 else -1
        self.follower.set_path(path.pts, now, self.pos_map[:3], self.pos_map[3], speed_mps=self.p.speed_mps)
        self.seg_t0 = now
        length = sum(math.dist(path.pts[i][:2], path.pts[i + 1][:2]) for i in range(len(path.pts) - 1))
        self._set("fly", now, out, f"segment {self.seg_i + 1}/{len(self.segments)} {seg.name}"
                                   f"{' (tunnel)' if seg.tunnel else ''}: {length:.1f} m at z {seg.z:.2f} m")

    def _scale(self, now):
        """Speed scale from tunnel mode, VIO state and depth staleness (0 = hold the carrot)."""
        p = self.p
        if self.tunnel_active:
            return min(1.0, p.tunnel_speed_mps / p.speed_mps), ""
        scale, why = 1.0, ""
        if self.vio_state == "WITHHELD":
            scale, why = 0.0, "VIO WITHHELD"
        elif self.vio_state == "DEGRADED":
            scale = p.vio_degraded_scale
        if p.depth_stale_hover_s > 0 and self._depth_matters() and self._depth_age(now) >= p.depth_stale_hover_s:
            scale, why = 0.0, "no depth points"
        return scale, why

    def _fly(self, now, out):
        p, seg, pm = self.p, self._seg(), self.pos_map
        if seg.tunnel and not self.tunnel_active and not self.tunnel_done and self._in_tube(pm):
            self.tunnel_active, self.t_tunnel = True, now
            self._event(out, f"tunnel enter at ({pm[0]:.2f}, {pm[1]:.2f}): aligner sigma {self.aligner.sigma_xy():.3f} m, "
                             f"depth ignored, {p.tunnel_speed_mps:.2f} m/s")
        if seg.kind == "gate" and self.prev_odom is not None:
            # both ends through the current transform: an aligner correction between the two ticks
            # moves the map, not the vehicle, and must not count as a crossing
            c = crossing(self._to_map(*self.prev_odom)[:2], pm, *seg.cp.line)
            if c == self.cross_sign:
                self.crossed = True
                if all(q["name"] != seg.name for q in self.passed):
                    self.passed.append({"id": seg.cp.id, "name": seg.name, "t": now, "elapsed_s": self._elapsed(now)})
                self._event(out, f"checkpoint {seg.cp.id} {seg.name} crossed at {self._elapsed(now):.1f} s "
                                 f"({pm[0]:.2f}, {pm[1]:.2f})")
                if self.tunnel_active:
                    self.tunnel_active, self.tunnel_done = False, True
                    self._event(out, f"tunnel exit after {now - self.t_tunnel:.1f} s: re-aligning from depth")
            elif c:
                self.crossed = False
                self._event(out, f"{seg.name} crossed in the WRONG direction at ({pm[0]:.2f}, {pm[1]:.2f}): not counted")

        if self.phase == "fly":
            scale, why = self._scale(now)
            if why != self.hold_why:
                self._event(out, f"hold: {why} (depth age {self._depth_age(now):.1f} s, vio {self.vio_state})"
                            if why else f"resume ({self.hold_why} cleared)")
                self.hold_why = why
        else:
            scale = 0.0
        self.speed_scale = scale
        sp = self.follower.step(now, pm[:3], pm[3], scale)
        self.sp_map = [sp.x, sp.y, sp.z, sp.yaw]

        if self.phase == "fly":
            if sp.done and (seg.kind != "gate" or self.crossed):
                if seg.kind == "gate":
                    self.seg_i += 1
                    self._begin_segment(now, out)
                else:
                    self.sp_map[0], self.sp_map[1] = seg.target
                    self._set("descend", now, out, f"over {seg.name}, down to {p.land_descend_z_m:.2f} m")
            elif now - self.seg_t0 > p.segment_timeout_s:
                if self.replans_used < p.replans:
                    self.follower.hold()
                    self._set("hover", now, out, f"{seg.name} timed out after {now - self.seg_t0:.0f} s "
                                                 f"(progress {sp.progress_m:.2f}/{sp.length_m:.2f} m): holding {p.hover_s:.0f} s")
                else:
                    self._land(now, out, f"{seg.name} timed out after {now - self.seg_t0:.0f} s and "
                                         f"{self.replans_used} replan(s)")
        elif self.phase == "hover" and now - self.t_phase >= p.hover_s:
            self.follower.back_off(p.back_off_m)
            self._set("back_off", now, out, f"backing off {p.back_off_m:.2f} m")
        elif self.phase == "back_off" and now - self.t_phase >= p.back_off_s:
            self.replans_used += 1
            self._event(out, f"replan {self.replans_used}/{p.replans} for {seg.name} from ({pm[0]:.2f}, {pm[1]:.2f})")
            self._plan(now, out)

    def _descend(self, now, dt, out):
        target = self.p.land_descend_z_m
        z = self.sp_map[2]
        self.sp_map[2] = max(target, z - self.p.descend_rate_mps * dt)
        if self.sp_map[2] == target and abs(self.pos_map[2] - target) < self.p.climb_tol_m:
            seg = self._seg()
            self._land(now, out, f"on {seg.name}", kind=seg.kind)

    def _landing(self, now, out, v):
        p = self.p
        if v.nav_state == NAV_AUTO_LAND and not self.land_auto_seen:
            self.land_auto_seen = True
            self._event(out, "PX4 AUTO_LAND: setpoint stream stopped")
        if not self.land_auto_seen and self._resend(now):
            out.commands.append("land")
        if self.land_auto_seen and v.armed is False:
            self._landed(now, out)
            return
        if self.land_auto_seen and v.landed:
            self.landed_since = now if self.landed_since is None else self.landed_since
            if now - self.landed_since >= p.disarm_delay_s and self._resend(now):
                out.commands.append("disarm")
        if now - self.t_phase > p.land_timeout_s:
            self._end(now, out, f"aborted: {self.land_reason}; landing not confirmed within {p.land_timeout_s:.0f} s")

    def _landed(self, now, out):
        kind, pm = self.land_kind, self.pos_map
        if kind == "abort":
            self._end(now, out, f"aborted: {self.land_reason}")
            return
        seg = self._seg()
        d = math.hypot(pm[0] - seg.target[0], pm[1] - seg.target[1])
        self.passed.append({"id": seg.cp.id if seg.cp else None, "name": seg.name, "t": now,
                            "elapsed_s": self._elapsed(now)})
        self._event(out, f"landed and disarmed on {seg.name} at {self._elapsed(now):.1f} s, {d:.2f} m from its centre")
        if kind == "pad" and self.p.return_leg and self.stop_request is None and self.seg_i + 1 < len(self.segments):
            self.seg_i += 1
            self._set("ground", now, out, f"waiting {self.p.ground_wait_s:.0f} s before the return leg")
        else:
            self._end(now, out, "complete")

    # ------------------------------------------------------------------ output
    def _setpoint_ned(self, out):
        if self.sp_map is None:
            return tuple(self.sp_ned) if self.sp_ned is not None else None
        x, y, z, yaw = self.sp_map
        if self.leg == "main" and z > self.z_cap:                   # rule 7.7, on the height above the floor
            if not self.clamping:
                self.counts["z_clamps"] += 1
                self._event(out, f"rule 7.7: commanded z {z:.2f} m clamped to {self.z_cap:.2f} m")
            self.clamping = True
            z = self.z_cap
        else:
            self.clamping = False
        xo, yo, zo, yawo = self._to_odom(x, y, z, yaw)
        n, e, d = F.enu_to_ned(xo, yo, zo)
        return (n, e, d, F.yaw_to_heading(yawo))

    def status(self, now) -> dict:
        def r(x, k=3):
            return None if x is None or not math.isfinite(x) else round(float(x), k)
        seg = self._seg() if self.seg_i < len(self.segments) else None
        T = self.aligner.T if self.aligner_ready else None
        bad = self.health.bad_for(now)
        la = self.last_align
        return {
            "phase": self.phase, "leg": self.leg, "why": self.why, "result": self.result,
            "segment": None if seg is None else {"index": self.seg_i, "count": len(self.segments), "name": seg.name,
                                                  "kind": seg.kind, "z": r(seg.z), "tunnel": seg.tunnel},
            "segments": [s.name for s in self.segments],
            "tunnel_active": self.tunnel_active, "gate_crossed": self.crossed, "replans_used": self.replans_used,
            "passed": [{"id": q["id"], "name": q["name"], "t": r(q["t"], 2), "elapsed_s": r(q["elapsed_s"], 2)}
                       for q in self.passed],
            "aligner": None if T is None else {"x": r(T.x), "y": r(T.y), "yaw_deg": r(math.degrees(T.yaw), 2),
                                               "sigma_xy_m": r(self.aligner.sigma_xy()),
                                               "last": None if la is None else {"accepted": bool(la.accepted),
                                                                                "why": la.why, "n": int(la.n)}},
            "vio": {"state": self.vio_state, "lost_s": r(self.vio_lost_s, 2)},
            "speed_scale": r(self.speed_scale, 2), "hold": self.hold_why,
            "depth_age_s": None if self.pts_t is None else r(now - self.pts_t, 2),
            "camera_age_s": None if self.frame_t is None else r(now - self.frame_t, 2),
            "health_bad_s": r(bad, 2),
            "setpoint_map": None if self.sp_map is None else [r(c) for c in self.sp_map],
            "pos_map": None if self.pos_map is None else [r(c) for c in self.pos_map],
            "takeoff_odom": None if self.takeoff_odom is None else [r(c) for c in self.takeoff_odom],
            **self.counts,
        }
