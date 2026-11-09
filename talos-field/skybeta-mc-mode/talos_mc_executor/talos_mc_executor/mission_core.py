"""ROS-free core of the TALOS MC executor.

This is the ROSCon '25 workshop's multicopter Mode / ModeExecutor flow
(``custom_executor_demo``: Takeoff -> CustomWaypoints -> CustomYaw -> Land ->
WaitUntilDisarmed), rebuilt on the skybeta contract:

* we publish ENU pose setpoints; ``skybeta_flight_interface`` clamps them,
  converts them to NED and owns Offboard. Nothing here touches ``/fmu/*``.
* the safety pilot arms, then switches to Position; Offboard starting is
  our "executor activated" signal (the workshop's ``onActivate()``).
* there is no land command, so "Land" becomes: descend to a hand-off height
  and hold for the pilot.

Workshop (px4_ros2, C++)              here
------------------------              ----
ModeBase::updateSetpoint(dt)          Mode.update(ctx, dt) -> (Setpoint, result)
ModeBase::completed(Result)           Mode.update returns SUCCESS / TIMEOUT
ModeExecutorBase::onActivate()        offboard_active goes true (rising edge)
ModeExecutorBase::onDeactivate()      offboard_active goes false -> PAUSED
takeoff()                             TakeoffMode
scheduleMode(CustomWaypoints)         WaypointsMode
scheduleMode(CustomYaw)               YawSpinMode
land()                                DescendMode, then HOLD for the pilot
waitUntilDisarmed()                   HOLD until /skybeta/state armed=false

One deliberate difference: the workshop executor carries on to the next state
even when a step fails. Here a failed (timed-out) step stops the mission and
holds, because on a real vehicle "keep going anyway" is the wrong default.

Kept free of rclpy so it can be unit-tested on any machine.
"""
import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

Vec3 = Tuple[float, float, float]

SUCCESS = 'success'
TIMEOUT = 'timeout'

# Executor states.
WAIT_FOR_OFFBOARD = 'WAIT_FOR_OFFBOARD'
RUNNING = 'RUNNING'
HOLD = 'HOLD'
PAUSED = 'PAUSED'

# Longest dt a mode is ever handed. A stalled timer must not turn into one
# huge carrot step when it resumes.
MAX_DT_S = 0.5

DEFAULT_WAYPOINTS: List[Vec3] = [
    # ENU offsets from the mission start, metres; z = height above the start.
    # A 2 m square at 1.5 m. Size it to the room: the flight interface has no
    # horizontal geofence.
    (2.0, 0.0, 1.5),
    (2.0, 2.0, 1.5),
    (0.0, 2.0, 1.5),
    (0.0, 0.0, 1.5),
]


@dataclass
class Params:
    hover_altitude_m: float = 1.5
    takeoff_settle_s: float = 3.0
    climb_speed_mps: float = 0.5
    cruise_speed_mps: float = 0.5
    descend_speed_mps: float = 0.3
    max_lead_m: float = 1.0
    arrival_radius_m: float = 0.3
    waypoints_enu_m: List[Vec3] = field(
        default_factory=lambda: list(DEFAULT_WAYPOINTS))
    # Off until the odom-vs-PX4 yaw offset has been measured on the bench
    # (competitor guide section 1): an uncorrected offset makes the spin
    # start with a sudden turn of that size.
    yaw_spin_enabled: bool = False
    yaw_rate_dps: float = 10.0
    yaw_max_lead_deg: float = 25.0
    yaw_settle_s: float = 2.0
    handoff_altitude_m: float = 0.5
    step_timeout_s: float = 30.0
    odom_timeout_s: float = 0.5


def parse_waypoints(flat: Sequence[float]) -> List[Vec3]:
    """``[x1, y1, z1, x2, y2, z2, ...]`` -> ``[(x1, y1, z1), ...]``.

    ROS 2 parameters cannot hold a list of tuples, so the YAML stores a flat
    list. Raises ValueError on anything that is not a whole number of
    finite triples.
    """
    values = [float(v) for v in flat]
    if not values or len(values) % 3 != 0:
        raise ValueError(
            f'waypoints_enu_m needs x, y, z triples; got {len(values)} numbers')
    if not all(math.isfinite(v) for v in values):
        raise ValueError('waypoints_enu_m contains NaN or inf')
    return [tuple(values[i:i + 3]) for i in range(0, len(values), 3)]


# Sanity limits. The heights are the flight interface's default envelope
# (min_altitude_m / max_altitude_m, measured from the odometry origin, which
# is the start point if the drone has not been moved since boot). If you
# tighten max_altitude_m in the launch file, keep the mission below it too:
# a clamped target is never reached and ends in a timeout and a hold.
MIN_HEIGHT_M = 0.3
MAX_HEIGHT_M = 3.0
MAX_SPEED_MPS = 1.5     # the interface's own twist cap
MAX_LEAD_M = 2.0


def validate(params: Params) -> List[str]:
    """Every reason these parameters are unsafe to fly. Empty means OK."""
    errors = []

    def check(name, low, high, low_inclusive=True):
        value = getattr(params, name)
        above = value >= low if low_inclusive else value > low
        if not (math.isfinite(value) and above and value <= high):
            bracket = '[' if low_inclusive else '('
            errors.append(f'{name} must be in {bracket}{low}, {high}] '
                          f'(got {value})')

    for name in ('climb_speed_mps', 'cruise_speed_mps', 'descend_speed_mps'):
        check(name, 0.0, MAX_SPEED_MPS, low_inclusive=False)
    check('max_lead_m', 0.0, MAX_LEAD_M, low_inclusive=False)
    check('arrival_radius_m', 0.0, 1.0, low_inclusive=False)
    check('hover_altitude_m', MIN_HEIGHT_M, MAX_HEIGHT_M)
    check('handoff_altitude_m', MIN_HEIGHT_M, MAX_HEIGHT_M)
    check('yaw_rate_dps', 0.0, 45.0, low_inclusive=False)
    check('yaw_max_lead_deg', 0.0, 90.0, low_inclusive=False)
    check('step_timeout_s', 0.0, 600.0, low_inclusive=False)
    check('odom_timeout_s', 0.0, 5.0, low_inclusive=False)
    check('takeoff_settle_s', 0.0, 60.0)
    check('yaw_settle_s', 0.0, 60.0)
    if not params.waypoints_enu_m:
        errors.append('waypoints_enu_m is empty')
    for i, (_, _, z) in enumerate(params.waypoints_enu_m):
        if not MIN_HEIGHT_M <= z <= MAX_HEIGHT_M:
            errors.append(
                f'waypoint {i + 1} has height {z} m: heights are above the '
                f'start point and must be in [{MIN_HEIGHT_M}, {MAX_HEIGHT_M}]')
    return errors


# ----------------------------------------------------------------------
# geometry
# ----------------------------------------------------------------------
def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_xyzw(x: float, y: float, z: float, w: float) -> float:
    """ENU yaw of a ROS (x, y, z, w) quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def advance_carrot(carrot: Vec3, target: Vec3, vehicle: Optional[Vec3],
                   speed: float, dt: float, max_lead: float) -> Vec3:
    """Move the commanded point ("carrot") towards ``target``.

    The flight interface does not speed-limit pose setpoints, and PX4 flies
    a far-away pose at up to MPC_XY_VEL_MAX (12 m/s on these airframes). So
    instead of sending the waypoint itself we slide a carrot along the
    straight line at ``speed``, and stop sliding while it is ``max_lead``
    or more ahead of the vehicle. The vehicle therefore never sees a
    setpoint much more than ``max_lead`` away.
    """
    delta = tuple(t - c for t, c in zip(target, carrot))
    dist = math.sqrt(sum(d * d for d in delta))
    if dist < 1e-6:
        return tuple(target)
    # Written as "not <" so a NaN distance also stops the carrot.
    if vehicle is not None and not math.dist(carrot, vehicle) < max_lead:
        return carrot
    step = speed * dt
    if step >= dist:
        return tuple(target)
    return tuple(c + d * step / dist for c, d in zip(carrot, delta))


# ----------------------------------------------------------------------
# what the executor sees and produces
# ----------------------------------------------------------------------
@dataclass
class Inputs:
    now_s: float
    position: Optional[Vec3]       # ENU, odom frame, from cuVSLAM
    yaw: Optional[float]           # ENU yaw, rad, from cuVSLAM
    odom_age_s: float              # seconds since the last odometry message
    state_received: bool           # at least one /skybeta/state seen
    px4_link: bool = False
    armed: bool = False
    offboard_active: bool = False


@dataclass
class Setpoint:
    position: Vec3
    yaw: Optional[float] = None    # ENU rad; None = do not command yaw


@dataclass
class Context:
    """Shared by every mode: the "base class" inputs, in one place."""
    params: Params
    log: Callable[[str, str], None]
    inputs: Optional[Inputs] = None
    start: Vec3 = (0.0, 0.0, 0.0)      # ENU position when Offboard began
    carrot: Vec3 = (0.0, 0.0, 0.0)     # last commanded ENU position


# ----------------------------------------------------------------------
# modes: what to do (the workshop's px4_ros2::ModeBase subclasses)
# ----------------------------------------------------------------------
class Mode:
    name = 'Mode'

    def on_activate(self, ctx: Context) -> None:
        self.activated_at = ctx.inputs.now_s

    def update(self, ctx: Context, dt: float):
        raise NotImplementedError

    def _arrived(self, ctx: Context, target: Vec3) -> bool:
        return (ctx.carrot == tuple(target) and
                math.dist(ctx.inputs.position, target) <=
                ctx.params.arrival_radius_m)


class TakeoffMode(Mode):
    """Workshop ``takeoff()``: climb above the start point, then settle."""

    name = 'TAKEOFF'

    def on_activate(self, ctx):
        super().on_activate(ctx)
        x, y, z = ctx.start
        self.target = (x, y, z + ctx.params.hover_altitude_m)
        self.settled_since = None
        self.timeout_s = (ctx.params.step_timeout_s +
                          ctx.params.hover_altitude_m /
                          ctx.params.climb_speed_mps)
        ctx.log('info', f'takeoff: climbing to {self.target[2]:.2f} m ENU z')

    def update(self, ctx, dt):
        p = ctx.params
        ctx.carrot = advance_carrot(ctx.carrot, self.target,
                                    ctx.inputs.position, p.climb_speed_mps,
                                    dt, p.max_lead_m)
        now = ctx.inputs.now_s
        if self._arrived(ctx, self.target):
            # The settle clock only runs while we are actually there, like
            # square_mission's hover clock.
            if self.settled_since is None:
                self.settled_since = now
            if now - self.settled_since >= p.takeoff_settle_s:
                return Setpoint(ctx.carrot), SUCCESS
        else:
            self.settled_since = None
            if now - self.activated_at > self.timeout_s:
                return Setpoint(ctx.carrot), TIMEOUT
        return Setpoint(ctx.carrot), None


class WaypointsMode(Mode):
    """Workshop ``CustomWaypoints``: visit each waypoint in order."""

    name = 'WAYPOINTS'

    def on_activate(self, ctx):
        super().on_activate(ctx)
        sx, sy, sz = ctx.start
        self.points = [(sx + x, sy + y, sz + z)
                       for x, y, z in ctx.params.waypoints_enu_m]
        self.index = 0
        self._start_leg(ctx)

    def _start_leg(self, ctx):
        target = self.points[self.index]
        self.leg_started = ctx.inputs.now_s
        # Allow for the leg's length at cruise speed, plus the fixed margin.
        self.leg_timeout_s = (ctx.params.step_timeout_s +
                              math.dist(ctx.carrot, target) /
                              ctx.params.cruise_speed_mps)
        ctx.log('info',
                f'waypoint {self.index + 1}/{len(self.points)}: ENU '
                f'({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f})')

    def update(self, ctx, dt):
        p = ctx.params
        target = self.points[self.index]
        ctx.carrot = advance_carrot(ctx.carrot, target, ctx.inputs.position,
                                    p.cruise_speed_mps, dt, p.max_lead_m)
        if self._arrived(ctx, target):
            self.index += 1
            if self.index >= len(self.points):
                return Setpoint(ctx.carrot), SUCCESS
            self._start_leg(ctx)
        elif ctx.inputs.now_s - self.leg_started > self.leg_timeout_s:
            ctx.log('error',
                    f'waypoint {self.index + 1} not reached in '
                    f'{self.leg_timeout_s:.0f} s (still '
                    f'{math.dist(ctx.inputs.position, target):.2f} m away). '
                    'Check "clamped" in /skybeta/state and cuVSLAM tracking.')
            return Setpoint(ctx.carrot), TIMEOUT
        return Setpoint(ctx.carrot), None


class YawSpinMode(Mode):
    """Workshop ``CustomYaw``: one full turn in place, slowly.

    Holds the current carrot position and steps the commanded heading at
    ``yaw_rate_dps``. Slow on purpose: cuVSLAM is stereo-only, and a fast
    spin blurs the images it tracks.

    Closed loop, like the position carrot: the commanded heading only
    advances while the measured (odom) heading is within
    ``yaw_max_lead_deg`` of it. A vehicle that cannot follow, or a large
    odom-vs-PX4 yaw offset, stalls the spin into a timeout and a hold
    instead of running away.
    """

    name = 'YAW_SPIN'

    def on_activate(self, ctx):
        super().on_activate(ctx)
        self.hold = ctx.carrot
        self.yaw0 = ctx.inputs.yaw if ctx.inputs.yaw is not None else 0.0
        self.turned = 0.0
        self.done_since = None
        self.last_following = ctx.inputs.now_s
        # Stall timeout: step_timeout_s without the heading following.
        # Overall cap: three times the nominal spin time, plus the margin.
        rate = math.radians(ctx.params.yaw_rate_dps)
        self.timeout_s = ctx.params.step_timeout_s + 6.0 * math.pi / rate
        ctx.log('info',
                f'yaw spin: 360 deg at {ctx.params.yaw_rate_dps:.0f} deg/s '
                f'from ENU yaw {math.degrees(self.yaw0):.0f} deg')

    def _following(self, ctx) -> bool:
        measured = ctx.inputs.yaw
        if measured is None or not math.isfinite(measured):
            return False
        command = wrap_pi(self.yaw0 + self.turned)
        return (abs(wrap_pi(command - measured)) <
                math.radians(ctx.params.yaw_max_lead_deg))

    def update(self, ctx, dt):
        p = ctx.params
        following = self._following(ctx)
        if following and self.turned < 2.0 * math.pi:
            self.turned = min(2.0 * math.pi,
                              self.turned + math.radians(p.yaw_rate_dps) * dt)
        sp = Setpoint(self.hold, wrap_pi(self.yaw0 + self.turned))
        now = ctx.inputs.now_s
        if following:
            self.last_following = now
        if self.turned >= 2.0 * math.pi and following:
            if self.done_since is None:
                self.done_since = now
            if now - self.done_since >= p.yaw_settle_s:
                return sp, SUCCESS
        else:
            self.done_since = None
        if (now - self.last_following > p.step_timeout_s or
                now - self.activated_at > self.timeout_s):
            ctx.log('error',
                    f'yaw spin stalled at {math.degrees(self.turned):.0f} deg: '
                    'the measured heading is not following. Check the '
                    'odom-vs-PX4 yaw offset (competitor guide section 1).')
            return sp, TIMEOUT
        return sp, None


class DescendMode(Mode):
    """Workshop ``land()``, as far as skybeta allows.

    The flight interface has no land command and the envelope floors every
    pose at ``min_altitude_m``, so this descends to the hand-off height and
    then the executor holds there for the safety pilot to land.
    """

    name = 'DESCEND'

    def on_activate(self, ctx):
        super().on_activate(ctx)
        x, y, _ = ctx.carrot
        self.target = (x, y, ctx.start[2] + ctx.params.handoff_altitude_m)
        self.timeout_s = (ctx.params.step_timeout_s +
                          abs(ctx.carrot[2] - self.target[2]) /
                          ctx.params.descend_speed_mps)
        ctx.log('info',
                f'descend: to {ctx.params.handoff_altitude_m:.2f} m above the '
                'start, then hold for the pilot to land')

    def update(self, ctx, dt):
        p = ctx.params
        ctx.carrot = advance_carrot(ctx.carrot, self.target,
                                    ctx.inputs.position, p.descend_speed_mps,
                                    dt, p.max_lead_m)
        if self._arrived(ctx, self.target):
            return Setpoint(ctx.carrot), SUCCESS
        if ctx.inputs.now_s - self.activated_at > self.timeout_s:
            return Setpoint(ctx.carrot), TIMEOUT
        return Setpoint(ctx.carrot), None


def build_plan(params: Params) -> List[Mode]:
    """The executor's sequence (the workshop's switchToState chain)."""
    plan: List[Mode] = [TakeoffMode(), WaypointsMode()]
    if params.yaw_spin_enabled:
        plan.append(YawSpinMode())
    plan.append(DescendMode())
    return plan


# ----------------------------------------------------------------------
# the executor: when to do it (the workshop's ModeExecutorBase subclass)
# ----------------------------------------------------------------------
class MissionExecutor:
    """Sequences the modes. Call ``step()`` at a fixed rate (10 Hz).

    ``step()`` returns the setpoint to publish, or None to publish nothing.
    """

    def __init__(self, params: Params,
                 log: Callable[[str, str], None] = lambda level, msg: None):
        self.params = params
        self._log = log
        self._logged_once = set()
        self.ctx = Context(params=params, log=log)
        self.transitions: List[str] = []   # history, for logs and tests
        self._reset()

    # -- bookkeeping ---------------------------------------------------
    def _reset(self):
        self.state = WAIT_FOR_OFFBOARD
        self.plan = build_plan(self.params)
        self.mode_index = 0
        self.last_now = None
        self.last_setpoint: Optional[Setpoint] = None
        # A mission starts only on Offboard going false -> true while this
        # executor watches. Cleared on every reset, so each flight needs a
        # fresh "not in Offboard" observation first.
        self.offboard_seen_false = False
        self._logged_once.discard('waiting')
        self._record(WAIT_FOR_OFFBOARD)

    def _out(self, setpoint: Setpoint) -> Setpoint:
        self.last_setpoint = setpoint
        return setpoint

    def _record(self, name):
        if not self.transitions or self.transitions[-1] != name:
            self.transitions.append(name)

    def _once(self, key, level, message):
        if key not in self._logged_once:
            self._logged_once.add(key)
            self._log(level, message)

    @property
    def mode(self) -> Optional[Mode]:
        if self.state == RUNNING:
            return self.plan[self.mode_index]
        return None

    def _activate_mode(self, index):
        self.mode_index = index
        mode = self.plan[index]
        mode.on_activate(self.ctx)
        self._record(mode.name)

    def _hold(self, reason, level='info'):
        self.state = HOLD
        self._record(HOLD)
        self._log(level, f'{reason} -- holding. Safety pilot: raise the '
                         'throttle towards centre to take over, then land.')

    # -- the 10 Hz tick ------------------------------------------------
    def step(self, inp: Inputs) -> Optional[Setpoint]:
        self.ctx.inputs = inp
        dt = 0.0 if self.last_now is None else inp.now_s - self.last_now
        dt = min(max(dt, 0.0), MAX_DT_S)
        self.last_now = inp.now_s

        if inp.position is None or not all(
                math.isfinite(v) for v in inp.position):
            self._once('no_odom', 'warn',
                       'no usable /visual_slam/tracking/odometry -- is '
                       'cuVSLAM up and tracking? Nothing new is commanded '
                       'until it is.')
            return None
        self._logged_once.discard('no_odom')
        if not inp.state_received:
            # Until we know (again) whether PX4 is in Offboard, publish
            # nothing: the interface keeps holding its last pose.
            self._once('no_state', 'warn',
                       'no fresh /skybeta/state -- is flight_interface '
                       'running? Nothing new is commanded until it is.')
            return None
        self._logged_once.discard('no_state')

        stale = inp.odom_age_s > self.params.odom_timeout_s
        if stale:
            self._once('stale_odom', 'warn',
                       f'odometry is {inp.odom_age_s:.1f} s old -- mission '
                       'frozen until cuVSLAM recovers')
        else:
            self._logged_once.discard('stale_odom')

        # waitUntilDisarmed(): a disarm ends this mission whatever it was
        # doing. The next arm-then-Position starts a fresh one.
        if self.state != WAIT_FOR_OFFBOARD and inp.px4_link and not inp.armed:
            self._log('info', 'disarmed -- mission reset, waiting for the '
                              'next arm, then Position')
            self._reset()

        if self.state == WAIT_FOR_OFFBOARD:
            if not inp.offboard_active:
                self.offboard_seen_false = True
            if stale:
                # Never capture a start point from old data. Publishing
                # nothing also means the interface will not enter Offboard
                # (it needs a setpoint from us in the last second).
                return None
            if not inp.offboard_active:
                self._once('waiting', 'info',
                           'waiting for the safety pilot to arm, then '
                           'switch to Position')
                # Command where we already are, so Offboard starts with a
                # hold, and so the interface sees a live setpoint stream.
                return self._out(Setpoint(inp.position))
            if not inp.armed:
                # Inconsistent or transitional state: wait for it to settle.
                return self._out(Setpoint(inp.position))
            if not self.offboard_seen_false:
                # We started (or restarted) while already in Offboard,
                # probably in the air. Never start a mission from there.
                self.ctx.carrot = inp.position
                self.ctx.start = inp.position
                self._hold('Offboard was already active when this node '
                           'started; not starting a mission', 'warn')
                return self._out(Setpoint(self.ctx.carrot))
            # onActivate(): Offboard just started.
            self.ctx.start = inp.position
            self.ctx.carrot = inp.position
            self.state = RUNNING
            self._log('info',
                      'Offboard active -- mission start at ENU '
                      f'({inp.position[0]:.2f}, {inp.position[1]:.2f}, '
                      f'{inp.position[2]:.2f})')
            self._activate_mode(0)

        if self.state in (RUNNING, HOLD) and not inp.offboard_active:
            # onDeactivate(): the pilot took over (or a failsafe did).
            # Offboard does not come back until disarm and re-arm.
            self.state = PAUSED
            self._record(PAUSED)
            self._log('warn', 'Offboard lost -- the pilot has control. '
                              'Mission stopped until disarm and re-arm.')

        if self.state == PAUSED:
            # PX4 ignores setpoints outside Offboard; keep the stream live.
            return None if stale else Setpoint(inp.position)

        if stale:
            # Do not advance anything; repeat the last command exactly
            # (including its yaw).
            return self.last_setpoint or Setpoint(self.ctx.carrot)

        if self.state == HOLD:
            return self._out(Setpoint(self.ctx.carrot))

        setpoint, result = self.mode.update(self.ctx, dt)
        if result == SUCCESS:
            self._log('info', f'{self.mode.name} complete')
            if self.mode_index + 1 < len(self.plan):
                self._activate_mode(self.mode_index + 1)
            else:
                self._hold('mission complete')
        elif result == TIMEOUT:
            # Hold where the vehicle actually is, not at the carrot, which
            # may be up to max_lead_m further towards whatever blocked us.
            self.ctx.carrot = inp.position
            self._hold(f'{self.mode.name} timed out', 'error')
            setpoint = Setpoint(self.ctx.carrot)
        return self._out(setpoint)
