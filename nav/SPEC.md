# nav/ interface spec (implements ../ARCHITECTURE.md)

Every module in `nav/` except `node.py` is **pure**: numpy + stdlib (+ `yaml` in `course.py`),
no ROS, no clock reads, no file writes. Time always arrives as an argument (`now`, seconds, one
clock). Python 3.10 syntax (lab is 3.12, local sim is 3.10). No scipy.

**No course coordinates in code** (rule 9.5). Every position comes from `nav.course.Course`
(loaded from `course_map.yaml`) or from parameters. Tests may build small synthetic courses.

Already written (read them, do not change their API without asking the lead):
- `nav/course.py`: `Course`, `Box`, `Cylinder`, `Checkpoint`, `load()`, `default_map_path()`,
  `Grid`, `occupancy(course, z0, z1, inflate, res=0.05)`. `course.faces_at(z)` = wall faces
  (segments) at height z. `course.checkpoint(name)`.
- `nav/frames.py`: `SE2` (p_map = T.apply(p_odom)), `ned_to_enu`, `enu_to_ned`,
  `heading_to_yaw`, `yaw_to_heading`, `wrap`.

Frames: **map** (course ENU), **odom** (PX4 local frame expressed in ENU, origin at PX4's local
origin), **PX4** (NED). The aligner owns `T_map_odom: SE2`. Map corrections never go into PX4:
targets are converted map → odom → NED right before sending.

Existing code to reuse, not copy-paste: `refnav/core.py` (`Vehicle`, `Output`, `Health`, arming /
landing / yield logic), `refnav/depth.py` (`decode_depth`, `level_points`, `quat_to_mat`,
`level_rotation`). `refnav/` itself is not edited.

---

## planner.py

```python
@dataclass
class PlannerParams:
    res: float = 0.05
    drone_span_m: float = 0.40     # tip-to-tip; half of it is inflated
    tilt_margin_m: float = 0.05
    loc_margin_m: float = 0.08     # localization error allowance
    z_band_m: float = 0.25         # obstacles within z +- this count at a segment altitude
    gate_lead_m: float = 0.35      # entry/exit points this far before/after a gate line
    min_inflate_m: float = 0.15    # if no path, retry with inflation shrinking down to this

@dataclass
class Path:
    pts: list            # [(x, y, z)] map frame, first = start, polyline after shortcutting
    ok: bool
    why: str = ""        # on failure: which gate / which gap closed, with numbers
    gate: str | None = None          # checkpoint name this path crosses (None for a plain goal)
    cross_index: int | None = None   # pts[cross_index] -> pts[cross_index+1] crosses the gate line
    inflate_m: float = 0.0           # inflation actually used

class Planner:
    def __init__(self, course, params=PlannerParams()): ...
    def through_gate(self, start_xy, gate_name, z) -> Path
        # A* (8-connected, 5 cm inflated grid at band [z-z_band, z+z_band]) from start to the
        # gate's ENTRY point, then straight to its EXIT point. Entry/exit = centre of the FREE part
        # of the gate line (in the inflated grid) -/+ gate_lead_m along the line normal; the
        # crossing direction is "away from the side start is on". Window: z must lie inside the
        # opening (checkpoint.zmin/zmax), else ok=False with why.
    def to_point(self, start_xy, goal_xy, z) -> Path
    # Both: string-pull (line-of-sight shortcut on the grid) the A* cells into a short polyline;
    # every polyline edge must stay in free cells. If A* fails at full inflation, retry with less
    # (down to min_inflate_m) and report inflate_m; if it still fails, ok=False and `why` names
    # the gate and the narrowest blocked gap.
    def inflate_m(self) -> float  # span/2 + tilt + loc
```

## follower.py

```python
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
    corner_stop_deg: float = 45.0  # stop at vertices turning more than this, hold corner_hold_s
    corner_hold_s: float = 1.0

@dataclass
class Setpoint:
    x: float; y: float; z: float; yaw: float   # map frame, yaw ENU
    progress_m: float      # vehicle progress along the path
    carrot_m: float        # carrot arclength
    length_m: float
    cross_track_m: float
    speed_mps: float       # carrot speed this step
    done: bool             # carrot at the end AND vehicle within arrive_m of the end

class Follower:
    def __init__(self, params=FollowerParams()): ...
    def set_path(self, pts, now, pos_xyz, yaw, speed_mps=None) -> None  # starts at vehicle progress
    def step(self, now, pos_xyz, yaw, speed_scale=1.0) -> Setpoint
        # speed_scale in [0, 1] from the VIO state: 0 = hold the carrot.
        # Carrot speed: accel-limited ramp, <= speed_mps * speed_scale, slowed for the heading change
        # within lookahead_m ahead. Yaw setpoint rate-limited toward the tangent at the carrot.
    def hold(self) -> None    # freeze the carrot (mission hover)
    def back_off(self, dist_m) -> None  # move the carrot back along the path by dist_m
```

## landmarks.py

```python
@dataclass
class LandmarkParams:
    stride: int = 4; border_px: int = 16; rmin: float = 0.25; rmax: float = 4.0
    band_m: float = 0.20     # keep points within z_drone +- band (level frame)
    cell_m: float = 0.05     # voxel-downsample to one point per cell
    min_valid_frac: float = 0.2

def wall_points(depth_m, K, R_body_cam, t_body_cam, q_att_wxyz, pose_odom, params) -> np.ndarray | None
    # depth_m HxW metres (NaN/0 = no return), K=(fx,fy,cx,cy), R/t camera->base_link (FLU body,
    # from TF), q_att_wxyz = PX4 vehicle_attitude.q (FRD body -> NED). pose_odom = (x, y, z, yaw)
    # in odom ENU. Returns Nx2 odom-frame xy points of surfaces at the drone's height, or None
    # if the frame is unusable (too few valid pixels). Reuse refnav.depth.level_points: read it
    # for its frame convention and convert carefully; unit-test the full chain with a synthetic
    # depth image of a known wall.
```

## aligner.py

```python
@dataclass
class AlignerParams:
    init_sigma_xy_m: float = 0.10; init_sigma_yaw_deg: float = 3.0
    drift_per_m: float = 0.02          # process noise: sigma grows this much per metre flown
    drift_yaw_deg_per_m: float = 0.3
    meas_sigma_m: float = 0.05         # per-point point-to-face noise
    match_max_m: float = 0.25          # point-to-face association gate
    min_points: int = 30
    max_step_xy_m: float = 0.30        # reject an update that moves T more than this
    max_step_yaw_deg: float = 8.0
    max_points: int = 300              # subsample per update
    icp_iters: int = 5

@dataclass
class AlignResult:
    accepted: bool; why: str; n: int; dx: float; dy: float; dyaw: float

class Aligner:
    def __init__(self, course, params=AlignerParams()): ...
    def init_at_launch(self, odom_xy, launch_xy, yaw_offset=0.0) -> None
        # T such that odom_xy (vehicle position at takeoff) maps to launch_xy; rotation yaw_offset.
    def predict(self, dist_m) -> None          # grow covariance for distance flown
    def update(self, points_odom_xy, z, now) -> AlignResult
        # 6.1: EKF over (tx, ty, yaw). Point-to-face ICP against course.faces_at(z): the
        # measurement Jacobian makes only observable directions move (straight corridor: only
        # cross-corridor + yaw). Gate on match_max_m, min_points, max_step_*. Never touches VIO.
    T: SE2                                     # current map <- odom
    cov: np.ndarray                            # 3x3
    def sigma_xy(self) -> float                # sqrt of the larger xy eigenvalue
    def to_map(self, x, y, z, yaw) -> tuple    # odom -> map
    def to_odom(self, x, y, z, yaw) -> tuple   # map -> odom
```

## avoidance.py

Removed (team decision 6, 2026-11-06: no safety layer for now).

## eventlog.py

```python
class EventLog:            # the ONLY nav module that writes files (logger component)
    def __init__(self, directory, node_name): ...   # <dir>/<node>_<YYYYmmdd-HHMMSS>.jsonl
    def event(self, t, kind, **fields) -> None      # one JSON line, flushed
    def close(self) -> None
```
Plus `tools/record.sh`: `ros2 bag record -a` into `logs/bags/<stamp>` (development only, rule 6.4:
bags are never read by flight code).

## mission.py (state machine on the refnav pattern)

```python
@dataclass
class MissionParams:
    cruise_z_m: float = 0.80
    return_clearance_m: float = 0.50   # return altitude = wall_top + this (rule 7.10 exception)
    chicane: str = "window"            # "window" | "chicane_gate_b" style: name of a checkpoint
    chicane_fallback: str = ""         # if planning through `chicane` fails, try this one
    return_leg: bool = True
    speed_mps: float = 0.5
    tunnel_speed_mps: float = 0.3
    segment_timeout_s: float = 60.0
    back_off_m: float = 0.4
    replans: int = 1
    land_descend_z_m: float = 0.30     # descend to this over the target, then PX4 land
    ground_wait_s: float = 2.0         # on the pad after disarm before the return takeoff
    vio_degraded_scale: float = 0.5
    depth_stale_hover_s: float = 1.0   # outside tunnel mode
    # + arming / health params as in refnav.core.Params (reuse its Health)

class Mission:
    def __init__(self, course, planner, follower, aligner, params): ...
    def on_health(self, t, ok) -> None
    def on_vio_state(self, t, state: str, lost_s: float) -> None   # "OK" | "DEGRADED" | "WITHHELD"
    def on_points(self, t, points_odom_xy, z) -> None   # depth wall points -> aligner
    def request_land(self, why) -> None
    def step(self, now, v: refnav.core.Vehicle) -> refnav.core.Output   # setpoint NED (x,y,z,heading)
    def status(self, now) -> dict
    phase: str; result: str | None; events (in Output.events): str lines
```
Segments are built from the course's checkpoints in id order (one per id; for the chicane id the
`chicane` param picks which), then the landing circle, then (if return_leg) the launch mark at
return altitude. Each segment: goal, completion condition (gate crossed in the right direction by
the map-frame track / within arrive of the target), timeout, failure action
hover → back off → replan → land. The tunnel segment (the one whose gate lies on the course's
`tube` box) runs tunnel mode: slow, straight, commit, depth-staleness ignored,
re-align after exit.
The follower stops at sharp corners by itself (corner_stop_deg, corner_hold_s) so the aligner
corrects while the drone is still. PX4 owns every landing (AUTO_LAND, stop streaming), and the return leg
re-arms by itself after the pad disarm.
