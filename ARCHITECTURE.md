# DroneHacks system architecture

Last updated 2026-11-06. Decisions behind changes are in `decision-1.md` (VIO loss),
`decision-2.md` (doc revision) and `decision-3.md` (implementation, `workspace/nav/`).

## Core idea

Localization is the hard part, not planning or control:

- Takeoff heading error hurts most.
- VIO drifts about 1% of distance (about 0.16 m over the 16 m route).
- In the dark tunnel, VIO and stereo depth both fail: the OAK-D S2 has no IR projector, and
  rule 3.4 bans adding lights.

Plan: keep the stock VIO and correct it against the published map.

## Course and route

Map frame: +x east, +y north, metres (`course/course_map.yaml`).

1. Launch (-2.51, 0.91) facing north, up the first corridor → **CP1** (y > 2.12)
2. Through the M1/M2/M3 maze corridors, east through Gate A → **CP2**
3. South through the dark tunnel (x 1.57 to 3.05, about 1.5 m) → **CP3** (y < -0.30)
4. Chicane, either Gate B or the window (centre (-0.47, -2.27), 0.91 × 0.91, 0.30 to 1.22 m
   high, +5) → **CP4**
5. Land on the pad (centre (-2.04, -0.79)), disarm → **CP5**
6. Return: take off, fly over the barrier, land within 2 ft of the launch mark, disarm

- Narrowest gaps are about 1 m. That leaves about 0.3 m per side for the 0.4 m drone and about
  0.19 m for the 0.63 m sim x500.
- Cruising at about 0.8 m needs no height change before the window.
- Gate A, Gate B and the checkpoint lines are **estimates** from the video. Confirm them against
  the official top view.

## Sensors

| Sensor | Mount | Used by |
|---|---|---|
| OAK-D S2 stereo + IMU | Facing straight ahead | cuVSLAM, landmark detector (depth) |
| VL53L1X range | Facing down | PX4 EKF2 directly (lab: `/fmu/out/distance_sensor`, 0–4 m) |
| A010 ToF | Add-on, not mounted yet | Tunnel only, see 6.4 |

The OAK-D faces forward because VIO features and the walls, gates and window are all ahead.
The pad is found from the map plus downward range, not from the camera.

## Data flow

```
OAK-D stereo + IMU ──► cuVSLAM ──► ev_bridge ──► PX4 EKF2 ◄── downward range
                                                     │ local position (VIO frame)
OAK-D depth ──► landmark detector ──► map aligner ──► map → VIO transform
course_map.yaml ──► planner ──► path ──► mission + path follower ──► setpoints 20 Hz ──► PX4
every node ──► logger
```

### Key rule: map corrections never go into PX4

PX4 only sees the VIO frame (smooth, but drifts). The map aligner tracks the offset between the
VIO frame and the map frame (shift + yaw). Map targets are converted to the VIO frame before being
sent to PX4, so an aligner update just moves the target slightly (the ROS `map → odom` pattern).

Pushing corrections through `ev_bridge` would make PX4's estimate jump and risk failsafe, and
rule 8.5 bans changing failsafe settings.

### VIO loss (decided, `decision-1.md`)

The top priority is that PX4 never triggers failsafe. `ev_bridge` keeps sending VIO with a larger
covariance rather than going silent:

- **OK**: send as is.
- **DEGRADED** (fewer than 30 tracked features, or a bad sample sent after the withhold limit):
  send, with σ growing 0.25 m/s up to 0.5 m (position and velocity).
- **WITHHELD** (jump > 1 m or speed > 5 m/s): stop sending for at most 2 s, then send at the
  σ cap with a reset. That is less than half of `EKF2_NOAID_TOUT` (5 s).

State is published on `/estimator/vio_state` for the mission to act on (hover, back off, land).
The σ cap and recovery behavior will be tuned from the lab tunnel test.

## Interfaces

Measured in the lab (`workspace/lab_info/topics.txt`). All names are parameters, not hard-coded,
because the lab (x86 + Jazzy) and the Jetson (ARM + Humble) may differ.

| Topic | Type | Direction |
|---|---|---|
| `/stereo/left/image`, `/stereo/right/image`, `/stereo/imu` | `sensor_msgs` | camera → cuVSLAM |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | camera → landmark detector |
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | cuVSLAM → ev_bridge |
| `/visual_slam/vis/observations_count` | `std_msgs/Int32MultiArray` | cuVSLAM → ev_bridge |
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` | ev_bridge → PX4 |
| `/estimator/healthy`, `/estimator/vio_state` | `Bool`, `DiagnosticStatus` | ev_bridge → mission |
| `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command` | `px4_msgs` | mission → PX4 |
| `/fmu/out/vehicle_local_position_v1`, `/fmu/out/vehicle_status_v1` | `px4_msgs` | PX4 → mission |

- QoS: PX4 topics use `PX4_QOS` (best effort), sensors use `qos_profile_sensor_data`
  (`onboard/vio/ros_common.py`).
- Frames: ROS side is ENU/FLU, PX4 side is NED/FRD. Convert only with the existing helpers
  (`onboard/vio/frames.py` for vision input, `onboard/adapters/` for PX4 output); new nodes
  reuse them.
- Time: `use_sim_time` in sim and lab, system clock on hardware.

## Components

| Component | Status | Job |
|---|---|---|
| VIO (cuVSLAM) | Keep | Tune `vio.yaml` only |
| `ev_bridge.py`, `frames.py` | Keep | VIO-loss behavior changed (`decision-1.md`), nothing else |
| Landmark detector | New | Walls, gates, window, pad from depth |
| Map aligner | New | map → VIO transform + confidence |
| Planner | New | Inflated 5 cm 2D grid + A* through the required gates, per-segment altitude |
| Path follower | New | 20 Hz lookahead, stops at sharp corners so the aligner corrects while still; outputs VIO-frame setpoints |
| Mission | New | State machine on the `refnav/node.py` loop |
| Logger | New | rosbag2 + per-node event log, from day one |

Also keep: `refnav/core.py` (reference only), `tests/`, `course/course_map.yaml` (the only source
of course geometry), `a010/` driver (add-on only).

### Rules per component

- **Aligner**: start by matching the launch mark to the estimator's takeoff pose (don't assume
  VIO starts at zero). Update shift and yaw from each matched landmark. Reject and log updates
  that are too large. Never reset VIO.
- **Planner**: plan in 2D at cruise height (about 0.8 m); each segment carries its own altitude
  (window height, climb over the barrier on the return). Inflate obstacles by half the drone
  span + tilt + localization error. Enforce gate order and crossing direction. If A* finds no
  path, log which gap closed.
- **Follower**: compare positions in the map frame. Search progress only near the last progress,
  so parallel corridors can't make it skip ahead. Stop at sharp corners (> 45°) for about 1 s so
  the aligner corrects while still. Limit speed, acceleration and yaw rate.
- **Mission**: each segment has a goal, completion condition, timeout and failure action
  (hover → back off → land). A gate counts only if the track crossed its opening in the right
  direction. Build the return leg (×1.25) from day one. Window vs Gate B and conservative vs
  aggressive speed are parameters. No hard-coded waypoints (rule 9.5). React to
  `/estimator/vio_state`.
- **Tunnel mode**: align before entry, fly slowly and straight on IMU only with downward range
  for height (about 3 s), re-align against the map after exit. Expect the estimate to jump on
  VIO recovery; the aligner absorbs it.
- **Logging**: record every topic on every flight, in the same format in sim and lab, so lab runs
  can be replayed against sim. Bags stay in development and are never fed to a later run
  (rule 6.4).

## Advanced techniques

Only after the basic version is stable. Ordered by payoff.

1. **6.1 Probabilistic alignment** (most worth doing): EKF over the transform, so only observable
   directions get corrected (in a straight corridor, only cross-corridor position and yaw).
   Scan-match depth against the map. Later, a particle filter for recovery.
2. **6.2 Uncertainty drives speed and margins** (best highlight): margin = wall distance − 3σ,
   replacing the planner's fixed inflation; slowing at corners then emerges instead of being
   hard-coded.
3. **6.3 Honest VIO covariance**: inflate it when tracking is weak, add a health score and
   latency compensation. The minimal version is done (`decision-1.md`); the rest is
   `note/05-add-ons.md` section 5.
4. **6.4 A010 ToF in the tunnel**: only if lab tunnel drift exceeds about 20 cm sideways
   (see `note/05-add-ons.md` section 4). Mount and direction are decided then.
5. **6.5 Validation**: fault injection, 50–100 Monte Carlo runs with confidence intervals,
   ablations, failure taxonomy.
6. **6.6 Trajectory optimization**: lowest payoff, speed is worth at most 20 points.

Not recommended: neural networks for geometry, replacing the VIO, trading stability for
sophistication.

Caveats: build 6.1 and 6.2 in sim with fake noise before the lab. 6.1 relies on gate positions
that are still estimates, so wrong positions would pull the drone off course.

## Decided

| Question | Decision | Where |
|---|---|---|
| `ev_bridge` on VIO loss | Keep sending with larger σ; withhold bad samples for at most 2 s | `decision-1.md` |
| Planner | 2D A* at 0.8 m with per-segment altitude | `decision-2.md` |
| OAK-D mount | Straight ahead | `decision-2.md` |
| A010 | Stays an add-on; build only if lab tunnel drift > 20 cm | `decision-2.md` |
| Topic names and QoS | Parameters; lab names in the Interfaces table | `decision-2.md` |
| Follower | Lookahead, stop at sharp corners to align | `decision-3.md` |
| Safety layer | None for now (team decision 6); local avoidance removed | `decision-3.md` |

## Open questions

- VIO-loss tuning: is the 0.5 m σ cap enough in the real tunnel (sim drift reached 1.8–2.25 m)?
  Is a gradual σ ramp-down on recovery needed? Decide from the lab tunnel test
  (`decision-1.md`, 「給 lab 測試的觀察重點」).
- How to split the 0.3 m per side between tilt, localization and tracking error? Measure tracking
  error in sim first.
- Exact Gate A, Gate B and checkpoint positions: waiting for the official top view.
- Real aircraft: PX4 version, and whether the VL53L1X connects to the Pixhawk or the Jetson
  (`strategy.md` section 5).
