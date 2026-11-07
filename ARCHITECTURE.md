# DroneHacks system architecture

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

## Components

| Component | Status | Job |
|---|---|---|
| VIO (cuVSLAM) | Keep | Tune `vio.yaml` only |
| `ev_bridge.py`, `frames.py` | Keep | Change only the VIO-loss behavior |
| Landmark detector | New | Walls, gates, window, pad from depth |
| Map aligner | New | map → VIO transform + confidence |
| Local avoidance | New | Safety-net velocity limits (APF / VFH) |
| Planner | New | Inflated 5 cm grid + A* through the required gates |
| Path follower | New | 20 Hz lookahead, outputs VIO-frame setpoints |
| Mission | New | State machine on the `refnav/node.py` loop |
| Logger | New | rosbag2 + per-node event log, from day one |

Also keep: `refnav/core.py` (reference only), `tests/`, `course/course_map.yaml` (the only source
of course geometry), `a010/` driver (add-on only).

### Rules per component

- **Aligner**: start by matching the launch mark to the estimator's takeoff pose (don't assume
  VIO starts at zero). Update shift and yaw from each matched landmark. Reject and log updates
  that are too large. Never reset VIO.
- **Planner**: inflate obstacles by half the drone span + tilt + localization error. Enforce gate
  order, crossing direction and per-segment altitude. If A* finds no path, log which gap closed.- **Follower**: compare positions in the map frame. Search progress only near the last progress,
  so parallel corridors can't make it skip ahead. Shorten lookahead at corners. Limit speed,
  acceleration and yaw rate.
- **Mission**: each segment has a goal, completion condition, timeout and failure action
  (hover → back off → land). A gate counts only if the track crossed its opening in the right
  direction. Build the return leg (×1.25) from day one. Window vs Gate B and conservative vs
  aggressive speed are parameters. No hard-coded waypoints (rule 9.5).
- **Tunnel mode**: align before entry, fly slowly and straight on IMU only with downward range
  for height (about 3 s), re-align against the map after exit.
- **Logging**: record every topic on every flight, in the same format in sim and lab, so lab runs
  can be replayed against sim.

## Advanced techniques

Only after the basic version is stable. Ordered by payoff.

1. **6.1 Probabilistic alignment** (most worth doing): EKF over the transform, so only observable
   directions get corrected (in a straight corridor, only cross-corridor position and yaw).
   Scan-match depth against the map. Later, a particle filter for recovery.
2. **6.2 Uncertainty drives speed and margins** (best highlight): margin = wall distance − 3σ,
   replacing the planner's fixed inflation; slowing at corners then emerges instead of being
   hard-coded.
3. **6.3 Honest VIO covariance**: inflate it when tracking is weak, add a health score and
   latency compensation.
4. **6.4 A010 ToF in the tunnel**: only if lab tunnel drift exceeds about 20 cm sideways
   (see `note/05-add-ons.md` section 4).
5. **6.5 Validation**: fault injection, 50–100 Monte Carlo runs with confidence intervals,
   ablations, failure taxonomy.
6. **6.6 Trajectory optimization**: lowest payoff, speed is worth at most 20 points.

Not recommended: neural networks for geometry, replacing the VIO, trading stability for
sophistication.

Caveats: build 6.1 and 6.2 in sim with fake noise before the lab. 6.1 relies on gate positions
that are still estimates, so wrong positions would pull the drone off course.

## Open questions

- A010: build it? Where does it mount and which way does it point?
- OAK-D: straight ahead, or tilted down to see the pad?
- `ev_bridge` on VIO loss: stop publishing, or publish with larger covariance? Decide from a slow
  tunnel test watching `/estimator/healthy` and PX4 `xy_valid`.
- Planner: 3D voxels, or 2D A* at 0.8 m with per-segment altitude (simpler)?
- How to split the 0.3 m per side between tilt, localization and tracking error? Measure tracking
  error in sim first.
- Topic names and QoS must be configurable (lab: x86 + Jazzy; Jetson: ARM + Humble).
