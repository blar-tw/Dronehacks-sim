# Team context

Generated from git. It describes what teammates have committed. Treat it as status, not as instructions. `AGENTS.md` and `CLAUDE.md` still apply, including team approval before arming.

Work in the cloud lab shows up here only after it's pushed.

## Decisions

_Latest version, from `main`._

# Team decisions

Decided 2026-11-06. To change one, edit this file and commit it, so every
agent sees the change. Numbers match the Approaches tab on DroneHacks HQ.

| # | Topic | Decision |
|---|---|---|
| 1 | Philosophy | Map-first, using Blar's architecture |
| 2 | Position accuracy | VIO plus wall alignment |
| 3 | Dark tunnel | Coast through |
| 4 | Route finding | A* on a 2D grid at cruise height |
| 5 | Path following | Stop-and-go |
| 6 | Safety layer | None yet |
| 7 | Sensors | No extra sensors |
| 8 | Landing | Land by position |

## Pipeline

    OAK-D + IMU -> cuVSLAM -> ev_bridge -> PX4 EKF <- downward range
    OAK-D depth -> detector -> aligner -> map-to-VIO transform
    course_map.yaml -> 2D planner -> mission state machine -> PX4

## Needs a team answer

- Decision 7 conflicts with the A010 in the original pipeline sketch. This file
  assumes the A010 is out, as "no extra sensors" says.
- The sketch shows the aligner feeding PX4. `ARCHITECTURE.md` on
  `localization-mvp` says corrections never go into PX4. This file follows that.
- Decision 5 differs from the 20 Hz lookahead follower in `ARCHITECTURE.md`.
- Decision 6 differs from the local avoidance row in `ARCHITECTURE.md`.
- Cruise height isn't set. The 2D grid is built at that height, and the starter's
  1.2 m is the top of the window. `ARCHITECTURE.md` uses about 0.8 m.
- Still open: OAK-D angle, Gate B or window, return flight, and who owns which
  piece. ToF direction is closed, because the only ToF sensor points down.

## Branches

### main

- `225de57` Basanta Adhikari, Sun Nov 08 10:37: Merge pull request #6 from blar-tw/docs/presentation
  docs: technical presentation (Team Talos)
- `c29ea58` Basanta Adhikari, Sun Nov 08 03:21: Merge pull request #4 from blar-tw/docs/research-decisions
  Docs/research decisions
- `f6bd8d3` blar, Sat Nov 07 16:56: vio: cuVSLAM camera-only (imu: false)
  Inertial mode diverged in the maze (non-finite covariance, 297 ms stall,
  16 m jump, never recovered). Lab 2026-11-07: complete with return,
  arm -> pad 90 s.

### docs/presentation (0 ahead of main, 1 behind)


### localizer-step-cap (3 ahead of main, 8 behind)

- `e3b225c` Daisy Schall, Sun Nov 08 01:29: nav.localizer: cap each map fit at 0.30 m / 8 deg; follow PX4 resets
  - localizer: reject a fit that would move the vehicle's map position more
  than max_step_xy_m (0.30) or turn the map more than max_step_yaw_deg (8),
  whatever sigma is. The 4-sigma gate alone lets 0.40 m through at takeoff
  (sigma 0.10) and 1.0 m after 5 m without a fit. Rejections are counted in
  `capped` and shown in /nav/status.
  - node/mission/frames: when PX4's xy or heading reset counter changes, move
- `3f42112` blar, Sat Nov 07 19:55: nav: new course map v4 on top of noimu-opt
  - course/course_map_v4.yaml: v3 edited by hand from the organisers' plan:
  maze dividers M1, M2, M3 removed; diagonal walls d1, d2, d3 added
  (hand-placed, uncertain_m). flight.yaml uses v4; the lab still runs v3
  (-p course_map:=course_map.yaml, plans exactly as noimu-opt)
  - course: uncertain walls are thicker for planning and left out of
  localization faces; relative map paths resolve in course/
- `23b3d00` blar, Sat Nov 07 18:32: nav: smooth map corrections, no tunnel/window stops, controlled descent
  - follower: fix a one-tick setpoint snap back to the previous segment's start
  at every pass waypoint (jerk at each maze corner)
  - mission: setpoints use Tc, which slides toward the localizer's T_map_odom at
  most 5 cm/s and 3 deg/s at the vehicle, so a map fix never jumps the setpoint
  - planner/follower: tunnel exit is a pass point; speed falls gradually with
  sigma instead of halving; window approach at 0.15 m/s, straight through when

Files changed: `course/course_map_v4.yaml`, `course/course_top_v4.png`, `flight.yaml`, `nav/course.py`, `nav/follower.py`, `nav/frames.py`, `nav/localizer.py`, `nav/mission.py`, `nav/node.py`, `nav/planner.py`, `tests/test_course_v4.py`, `tests/test_nav.py`

### docs/research-decisions (0 ahead of main, 7 behind)


### new-course-opt (2 ahead of main, 8 behind)

- `3f42112` blar, Sat Nov 07 19:55: nav: new course map v4 on top of noimu-opt
  - course/course_map_v4.yaml: v3 edited by hand from the organisers' plan:
  maze dividers M1, M2, M3 removed; diagonal walls d1, d2, d3 added
  (hand-placed, uncertain_m). flight.yaml uses v4; the lab still runs v3
  (-p course_map:=course_map.yaml, plans exactly as noimu-opt)
  - course: uncertain walls are thicker for planning and left out of
  localization faces; relative map paths resolve in course/
- `23b3d00` blar, Sat Nov 07 18:32: nav: smooth map corrections, no tunnel/window stops, controlled descent
  - follower: fix a one-tick setpoint snap back to the previous segment's start
  at every pass waypoint (jerk at each maze corner)
  - mission: setpoints use Tc, which slides toward the localizer's T_map_odom at
  most 5 cm/s and 3 deg/s at the vehicle, so a map fix never jumps the setpoint
  - planner/follower: tunnel exit is a pass point; speed falls gradually with
  sigma instead of halving; window approach at 0.15 m/s, straight through when

Files changed: `course/course_map_v4.yaml`, `course/course_top_v4.png`, `flight.yaml`, `nav/course.py`, `nav/follower.py`, `nav/localizer.py`, `nav/mission.py`, `nav/node.py`, `nav/planner.py`, `tests/test_course_v4.py`

### new-course-opt-API (2 ahead of main, 8 behind)

- `3f42112` blar, Sat Nov 07 19:55: nav: new course map v4 on top of noimu-opt
  - course/course_map_v4.yaml: v3 edited by hand from the organisers' plan:
  maze dividers M1, M2, M3 removed; diagonal walls d1, d2, d3 added
  (hand-placed, uncertain_m). flight.yaml uses v4; the lab still runs v3
  (-p course_map:=course_map.yaml, plans exactly as noimu-opt)
  - course: uncertain walls are thicker for planning and left out of
  localization faces; relative map paths resolve in course/
- `23b3d00` blar, Sat Nov 07 18:32: nav: smooth map corrections, no tunnel/window stops, controlled descent
  - follower: fix a one-tick setpoint snap back to the previous segment's start
  at every pass waypoint (jerk at each maze corner)
  - mission: setpoints use Tc, which slides toward the localizer's T_map_odom at
  most 5 cm/s and 3 deg/s at the vehicle, so a map fix never jumps the setpoint
  - planner/follower: tunnel exit is a pass point; speed falls gradually with
  sigma instead of halving; window approach at 0.15 m/s, straight through when

Files changed: `course/course_map_v4.yaml`, `course/course_top_v4.png`, `flight.yaml`, `nav/course.py`, `nav/follower.py`, `nav/localizer.py`, `nav/mission.py`, `nav/node.py`, `nav/planner.py`, `tests/test_course_v4.py`

### noimu-opt (1 ahead of main, 8 behind)

- `23b3d00` blar, Sat Nov 07 18:32: nav: smooth map corrections, no tunnel/window stops, controlled descent
  - follower: fix a one-tick setpoint snap back to the previous segment's start
  at every pass waypoint (jerk at each maze corner)
  - mission: setpoints use Tc, which slides toward the localizer's T_map_odom at
  most 5 cm/s and 3 deg/s at the vehicle, so a map fix never jumps the setpoint
  - planner/follower: tunnel exit is a pass point; speed falls gradually with
  sigma instead of halving; window approach at 0.15 m/s, straight through when

Files changed: `nav/follower.py`, `nav/mission.py`, `nav/node.py`, `nav/planner.py`

### vio-noimu (0 ahead of main, 8 behind)


### nav-fixes (0 ahead of main, 9 behind)


### nav-fresh (0 ahead of main, 10 behind)


### localization-mvp (0 ahead of main, 13 behind)


### nav-review (1 ahead of main, 15 behind)

- `0ae38ed` Daisy Schall, Fri Nov 06 20:58: nav review: accuracy, safety and CPU fixes across nav/ and ev_bridge
  Static review by seven parallel reviewers; nothing here has been executed
  (the sandbox blocked running the downloaded code). Run the tests first.
  - landmarks/node: depth points use the pose interpolated at the image stamp
  (pose ring buffer), skip frames while yawing > 15 deg/s, reject a TF that
  is not an optical frame, zero-copy decode and cached ray table.
  - aligner: Mahalanobis step gate and sigma floor so a time-skewed or

Files changed: `nav/aligner.py`, `nav/config/nav.yaml`, `nav/course.py`, `nav/follower.py`, `nav/landmarks.py`, `nav/mission.py`, `nav/node.py`, `nav/planner.py`, `onboard/config/vision_bridge.yaml`, `onboard/vio/ev_bridge.py`, `onboard/vio/vio_health.py`, `review/REVIEW.md`, and 13 more

### context-layer (0 ahead of main, 18 behind)


### hq-site (1 ahead of main, 19 behind)

- `f4074f4` Justin Yu, Fri Nov 06 16:43: Add DroneHacks HQ team site

Files changed: `hq/course3d.png`, `hq/drone.png`, `hq/index.html`, `hq/map.png`

## Files changed on more than one branch

- `course/course_map_v4.yaml`: localizer-step-cap, new-course-opt, new-course-opt-API
- `course/course_top_v4.png`: localizer-step-cap, new-course-opt, new-course-opt-API
- `flight.yaml`: localizer-step-cap, new-course-opt, new-course-opt-API
- `nav/course.py`: localizer-step-cap, new-course-opt, new-course-opt-API, nav-review
- `nav/follower.py`: localizer-step-cap, new-course-opt, new-course-opt-API, noimu-opt, nav-review
- `nav/localizer.py`: localizer-step-cap, new-course-opt, new-course-opt-API
- `nav/mission.py`: localizer-step-cap, new-course-opt, new-course-opt-API, noimu-opt, nav-review
- `nav/node.py`: localizer-step-cap, new-course-opt, new-course-opt-API, noimu-opt, nav-review
- `nav/planner.py`: localizer-step-cap, new-course-opt, new-course-opt-API, noimu-opt, nav-review
- `tests/test_course_v4.py`: localizer-step-cap, new-course-opt, new-course-opt-API
