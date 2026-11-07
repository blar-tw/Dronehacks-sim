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

- `fa34d42` Justin Yu, Fri Nov 06 19:48: Merge pull request #2 from blar-tw/decisions-2d-grid
  Record the 2D grid decision
- `0d73545` Justin Yu, Fri Nov 06 19:48: Record the 2D grid decision
  The team chose A* on a 2D grid at cruise height over a 3D grid. Adds cruise
  height as an open question, since the grid depends on it, and closes ToF
  direction, because no extra sensors means the only ToF sensor points down.
- `afb102e` Justin Yu, Fri Nov 06 19:20: Merge pull request #1 from blar-tw/context-layer
  Add a combined team context layer
- `7779159` Justin Yu, Fri Nov 06 17:11: Add a combined team context layer
  tools/team_context.py merges every branch's recent commits, changed files
  and context/DECISIONS.md into one view. Claude Code loads it at session
  start, and a workflow publishes it to the team-context branch on each push.
  Records the team's decisions so far, with open conflicts listed.
  The workflow and hook only apply to branches that contain them, so this
  needs merging into main, and existing branches need main merged in.
- `c0b8e90` blar, Thu Nov 05 12:31: Lab workspace snapshot 2026-11-05 (Vision world, READY, disarmed)
  Project files from /var/lib/flyx-lab/workspace plus read-only
  inspection output in lab_info/ (lab_inspect.sh).

### context-layer (0 ahead of main, 3 behind)


### localization-mvp (2 ahead of main, 4 behind)

- `ddbd236` blar, Fri Nov 06 16:48: Add the DroneHacks course files
  Official v3 course (OBJ/MTL, GLB, STEP) and course_map.yaml, the course
  geometry ARCHITECTURE.md refers to (generated from the OBJ by the local
  sim's build_world.py), plus the top-view render.
- `5546e5d` blar, Fri Nov 06 16:38: ev_bridge: keep PX4 aided through VIO loss
  Weak tracking (fewer than 30 cuVSLAM features per frame) keeps sending
  with position/velocity sigma growing up to 0.5 m / 0.5 m/s. Insane
  samples are withheld at most 2 s, then sent at the max sigma with one
  reset_counter bump, so PX4 never reaches EKF2_NOAID_TOUT (5 s). The old
  5 s settle after a VIO jump hit that timeout exactly.
  New /estimator/vio_state (OK / DEGRADED / WITHHELD) for the mission;

Files changed: `ARCHITECTURE.md`, `course/course-v3.glb`, `course/course_map.yaml`, `course/course_top.png`, `course/dronehacks-course-v3.step`, `course/obj/README.txt`, `course/obj/dronehacks-course-v3.mtl`, `course/obj/dronehacks-course-v3.obj`, `onboard/config/vision_bridge.yaml`, `onboard/vio/ev_bridge.py`, `onboard/vio/vio_health.py`, `tests/test_vio_loss.py`

### hq-site (1 ahead of main, 4 behind)

- `f4074f4` Justin Yu, Fri Nov 06 16:43: Add DroneHacks HQ team site

Files changed: `hq/course3d.png`, `hq/drone.png`, `hq/index.html`, `hq/map.png`

## Files changed on more than one branch

None.
