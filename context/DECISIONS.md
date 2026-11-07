# Team decisions

Decided 2026-11-06. To change one, edit this file and commit it, so every
agent sees the change. Numbers match the Approaches tab on DroneHacks HQ.

| # | Topic | Decision |
|---|---|---|
| 1 | Philosophy | Map-first, using Blar's architecture |
| 2 | Position accuracy | VIO plus wall alignment |
| 3 | Dark tunnel | Coast through |
| 4 | Route finding | A* on a 3D grid |
| 5 | Path following | Stop-and-go |
| 6 | Safety layer | None yet |
| 7 | Sensors | No extra sensors |
| 8 | Landing | Land by position |

## Pipeline

    OAK-D + IMU -> cuVSLAM -> ev_bridge -> PX4 EKF <- downward range
    OAK-D depth -> detector -> aligner -> map-to-VIO transform
    course_map.yaml -> planner -> mission state machine -> PX4

## Needs a team answer

- Decision 7 conflicts with the A010 in the original pipeline sketch. This file
  assumes the A010 is out, as "no extra sensors" says.
- The sketch shows the aligner feeding PX4. `ARCHITECTURE.md` on
  `localization-mvp` says corrections never go into PX4. This file follows that.
- Decision 5 differs from the 20 Hz lookahead follower in `ARCHITECTURE.md`.
- Decision 6 differs from the local avoidance row in `ARCHITECTURE.md`.
- Still open: OAK-D angle, ToF direction, Gate B or window, return flight, and
  who owns which piece.
