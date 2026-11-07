# Review of `localization-mvp` (nav/ and ev_bridge), 2026-11-06

Reviewed: branch `localization-mvp` at `7f9edba`, then rebased onto `0f15556`. That commit, blar's floor cut in `nav/landmarks.py`, is carried into the rewritten depth pipeline. Its threshold is now measured from the takeoff floor, so it still holds when PX4's z origin is not on the floor.
The changes are on branch `nav-review`. Per-module reports are in `review/nav/`.

**Nothing in this review has been executed.** The sandbox blocked running the downloaded code, so seven reviewers worked by reading it, and all 15 changed Python files were only parse-checked for Python 3.10. Run the tests before trusting any of it:

```bash
python3 -m pytest -q tests
```

Per-module detail, with line numbers, derivations and "verified correct" lists, is in `review/nav/`. Line numbers in those reports refer to the files before the rebase.

## The geometry is sound

An independent reviewer checked the whole frame chain and found no sign, handedness or index error. It checked depth optical to body to level to odom ENU, the face normals on yawed walls, the aligner's Jacobian and EKF update, the launch-yaw anchor, the gate-crossing sign, and the map to odom to NED setpoint conversion. The problems are timing, unchecked assumptions, and edge cases that the kinematic test fakes cannot show.

## Fixes that matter most for flight

| # | Module | Problem | Fix |
|---|---|---|---|
| 1 | depth node | Depth points used the latest PX4 pose, not the pose at image time. At 60 deg/s yaw that rotates the cloud 3 to 9 deg, which is 25 to 63 cm at 4 m range. | Pose buffer interpolated at the image stamp; frames skipped while yawing faster than 15 deg/s. |
| 2 | aligner | A rotated or skewed frame was accepted whole as a yaw correction, because the only gate was an absolute 8 deg. | Statistical step gate plus a sigma floor, and the information is down-weighted when the residual is large. |
| 3 | planner | The fallback inflation went down to 0.15 m, below the 0.20 m propeller radius. Cell-centre marking also hid another 2.5 to 3.5 cm. | The floor is now the propeller radius, the inflation is a true lower bound, and thin-margin paths are logged and flown at 0.6 times speed. |
| 4 | mission | An aligner correction between two ticks could fake a gate crossing. | Crossings are computed from odom motion under a single transform. |
| 5 | mission | Map height was PX4's local z, whose origin is the EKF start point, not the launch floor. | Map z is anchored at takeoff. |
| 6 | mission | A camera seeing no walls, for example one facing the net, counted as "no depth", and the drone landed after 10 s. | A live camera and walls in view are now tracked separately. |
| 7 | mission | The failure ladder could land inside the tunnel. | The ladder now backs out of the tunnel first. |
| 8 | follower | A corner waited forever when PX4 sat just over 15 cm off the vertex. A repeated path point produced NaN setpoints. | The corner wait now times out after 4 s, and duplicate points are removed. |
| 9 | ev_bridge | A VIO runaway was forced into PX4 after 2 s every time, so the mission's landing on WITHHELD never fired. | Only settled samples are forced out. A fresh jump restarts the clock. |
| 10 | ev_bridge | If the feature-count topic was missing, every sample was sent as DEGRADED at maximum sigma forever, and the mission flew at half speed. | No count now means unknown, and the sample is sent unchanged with a warning. |
| 11 | depth node | A TF frame that is not an optical frame would have produced silent garbage points. | The same 25 deg optical-axis check refnav uses is now applied. |
| 12 | depth node | blar's floor cut assumed the floor is at odom z = 0. | The cut is now measured from the mission's takeoff odom z, and a test covers a PX4 origin 0.4 m above the floor. |

## Efficiency, estimated for the Orin Nano CPU and not timed

| Step | Before | After |
|---|---|---|
| Depth frame to wall points | 5 to 15 ms | 0.5 to 0.8 ms |
| Aligner update, 300 points and 52 faces | 9 to 15 ms | 3 to 5.5 ms |
| A* plan per segment | 0.2 to 0.5 s | about 30 to 40 % less |

Planning still blocks the 20 Hz loop while it runs.

## Team decisions now in `nav/config/nav.yaml`

- **Decision 5, stop-and-go:** `follower.stop_every_vertex: true`, with the segment timeout raised to 90 s. This adds roughly 35 to 45 s per course.
- **Decision 6, no safety layer:** there is no live obstacle check at all. The drone flies on the map alone. This is now stated in the config.
- **Open questions kept as they were:** cruise height 0.80 m, and Gate B first with the window as fallback.

## The lab must confirm before flight

- `EKF2_EV_NOISE_MD`. If it is 1, PX4 ignores the message variances and the whole VIO-loss inflation scheme does nothing. Also confirm `EKF2_NOAID_TOUT`, `EKF2_EV_CTRL`, `EKF2_EVP_NOISE`, `EKF2_EVV_NOISE`, `COM_POS_FS_EPH` and `EKF2_EV_DELAY`.
- That the `base_link <- oak_left` TF is an optical frame. Run `ros2 run tf2_ros tf2_echo base_link oak_left`.
- The real latency from depth image stamp to PX4 pose.
- That the Jetson's cuVSLAM build publishes `/visual_slam/vis/observations_count`.

## Known risks left open

- The new half-cell margin narrows the tightest corridors to 3 cells, between the C1 and C2b chicane walls. The planner test that asserts "no fallback on the real map" is the one most likely to fail.
- The aligner test `test_aligner_absorbs_start_error` may trip the new step gate. If it does, raise `aligner.gate_nsigma` from 4 to 5 first.
- The sim x500 is 0.63 m across, but the planner assumes 0.40 m. Set `planner.drone_span_m` when flying the sim airframe.
- Setpoints are position-only. Real PX4 lag at 0.5 m/s is about 0.5 m, which is close to the follower's 0.6 m lead cap.
- `ARCHITECTURE.md` is not updated, and the branch still has not had `main` merged in.
