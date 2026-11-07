# ev_bridge VIO-loss review (commit 5546e5d, branch nav-review)

Static review only (no Python executed). Files edited: `onboard/vio/ev_bridge.py`,
`onboard/vio/vio_health.py`, `onboard/config/vision_bridge.yaml`, `tests/test_vio_loss.py`.
Nothing committed. Line numbers are post-edit.

## Findings, by severity

### A. Could cause a failsafe or a wall strike

1. **Forced samples could be garbage** (`vio_health.py:95-105`, was `decide()`): after 2 s withheld
   the bridge sent whatever sample came next, even one that was itself a fresh >1 m jump or >5 m/s
   slide. A VIO runaway (impossible kinematics every frame) was therefore fed to EKF2 at 30 Hz with
   sigma 0.5 m, and `/estimator/vio_state` turned DEGRADED, so the mission resumed at half speed on
   garbage and its `withheld_land_s=3` landing could never fire (WITHHELD lasted at most 2 s).
   **Changed:** `VioHealth.update` now sets `health.fresh` (this very sample violated jump/speed/
   time); `VioLoss.decide(t, sane, fresh)` only forces *settling* samples (consistent with their
   predecessor, objected to only by the 5 s quiet / 10 s window timers). A fresh violation restarts
   the 2 s clock, is never sent, and starts a new episode (next forced sample gets its own reset).
   Consequence: a real relocalisation (one jump, then consistent) still reaches PX4 at +2 s as
   designed; continuous nonsense never does, the mission lands at 3 s WITHHELD, before
   EKF2_NOAID_TOUT (5 s). Tests: `test_a_fresh_violation_restarts_the_clock...`,
   `test_runaway_vio_is_never_fed_to_px4`, `test_tunnel_timeline...`.
2. **Reset-on-forced is the safer choice, kept** (question 1b). Sending the *last good* pose with a
   large variance instead was rejected: at 30 Hz with R=0.25 m² EKF2's position state is pinned to
   a pose that does not move while the drone moves 0.3-0.5 m/s; the controller then sees "no
   progress" and drives the vehicle into whatever is there (classic VIO-freeze runaway). A reset
   is gentler than smooth convergence too: EKF2 `resetHorizontalPositionTo()` publishes
   `delta_xy` and mc_pos_control shifts the pending trajectory setpoint by it, so the vehicle does
   not lunge; the mission then re-plans from the corrected position at DEGRADED (half) speed.
   Smooth fusion of a 1.8 m innovation would instead move the vehicle 1.8 m uncommanded.
   The forced sample is the *current* VIO pose (consistent for >=2 s thanks to finding 1), not a
   "known insane" one. Note `ev_bridge.py:143` already bumped `reset_counter` on `jumped`; `d.reset`
   only adds the case where the forced pose is within `jump_reset_m` of the last published one.
3. **Cold start / absent feature topic = DEGRADED forever** (`vio_health.py:81-83,92-99`,
   `ev_bridge.py:130-136`): `weak` started True and `lost_s` was `inf`, so every sample went out at
   the 0.5 m cap until the first count arrived, and forever if `/visual_slam/vis/observations_count`
   is not published by the Jetson's cuVSLAM build; the mission would fly at half speed all course.
   **Changed:** new `features_timeout_s` (1.0 s, yaml): no count for that long = UNKNOWN, not weak:
   samples go out unchanged (the October behaviour; PX4 applies its EKF2_EVP/EVV_NOISE floors),
   state OK, reason says "no feature counts, weak-tracking inflation off", one warning per outage
   (after 30 published samples, so the first-frame race does not warn). `lost_s` is always finite
   (0 when unknown; counted from the first count heard when VIO starts weak), so `inf` no longer
   goes on the wire. Tests: `test_no_feature_count_yet_is_unknown_not_weak`,
   `test_feature_counts_going_silent...`, `test_weak_from_the_first_frame...`.
4. **Empty `Int32MultiArray` killed the node** (`ev_bridge.py:101`): `msg.data[0]` on an empty
   array raises in the callback, rclpy exits, PX4 loses vision -> failsafe. Guarded.

### B. Wrong but hidden

5. **Inflation lost on a bad cuVSLAM covariance** (`vio_health.py:117-124`): cuVSLAM can emit a
   NaN/negative diagonal (`cov_bad` counter); `frames._diag` maps those to 0, so NaN+var -> NaN -> 0
   and the inflation vanished exactly when the covariance was least trustworthy. `inflate_xyz`
   now zeroes a non-finite/negative diagonal before adding. Test: `test_inflate_xyz_survives...`.
6. **WITHHELD only reached the mission on the 5 Hz tick** (`ev_bridge.py:90-99,124-127,150`):
   state is now also published on every transition from `_on_odom` (<= 1 frame, ~33-67 ms,
   instead of up to 200 ms); the 5 Hz tick stays. `/estimator/healthy` cadence is unchanged.
7. `pos_sigma_add_m` reported the last sent sigma while WITHHELD; now 0 (`ev_bridge.py:125`).
8. Parameter validation (`vio_health.py:76-79`): rates, caps, `withhold_max_s`,
   `features_timeout_s` must be positive finite (a 0 cap previously made `decide` force every
   insane sample immediately). Test: `test_bad_parameters_are_refused`.

### C. Minor / noted, not changed

9. Callback ordering: cuVSLAM publishes the count (`cuvslam_lib_node.py:279`) *before* the
   odometry (`:358`) of the same frame, and the single-threaded executor serialises the two
   callbacks, so `weak` is normally up to date when the odometry arrives; a DDS reorder lags it by
   one frame (<= 67 ms, sigma error <= 0.017 m). Acceptable.
10. `decide` uses receive time, `VioHealth` capture time; they differ by < `max_sample_age_s` (0.3 s).
11. `n_obs = -1` (cuVSLAM could not query observations) counts as weak, as cuVSLAM's own
    `visually_tracked` does. Documented in `features()`.
12. Per-sample cost unchanged: one 36-element list copy per covariance (as before), no new
    allocations on the hot path; the extra publish happens only on transitions.
13. ARCHITECTURE.md "VIO loss" should gain one line on findings 1 and 3 (not one of my four files).

## Tunnel timeline (15 Hz, 0.4 m/s, dark 2.0-5.0 s, 1.8 m relocalisation at exit; = the test)

| t (s) | VIO / VioHealth | VioLoss -> PX4 | vio_state (mission) |
|---|---|---|---|
| 0-2.0 | 200 features, sane | OK, raw covariance | OK (full speed, or tunnel speed) |
| 2.0 | enter dark, ~0 features | DEGRADED, sigma = 0.25*(t-1.93) m | DEGRADED published at once; tunnel mode ignores it |
| 3.9 | still dark | sigma hits the 0.5 m cap (var 0.25 m²), stays | DEGRADED |
| 5.0 | exit: jump 1.8 m, `fresh` | WITHHELD, nothing to PX4; EKF2 dead-reckons on IMU | WITHHELD at once; outside tunnel mode: carrot held (speed 0) |
| 5.07-6.93 | consistent samples, "settling" | WITHHELD (clock from 5.0) | WITHHELD; `/estimator/healthy` False from 5.5 |
| 7.0 | settling | forced: current pose, sigma 0.5, reset_counter+1 -> EKF2 resets position to it, mc_pos_control shifts the setpoint by delta_xy (no lunge) | DEGRADED (half speed); unhealthy lasted 1.5 s < 3 s, no landing |
| 7.0-10.0 | settling | forced samples, no further reset | DEGRADED |
| 10.0 | quiet over, sane | OK | OK |

PX4 is without vision 2.0 s (< NOAID_TOUT 5 s). If the relocalisation keeps jumping (fresh every
frame) the row at 7.0 never happens: WITHHELD continues, the mission lands at 3 s unhealthy
(8.5 s), PX4 would time out at 10 s. Open: whether PX4 Land without a position still behaves.

## PX4 parameters the lab must confirm (none found in lab_info/; `grep EKF2 lab_info/` is empty)

- **EKF2_EV_NOISE_MD** (expect 0): 0 = use the message variances with EVP/EVV_NOISE as lower
  bounds; 1 = parameters only. With 1 the whole inflation is a no-op and the scheme degrades to
  "keep sending" (still prevents the timeout; WITHHELD/forcing still work). Note the question in
  the brief had this inverted: it is MD=1 that disables the inflation.
- **EKF2_NOAID_TOUT** (5 s default, µs): the budget `withhold_max_s=2` is sized against.
- **EKF2_EV_CTRL** (bits: 0 hpos, 1 vpos, 2 vel, 3 yaw): the lab twist covariance is all zero
  (`lab_info/vio_odom.txt`), so if velocity fusion is on, PX4 trusts EV velocity at EVV_NOISE
  (0.1 m/s) until our inflation exceeds it (after 0.4 s of weak tracking).
- **EKF2_EVP_NOISE** (0.1 m), **EKF2_EVV_NOISE** (0.1 m/s): the floors; the lab position
  covariance diagonal is 0.0, so PX4 runs at these floors whenever we do not inflate.
- **COM_POS_FS_EPH** (5 m): eph after a forced reset is ~0.5 m (our cap), far below it; confirm.
- **EKF2_EV_DELAY**: cuVSLAM latency (gate allows up to 0.3 s); a wrong delay makes EKF2 fuse the
  forced pose against the wrong IMU history and inflates the apparent jump.
- Also useful: **EKF2_EV_QMIN** (quality gate; we send quality 0, so it must be 0),
  **EKF2_EVP_GATE** (5 sigma: at R=0.25 m² a 2.5 m innovation still passes).

## Verified (by reading)

- Variances land as variances: `inflate_xyz` adds var*I3 to the ROS 6x6 diagonal ->
  `align_to_enu` rotates the position block (R(C+vI)R^T = RCR^T + vI, isotropic, invariant) ->
  `position_variance` picks the diagonal and permutes axes -> `apply_floor` -> `fill_vehicle_odometry`
  copies into `position_variance`/`velocity_variance`, documented as variances in px4_msgs.
  Body-frame twist is not rotated; isotropic inflation is frame-free anyway.
- DEGRADED is never reported while `/estimator/healthy` is False (`_publish_state` masks to WITHHELD).
- One `reset_counter` bump per sample (`changed or jumped or d.reset`); `_observe_clock` rebuilds
  VioLoss on a ROS clock restart; existing tests' expectations hold except the one intentionally
  rewritten (`test_no_feature_count_yet...`).

## Not verified

- Nothing executed (sandbox): tests and node were checked by hand only; run `pytest tests/test_vio_loss.py`.
- PX4 firmware version and the EKF2 parameters above; mc_pos_control's setpoint shift on
  `xy_reset_counter` in the exact lab firmware; cuVSLAM relocalisation behaviour at the tunnel exit
  (sim drift 1.8-2.25 m is the only evidence). Whether the real Jetson build publishes
  `/visual_slam/vis/observations_count` (the lab topic list has it; the Jetson may differ).
