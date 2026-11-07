# Depth -> wall points pipeline review (nav/landmarks.py, nav/node.py, tests/test_nav_landmarks.py)

Branch nav-review, static review + edits; NO Python was executed (sandbox). Line numbers refer to the
files as they were before my edits unless marked (new).

## Findings, by severity

1. HIGH - pose/image time misalignment (node.py:179-182). `_on_depth` used the LATEST PX4 position,
   heading and quaternion for a frame stamped 50-150 ms earlier (stereo + transport, plus 10-20 ms
   PX4->DDS). Translation error at 0.5 m/s: 2.5-7.5 cm along track. Yaw error at 60 deg/s:
   0.06-0.15 s x 60 = 3.6-9 deg -> lateral error r*theta = 13-31 cm at 2 m, 25-63 cm at 4 m, i.e.
   beyond the aligner's match_max_m 0.25: the far points are dropped or mis-associated and the yaw
   estimate is biased by latency x yaw rate. Fix (new): `landmarks.PoseBuffer` keeps ~1 s of
   (stamp, x, y, z, heading) from vehicle_local_position and (stamp, q) from vehicle_attitude with
   SEPARATE stamps (they arrive at different rates, ~10 ms apart) and interpolates each at the image
   header stamp (linear position, circular heading, normalised-lerp quaternion). Stamps are the node
   clock at receipt minus PX4's own (timestamp - timestamp_sample) age, the repo's convention
   (onboard/adapters/px4_odom.py: PX4 timestamps are on PX4's clock; the lab's 120.614 s vs the
   image's 120.54 s only match because SITL lockstep starts PX4's clock with the sim). Fallbacks:
   stamp outside the history by > pose_max_dt_s (0.10) -> nearest sample + rate-limited warning,
   counted in frames.pose_fallback; stamp <= 0 -> arrival time (frames.unstamped); frame older than
   max_latency_s (0.5) on arrival -> reject "stale" (message names use_sim_time when > 2 s: camera
   and node on different clocks); stamp in the future -> reject "future".
2. HIGH - yaw-rate guard (new, coordinator finding). Even the interpolated pose is only as good as
   the stamps (10 ms error at 60 deg/s = 0.6 deg = 2 cm at 2 m, 4 cm at 4 m). `PoseBuffer.yaw_rate`
   measures the heading rate over the samples within +-0.1 s of the image stamp; above
   max_yaw_rate_dps (15) the frame is rejected as "turning" and on_points is not called.
3. HIGH - no optical-frame check on the TF (node.py:153-165, coordinator finding). A body-aligned
   TF would have produced silent garbage. Copied refnav/node.py:194-199: the optical axis R[:,2]
   must lie within optical_axis_tol_deg (25) of body-forward, else reject "camera_not_forward" and
   do not cache. NOTE for the lab: the depth frame_id is `oak_left`, not `..._optical_frame`; refnav
   uses the same check against the same TF, so if refnav's depth worked there the TF is optical.
   Verify with `ros2 run tf2_ros tf2_echo base_link oak_left` (expect q ~ (x,y,z,w) =
   (-0.5, 0.5, -0.5, 0.5) for a forward optical frame).
4. MEDIUM - attitude vs heading residual (landmarks.py:50-56). level_rotation strips the yaw of q;
   the odom rotation used lpos.heading. In PX4 EKF2 heading IS the yaw of that quaternion, so the
   only residual was the sampling skew (~10 ms at 60 deg/s = 0.6 deg = 2-4 cm at 2-4 m). Now the
   odom yaw is `heading_from_quat(q)` of the same interpolated q: residual exactly 0 by construction;
   |lpos.heading - yaw(q)| at the stamp is still published as `heading_skew_deg` (expect ~0).
5. MEDIUM - empty vs unusable (landmarks.py:47-48,57-58; node.py:184-186). wall_points returned None
   only for < min_valid_frac; a valid frame with nothing in the band returned a (0,2) array already,
   but through `len(xy)==0` after the full chain. Now: None = unusable (reject "unusable"), (0,2) =
   "no walls in view" (frames.empty, not a reject); both paths tested, 32FC1 and 16UC1. The node
   still hands the (0,2) array to on_points (mission ignores it today; see proposal below).
6. MEDIUM - a depth callback exception killed the node (node.py:167). `_on_depth` now wraps `_depth`
   in try/except: reject "exception", traceback logged for the first 3.
7. LOW - decode_depth (refnav/depth.py:37-50, not my file) is correct for 16UC1 mm and 32FC1 and for
   `step` >= width*itemsize (reshape(h, step//isz)[:, :w]); a byte count != h*step raises ValueError
   (caught as "undecodable"); is_bigendian was ignored. But it copies the image three times
   (bytes(m.data), .astype, .copy()) and runs 2 full-image mask passes. Replaced in node.py by
   `landmarks.depth_view` (new): np.frombuffer(m.data) read-only view, padding sliced off, 16UC1
   kept as uint16 (scaled after sampling), is_bigendian handled, documented "use inside the callback,
   never keep".
8. LOW - border (landmarks.py:25). border_px=16 covers the matcher's window edge, not the OAK stereo
   invalid band: the left ~95 px of a left-aligned 640-wide depth image can never match (0-95 px
   disparity search; 190 with extended disparity). Those pixels are 0 -> already dropped by the
   validity test, so they only lower the valid fraction (by ~13 %); the hazard is false matches at
   the band's edge. Added border_left/right/top/bottom_px (-1 = border_px) and documented
   border_left_px ~ 96 for the hardware; the default is unchanged (the simulator has no band).
9. LOW - pose quality gating (new). Only lpos samples with pos_ok (xy_valid, z_valid, heading ok,
   finite) enter the history; a frame before the first such sample is rejected "no_pose".
10. LOW - node robustness: `main()` constructs the node inside the try (a config error no longer
    raises NameError in `finally`), handles ExternalShutdownException (launch Ctrl-C) as 130,
    destroy_node guarded; vio_state parsing tolerates non-numeric/NaN lost_s; CameraInfo K cast to
    float; `_on_health` casts to bool. Depth QoS is BEST_EFFORT depth 1 (matches a reliable or
    best-effort camera publisher; never works through a backlog). use_sim_time: main.py passes the
    deployment params file, node logs the value; nothing to change. PX4_QOS import unchanged.
11. INFO - sign/frame: the folded chain reproduces refnav.level_points point for point (test). No new
    sign issue found; the TF check (3) is the frame risk.

## Per-frame CPU cost (640x400, stride 4, 32FC1; Orin Nano A78AE, numpy only)

Before: bytes(m.data) 1.0 MB copy (~0.3 ms) + decode_depth astype copy + .copy() + 2 mask passes
(~5 full-image passes, 1-2 ms) + wall_points validity (~0.1 ms) + level_points: np.array(copy=True)
1 MB (~0.3 ms), 4 border NaN writes, mgrid 2x16k int64, 2 astype, stack (M,3), 2 float64 matmuls
(~0.5-1 ms) + np.unique(cells, axis=0): row sort through a structured void view on N = 1-16k
rows (~1-2 us/row = 2-10 ms) + np.add.at (~0.1 us/elt, 0.2-1.5 ms). Total ~5-15 ms/frame;
x 15 Hz = 75-225 ms/s = 8-22 % of one core, next to cuVSLAM's CPU share.

After: no image copy (view). Sampled pixels 92 x 152 = 13,984: valid + ok masks 5 passes (~70 us);
gathers z, rx, ry (~40 us); x, y (~10 us); `down` 5 ops + abs + compare (~50 us); survivors
S = 1-4k: 2x3 fold 10 ops (~20 us); voxel: floor/astype (~20 us), 1-D int64 np.unique S log S
(~150 us at 4k), 3 bincounts (~20 us); ~50 numpy calls x ~3 us overhead (~150 us). Total
~0.5-0.8 ms/frame (~10-20x less), ~1 % of a core at 15 Hz. The ray table (_rays) is built once per
(K, shape, stride, borders) (~0.3 ms) and reused; float32 is kept until the odom offset is added,
which happens in float64 (output float64 for the aligner). depth_ms EMA kept; added
depth_latency_ms, pose_dt_ms, heading_skew_deg and a `frames` counter dict to the status JSON.

## Verified (by reading)

- Sampling identity: my rows/cols (multiples of stride, >= border, < size - border) are exactly the
  pixels level_points keeps after NaN-ing the border and striding from 0; same C order, so the
  equivalence test compares point for point.
- Odom formula matches the module docstring and the old code (A = [[c, s], [s, -c]] on (forward,
  right)); origin t_body_cam rotated by the level rotation like level_points does.
- config.fill(): all new LandmarkParams fields are int/float with defaults; nav.yaml has no
  landmarks section, so nothing breaks; -1 per-side borders pass through int().
- PoseBuffer: sorted stamps (a backwards stamp voids the history), horizon trimming, bracket
  interpolation, +-pi heading seam, quaternion sign flip before lerp; yaw_rate always includes the
  two bracketing samples.
- Public API kept: wall_points signature/semantics (None = unusable), LandmarkParams fields (added
  with defaults), node parameters unchanged; nav/mission.py, aligner.py untouched.

## NOT verified (tests cannot run here)

- All new tests (tests/test_nav_landmarks.py: 13 functions, 20 cases). Risky spots I hand-checked:
  nlerp heading at 3/4 between 0 and 20 deg (computed 15.01 deg, tol 0.1); band-edge points in the
  level_points equivalence test (level attitude facing the wall -> heights per row, 5e-4 m margin);
  the 2.8 m wall line sits on a 5 cm cell boundary, so only uniqueness, never counts, is asserted.
- numpy 1.21 (Humble) vs 2.x: depth_view uses view-then-reshape (allowed everywhere); the odd-step
  branch copies. float32 arithmetic relies on python-float scalars (both casting rules keep float32).
- rclpy specifics: m.data as array.array (frombuffer works; a list falls back to bytes()); the
  SystemExit-from-timer exit path is the pre-existing refnav pattern; depth QoS depth 1.
- VehicleAttitude.timestamp_sample exists in the lab's px4_msgs (getattr fallback otherwise).
- The lab TF for `oak_left` being an optical frame (finding 3).

## Proposal for nav/mission.py (owner: mission agent) - on_points

`on_points(self, t, points_odom_xy, z)` -> `on_points(self, t, points_odom_xy, z, stamp=None)`:
`t` stays the arrival time on the mission clock (what the aligner's t_update and the staleness
timers use), `stamp` the image capture time (node passes it) for ordering: drop a frame stamped no
later than the last one used, like refnav.core.Mission.on_depth. Independently, set `self.pts_t = t`
BEFORE the `len(pts) == 0: return` (mission.py:228-232): today a live camera that sees nothing at
flight height (net, open area, far wall) counts as "no depth", and mission.py:369 lands the drone
after depth_lost_land_s of that. The node already passes the (0,2) array so this needs no node change.
