# Frame and geometry chain review (static, not executed)

Scope: nav/frames.py, refnav/depth.py, nav/landmarks.py, nav/aligner.py, mission pose/crossing code, nav/node.py depth path, tests/px4_fake.py, nav/course.py.

Verdict: no sign, handedness or index error in the core geometry. Verified correct:
- (forward, right, down) -> ENU in landmarks.wall_points; right = (sin psi, -cos psi).
- level_rotation gives (forward, right, down) with yaw removed; t_body_cam applied in FLU before leveling.
- Box.corners is CCW for any yaw; N = (Dy, -Dx)/L is outward; face culling drops back-facing faces correctly.
- Aligner yaw Jacobian column -nx*vy + ny*vx; IEKF step and posterior covariance; _lever in predict/init.
- init_at_launch yaw: T.yaw = launch_yaw - odom_yaw; consistent with FakePX4.
- crossing() sign vs cross_sign: only planned-direction crossings count.
- _setpoint_ned map->odom->NED; z clamp; decode_depth; voxel mean.

Findings (ranked):
1. No time alignment of depth vs pose (nav/node.py _on_depth). Latest PX4 pose used for an image that is L older.
   At 0.5 m/s, L=50-150 ms: 2.5-7.5 cm along heading. At 60 deg/s yaw: 3-9 deg -> 10-31 cm lateral at 2 m, 21-63 cm at 4 m.
   Exceeds match_max_m 0.25 and corner_m 0.05; yaw bias can pass the 8 deg step gate with confident weight.
   Hidden by tests: px4_fake generates odom points directly. Fix: pose ring buffer interpolated at image stamp, or skip updates when |yaw rate| > ~15 deg/s.
2. oak_left optical-frame convention assumed, not checked, in nav/node.py _calibration. refnav/node.py has a 25 deg optical-axis-forward check; nav does not. Wrong TF -> silent garbage points.
3. Gate crossing uses prev_map -> pos_map where T may have changed in between (aligner step up to 0.30 m): a fake crossing or a fake wrong-direction event. Fix: test crossing in odom with one T, or map prev_odom through the current T.
4. z not anchored: init_at_launch/_takeoff never set T.z; map z = PX4 local z (EKF origin, not launch floor). Feeds faces_at(z), cruise height, wall clamp, descend floor.
5. faces_at uses a +-0.01 band while points use +-0.20 band (window, tube ellipse profiles). Acceptable, unverified.
6. crossing() ignores Checkpoint zmin/zmax (planner checks z for the window; mission does not).
7. on_points uses pos_odom from the last 20 Hz tick, not the depth-time pose (~2.5 cm).
8. Takeoff event prints ENU yaw labelled "heading" (cosmetic).

Test gaps: no test composes wall_points with the aligner on a moving/yawing drone; no non-optical TF test; no aligner-jump-near-gate test; no non-zero PX4 z origin test; aligner tests do not use yawed boxes.

Lab verification needed: `ros2 run tf2_ros tf2_echo base_link oak_left` must show an optical rotation; measure depth stamp vs local_position arrival latency.
