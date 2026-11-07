# nav/mission.py review (branch nav-review)

Files edited: `nav/mission.py`, `nav/config/nav.yaml`, `tests/test_nav_mission.py`. Nothing committed.
Static review only: no Python was executed (sandbox). Line numbers are the NEW file.

## Findings, by severity

### A. Could crash or lose the aircraft

1. **Gate crossing fooled by aligner corrections** (`_fly`, old `crossing(self.prev_map, pm, ...)`).
   `on_points` may move `aligner.T` between two ticks (accepted step up to `max_step_xy_m` 0.30 m);
   prev_map was mapped under the old T, pos_map under the new one, so the fake 0.30 m "motion" could
   count a gate, or log a WRONG-direction crossing and clear `crossed` while the aircraft sat still past
   the line, stranding the segment until the 60 s ladder (replan from the far side plans a crossing BACK
   through the gate). Fix (mission.py:574): the track is `aligner.to_map(*prev_odom) -> pos_map`, both
   under the current T. Test `test_an_aligner_jump_is_not_a_gate_crossing`.
2. **Map z not anchored** (`_takeoff`). `pos_map[2]` was PX4 local z (EKF origin, lab shows 0.037 m
   at rest, but the origin is wherever the EKF started), so cruise 0.80 m, the rule 7.7 cap, the window
   height, `faces_at(z)` and the 0.30 m descend floor were all relative to the EKF origin, not the floor.
   Fix (mission.py:457): `init_at_launch(..., odom_z=oz, launch_z=0.0)` (the aligner's new keyword,
   present on this branch) with a `TypeError` fallback to the old call that logs "floor z NOT anchored".
   `on_points` now passes `z + T.z` to the aligner (faces are in the map frame). Test
   `test_map_floor_is_anchored_at_the_launch_mark` (z origin +0.5 / -0.4 m).
3. **Rule 7.7 clamp mixed frames** (`_setpoint_ned`): `max(z, zo) > z_cap` compared the ODOM z against
   the MAP cap; with T.z != 0 (finding 2) that over- or under-clamped. Now clamps map z, then converts.
   Covered by the same test (truth max z <= cap, == cruise).
4. **Ladder could land inside the tube.** Second timeout / planner failure inside the tunnel landed where
   it was. `_ladder_land` (mission.py:300): if `tunnel_active`, hold and reverse the carrot along the path
   at `tunnel_speed_mps` (`follower.back_off(v*dt)` per tick, no jump for PX4 to chase) until the
   aircraft is outside the tube footprint + margin, or `tunnel_back_out_s` 15 s, then land. Test
   `test_ladder_landing_inside_the_tube_backs_out_first`. Health/position failures inside the tube still
   land at once: with VIO dead, flying on is as blind as landing (ARCHITECTURE "coast on IMU ~3 s" is
   PX4's EKF2_NOAID_TOUT 5 s, which the mission yields to).
5. **Empty depth frames counted as a dead camera** (depth-pipeline finding). `on_points` returned before
   recording the time on a `(0,2)` array, so a live camera facing the net or open space aged
   `depth_lost_land_s` (10 s) toward a mid-course landing. Now `depth_alive_t` (any call) drives the
   landing, `walls_t` (non-empty) drives `depth_stale_hover_s` and `status.walls_age_s`. Optional
   `stamp=` drops out-of-order frames. Tests `test_empty_depth_frames_keep_the_stream_alive_but_silence_lands`,
   `test_out_of_order_depth_frames_are_dropped`.
6. **Climb / descend had no timeout.** A climb PX4 never completes (motors not up, z disagreement) hovered
   for ever; a descend whose 0.30 m target is below PX4's floor (z drift) pressed down for ever. New
   `vertical_timeout_s` 20 s: climb -> land where it is; descend -> hand the planned landing to PX4
   (`kind` kept, so the pad landing still counts). Test `test_descent_that_never_arrives_times_out_into_px4_land`.

### B. Wrong but hidden by tests

7. **Window crossing at any height counted** (coordinator finding 3). `in_opening(cp, z)` gates
   `_gate_crossed`; a crossing above/below `Checkpoint.zmin/zmax` is logged "not counted". Test
   `test_window_crossing_outside_the_opening_is_not_counted`.
8. **Missed track crossing strands the segment.** If the follower is `done` (aircraft at the exit point,
   0.35 m past the line) and no crossing was counted (skipped tick, jitter back over the line after a
   wrong-way event), the aircraft is past the gate by geometry: now counted "from the position past the
   line" (side + projection within the line, z inside the opening). Logged with that suffix.
9. **Stale ground hold setpoint while arming.** `sp_ned` was frozen at the wait->arm tick, streamed up to
   `arm_wait_s` 30 s. Now refreshed from PX4 until the first Offboard request (`_arm`), frozen after.
   Test `test_arm_hold_setpoint_tracks_px4_until_the_first_request`. The remaining race (PX4 accepts
   Offboard+arm on a ground-level setpoint, then the climb starts from the CURRENT pos_map) is benign: the
   first climb setpoint is rebuilt from the live position in `_takeoff`.
10. **Land timeout left an armed aircraft.** `_landing` timing out (no land-detected / disarm report) ended
    the mission with the aircraft possibly armed on the ground. One checked `disarm` is sent with the
    abort (PX4 refuses it in the air; never forced).
11. **`health_bad_land_s` is effectively dead.** ev_bridge publishes `vio_state = WITHHELD` whenever
    `healthy` is False (`_publish_health`), so the mission's limit is always `withheld_land_s` 3.0 s for an
    unhealthy-but-publishing bridge; 1.0 s only applies to a SILENT bridge (plus 1.0 s stale). 3 s is
    still < EKF2_NOAID_TOUT 5 s and > the 2 s withhold cap, so no change; comments say so.
12. Takeoff event labelled the ENU yaw "heading" (coordinator minor): now "yaw ... (ENU)".

### C. Minor
13. `pad_miss_abort_m` (default 0 = off): if the estimate says the pad landing was farther than this from
    the centre, the return leg is not flown (the launch mark is 0.53 m from the west net; a bad map pose
    over the walls is the risk). Team to decide the default. Test `test_pad_miss_by_the_estimate_skips_the_return_leg`.
14. `status()` gains `walls_age_s`, `back_out`, `aligner.z`, `aligner.z_anchored`; `segment.tunnel` cast
    to bool. Return takeoff logs the map z on the pad (= PX4 z drift since launch).

## Team decision -> what the code does now

| Decision | Code |
|---|---|
| 3 coast through the tunnel | tunnel mode: `tunnel_speed_mps` 0.3, depth/WITHHELD ignored, aligner frozen; ladder backs out before landing (new) |
| 5 stop-and-go | `nav.yaml follower.stop_every_vertex: true`; `mission.segment_timeout_s` 90 (yaml + default). Course ~16 m at 0.5 m/s = 32 s + ~45 s of holds + climbs/descents/landings ~30 s = ~110 s; e2e `t_max` 900 s is ample, `tests/test_nav_e2e.py` untouched |
| 6 no safety layer | `depth_stale_hover_s` 0 (yaml + default). **Risk stated plainly:** nothing checks live free space; the planned path is flown against the map only. An unmapped object (person, moved boulder, mis-surveyed gate) will be hit. Depth loss lands only because the aligner is then blind (10 s) |
| 7 no extra sensors | nothing in mission reads a ToF; height = PX4's estimate anchored to the floor at takeoff |
| 8 land by position | pad = map target: follower `done` (0.15 m), descend to 0.30 m, AUTO_LAND, PX4 owns it |
| open: cruise height | 0.80 m kept, pinned in yaml (window 0.30..1.22 m, centre 0.76; rule 7.7 cap 1.43) |
| open: Gate B vs window | defaults now `chicane_gate_b`, fallback `chicane_window` (dataclass == yaml); SPEC said window first |

## Verified correct (trace)
- Yield: `offboard_seen and nav_state not in allowed` in every AIRBORNE phase and `land` before AUTO_LAND;
  after AUTO_LAND nothing streams, only checked disarms. `ground`/return `wait` catch an external arm via
  the pre-command guard (`t_cmd` reset in `ground`). Same as refnav; refnav's `return out` after yield is
  not needed since no later branch acts on `done`.
- Takeoff climb at the current xy; return climb to wall_top + 0.5 = 2.23 m happens over the PAD (1.2 m
  south of barrier_1), under the 3.05 m net; `_depth_matters` is False above wall_top.
- Tunnel entry/exit: geometric (`on_tube`, margin 0.30); exit only on a right-direction crossing. After
  exit a T jump of <= 0.30 m can log a second right-way crossing (cosmetic, `passed` dedups) but no longer a
  wrong-way one (finding 1).
- Health: 5 Hz Bool, `health_stale_s` 1.0 tolerates 4 consecutive drops (reliable QoS anyway).
- Re-arm on the pad: AUTO_LAND -> `armed False` -> `_landed` -> `ground` -> `wait` (needs `arm_ok`,
  `armed is False`, `pos_ok`) -> `arm` -> `_takeoff` without re-anchoring xy (aligner kept) - correct.
- `dt`/`now`: `t_last/t_phase/t_wait0` set on the first step before use; `t_airborne` set in `_takeoff`
  before any `_depth_age`; `_landed` only after flight (pos_map set). `status()` is JSON-safe (floats
  rounded, inf -> None, bools, ints, str).
- 20 Hz cost: one extra `to_map` per fly tick; no new allocations in the loop.

## Proposed API changes for other owners
- aligner (optional): `set_floor(odom_z, map_z=0.0)` to re-anchor z on the pad before the return takeoff
  (PX4 z drift over the run); today the return descends to 0.30 m by a possibly drifted z, and the new
  `vertical_timeout_s` covers the hang.
- node: pass `stamp=<capture time>` to `on_points` to use the out-of-order guard.

## Not verified (tests could not run)
- Every new test and the three edited behaviours were traced by hand against the fakes and
  `tests/px4_fake.py`; margins in `test_an_aligner_jump_is_not_a_gate_crossing` (jump 1.25 m, exit 0.8 m
  past the line) were chosen so the done radius cannot fire; not executed.
- e2e (`tests/test_nav_e2e.py`) with `stop_every_vertex: true` and the real aligner's `odom_z`: timing
  estimated only.
- Real PX4 behaviour on arming in Offboard with a ground-level setpoint, and whether the lab's EKF z origin
  is at the floor (lab shows z 0.037 m at rest).
