# skybeta-05 crash, 2026-11-07 22:43 (flight `20261107-224309_blar`)

Source: the bag the team's `team13_onboard.sh fly` recorded at
`~/team_ws/team13_flights/20261107-224309_blar/bag/bag_0.mcap` (8.1 GB), plus `launch.log` beside it.
`crash_timeline.py` and `frames_att.py` (in this folder) read the bag without changing it. `timeline.txt` is
the full output. Times are seconds after Offboard started (unix 1794120218.37).

This flight ran **before** the `anchor_setpoints` launch change (23:17). It ran **with** the 21:46 `frame_comp`
patch in `team13_mission/node.py`.

## Timeline

| t (s) | What happened | Evidence |
|---|---|---|
| −2.4 | Armed | vehicle_status arming 2 |
| −0.06 | Offboard | nav_state 14 |
| 1.0 | Airborne | land_detected false |
| 2–4 | Climb. The camera says 0.08 → 0.58 m. PX4 and the rangefinder say it peaked at about 0.25–0.30 m. | odom z vs PX4 z, dist_bottom 0.29–0.31 |
| 4–7 | Flies forward (odom +x, "up the first corridor" in the map) at 0.4 m/s. The camera height keeps rising to 0.92 m, so `frame_comp` lowers PX4's target from +0.58 to −0.30 m. PX4 descends to floor level (z −0.15; dist_bottom 0.09 → 0.02). | setpoint z, PX4 z, dist_bottom |
| 5.0 | The camera already sees a wall panel close ahead and to the right, and open space and a person to the left | `frames/left_05.01.png` |
| 7.0 | The camera is pressed against the wall | `frames/left_07.02.png` |
| 7.3 | Impact: vertical acceleration spike (az −3.1 m/s²) | vehicle_local_position.az |
| 7.55–8.3 | Flips: roll 24° → 122° → −146° → −170°, with a −22.7 m/s² impact at 8.32 | vehicle_attitude |
| 8.1–8.3 | *Consequence:* the tumbling camera produces a 38 m jump. cuVSLAM briefly loses tracking (vo_state 1→2→1), the bridge bumps `reset_counter`, and PX4 resets its position to the bad pose. Afterwards PX4's position alternates between several bad poses. | odom, reset counters |
| 8.5–12.3 | Lying **upside down**, still armed in Offboard (roll −179°). PX4's "falling at 3.5 m/s" is the upside-down accelerometer. | attitude, vz |
| 12.33 | Disarmed | arming 1 |
| 13.3 | Land detector: landed | |

The horizontal path PX4 measured after takeoff was 4.1 m, at up to 0.87 m/s.

## Causes

1. **It flew blind into a wall about 1–1.3 m ahead of the start.**
   - The mission flies `course_map_v4` from the start pose, and there's no depth topic, so nothing senses obstacles.
   - The first two waypoints are 0.81 m and 1.29 m straight ahead ("facing up the first corridor").
   - The 5.0 s frame shows a wall face ahead-right instead of a corridor. Either the drone wasn't on the launch
     mark facing up the first corridor *when the camera pipeline last started* (the camera fixes its frame at
     that moment), or the map doesn't match the real layout.
2. **It was flying at floor level.**
   - The camera's height drifted up to about 0.9 m, while PX4 and the rangefinder read about 0.3 m and then less.
   - `frame_comp` adds the live (PX4 − camera) offset to every setpoint, so PX4 followed the camera's drift:
     the higher the camera thought the drone was, the lower PX4 was told to fly. That put the drone at floor
     level while it moved forward.
3. **Nothing stopped the motors** for 4 s while the drone lay upside down (disarm at 12.3 s).

Not causes: the 38 m position jump and PX4's resets (they happened after the flip), and the 1.3 m height
offset (frame_comp was handling the offset at this point).

## What to fix before the next flight

1. **Don't compensate twice.** As of 23:17, `team13_mission.launch.py` sets `anchor_setpoints: true` *and* the
   node still runs `frame_comp` (default true). Both shift setpoints by PX4 − camera, so the shift is applied
   twice. Keep `anchor_setpoints` and set `frame_comp: false`:
   - The anchor uses a fixed offset recorded once, so the camera's later height drift no longer pushes PX4's
     target up or down.
   - `frame_comp` follows that drift live, which is what flew it into the floor.
2. **Start pose:** place the drone on the launch mark, facing up the first corridor, *then* restart the stack
   (`docker rm -f skybeta-stack`), and don't move it again. Before arming, look at the camera (or
   `/team13/status`) and confirm it sees down the corridor.
3. **First flights:** set `height_m` higher, use a short first leg, and keep the safety pilot's hand on the kill
   switch. A 4 s delay is long.
4. **Height source:** `dist_bottom_valid` stayed false for the whole flight. Lift the drone by hand with props
   off and check that the rangefinder becomes valid. Until PX4's height is trusted, don't let the mission's
   height control follow the camera's height.
5. **For the organisers** (seen after the flip, but it could cause a fly-away in flight): the bridge passed a
   38 m vision jump to PX4 as a reset, and a held pose target in the old frame would then be 38 m away.
