# Height fix for skybeta-05: anchor setpoints on PX4's own position

**Status (2026-11-08 00:35):**
- Both patches are applied on skybeta-05: the anchor (2026-11-07 23:17) and `frame_comp` off (2026-11-08 00:29).
- Loaded with the real launch libraries, the file gives `anchor_setpoints: True` and `frame_comp: False`.
- After the stack restart at 00:30:47, the interface captured the anchor at 00:32:15 with PX4's position valid.
- Offsets on that boot were small: yaw −0.02°, height −0.04 m (on the 2026-11-07 21:40 boot they were 9° and 1.16 m).
- Not yet checked: the setpoint PX4 receives vs PX4's own position. A teammate stopped the competitor window at
  00:33 for a manual recording before that check could run.

**Setpoint check, run 2026-11-08 02:01 (boot of about 01:18, drone landed, disarmed, still): FAILED in z.**
- Runtime params are right: `anchor_setpoints` True, `frame_comp` False, yaw offset 0.
- PX4 receives `trajectory_setpoint` at about 10 Hz. x and y match PX4's position within 0.03 m.
- z: the target is NED −3.0 m while PX4 is at +0.02 m, so the target is 3 m above the drone.
- Cause 1: the camera odometry reads z = 30.0 m with the drone on the floor (`vo_state` still 1). PX4's height
  is correct because EKF2 takes height from the rangefinder, not from vision. team13 holds its current
  odometry pose, 30 m, and the interface clamps that to its 3 m ceiling.
- Cause 2: this boot's anchor fell back to the NED origin (PX4 was not linked at 01:20:56), so the check
  doesn't test the anchor.
- Don't arm and switch to Position on this boot. Offboard would climb to 3 m.
- A valid check needs a full stack restart on the ground (VIO starts from zero, and the anchor is captured
  60 s later with PX4 linked). Respawning only the competitor window would anchor on top of the 30 m error.

**Rerun after a full stack restart, 2026-11-08 02:08: PASSED.**
- At 02:06:03 the stack was removed with `docker rm -f`, with the drone landed, disarmed, still and nothing
  recording. systemd brought it back. PX4 was linked at 02:07:49, and the anchor was captured at 02:07:50 with
  no fallback warning.
- Over 15 s, 152 setpoints: the target PX4 receives equals PX4's own position (NED −0.001, −0.002, 0.024)
  to within 0.5 mm on every axis. There were no clamp warnings and no camera errors.
- Odometry vs PX4 on this boot: yaw 0.00° ± 0.02°, height −0.03 m. VIO tracked on all 424 status messages at 30 Hz.
- What this proves: the anchor is captured and the settings are right. It is a check at rest, and this boot's
  offsets are close to zero, so it does not test the correction in flight.
- The launch file was edited again by a teammate at 01:46 (sha `ba040357…`). It added a CBF wall filter,
  `cbf` (default true), that needs `depth_topic`. Since there is no depth topic, team13 logs "the mission will
  NOT offer a takeover", so it won't fly the course in this configuration. The anchor and `frame_comp` settings
  are still in force at runtime.

**No takeoff, 2026-11-08 03:12–03:15 (`use blar`, three attempts, read from their bags):**
- The pilot armed within seconds of each launch. The 60 s `interface_start_delay_s` from this fix meant
  `flight_interface` and team13 weren't running yet, so nothing switched PX4 to Offboard.
- Attempts 1 and 2 were relaunched after about 27 s, and no setpoint was ever sent.
- Attempt 3:
  - 03:14:25 armed.
  - 03:14:26 switched to Position.
  - 03:14:36 PX4 disarmed itself, after 10 s armed on the ground.
  - 03:15:20 the interface came up.
- Two more problems would have hit even with a wait:
  - From 03:15:15 the camera odometry ran away about 3 m (1.8 m up) while the drone sat still and `vo_state`
    stayed 1.
  - The anchor fell back to the NED origin at 03:15:21, because the interface received odometry before its
    first PX4 position. That is a race, and the start delay doesn't prevent it.
- The Orin went down at about 03:15:36 and booted again at 03:17:00.
- At 03:20 the team lowered the delay default to 10 s.
- Before arming: wait for "envelope origin captured", check there is no "anchored at the NED origin" warning,
  and check that `/skybeta/state` odom_enu is still near zero.

## The problem

Measured on 2026-11-07 with the drone landed:
- PX4's height estimate was −1.16 m and the camera odometry's was −0.24 to +0.18 m. They disagreed by 1.15–1.33 m.
- `skybeta_flight_interface` runs with `anchor_setpoints: false`, so it passes odometry-frame setpoints to PX4
  unchanged. Every commanded height was off by that 1.2–1.3 m:
  - `team13_mission`'s 0.6 m flight would climb to about 1.9 m.
  - Its landing push would stop about 1 m above the floor.

## The fix

`patch_team13_launch.py` edits `~/team_ws/src/team13_mission/launch/team13_mission.launch.py` on the drone:

1. **`anchor_setpoints:=true`** is passed to `flight_interface`. This is the organisers' own correction
   (competitor guide §1). The interface records PX4's position at the same instant as the odometry origin,
   and sends every setpoint as a displacement from that pair. The envelope clamp still works in the odometry
   frame, so `min_altitude_m: -0.5` and the landing logic are unchanged.
2. **`interface_start_delay_s:=60`** starts both nodes 60 s after the launch. The interface records that
   position pair only once, at its first odometry message. Right after a boot or a stack restart, PX4 may
   not have a valid position yet, and the interface then quietly falls back to no correction. A one-minute
   delay matches the guide's "keep the drone still for the first minute".

Nothing else changes. The script makes exact-text edits and refuses to write if the file isn't the version
it was written for, or if it's already patched. It was tested on a copy of the file as it was on the drone
at 21:40, and the patched launch file was loaded against stand-in launch modules.

The same change is in `talos_mc_executor/launch/talos_mission.launch.py`.

## Apply (drone disarmed, on the ground, still)

```bash
# from the Mac
scp patch_team13_launch.py dronehacks@<drone-ip>:/tmp/
ssh dronehacks@<drone-ip>
mkdir -p ~/team_ws/backups/height-fix && cp ~/team_ws/src/team13_mission/launch/team13_mission.launch.py ~/team_ws/backups/height-fix/
python3 -I /tmp/patch_team13_launch.py ~/team_ws/src/team13_mission/launch/team13_mission.launch.py
docker exec skybeta-stack tmux respawn-window -k -t skybeta:competitor   # rebuilds team_ws, relaunches; VIO keeps running
```

## Check (about 2 minutes later)

```bash
host/run.sh shell
ros2 param get /flight_interface anchor_setpoints          # True
# The flight_interface log must NOT say "anchor_setpoints is true but EKF2 had no valid local position"
ros2 topic echo /fmu/in/trajectory_setpoint --once --field position
ros2 topic echo /fmu/out/vehicle_local_position --once --qos-reliability best_effort   # compare x, y, z
```

While the mission waits on the ground, the setpoint PX4 receives should match PX4's own position to within
a few tenths of a metre. Before the fix it was about 1.3 m off in z.

## Roll back

```bash
cp ~/team_ws/backups/height-fix/team13_mission.launch.py ~/team_ws/src/team13_mission/launch/
docker exec skybeta-stack tmux respawn-window -k -t skybeta:competitor
```

## Second patch: turn off team13's own frame compensation

`team13_mission/node.py` gained `frame_comp` (default true) at 2026-11-07 21:46, after the 21:40 snapshot the
anchor patch was written against. It adds the live, low-passed `(PX4 local − odom)` offset from
`/skybeta/state` to every setpoint, but only while every axis is under 1.0 m. With `anchor_setpoints` on as well,
an offset under 1 m is corrected twice, and the drone flies off by that offset in the other direction. (At the
1.16 m measured on 2026-11-07, `frame_comp` switched itself off, so the problem didn't show.)

The decision (2026-11-08) is to keep the anchor and drop `frame_comp`. The anchor flies heights on PX4's
rangefinder-fused estimate, not the drifting camera height, and it has no 1 m cut-off.

`patch_team13_frame_comp.py` adds a `frame_comp` launch argument (default `false`) and passes it to the
`team13_mission` node. It refuses to run unless the anchor patch is already in, or if `frame_comp` is already
there. Nothing in `node.py` changes. To use team13's compensation instead for a run, pass
`frame_comp:=true anchor_setpoints:=false`. Never set both to true.

```bash
scp patch_team13_frame_comp.py dronehacks@<drone-ip>:/tmp/
ssh dronehacks@<drone-ip>
mkdir -p ~/team_ws/backups/frame-comp && cp -p ~/team_ws/src/team13_mission/launch/team13_mission.launch.py ~/team_ws/backups/frame-comp/
python3 -I /tmp/patch_team13_frame_comp.py ~/team_ws/src/team13_mission/launch/team13_mission.launch.py
```

It takes effect at the next launch, after a stack restart on the ground with the drone still. Check:

```bash
ros2 param get /team13_mission frame_comp       # False
ros2 param get /flight_interface anchor_setpoints  # True
# the competitor log must NOT contain "frame_comp: PX4 local - odom"
```

To roll back only this patch, copy `~/team_ws/backups/frame-comp/team13_mission.launch.py` back.

## Third and fourth patches: the team13_nav paths (applied 2026-11-08 01:25)

`team13_onboard.sh` has more flight paths than `use blar`:

| Command | Launches | Height correction now |
|---|---|---|
| `use blar` | `team13_mission.launch.py` | anchor on, `frame_comp` off (patches 1 + 2) |
| `fly <stage>` (also Mac-side `team13.sh`) | `team13_nav/launch/team13.launch.py` | anchor on, `sky_frame_comp` off (patch 3) |
| `axis`, `wall` | `team13.launch.py` + `tools/axis_test.py` / `wall_hop.py` | unchanged: the tools' own live offset, anchor off (patch 4) |

- `patch_team13_nav_launch.py` adds `anchor_setpoints` (default true), `interface_start_delay_s` (default 60;
  flight_interface **and** the bag, stereo and mission nodes start that late), and `sky_frame_comp` (default false,
  passed to `official_mission`). It also logs "flight_interface starts in 60 s - wait for 'envelope origin captured'
  before arming". It is SHA-guarded against the 14:44 file (`7264bee6…`).
- `patch_team13_onboard.py`: `axis_test.py` and `wall_hop.py` add their own live (PX4 − VIO) offset and have no
  switch to turn it off. So `axis` and `wall` now launch with `anchor_setpoints:=false interface_start_delay_s:=0`,
  exactly as before. The `fly` and `use blar` messages tell the pilot to wait about 60 s for "envelope origin
  captured". It is SHA-guarded against the 23:15 script (`a44a1fdb…`).

Applied at 01:25:30. Backups are in `~/team_ws/backups/nav-height-fix/*.orig`. The drone's files are byte-identical
to the locally tested versions (`88483b71…` launch, `6b798d02…` script). `ros2 launch team13_nav team13.launch.py
--show-args` loads with the real launch libraries (exit 0). The installed launch file is a symlink to the source, so
no rebuild is needed. Nothing was restarted, and none of this has flown yet.

Roll back:

```bash
cp -p ~/team_ws/backups/nav-height-fix/team13.launch.py.orig ~/team_ws/team13_pkgs/team13_nav/launch/team13.launch.py
cp -p ~/team_ws/backups/nav-height-fix/team13_onboard.sh.orig ~/team_ws/team13_pkgs/team13_nav/official/realdrone/team13_onboard.sh
```

## What this does not fix

- The camera height still drifts at rest (about 0.4 m over 6 min, measured). Keep flights short, and consider
  using PX4's rangefinder height (`dist_bottom`) for the descent.
- PX4 height resets after the anchor is recorded aren't followed by the interface. PX4's reset counter rose
  4 → 6 in 6 min at rest, while its z stayed at −1.16 m.
- Ask the organisers why PX4's height on drone 05 sits 1.2 m below its own zero point on the floor.
- The heading offset (+9° → −3.4°, not yet aligned) is a separate fix.
