# The MC mode that fits skybeta-05's Orin

Source: the organisers' repo that runs on the drone (`~/skybeta` on the Orin), see `SOURCE.txt`.
Not yet checked against the Orin itself: it could not be reached on enduroram (old IP 10.8.116.223 timed
out, `skybeta-05.local` did not resolve). Confirm with `cd ~/skybeta && git log -1` on the drone.

## Which file is "the MC mode" here

| Workshop (ROSCon deck) | On skybeta | File |
|---|---|---|
| Custom MC mode (`px4_ros2::ModeBase`, e.g. `CustomWaypoints`) | **"Skybeta Goto"**: a `px4_ros2::ModeBase` that flies to one NED target with `GotoSetpointType`, then calls `completed()` | `competitor_advanced_cpp/include/skybeta_goto_mode.hpp`, `src/main.cpp` |
| MC mode executor (Takeoff → modes → Land) | **No executor.** The supported equivalent is a state machine in your own node: WAIT_FOR_OFFBOARD → HOVER → SQUARE → HOLD | `competitor_example/competitor_example/square_mission.py` |

Both build against what the skybeta image already contains (`px4_msgs` and `px4-ros2-interface-lib` at
`release/1.16`, matching PX4 v1.16.2). **Do not add `px4_msgs` to `~/team_ws`** (competitor guide §5).

## Why the Python track is the main one

Rules from `competitor_guide.md`:

- Your code publishes ENU setpoints to `/skybeta/setpoint/pose` or `/skybeta/setpoint/twist`.
  `skybeta_flight_interface` clamps them (altitude 0.3–3 m, twist ≤ 1.5 m/s) and sends them to PX4.
  **Do not publish to `/fmu/in/*` yourself.**
- The safety pilot arms on Altitude, then switches to Position. The interface then enters Offboard once,
  and your node does the takeoff. There is no arm service and no land command: the pilot lands.
- "Skybeta Goto" (§7) is the advanced track. It has **no safety envelope**, uses NED (+z is down), and
  **nothing selects it as shipped**. Putting it on the RC switch is an FC parameter change, at your own risk.
- A workshop-style `ModeExecutor` would need its owned mode selected and would command takeoff/land itself.
  That doesn't fit the skybeta arming and Offboard flow.

## Workshop executor steps → skybeta equivalents

| Workshop executor step | Python track (`square_mission.py` pattern) |
|---|---|
| `onActivate()` when the owned mode is selected | wait for `offboard_active: true` in `/skybeta/state` |
| `takeoff()` | publish a pose at start + hover height; Offboard start lifts it |
| `scheduleMode(CustomWaypoints)` | step through an ENU waypoint list, checking arrival with `math.dist` |
| `scheduleMode(CustomYaw)` | send poses with a yaw quaternion (`yaw_enu`), stepping the heading |
| `land()` | not available: hold, then the pilot lands |
| `onDeactivate()` / failsafe | `offboard_active` goes false → stop advancing; no re-entry until re-armed |
