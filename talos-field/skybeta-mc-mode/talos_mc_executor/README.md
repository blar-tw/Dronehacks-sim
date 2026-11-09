# talos_mc_executor

The ROSCon '25 workshop's multicopter mode executor (`custom_executor_demo`), ported to the skybeta flight
interface on skybeta-05's Orin.

| Workshop executor (px4_ros2) | Here | Notes |
|---|---|---|
| `onActivate()` when the owned mode is selected | Offboard starts (`offboard_active` rising edge, while armed) | Pilot arms on Altitude, then switches to Position |
| `takeoff()` | `TAKEOFF` | Carrot climbs at `climb_speed_mps` to `hover_altitude_m`, settles `takeoff_settle_s` |
| `scheduleMode(CustomWaypoints)` | `WAYPOINTS` | ENU offsets from the start, carrot at `cruise_speed_mps` |
| `scheduleMode(CustomYaw)` | `YAW_SPIN` | 360° in place at `yaw_rate_dps`. **Off by default** (see below) |
| `land()` | `DESCEND` then `HOLD` | No land command on skybeta: descends to `handoff_altitude_m`, then the pilot lands |
| `waitUntilDisarmed()` | disarm resets the mission | The next arm → Position runs it again from the new start point |
| `onDeactivate()` | `PAUSED` | Pilot took over: the mission stops and never resumes in that flight |

It talks only to `/skybeta/setpoint/pose`, `/skybeta/state` and `/visual_slam/tracking/odometry`. Nothing
touches `/fmu/*`, nothing arms, nothing changes mode.

**Safety behaviour that differs from the workshop:**
- A step that times out holds at the vehicle's current position. The workshop carries on to the next step.
- Setpoints move as a "carrot" no faster than the configured speeds and no more than `max_lead_m` ahead of
  the vehicle. Pose setpoints aren't speed-limited by the interface, and PX4 would fly a far pose at up to 12 m/s.
- The yaw spin is closed-loop: the commanded heading only advances while the measured heading is within
  `yaw_max_lead_deg`. It stops after `step_timeout_s` without following.
- If the node starts while Offboard is already active (a restart mid-flight), it holds where it is and never
  starts a mission.
- Inputs it doesn't trust:
  - NaN odometry is dropped.
  - Odometry older than `odom_timeout_s` freezes the last command, and no start point is captured from it.
  - A `/skybeta/state` older than `state_timeout_s` means nothing new is published.
- Limits checked at startup: speeds ≤ 1.5 m/s, `max_lead_m` ≤ 2 m, every height in [0.3, 3.0] m, publish rate
  above 2 Hz. If any is broken, the node refuses to start.

## Install on the drone

From the Mac, with the drone's Wi-Fi IP:

```bash
scp -r talos_mc_executor dronehacks@<drone-ip>:~/team_ws/src/
```

On the drone:

```bash
host/run.sh shell
team_build.sh --packages-select talos_mc_executor
```

To make the stack's `competitor` window run it, set these in `/etc/skybeta/drone.env`, then restart the
stack on the ground with the drone still:

```
COMPETITOR_PKG=talos_mc_executor
COMPETITOR_LAUNCH=talos_mission.launch.py
```

To run it by hand instead: stop what the `competitor` window is running first, because two flight interfaces
fight. Then:

```bash
ros2 launch talos_mc_executor talos_mission.launch.py
```

The launch file starts `flight_interface` exactly as `competitor.launch.py` does, with
`auto_offboard_on_arm:=true`. Envelope arguments work the same, e.g. `max_altitude_m:=2.0`. If you tighten the
envelope, keep the mission heights below it: a clamped target is never reached and ends in a hold.

## Before the first flight

1. Edit `config/mission.yaml`. The default is a 2 m square at 1.5 m. **There is no horizontal geofence**,
   so make the waypoints fit the room. Write numbers as floats (`2.0`).
2. Run the tests: `PYTHONPATH=. python3 -m unittest discover -s test -v` (no ROS needed).
3. Props off: arm on Altitude, switch to Position, and check the log reaches `TAKEOFF` and the setpoint z
   rises: `ros2 topic echo /skybeta/setpoint/pose`.
4. Then low and tethered. Record a bag every flight: `host/run.sh bag`.
5. To turn on the yaw spin, first measure the odom vs PX4 yaw offset with props off (competitor guide §1).
   If the offset is more than a few degrees, set `odom_to_ned_yaw_offset_rad` on `flight_interface`. Then
   set `yaw_spin_enabled: true`. Without that, the spin starts with a sudden turn the size of the offset,
   or stalls into a hold if the offset is over 25°.

## Tested so far

28 unit tests ran against a crude simulated vehicle. They cover:
- the step order
- every waypoint visited
- setpoint step size and how far the carrot leads the vehicle
- a full yaw turn, with a small offset, a large offset and a slow vehicle
- leaving the ground within 4 s of arming
- Offboard loss and re-arming
- a restart mid-flight
- stale and NaN odometry, including stale odometry at the switch
- stale state
- an unreachable waypoint
- the parameter limits

The ROS wiring was checked with stand-in `rclpy` modules. An independent review found no blocking defects,
and its fixes are applied.

**Not yet done:** building or running it on the Orin, in SITL, or in flight.
