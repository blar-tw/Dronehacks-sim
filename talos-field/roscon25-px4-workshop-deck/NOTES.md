# ROSCon '25 PX4 workshop deck: scrape and executor notes

Source: Google Slides deck "Advanced Aerial Robotics with PX4 and ROS: Custom Flight Modes and Beyond"
(Dronecode workshop; Pozzan / Pordi / Guijarro), deck id `1Dkp2tvANRnrUpToBEG1AI2GcibONjgqqOaggqyTtFVc`.
Scraped 2026-11-07. The linked slide (`h33f0ca872e7eafa7_1_7`) is slide 102, "VTOL executor video".

## What is in this folder

| Path | What it is |
|---|---|
| `deck.pdf` | Full deck, 142 slides, as rendered by Google |
| `deck.pptx` | Editable export (images included; videos are links only) |
| `deck.txt` | Plain-text export of every slide |
| `target-slide.png` | The slide the link pointed at |
| `slide-images/slide-097..106.png` | The Mode Executors / VTOL / MC section |
| `videos/` | All 9 embedded Google Drive videos, named by slide |
| `reference-code/` | `vtol_executor` and `custom_executor_demo` from Dronecode/roscon-25-workshop (see `SOURCE.txt`, `LICENSE`) |

Videos:

| Slide | File | Length | Topic |
|---|---|---|---|
| 93 | `slide093-whats-new_rover_teleop.mp4` | 1:04 | Rover teleop (new rover setpoints) |
| 96 | `slide096-custom-mode_custom_modes.mp4` | 2:52 | Custom modes demo |
| **102** | **`slide102-vtol-executor_vtol.mp4`** | **4:53** | **VTOL executor** (`make px4_sitl gz_standard_vtol`) |
| **103** | **`slide103-mc-mode-executor_custom.webm`** | short | **MC (multicopter) mode executor** |
| 112 | `slide112-sar-modes_sar_modes.mp4` | 2:17 | SAR modes + SAR auto executor |
| 119 | `slide119-aruco_aruco.webm` | short | ArUco detection |
| 123 | `slide123-teleop-lidar_teleop.webm` | short | Teleop + LiDAR scan |
| 125 | `slide125-precision-land_precision.webm` | short | Precision landing mode |
| 130 | `slide130-precision-executor_precision_executor.webm` | short | Precision landing executor |

## Mode vs Mode Executor (px4-ros2-interface-lib)

- **Mode** (`px4_ros2::ModeBase`): one behaviour. It streams setpoints from `updateSetpoint()` and calls
  `completed()` when it is done. It shows up in QGC like a built-in PX4 mode.
- **Mode Executor** (`px4_ros2::ModeExecutorBase`): a state machine that sequences modes. It owns exactly one
  mode. When the pilot selects that mode, the executor activates and can chain PX4's built-in actions
  (`takeoff()`, `land()`, `rtl()`, `waitUntilDisarmed()`) and custom modes (`scheduleMode()`). Each step
  reports a `Result` to a callback, which picks the next state.

Why it matters: it replaces "offboard mode + a script firing MAVLink commands" with something PX4 still supervises.

- The executor is in charge only while its owned mode is selected. If the pilot switches mode (RC or GCS) or a
  failsafe triggers, it drops out. If the failsafe clears, it is reactivated.
- If the ROS node crashes or stops responding while its mode is selected, PX4 triggers a failsafe instead of
  waiting for setpoints that never arrive.
- Arming checks and mode requirements (for example "needs a valid local position") are enforced by PX4.
- Sequencing is event-driven (step completed → next step), not timer-driven.

## VTOL executor (slides 99–102, `reference-code/vtol_executor`)

Executor: **Takeoff (MC) → `VtolCruise` custom mode → RTL → WaitUntilDisarmed**.

The custom mode owns both transitions:
1. `toFixedwing()`. While transitioning, it streams a `TrajectorySetpointType` (acceleration from
   `computeAccelerationSetpointDuringTransition()`) and a `FwLateralLongitudinalSetpointType` course on every cycle.
2. Once `VTOL::getCurrentState() == FixedWing`, it flies course, altitude and airspeed for `cruise_duration_s`.
3. `toMulticopter()`, the same transition streaming, then `completed()`.

The point the slides make: **if an external mode commands the transition, it must stream the correct setpoints
during the transition.** PX4 only does this automatically for onboard-triggered transitions, such as an RC switch.
The slide 101 exercise, "change land to RTL", is already applied in the repo, because the back-transition ends far from home.

Requirements: a VTOL airframe, and the `px4_ros2::VTOL` + `FwLateralLongitudinalSetpointType` API, which the PX4
docs mark **PX4 v1.17**. The workshop builds against `release/1.18`. Altitudes are AMSL.

## MC mode executor (slide 103, `reference-code/custom_executor_demo`)

The same pattern on a quad: **Takeoff (2 m) → CustomWaypoints → CustomYaw (360°) → Land → WaitUntilDisarmed**.
The pilot selects *CustomWaypoints* in QGC, then arms, and the executor does the rest. The frame in the video
shows this demo's README; it was called `custom_mode_demo` in the older repo layout.

## Relevance to TALOS / skybeta-05

Facts from the drone's own parameter backup (`../pixhawk-accel-cal-2026-11-07/parameters-153659.json`) and the
cal summary:

| Param | Value | Meaning |
|---|---|---|
| firmware | 1.16.2 | older than the 1.17 VTOL API |
| `SYS_AUTOSTART` / `MAV_TYPE` / `CA_ROTOR_COUNT` | 4001 / 2 / 4 | generic quadrotor; no `VT_TYPE` → **not a VTOL** |
| `UXRCE_DDS_CFG` / `SER_TEL2_BAUD` / `UXRCE_DDS_DOM_ID` | 102 (TELEM2) / 921600 / 5 | the PX4↔ROS 2 DDS bridge is on, over serial |
| `EKF2_GPS_CTRL` / `EKF2_EV_CTRL` / `EKF2_HGT_REF` | 0 / 9 / 2 | GPS not fused; external vision for position and yaw; rangefinder height |
| `COM_MODE_ARM_CHK` | 0 | external modes **cannot register while armed** |

**VTOL executor: not usable here.** The airframe is a quad, the firmware lacks the 1.17 VTOL setpoint API, and the
demo relies on AMSL altitudes and RTL, which assume GPS. Treat it only as a reference for how a custom mode can
run its own internal phases inside an executor.

**MC mode executor: directly relevant.** It is the template for an autonomous run on skybeta-05: a ROS 2 node
on the Orin, over the TELEM2 DDS link that is already configured, does takeoff → your mission modes → land. The
pilot can take over at any time by switching modes. Things to get right first:

1. **Version match.** Build `px4_msgs` and `px4-ros2-interface-lib` from their `release/1.16` branches. The workshop's
   1.18 workspace will fail the library's message-compatibility check against 1.16.2 firmware. The alternative is
   PX4's message translation node with that check disabled.
2. **Start order.** With `COM_MODE_ARM_CHK = 0`, start the node and let its mode register *before* arming.
3. **Indoor positioning.** Built-in Takeoff/Land/RTL need a valid position estimate, and RTL needs a home
   position. With GPS fusion off, prefer `land()` over `rtl()` and avoid AMSL altitudes unless the EKF has a
   global origin set. Prove the sequence in SITL with an EV-only setup before flying.
4. The closest workshop example to an indoor quad mission is `precision_land_executor`
   (Takeoff → waypoints → ArUco precision land; slides 125/130).

Not verified: what ROS 2 distro and packages are already in `~/team_ws` on the Orin, and the TALOS mission
goals beyond the drone's configuration.
