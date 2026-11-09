# cuVSLAM IMU fusion: offline replay test (2026-11-08, skybeta-05)

This replays the 22:43 wall-crash flight (`team13_flights/20261107-224309_blar`) through one
cuVSLAM, with IMU fusion off or on. It runs on the Orin on ROS_DOMAIN_ID=77, so nothing reaches
PX4. The IMU-on runs use the team's Kalibr candidate (`oak-calibration/releases/2026-11-07-motion-003`):
the camera-IMU transform, a 3.8 ms time shift, and the measured noise. Times are epoch − 1794120000.
On the Orin: `~/team_ws/imu-ab-20261108/` (`run_ab2.sh <bag> 70 <true|false> <out> [noise_scale]`, then `ab_report.py`).

| run | normal flight: biggest step per frame | flight, RMS difference from IMU off | camera touching the wall, 225.4–226.2 | on the floor, 227–240 (truth: still) |
|---|---|---|---|---|
| IMU off | 1.8 cm (real motion) | — | jumps 13 m (441 m/s) | still, but 12 m wrong |
| IMU on, measured noise | 11.4 cm (3.4 m/s jerks at 223.8, 224.1, 225.1) | 2.9 cm | no jump (0.10 m/s), holds 0.76 m | ends about 1.8 m off after the 11 g impact |
| IMU on, noise ×10 | 16.4 cm | 4.2 cm | jumps (18 m/s) | still |

- At rest (201–215.5) all runs drift 2–4 mm, and the live run 4 mm.
- IMU on prevents the blinded-camera jump.
- With the current calibration, IMU on adds 8–11 cm jerks in normal flight. Inflating the noise makes them worse, so the camera-IMU alignment or timing is the more likely cause.
- **Not flight-ready.** Next test: the factory IMU extrinsic (`oak_imu_frame` from the EEPROM/URDF), with no time shift, on this bag and on one axis flight.
