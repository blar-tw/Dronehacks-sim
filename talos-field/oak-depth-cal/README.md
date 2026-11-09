# OAK-D-S2 stereo depth: check, then (only if needed) recalibrate

Drone: skybeta-05. Tool on the Orin: `~/team_ws/oak-depth-cal/depth_check.py` (copy of this folder).
It only subscribes. The VIO -> PX4 path is not touched, and nothing is published.

## What we know (2026-11-08)

- The driver undistorts both mono cameras on the camera (`left/right.i_undistorted = true`).
  That is why `camera_info` has D = 0. That is correct for these images.
- `camera_info` has no stereo rectification: R = identity, and the baseline term P[3] is on the
  LEFT info (-47.9). ROS stereo tools such as stereo_image_proc read the baseline from the
  RIGHT info, so they would see a 0 baseline. depth_check.py therefore rectifies with the factory
  left->right extrinsics from `~/team_ws/oak-calibration/factory-calibration-backup.json`.
- The factory baseline is 75.29 mm. camera_info implies 75.28 mm.
- The organisers' camera config has no StereoDepth node (`i_pipeline_type: stereo`), so the OAK
  itself does not output depth. This tool computes depth on the Orin CPU (SGBM, 512x320, 2 Hz).

## 1. Accuracy check (do this first; it needs the AprilGrid board)

The board is the team's 6x6 tag36h11 AprilGrid: 20 mm tags, 6 mm gaps, held rigid and flat.
Its distance comes from PnP on the left image, which uses no stereo, so it is an independent
check of the stereo depth.

1. Drone on the ground with the stack running. Hold the board still, facing the camera, at about
   0.4 m, then 0.7 m, then 1.0 m. Beyond about 1.2 m the 20 mm tags get too small to detect.
2. At each distance, on the Orin:

```
docker exec -it skybeta-stack bash -c 'source /opt/ros/humble/setup.bash; cd /root/team_ws/oak-depth-cal; python3 depth_check.py --seconds 15 --out results.jsonl'
```

3. Read the last line, `stereo depth error median ±x.xx%`. Rule of thumb:
   - within ±2 % at all three distances: the factory calibration is fine; do NOT flash.
   - a consistent bias (for example +5 % everywhere): the stereo extrinsics are off; go to step 2.
   - an error that grows with distance or flips sign: suspect board detection or motion first.
     Re-run holding the board still.

Without the board, `centre` gives the median depth in the middle of the image. Use it against a
textured wall at tape-measured distances.

## 2. Recalibrate + flash (only if step 1 fails)

This changes the camera model that cuVSLAM, and so the drone's position estimate, depends on.
Plan for a full VIO re-check afterwards.

1. The backup already exists: `~/team_ws/oak-calibration/factory-calibration-backup.json`
   (Nov 6, 12 KB, board DM9098). Make a second copy off the drone.
2. The OAK can only be opened by one process. Calibrating means the stack's camera driver must
   be stopped, which kills VIO. Do it on the ground, with the FC disarmed.
3. Use Luxonis's calibration tool with a printed ChArUco board, measured with calipers. It needs a
   screen to guide the board poses. The easiest setup is the OAK on a laptop over USB-C.
4. After flashing, run step 1 again. Then restart the stack (`docker rm -f skybeta-stack`; the
   service brings it back) and re-check VIO at rest and on a hand-carried walk.
5. To undo, flash the backup JSON back with depthai's `CalibrationHandler` +
   `Device.flashCalibration`.

## 3. Camera-IMU (Kalibr): already done, needs one more step

`~/team_ws/oak-calibration/releases/2026-11-07-motion-003`. The camera-IMU extrinsic agrees with the
factory IMU extrinsic within about 5 mm, and the time shift is 3.8 ms. It is still marked
"candidate". See `../imu-fusion-ab/` for the replay test of IMU fusion that uses it.
