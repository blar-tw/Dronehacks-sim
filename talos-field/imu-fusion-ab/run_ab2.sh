#!/usr/bin/env bash
# Bounded version of run_ab.sh: every step ends on a timeout, children are killed by PID.
# Replays the first <dur_s> s of a flight bag at 0.5x through ONE cuVSLAM (imu on/off) on
# ROS_DOMAIN_ID=77 and records /replay/* to <out_dir>. Nothing reaches the drone's domain.
#   run_ab2.sh <bag.mcap> <dur_s> <true|false> <out_dir>
source /opt/ros/humble/setup.bash; source /root/skybeta_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID=77
BAG=$1; DUR=$2; IMU=$3; OUT=$4; NS=${5:-1.0}
HERE=$(cd "$(dirname "$0")" && pwd)
rm -rf "$OUT"
setsid ros2 launch "$HERE/vio_replay.launch.py" imu:="$IMU" noise_scale:="$NS" > "$OUT.launch.log" 2>&1 & LP=$!
sleep 10
timeout -s INT -k 5 $(( DUR * 2 + 15 )) ros2 bag record -s mcap -o "$OUT" /replay/odometry /replay/status > "$OUT.rec.log" 2>&1 & RP=$!
sleep 3
timeout -s INT -k 5 $(( DUR * 2 )) ros2 bag play "$BAG" -s mcap --rate 0.5 --topics /oak/left/image_raw /oak/right/image_raw \
  /oak/left/camera_info /oak/right/camera_info /oak/imu/data /tf_static > "$OUT.play.log" 2>&1
wait $RP
kill -INT -- -$LP 2>/dev/null; sleep 4; kill -9 -- -$LP 2>/dev/null
echo "done imu=$IMU -> $OUT ($(du -sh "$OUT" 2>/dev/null | cut -f1))"
