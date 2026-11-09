#!/usr/bin/env bash
# Inside the skybeta container. Replays the first <dur_s> seconds of a flight bag through ONE
# cuVSLAM (imu fusion on or off) on ROS_DOMAIN_ID=$AB_DOMAIN (default 77), records /replay/* to
# <out_dir>. Nothing is published on the drone's own domain, so PX4 and flight_interface never see it.
# Only one replay may run per domain: a second replay's cuVSLAM also eats this bag's images.
#   run_ab.sh <bag.mcap> <dur_s> <true|false> <out_dir>
set -uo pipefail
source skybeta_env.sh >/dev/null 2>&1
DRONE_DOMAIN=$ROS_DOMAIN_ID
export ROS_DOMAIN_ID=${AB_DOMAIN:-77}
BAG=$1; DUR=$2; IMU=$3; OUT=$4
HERE=$(cd "$(dirname "$0")" && pwd)
rm -rf "$OUT"
ros2 launch "$HERE/vio_replay.launch.py" imu:="$IMU" > "$OUT.launch.log" 2>&1 & LP=$!
sleep 15
ros2 bag record -s mcap -o "$OUT" /replay/odometry /replay/status > "$OUT.rec.log" 2>&1 & RP=$!
sleep 3
# -s mcap: crash bags have no metadata.yaml. The QoS override keeps /tf_static latched, so cuVSLAM's
# transient-local TF listener still gets the camera frames.
ros2 bag play "$BAG" -s mcap --rate 0.5 --qos-profile-overrides-path "$HERE/qos_overrides.yaml" \
  --topics /oak/left/image_raw /oak/right/image_raw \
  /oak/left/camera_info /oak/right/camera_info /oak/imu/data /tf_static > "$OUT.play.log" 2>&1 & PP=$!
# Stop this replay (and only this one) if the drone is armed. Reads /skybeta/state on the drone's domain.
ROS_DOMAIN_ID=$DRONE_DOMAIN python3 "$HERE/arm_guard.py" $PP $RP $LP > "$OUT.guard.log" 2>&1 & GP=$!
sleep $(( DUR * 2 ))
kill -INT $PP 2>/dev/null; sleep 3
kill -INT $RP 2>/dev/null; sleep 3
# A job started with & in a script inherits SIGINT=ignored, so `ros2 launch` never reacts to it (the
# old version hung here in `wait`). Signal its children (the C++ container, the IMU shifter) instead;
# launch exits once they have. By PID only: `pkill -f` would also hit another replay on this drone.
pkill -INT -P $LP 2>/dev/null
for _ in $(seq 20); do kill -0 $LP 2>/dev/null || break; sleep 1; done
if kill -0 $LP 2>/dev/null; then pkill -KILL -P $LP 2>/dev/null; kill -KILL $LP 2>/dev/null; fi
kill $GP 2>/dev/null
wait 2>/dev/null
echo "done imu=$IMU -> $OUT"
