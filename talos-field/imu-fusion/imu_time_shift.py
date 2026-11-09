"""Republish /oak/imu/data on /replay/imu with Kalibr's camera-IMU time shift removed.
Kalibr convention: t_imu = t_cam + timeshift_cam_imu, so subtract the shift to put IMU samples on
the camera time axis. Frame set to oak_kalibr_imu_frame (the calibrated extrinsic)."""
import sys
from pathlib import Path
import rclpy, yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

shift_ns = round(float(yaml.safe_load(Path(sys.argv[1]).read_text())['left']['timeshift_cam_imu']) * 1e9)
rclpy.init()
node = Node('replay_imu_time_shift')
pub = node.create_publisher(Imu, '/replay/imu', QoSProfile(depth=400, reliability=ReliabilityPolicy.RELIABLE))

def cb(msg):
    t = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec - shift_ns
    if t <= 0:
        return
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(t, 10**9)
    msg.header.frame_id = 'oak_kalibr_imu_frame'
    msg.orientation_covariance[0] = -1.0
    pub.publish(msg)

node.create_subscription(Imu, '/oak/imu/data', cb, QoSProfile(depth=400, reliability=ReliabilityPolicy.RELIABLE))
node.get_logger().info(f'IMU stamps shifted by {-shift_ns} ns')
rclpy.spin(node)
