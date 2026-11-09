"""Stop one offline replay if the drone is armed, so a replay never competes with a live flight for
the GPU. Read-only on the drone's domain: it only subscribes to /skybeta/state.
    python3 arm_guard.py <pid> [<pid> ...]      (run_ab.sh starts it with its own PIDs)
Signals each PID and its children, never a name pattern, so another replay on the drone is untouched."""
import json, os, signal, subprocess, sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

pids = [int(p) for p in sys.argv[1:]]
rclpy.init()
node = Node('imu_ab_arm_guard')


def cb(msg):
    try:
        armed = json.loads(msg.data).get('armed') is True
    except ValueError:
        return
    if armed:
        node.get_logger().error(f'drone ARMED: stopping replay PIDs {pids}')
        for p in pids:
            subprocess.run(['pkill', '-INT', '-P', str(p)])
            try:
                os.kill(p, signal.SIGINT)
            except ProcessLookupError:
                pass
        raise SystemExit(0)


node.create_subscription(String, '/skybeta/state', cb, 10)
node.get_logger().info(f'watching /skybeta/state; will stop PIDs {pids} on arm')
try:
    rclpy.spin(node)
except SystemExit:
    pass
