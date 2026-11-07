"""nav.node - ROS 2 shell around nav.mission (PX4 in/out, depth -> localizer, status)."""
import json
import math
import sys
from collections import deque

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

from refnav import depth as D

from . import course as COURSE
from . import planner
from .frames import enu_to_ned, heading_to_yaw, ned_to_enu, yaw_to_heading
from .localizer import Localizer, wall_points
from .mission import Mission, Vehicle
from .yaw import ViewScorer

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
CMD_SET_MODE, CMD_ARM, CMD_LAND = 176, 400, 21
MAX_YAW_RATE = math.radians(15.0)       # depth frames are skipped while turning faster


class NavNode(Node):
    def __init__(self):
        super().__init__("nav", automatically_declare_parameters_from_overrides=True)
        prm = lambda k, d: self.get_parameter(k).value if self.has_parameter(k) else d
        course = COURSE.load(prm("course_map", str(COURSE.DEFAULT_MAP)))
        pp = planner.PlanParams(height_m=float(prm("height_m", 0.6)))
        wps = planner.plan(course, pp)
        z_map = sum(pp.band_z) / 2                     # wall faces at the body's height in the map
        self.loc = Localizer(course, z_map)
        self.mission = Mission(course, wps, self.loc, ViewScorer(course, z_map))
        self.get_logger().info(f"planned {len(wps)} waypoints: " + " ".join(
            f"({w.x:.2f},{w.y:.2f}{',' + w.label if w.label else ''}{'*' if w.kind == 'stop' else ''})" for w in wps))

        self.depth_topic = prm("depth_topic", "/camera/depth/image_raw")
        info_topic = prm("depth_info_topic", "/camera/depth/camera_info")
        health_topic = prm("health_topic", "/estimator/healthy")
        self.base_frame = prm("base_frame", "base_link")
        self.sys_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleAttitude, VehicleCommand,
                                  VehicleLocalPosition, VehicleStatus)
        self._OCM, self._TSP, self._CMD = OffboardControlMode, TrajectorySetpoint, VehicleCommand
        self.pub_ocm = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", PX4_QOS)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", PX4_QOS)
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", PX4_QOS)
        self.pub_status = self.create_publisher(String, "/nav/status", 10)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._on_lpos, PX4_QOS)
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self._on_att, PX4_QOS)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self._on_status, PX4_QOS)
        self.create_subscription(Bool, health_topic, lambda m: self.mission.on_health(self._now(), m.data), 10)
        self.create_subscription(CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self._on_depth, qos_profile_sensor_data)

        self.v = Vehicle()
        self.q = None
        self.K = None
        self.cal = None
        self.yaw_rate = 0.0
        self._prev_yaw = None
        self.poses = deque(maxlen=100)                 # (t, x, y, z, yaw): the pose at each depth frame's stamp
        self.create_timer(0.05, self._tick)
        self.create_timer(0.5, self._publish_status)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ inputs
    def _on_lpos(self, m):
        x, y, z = ned_to_enu(m.x, m.y, m.z)
        yaw = heading_to_yaw(m.heading)
        now = self._now()
        if self._prev_yaw is not None and now > self._prev_yaw[1]:
            self.yaw_rate = abs(math.remainder(yaw - self._prev_yaw[0], 2 * math.pi)) / (now - self._prev_yaw[1])
        self._prev_yaw = (yaw, now)
        self.v.pos, self.v.yaw = (x, y, z), yaw
        self.poses.append((m.timestamp_sample * 1e-6, x, y, z, yaw))
        self.v.pos_ok = bool(m.xy_valid and m.z_valid)

    def _on_att(self, m):
        self.q = tuple(float(c) for c in m.q)

    def _on_status(self, m):
        self.v.armed = m.arming_state == 2
        self.v.nav_state = int(m.nav_state)

    def _on_info(self, m):
        self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _on_depth(self, m):
        if self.mission.phase not in ("look", "follow") or self.K is None or self.q is None \
                or self.yaw_rate > MAX_YAW_RATE:
            return
        if self.cal is None:
            from rclpy.time import Time
            tf = self.tf_buffer.lookup_transform(self.base_frame, m.header.frame_id, Time())
            r, t = tf.transform.rotation, tf.transform.translation
            self.cal = (D.quat_to_mat(r.w, r.x, r.y, r.z), np.array([t.x, t.y, t.z]))
            self.get_logger().info(f"camera TF {self.base_frame} <- {m.header.frame_id}: t=({t.x:.3f},{t.y:.3f},{t.z:.3f})")
        d = D.decode_depth(m.encoding, m.height, m.width, m.step, bytes(m.data))
        pose = self._pose_at(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        pts = wall_points(d, self.K, self.cal[0], self.cal[1], self.q, pose, self.mission.floor_z())
        self.loc.update(pts)

    def _pose_at(self, stamp):
        """PX4 pose nearest the image stamp. PX4 stamps in its own clock: line the two clocks up on
        the newest sample (both streams arrive with little delay compared to each other)."""
        t_px4_now = self.poses[-1][0]
        target = t_px4_now - (self._now() - stamp)
        best = min(self.poses, key=lambda p: abs(p[0] - target))
        return best[1:]

    # ------------------------------------------------------------------ outputs
    def _sys_us(self):
        return self.sys_clock.now().nanoseconds // 1000

    def _command(self, cmd, p1=0.0, p2=0.0):
        c = self._CMD()
        c.timestamp = self._sys_us()
        c.command, c.param1, c.param2 = cmd, float(p1), float(p2)
        c.target_system = c.target_component = c.source_system = c.source_component = 1
        c.from_external = True
        self.pub_cmd.publish(c)

    def _tick(self):
        now = self._now()
        if now <= 0.0 or self.v.pos is None:
            return
        out = self.mission.step(now, self.v)
        if out.event:
            self.get_logger().info(out.event)
            self._publish_status()
        if out.stream and out.setpoint is not None:
            o = self._OCM()
            o.timestamp = self._sys_us()
            o.position = True
            self.pub_ocm.publish(o)
            s = self._TSP()
            s.timestamp = o.timestamp
            x, y, z, yaw = out.setpoint
            s.position = [float(c) for c in enu_to_ned(x, y, z)]
            s.velocity = [math.nan] * 3
            s.acceleration = [math.nan] * 3
            s.yaw = float(yaw_to_heading(yaw))
            self.pub_sp.publish(s)
        for c in out.commands:
            if c == "offboard":
                self._command(CMD_SET_MODE, 1.0, 6.0)
            elif c == "arm":
                self._command(CMD_ARM, 1.0)
            elif c == "land":
                self._command(CMD_LAND)
        if out.phase == "done":
            self.get_logger().info(f"mission done: {self.mission.result}")
            self._publish_status()
            raise SystemExit(0 if self.mission.result == "complete" else 1)

    def _publish_status(self):
        self.pub_status.publish(String(data=json.dumps(self.mission.status())))


def main(argv=None):
    rclpy.init(args=list(argv) if argv is not None else sys.argv)
    node = NavNode()
    code = 1
    try:
        rclpy.spin(node)
    except SystemExit as e:
        code = int(e.code or 0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)
