#!/usr/bin/env python3
"""nav.node - the ROS 2 shell around nav.mission. No flight decisions here.

Same pattern as refnav/node.py: every input arrives through standard ROS interfaces and is handed
to the pure mission with its time; the mission's Output is published as native PX4 messages.

  in   /fmu/out/{vehicle_local_position_v1, vehicle_attitude, vehicle_status_v1,
                 vehicle_land_detected, estimator_status_flags}
       health_topic (std_msgs/Bool), vio_state_topic (diagnostic_msgs/DiagnosticStatus)
       depth_topic + its CameraInfo, TF base_link <- depth optical frame (checked: optical axis
       within 25 deg of body forward, as refnav)
  out  /fmu/in/{offboard_control_mode, trajectory_setpoint, vehicle_command}
       status_topic (std_msgs/String JSON, 2 Hz), event log (nav.eventlog, JSON lines)

Topic names are parameters (lab: x86 + Jazzy, Jetson: ARM + Humble).
Run:  python3 fly.py            (or python3 -m nav.node --ros-args -p use_sim_time:=true)
"""
from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

from refnav import core as RC
from refnav import depth as D
from refnav.node import CMD_ARM, CMD_LAND, CMD_SET_MODE, PX4_MAIN_MODE_OFFBOARD, PX4_QOS

from . import config as CFG
from . import course as COURSE
from . import frames as F
from . import landmarks as LM
from .eventlog import EventLog
from .posebuf import PoseBuffer, yaw_rate_dps

CAMERA_MAX_OFF_FORWARD_DEG = 25.0      # refnav/node.py: a non-optical or non-forward frame is refused


class NavNode(Node):
    def __init__(self):
        super().__init__("nav")
        P = self.declare_parameter
        map_file = P("map_file", "").value or str(COURSE.default_map_path())
        cfg_file = P("nav_config", "").value or str(CFG.default_config_path())
        overrides = json.loads(P("overrides", "{}").value or "{}")
        self.course = COURSE.load(map_file)
        self.params = CFG.load(cfg_file, overrides)
        self.mission = CFG.build(self.course, self.params)
        self.lm_params = self.params["landmarks"]

        self.depth_topic = P("depth_topic", "/camera/depth/image_raw").value
        info = P("depth_info_topic", "").value
        self.info_topic = info or self.depth_topic.rsplit("/", 1)[0] + "/camera_info"
        self.health_topic = P("health_topic", "/estimator/healthy").value
        self.vio_state_topic = P("vio_state_topic", "/estimator/vio_state").value
        self.base_frame = P("base_frame", "base_link").value
        self.exit_when_done = bool(P("exit_when_done", True).value)
        self.max_yaw_rate_dps = float(P("depth_max_yaw_rate_dps", 15.0).value)   # skip frames while turning
        self.pose_match_s = float(P("depth_pose_match_s", 0.1).value)            # pose within this of the stamp
        status_topic = P("status_topic", "/nav/status").value
        log_dir = P("log_dir", "").value or str(Path(__file__).resolve().parent.parent / "logs")
        self.log = EventLog(log_dir, "nav")
        self.log.event(self._now(), "start", map_file=map_file, nav_config=cfg_file, overrides=overrides,
                       params={k: vars(v) for k, v in self.params.items()})
        # PX4-bound stamps use the system clock (see refnav/node.py)
        self.sys_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        from diagnostic_msgs.msg import DiagnosticStatus
        from px4_msgs.msg import (EstimatorStatusFlags, OffboardControlMode, TrajectorySetpoint, VehicleAttitude,
                                  VehicleCommand, VehicleLandDetected, VehicleLocalPosition, VehicleStatus)
        self._OCM, self._TSP, self._CMD = OffboardControlMode, TrajectorySetpoint, VehicleCommand
        self.pub_ocm = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", PX4_QOS)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", PX4_QOS)
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", PX4_QOS)
        self.pub_status = self.create_publisher(String, status_topic, 10)

        self.v = RC.Vehicle()
        self.lpos_buf, self.att_buf = PoseBuffer(), PoseBuffer()   # (pos NED, heading) / q, by receive time
        self.hdg_buf = PoseBuffer()                                  # heading alone, for the yaw rate
        self._yaw_align = (False, None)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._on_lpos, PX4_QOS)
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self._on_att, PX4_QOS)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self._on_status, PX4_QOS)
        self.create_subscription(VehicleLandDetected, "/fmu/out/vehicle_land_detected", self._on_land, PX4_QOS)
        self.create_subscription(EstimatorStatusFlags, "/fmu/out/estimator_status_flags", self._on_est_flags, PX4_QOS)
        self.create_subscription(Bool, self.health_topic, self._on_health, 10)
        self.create_subscription(DiagnosticStatus, self.vio_state_topic, self._on_vio_state, 10)
        self.create_subscription(CameraInfo, self.info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self._on_depth, qos_profile_sensor_data)

        self.K = None
        self.extrinsic = None
        self.rejects = {}
        self.depth_ms = 0.0
        self.done_at = None
        self.create_timer(0.05, self._tick)            # 20 Hz on the node clock
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(f"nav: map {map_file}; config {cfg_file}; depth {self.depth_topic}; "
                               f"sim time {self.get_parameter('use_sim_time').value}")

    # ------------------------------------------------------------------ inputs
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_est_flags(self, m):
        self._yaw_align = (bool(m.cs_yaw_align), self._now())

    def _on_lpos(self, m):
        self.v.pos = (m.x, m.y, m.z)
        self.v.heading = m.heading
        if all(math.isfinite(c) for c in (m.x, m.y, m.z, m.heading)):
            t = self._now()
            self.lpos_buf.add(t, ((m.x, m.y, m.z), m.heading))
            self.hdg_buf.add(t, m.heading)
        ya, ya_t = self._yaw_align
        heading_ok = bool(m.heading_good_for_control) or (ya and ya_t is not None and self._now() - ya_t <= 3.0)
        self.v.pos_ok = bool(m.xy_valid and m.z_valid and heading_ok
                             and all(math.isfinite(c) for c in (m.x, m.y, m.z, m.heading)))

    def _on_att(self, m):
        self.v.q = tuple(float(c) for c in m.q)
        self.att_buf.add(self._now(), self.v.q)

    def _on_status(self, m):
        self.v.armed = m.arming_state == 2
        self.v.nav_state = int(m.nav_state)

    def _on_land(self, m):
        self.v.landed = bool(m.landed)

    def _on_health(self, m):
        self.mission.on_health(self._now(), m.data)

    def _on_vio_state(self, m):
        lost = 0.0
        for kv in m.values:
            if kv.key == "lost_s":
                try:
                    lost = float(kv.value)
                except ValueError:
                    pass
        self.mission.on_vio_state(self._now(), m.message, lost)

    def _on_info(self, m):
        if m.k[0] > 0 and m.k[4] > 0:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _reject(self, key, detail):
        self.rejects[key] = self.rejects.get(key, 0) + 1
        if self.rejects[key] in (1, 10, 100, 1000):
            self.get_logger().warning(f"depth frame not used ({self.rejects[key]}x): {detail}")

    def _calibration(self, frame_id):
        if self.extrinsic is not None and self.extrinsic[0] == frame_id:
            if self.extrinsic[3] is not None:
                self._reject("camera_not_forward", self.extrinsic[3])
                return None
            return self.extrinsic
        from rclpy.time import Time
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame_id, Time())
        except Exception as e:
            self._reject("no_tf", f"no TF {self.base_frame} <- {frame_id} ({type(e).__name__})")
            return None
        r, t = tf.transform.rotation, tf.transform.translation
        R = D.quat_to_mat(r.w, r.x, r.y, r.z)
        fwd = R[:, 2]                                  # the optical axis (z) in the body frame
        ang = math.degrees(math.acos(max(-1.0, min(1.0, fwd[0] / np.linalg.norm(fwd)))))
        bad = None
        if ang > CAMERA_MAX_OFF_FORWARD_DEG:
            bad = (f"{frame_id}'s optical axis is {ang:.0f} deg off the body's forward axis: not a forward "
                   f"REP-103 optical frame (x right, y down, z forward)")
        self.extrinsic = (frame_id, R, np.array([t.x, t.y, t.z]), bad)
        self.log.event(self._now(), "calibration", frame=frame_id, xyz=[t.x, t.y, t.z],
                       q_wxyz=[r.w, r.x, r.y, r.z], optical_axis_off_forward_deg=round(ang, 1), ok=bad is None)
        if bad is not None:
            self._reject("camera_not_forward", bad)
            return None
        return self.extrinsic

    def _pose_at(self, stamp):
        """(pos NED, heading, q) of PX4 nearest the image stamp, or the newest one (counted as
        stamp_unmatched) when no pose is within depth_pose_match_s of it."""
        lp, at = self.lpos_buf.at(stamp, self.pose_match_s), self.att_buf.at(stamp, self.pose_match_s)
        if lp is None or at is None:
            self._reject("stamp_unmatched", f"no PX4 pose within {self.pose_match_s:.2f} s of the image stamp "
                                            "(using the newest pose)")
            return self.v.pos, self.v.heading, self.v.q
        return lp[1][0], lp[1][1], at[1]

    def _on_depth(self, m):
        now = self._now()
        self.mission.on_depth_frame(now)                # the camera is alive, whatever the frame holds
        if self.K is None:
            return self._reject("no_camera_info", f"no camera_info on {self.info_topic}")
        cal = self._calibration(m.header.frame_id)
        if cal is None:
            return None
        if self.v.q is None or self.v.pos is None or self.v.heading is None:
            return self._reject("no_pose", "no PX4 attitude/position yet")
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        rate = yaw_rate_dps(self.hdg_buf, stamp)
        if rate > self.max_yaw_rate_dps:
            return self._reject("turning", f"yawing {rate:.0f} deg/s > {self.max_yaw_rate_dps:.0f} deg/s")
        try:
            d = D.decode_depth(m.encoding, m.height, m.width, m.step, bytes(m.data))
        except ValueError as e:
            return self._reject("undecodable", str(e))
        pos, heading, q = self._pose_at(stamp)
        x, y, z = F.ned_to_enu(*pos)
        pose = (x, y, z, F.heading_to_yaw(heading))
        floor_z = self.mission.takeoff_odom[2] if self.mission.takeoff_odom is not None else z
        t0 = time.perf_counter()
        pts = LM.wall_points(d, self.K, cal[1], cal[2], q, pose, self.lm_params, floor_z=floor_z)
        self.depth_ms = 0.9 * self.depth_ms + 0.1 * (time.perf_counter() - t0) * 1e3
        if pts is None:
            return self._reject("unusable", "too few valid depth pixels")
        self.mission.on_points(now, pts, z)
        return None

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
        if now <= 0.0:
            return
        out = self.mission.step(now, self.v)
        for e in out.events:
            self.get_logger().info(e)
            self.log.event(now, "mission", text=e)
        if out.events:
            self._publish_status()
        if out.stream and out.setpoint is not None:
            o = self._OCM()
            o.timestamp = self._sys_us()
            o.position = True
            self.pub_ocm.publish(o)
            s = self._TSP()
            s.timestamp = o.timestamp
            s.position = [float(c) for c in out.setpoint[:3]]
            s.velocity = [math.nan] * 3
            s.acceleration = [math.nan] * 3
            s.yaw = float(out.setpoint[3])
            self.pub_sp.publish(s)
        for c in out.commands:
            if c == "offboard":
                self._command(CMD_SET_MODE, 1.0, PX4_MAIN_MODE_OFFBOARD)
            elif c == "arm":
                self._command(CMD_ARM, 1.0)
            elif c == "land":
                self._command(CMD_LAND)
            elif c == "disarm":
                self._command(CMD_ARM, 0.0)
            self.log.event(now, "command", command=c)
        if self.mission.result is not None and self.done_at is None:
            self.done_at = now
            self.get_logger().info(f"mission done: {self.mission.result}")
            self.log.event(now, "done", result=self.mission.result)

    def _publish_status(self):
        now = self._now()
        st = self.mission.status(now)
        st["rejects"] = dict(self.rejects)
        st["depth_ms"] = round(self.depth_ms, 1)
        txt = json.dumps(st, default=float)
        self.pub_status.publish(String(data=txt))
        self.log.event(now, "status", status=json.loads(txt))
        if self.exit_when_done and self.done_at is not None and now - self.done_at > 1.5:
            raise SystemExit(0 if self.mission.result == "complete" else 1)


def main(argv=None):
    rclpy.init(args=list(argv) if argv is not None else sys.argv)
    node = NavNode()
    code = 1
    try:
        rclpy.spin(node)
    except SystemExit as e:
        code = int(e.code or 0)
    except KeyboardInterrupt:
        code = 130
    finally:
        node.log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
