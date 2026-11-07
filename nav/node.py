#!/usr/bin/env python3
"""nav.node - the ROS 2 shell around nav.mission. No flight decisions here.

Same pattern as refnav/node.py: every input arrives through standard ROS interfaces and is handed
to the pure mission with its time; the mission's Output is published as native PX4 messages.

  in   /fmu/out/{vehicle_local_position_v1, vehicle_attitude, vehicle_status_v1,
                 vehicle_land_detected, estimator_status_flags}
       health_topic (std_msgs/Bool), vio_state_topic (diagnostic_msgs/DiagnosticStatus)
       depth_topic + its CameraInfo, TF base_link <- depth optical frame
  out  /fmu/in/{offboard_control_mode, trajectory_setpoint, vehicle_command}
       status_topic (std_msgs/String JSON, 2 Hz), event log (nav.eventlog, JSON lines)

Depth timing: a frame is stamped at capture and arrives 50-150 ms later (stereo + transport); the
drone has moved meanwhile (0.5 m/s -> 2.5-7.5 cm, 60 deg/s yaw -> 3-9 deg = 10-31 cm sideways at
2 m, more than the aligner's match gate). The PX4 pose is therefore not read "latest" but from a
short history (nav.landmarks.PoseBuffer) at the image header stamp. All stamps are on the node
clock: the camera driver stamps on it, and PX4 samples are stamped at receipt (PX4's own timestamps
are on PX4's clock, see onboard/adapters/px4_odom.py).

Topic names are parameters (lab: x86 + Jazzy, Jetson: ARM + Humble).
Run:  python3 main.py            (or python3 -m nav.node --ros-args -p use_sim_time:=true)
"""
from __future__ import annotations

import json
import math
import sys
import time
import traceback
from pathlib import Path

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
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

# depth: best effort (matches a reliable or best-effort camera publisher) and ONE frame of history,
# so a slow tick never works through a backlog of stale frames - the newest frame is the useful one
DEPTH_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
LOG_AT = (1, 10, 100, 1000)


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
        self.poses = LM.PoseBuffer(self.lm_params.pose_buffer_s)
        self._yaw_align = (False, None)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._on_lpos, PX4_QOS)
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self._on_att, PX4_QOS)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self._on_status, PX4_QOS)
        self.create_subscription(VehicleLandDetected, "/fmu/out/vehicle_land_detected", self._on_land, PX4_QOS)
        self.create_subscription(EstimatorStatusFlags, "/fmu/out/estimator_status_flags", self._on_est_flags, PX4_QOS)
        self.create_subscription(Bool, self.health_topic, self._on_health, 10)
        self.create_subscription(DiagnosticStatus, self.vio_state_topic, self._on_vio_state, 10)
        self.create_subscription(CameraInfo, self.info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self._on_depth, DEPTH_QOS)

        self.K = None
        self.extrinsic = None               # (frame_id, R_body_cam, t_body_cam)
        self.rejects = {}                   # frames NOT used, by reason
        self.frames = {"points": 0, "empty": 0, "pose_fallback": 0, "unstamped": 0}   # frames used, how
        self.depth_ms = 0.0                 # EMA: wall_points CPU time per frame
        self.depth_latency_ms = 0.0         # EMA: image stamp -> arrival on the node clock
        self.pose_dt_ms = 0.0               # EMA: image stamp distance to the PX4 samples used
        self.heading_skew_deg = 0.0         # EMA: |lpos.heading - yaw(q)| at the image stamp (expect ~0)
        self.done_at = None
        self.create_timer(0.05, self._tick)            # 20 Hz on the node clock
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(f"nav: map {map_file}; config {cfg_file}; depth {self.depth_topic}; "
                               f"sim time {self.get_parameter('use_sim_time').value}")

    # ------------------------------------------------------------------ inputs
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _px4_age(m):
        """Seconds between a PX4 message's sample and its publication, on PX4's clock (a difference,
        so clock-independent); clamped so a bad field cannot shift a sample far."""
        age = (int(m.timestamp) - int(getattr(m, "timestamp_sample", m.timestamp))) * 1e-6
        return min(max(age, 0.0), 0.05)

    def _on_est_flags(self, m):
        self._yaw_align = (bool(m.cs_yaw_align), self._now())

    def _on_lpos(self, m):
        now = self._now()
        self.v.pos = (m.x, m.y, m.z)
        self.v.heading = m.heading
        ya, ya_t = self._yaw_align
        heading_ok = bool(m.heading_good_for_control) or (ya and ya_t is not None and now - ya_t <= 3.0)
        self.v.pos_ok = bool(m.xy_valid and m.z_valid and heading_ok
                             and all(math.isfinite(c) for c in (m.x, m.y, m.z, m.heading)))
        if self.v.pos_ok:                   # only poses PX4 itself trusts enter the depth history
            self.poses.push_pos(now - self._px4_age(m), m.x, m.y, m.z, m.heading)

    def _on_att(self, m):
        q = tuple(float(c) for c in m.q)
        self.v.q = q
        if all(math.isfinite(c) for c in q):
            self.poses.push_att(self._now() - self._px4_age(m), q)

    def _on_status(self, m):
        self.v.armed = m.arming_state == 2
        self.v.nav_state = int(m.nav_state)

    def _on_land(self, m):
        self.v.landed = bool(m.landed)

    def _on_health(self, m):
        self.mission.on_health(self._now(), bool(m.data))

    def _on_vio_state(self, m):
        lost = 0.0
        for kv in m.values:
            if kv.key == "lost_s":
                try:
                    lost = float(kv.value)
                except (TypeError, ValueError):
                    pass
        if not math.isfinite(lost):
            lost = 0.0
        self.mission.on_vio_state(self._now(), str(m.message), lost)

    def _on_info(self, m):
        if m.k[0] > 0 and m.k[4] > 0:
            self.K = (float(m.k[0]), float(m.k[4]), float(m.k[2]), float(m.k[5]))

    def _count(self, table, key, detail, level="warning"):
        """Count an event by key; log it WITH its numbers the 1st, 10th, 100th and 1000th time."""
        table[key] = table.get(key, 0) + 1
        if table[key] in LOG_AT:
            getattr(self.get_logger(), level)(f"depth {key} ({table[key]}x): {detail}")

    def _reject(self, key, detail):
        self._count(self.rejects, key, "frame not used: " + detail)

    def _calibration(self, frame_id):
        if self.extrinsic is not None and self.extrinsic[0] == frame_id:
            return self.extrinsic
        from rclpy.time import Time
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame_id, Time())
        except Exception as e:                      # not (yet) on the TF tree: tf2 raises several types
            self._reject("no_tf", f"no TF {self.base_frame} <- {frame_id} ({type(e).__name__})")
            return None
        r, t = tf.transform.rotation, tf.transform.translation
        R = D.quat_to_mat(r.w, r.x, r.y, r.z)
        # an OPTICAL frame has its z axis (R[:, 2] in body) along body-forward; a body-aligned TF
        # (z up) would turn every point into silent garbage, so refuse it (refnav/node.py pattern)
        fwd = R[:, 2]
        ang = math.degrees(math.acos(max(-1.0, min(1.0, float(fwd[0]) / (np.linalg.norm(fwd) or 1.0)))))
        if ang > self.lm_params.optical_axis_tol_deg:
            self._reject("camera_not_forward", f"TF {self.base_frame} <- {frame_id}: optical axis {ang:.0f} deg off "
                         f"body-forward (optical_axis_tol_deg {self.lm_params.optical_axis_tol_deg:g}); not an optical "
                         f"frame or not a forward camera")
            return None
        self.extrinsic = (frame_id, R, np.array([t.x, t.y, t.z], float))
        self.get_logger().info(f"calibration from TF: {self.base_frame} <- {frame_id} t=({t.x:.3f},{t.y:.3f},{t.z:.3f}) "
                               f"optical axis {ang:.1f} deg off forward")
        self.log.event(self._now(), "calibration", frame=frame_id, xyz=[t.x, t.y, t.z], axis_off_deg=round(ang, 2))
        return self.extrinsic

    def _on_depth(self, m):
        try:
            self._depth(m)
        except Exception as e:              # one bad frame must never take the node (setpoint stream) down
            self._reject("exception", f"{type(e).__name__}: {e}")
            if self.rejects["exception"] <= 3:
                self.get_logger().error(traceback.format_exc())

    def _depth(self, m):
        p = self.lm_params
        if self.K is None:
            return self._reject("no_camera_info", f"no camera_info on {self.info_topic}")
        cal = self._calibration(m.header.frame_id)
        if cal is None:
            return None
        if not self.poses.t_pos or not self.poses.t_att:
            return self._reject("no_pose", "no valid PX4 position/attitude yet (pos_ok needs xy_valid, z_valid, heading)")
        try:
            d = LM.depth_view(m.encoding, m.height, m.width, m.step, m.data, bool(m.is_bigendian))
        except ValueError as e:
            return self._reject("undecodable", str(e))
        now = self._now()
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:                    # an unstamped image: arrival time is the best there is
            self._count(self.frames, "unstamped", "depth image has no header stamp; using arrival time")
            stamp = now
        latency = now - stamp
        if latency > p.max_latency_s:       # a backlog or a clock mismatch: the pose history cannot cover it
            return self._reject("stale", f"image {latency:.2f} s old on arrival (max_latency_s {p.max_latency_s:g}); "
                                         f"camera and node on different clocks (use_sim_time)?" if latency > 2.0
                                else f"image {latency:.2f} s old on arrival (max_latency_s {p.max_latency_s:g})")
        if latency < -p.pose_max_dt_s:
            return self._reject("future", f"image stamped {-latency:.2f} s in the future: clocks disagree")
        pose, q, dt_pos, dt_att = self.poses.at(stamp)
        dt = max(dt_pos, dt_att)
        if dt > p.pose_max_dt_s:
            self._count(self.frames, "pose_fallback", f"image stamp lies {dt * 1e3:.0f} ms outside the PX4 pose history "
                        f"(latency {latency * 1e3:.0f} ms, pose_max_dt_s {p.pose_max_dt_s:g}); using the nearest sample")
        self.pose_dt_ms = 0.9 * self.pose_dt_ms + 0.1 * dt * 1e3
        self.depth_latency_ms = 0.9 * self.depth_latency_ms + 0.1 * latency * 1e3
        rate = self.poses.yaw_rate(stamp)
        if rate is not None and abs(math.degrees(rate)) > p.max_yaw_rate_dps:
            # even an interpolated pose is only as good as the stamps; a 10 ms error at 60 deg/s is
            # 0.6 deg = 2 cm at 2 m, and the aligner's gate is 25 cm: skip the update while turning
            return self._reject("turning", f"yaw rate {math.degrees(rate):.0f} deg/s > max_yaw_rate_dps {p.max_yaw_rate_dps:g}")
        px, py, pz, heading = pose
        x, y, z = F.ned_to_enu(px, py, pz)
        # the odom yaw comes from the SAME quaternion level_rotation strips its yaw from, so the two
        # cannot disagree; lpos.heading is PX4's yaw of that quaternion, its skew is only a metric
        heading_q = LM.heading_from_quat(q)
        self.heading_skew_deg = 0.9 * self.heading_skew_deg + 0.1 * abs(math.degrees(F.wrap(heading_q - heading)))
        pose_odom = (x, y, z, F.heading_to_yaw(heading_q))
        t0 = time.perf_counter()
        # floor cut reference: the takeoff spot's odom z once the mission has one (PX4's local z
        # origin is where its EKF started, not necessarily the floor), else odom z = 0
        takeoff = getattr(self.mission, "takeoff_odom", None)
        floor_z = float(takeoff[2]) if takeoff is not None else 0.0
        pts = LM.wall_points(d, self.K, cal[1], cal[2], q, pose_odom, p, floor_z=floor_z)
        self.depth_ms = 0.9 * self.depth_ms + 0.1 * (time.perf_counter() - t0) * 1e3
        if pts is None:
            return self._reject("unusable", f"too few valid depth pixels (min_valid_frac {p.min_valid_frac:g})")
        # a usable frame with nothing at flight height (net, far wall) is NOT a reject: it is counted
        # apart and still handed over as a (0,2) array, so the mission can treat it as "depth alive"
        self.frames["empty" if len(pts) == 0 else "points"] += 1
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
            return                                   # sim time before the first /clock
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
        st["frames"] = dict(self.frames)
        st["depth_ms"] = round(self.depth_ms, 1)
        st["depth_latency_ms"] = round(self.depth_latency_ms, 1)
        st["pose_dt_ms"] = round(self.pose_dt_ms, 1)
        st["heading_skew_deg"] = round(self.heading_skew_deg, 2)
        txt = json.dumps(st, default=float)
        self.pub_status.publish(String(data=txt))
        self.log.event(now, "status", status=json.loads(txt))
        if self.exit_when_done and self.done_at is not None and now - self.done_at > 1.5:
            raise SystemExit(0 if self.mission.result == "complete" else 1)


def main(argv=None):
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=list(argv) if argv is not None else sys.argv)
    node = None
    code = 1
    try:
        node = NavNode()                            # inside the try: a config error still shuts rclpy down
        rclpy.spin(node)
    except SystemExit as e:
        code = int(e.code or 0)
    except (KeyboardInterrupt, ExternalShutdownException):
        code = 130
    finally:
        if node is not None:
            node.log.close()
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
