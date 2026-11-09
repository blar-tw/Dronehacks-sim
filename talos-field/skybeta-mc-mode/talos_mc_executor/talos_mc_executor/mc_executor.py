"""TALOS MC executor: the ROSCon '25 workshop's multicopter mode executor,
ported to the skybeta flight interface.

    Takeoff -> Waypoints -> Yaw spin -> Descend -> hold for the pilot

The contract is the same as competitor_example/square_mission.py:

    subscribe  /visual_slam/tracking/odometry   where we are (ENU, odom)
    subscribe  /skybeta/state                   what PX4 is doing (JSON)
    publish    /skybeta/setpoint/pose           where we want to be (ENU, odom)

This node never touches /fmu/*, never arms and never changes mode.
``skybeta_flight_interface`` owns Offboard and clamps every setpoint to the
safety envelope. The mission logic lives in ``mission_core.py`` (no rclpy),
so it can be tested off the drone; this file only wires it to ROS.

Flying it: the safety pilot arms on Altitude, then switches to Position.
The interface enters Offboard, and this node takes off. The pilot can take
over at any time; the mission stops and does not resume until the vehicle is
disarmed and armed again.
"""
import json
import math

import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from talos_mc_executor.mission_core import (
    DEFAULT_WAYPOINTS,
    Inputs,
    MissionExecutor,
    Params,
    parse_waypoints,
    validate,
    yaw_from_xyzw,
)


class McExecutorNode(Node):

    def __init__(self, **kwargs):
        super().__init__('talos_mc_executor', **kwargs)

        defaults = Params()
        scalar_params = (
            'hover_altitude_m', 'takeoff_settle_s', 'climb_speed_mps',
            'cruise_speed_mps', 'descend_speed_mps', 'max_lead_m',
            'arrival_radius_m', 'yaw_rate_dps', 'yaw_max_lead_deg',
            'yaw_settle_s', 'handoff_altitude_m', 'step_timeout_s',
            'odom_timeout_s',
        )
        # dynamic_typing: accept 2 as well as 2.0 from YAML; every value is
        # cast and validated below anyway.
        loose = ParameterDescriptor(dynamic_typing=True)
        for name in scalar_params:
            self.declare_parameter(name, float(getattr(defaults, name)), loose)
        self.declare_parameter('yaw_spin_enabled', defaults.yaw_spin_enabled)
        self.declare_parameter(
            'waypoints_enu_m',
            [float(v) for point in DEFAULT_WAYPOINTS for v in point], loose)
        self.declare_parameter('publish_rate_hz', 10.0, loose)
        self.declare_parameter('state_timeout_s', 1.0, loose)
        self.declare_parameter('odometry_topic',
                               '/visual_slam/tracking/odometry')

        params = Params(
            **{name: float(self.get_parameter(name).value)
               for name in scalar_params},
            yaw_spin_enabled=bool(
                self.get_parameter('yaw_spin_enabled').value),
            waypoints_enu_m=parse_waypoints(
                self.get_parameter('waypoints_enu_m').value),
        )
        errors = validate(params)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if not 2.0 < rate_hz <= 50.0:
            # PX4 drops out of Offboard below 2 Hz of setpoints.
            errors.append(f'publish_rate_hz must be in (2, 50] (got {rate_hz})')
        self.state_timeout_s = float(
            self.get_parameter('state_timeout_s').value)
        if not 0.3 <= self.state_timeout_s <= 5.0:
            errors.append('state_timeout_s must be in [0.3, 5.0] '
                          f'(got {self.state_timeout_s}); the state is 5 Hz')
        if errors:
            raise ValueError('unsafe mission parameters:\n  ' +
                             '\n  '.join(errors))

        self.executor_core = MissionExecutor(params, self._log)

        # -- the contract (same topics and QoS depth as square_mission) ---
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/skybeta/setpoint/pose', 10)
        # Keep this the same topic flight_interface measures altitude from.
        self.create_subscription(
            Odometry, str(self.get_parameter('odometry_topic').value),
            self.odometry_cb, 10)
        self.create_subscription(String, '/skybeta/state', self.state_cb, 10)

        self.position = None
        self.yaw = None
        self.last_odom_s = None
        self.state = None
        self.last_state_s = None
        self.logged_clamps = set()

        self.create_timer(1.0 / rate_hz, self.tick)

        legs = len(params.waypoints_enu_m)
        self.get_logger().info(
            f'talos_mc_executor ready: takeoff to {params.hover_altitude_m:.2f}'
            f' m, {legs} waypoints at {params.cruise_speed_mps:.2f} m/s, '
            + (f'360 deg yaw spin at {params.yaw_rate_dps:.0f} deg/s, '
               if params.yaw_spin_enabled else 'no yaw spin, ')
            + f'descend to {params.handoff_altitude_m:.2f} m and hold.')

    # ------------------------------------------------------------------
    def _log(self, level, message):
        logger = self.get_logger()
        {'info': logger.info, 'warn': logger.warn,
         'error': logger.error}.get(level, logger.info)(message)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # -- inputs --------------------------------------------------------
    def odometry_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        position = (float(p.x), float(p.y), float(p.z))
        yaw = yaw_from_xyzw(float(q.x), float(q.y), float(q.z), float(q.w))
        if not all(math.isfinite(v) for v in position + (yaw,)):
            # Drop it: the sample stays "missing", so it ages into stale.
            return
        self.position = position
        self.yaw = yaw
        self.last_odom_s = self.now_s()

    def state_cb(self, msg):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(state, dict):
            return
        self.state = state
        self.last_state_s = self.now_s()
        clamped = state.get('clamped') or []
        if set(clamped) == {'min_altitude'}:
            # Expected on the ground and early in the climb: the envelope
            # floor lifts every pose to min_altitude_m. Not worth a warning.
            return
        key = ','.join(sorted(clamped))
        if clamped and key not in self.logged_clamps:
            self.logged_clamps.add(key)
            self.get_logger().warn(
                f'the flight interface is CLAMPING our setpoints ({key}): we '
                f'are asking for something outside the envelope '
                f'{state.get("envelope")}. A clamped waypoint is never '
                'reached: expect a timeout and a hold.')

    # -- the tick ------------------------------------------------------
    def tick(self):
        now = self.now_s()
        # A /skybeta/state older than state_timeout_s is not trusted: if
        # flight_interface died, its last "armed / offboard_active" would
        # otherwise be believed forever.
        state_fresh = (self.last_state_s is not None and
                       now - self.last_state_s <= self.state_timeout_s)
        state = self.state if state_fresh else {}
        inputs = Inputs(
            now_s=now,
            position=self.position,
            yaw=self.yaw,
            odom_age_s=(math.inf if self.last_odom_s is None
                        else now - self.last_odom_s),
            state_received=state_fresh,
            px4_link=bool(state.get('px4_link', False)),
            armed=bool(state.get('armed', False)),
            offboard_active=bool(state.get('offboard_active', False)),
        )
        setpoint = self.executor_core.step(inputs)
        if setpoint is not None:
            self.send(setpoint.position, setpoint.yaw)

    # -- output --------------------------------------------------------
    def send(self, position, yaw_enu=None):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        if yaw_enu is None:
            # All-zero quaternion == "do not command yaw" (interface rule).
            msg.pose.orientation.w = 0.0
        else:
            msg.pose.orientation.z = math.sin(float(yaw_enu) / 2.0)
            msg.pose.orientation.w = math.cos(float(yaw_enu) / 2.0)
        self.setpoint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = McExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
