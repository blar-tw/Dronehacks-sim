"""flight_interface + the TALOS MC executor.

Forked from competitor_example/launch/competitor.launch.py. The
``flight_interface`` node below is kept exactly as the organisers ship it
(it owns the Offboard heartbeat and the safety envelope, and must run with
auto_offboard_on_arm:=true). Only the competitor node is swapped.

    ros2 launch talos_mc_executor talos_mission.launch.py
    ros2 launch talos_mc_executor talos_mission.launch.py max_altitude_m:=2.0
    ros2 launch talos_mc_executor talos_mission.launch.py \
        mission_config:=/root/team_ws/src/talos_mc_executor/config/mission.yaml

Mission parameters (waypoints, speeds, yaw spin) live in config/mission.yaml.
Set ``competitor:=false`` to bring up only the flight interface.

anchor_setpoints is on: on skybeta-05 the camera odometry and PX4 disagreed on
height by 1.1-1.3 m (2026-11-07), so the interface re-anchors every setpoint on
PX4's own position, captured with the odometry origin. Both nodes start
interface_start_delay_s after launch, so that capture happens after VIO and
EKF2 have settled.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_CONFIG = os.path.join(
    get_package_share_directory('talos_mc_executor'), 'config', 'mission.yaml')

# name -> (default, help). Envelope values are the flight interface's own
# defaults; tighten them here, do not loosen them without asking.
ARGUMENTS = [
    ('max_altitude_m', '3.0', 'ceiling above the odometry origin, metres'),
    ('min_altitude_m', '0.3', 'floor above the odometry origin, metres'),
    ('max_speed_mps', '1.5', 'cap on the norm of a twist setpoint, m/s'),
    ('setpoint_rate_hz', '10.0',
     'OffboardControlMode heartbeat rate -- PX4 needs > 2 Hz'),
    ('allow_mode_request', 'false',
     'enable /skybeta/request_offboard. SIM ONLY'),
    ('auto_offboard_on_arm', 'true',
     'enter Offboard once when the pilot arms, then switches to Position'),
    ('odometry_topic', '/visual_slam/tracking/odometry',
     'source of the ENU pose the altitude band is measured against'),
    ('anchor_setpoints', 'true',
     'send setpoints relative to PX4 own position (fixes the camera/PX4 '
     'height offset)'),
    ('interface_start_delay_s', '60.0',
     'start both nodes this long after launch, so the anchor is captured '
     'after VIO and EKF2 settle'),
    ('mission_config', DEFAULT_CONFIG, 'YAML with the mission parameters'),
    ('competitor', 'true', 'also start the TALOS MC executor'),
]


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=help_)
        for name, default, help_ in ARGUMENTS
    ]

    flight_interface = Node(
        package='skybeta_flight_interface',
        executable='flight_interface',
        name='flight_interface',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'max_altitude_m': LaunchConfiguration('max_altitude_m'),
            'min_altitude_m': LaunchConfiguration('min_altitude_m'),
            'max_speed_mps': LaunchConfiguration('max_speed_mps'),
            'setpoint_rate_hz': LaunchConfiguration('setpoint_rate_hz'),
            'allow_mode_request': LaunchConfiguration('allow_mode_request'),
            'auto_offboard_on_arm': LaunchConfiguration('auto_offboard_on_arm'),
            'anchor_setpoints': LaunchConfiguration('anchor_setpoints'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
        }],
    )

    mc_executor = Node(
        package='talos_mc_executor',
        executable='mc_executor',
        name='talos_mc_executor',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('competitor')),
        parameters=[
            LaunchConfiguration('mission_config'),
            # Same odometry as flight_interface's altitude band.
            {'odometry_topic': LaunchConfiguration('odometry_topic')},
        ],
    )

    def delayed(context, *args, **kwargs):
        # Resolved to a float here, so this works whatever TimerAction accepts.
        delay = max(0.0, float(
            LaunchConfiguration('interface_start_delay_s').perform(context)))
        return [TimerAction(period=delay,
                            actions=[flight_interface, mc_executor])]

    return LaunchDescription(declared + [OpaqueFunction(function=delayed)])
