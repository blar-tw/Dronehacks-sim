"""Offline A/B: replay a flight bag through ONE cuVSLAM, with or without IMU fusion.

Run on ROS_DOMAIN_ID=77 (not the drone's 5) so nothing reaches flight_interface, the PX4
bridge or PX4. Same converter + cuVSLAM settings as the organisers' oakd_vslam.launch.py,
except that output goes to /replay/* and no TF is published.

With imu:=true it adds the team's Kalibr camera-IMU calibration
(~/team_ws/oak-calibration/releases/2026-11-07-motion-003):
  * static TF oak_left_camera_optical_frame -> oak_kalibr_imu_frame (T_camera_raw_imu_m)
  * IMU timestamps shifted by -timeshift_cam_imu (imu_time_adapter.py)
  * measured noise densities / random walks (imu-initial.yaml)

    ros2 launch vio_replay.launch.py imu:=true
"""
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

CAL_DIR = Path('/root/team_ws/oak-calibration/releases/2026-11-07-motion-003')
HERE = Path(__file__).resolve().parent
W, H = 1024, 640


def _quat_from_matrix(m):
    # Shepperd's method; avoids a scipy dependency.
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = (t + 1.0) ** 0.5 * 2
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = (1.0 + m[0][0] - m[1][1] - m[2][2]) ** 0.5 * 2
        return (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    if m[1][1] > m[2][2]:
        s = (1.0 + m[1][1] - m[0][0] - m[2][2]) ** 0.5 * 2
        return ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    s = (1.0 + m[2][2] - m[0][0] - m[1][1]) ** 0.5 * 2
    return ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s)


def _build(context):
    use_imu = context.launch_configurations['imu'].lower() in ('1', 'true', 'yes')
    ns = float(context.launch_configurations.get('noise_scale', '1.0'))
    actions = []
    params = {
        'enable_image_denoising': False, 'rectified_images': False,
        'enable_imu_fusion': use_imu,
        'base_frame': 'base_link', 'map_frame': 'replay_map', 'odom_frame': 'replay_odom',
        'camera_optical_frames': ['oak_left_camera_optical_frame', 'oak_right_camera_optical_frame'],
        'publish_odom_to_base_tf': False, 'publish_map_to_odom_tf': False,
        'image_jitter_threshold_ms': 40.0, 'enable_localization_n_mapping': False,
        'enable_slam_visualization': False, 'enable_landmarks_view': False,
        'enable_observations_view': False, 'path_max_size': 100,
    }
    remaps = [
        ('visual_slam/image_0', '/oak/left/image_mono'), ('visual_slam/camera_info_0', '/oak/left/camera_info'),
        ('visual_slam/image_1', '/oak/right/image_mono'), ('visual_slam/camera_info_1', '/oak/right/camera_info'),
        ('visual_slam/tracking/odometry', '/replay/odometry'), ('visual_slam/status', '/replay/status'),
    ]
    if use_imu:
        cal = yaml.safe_load((CAL_DIR / 'camera-imu-raw-candidate.yaml').read_text())
        noise = yaml.safe_load((CAL_DIR / 'imu-initial.yaml').read_text())
        T = cal['left']['T_camera_raw_imu_m']
        q = _quat_from_matrix([row[:3] for row in T[:3]])
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher', name='replay_kalibr_imu_tf',
            arguments=['--x', str(T[0][3]), '--y', str(T[1][3]), '--z', str(T[2][3]),
                       '--qx', str(q[0]), '--qy', str(q[1]), '--qz', str(q[2]), '--qw', str(q[3]),
                       '--frame-id', 'oak_left_camera_optical_frame', '--child-frame-id', 'oak_kalibr_imu_frame']))
        actions.append(ExecuteProcess(
            cmd=['python3', str(HERE / 'imu_time_shift.py'), str(CAL_DIR / 'camera-imu-raw-candidate.yaml')],
            output='screen'))
        params.update({
            'imu_frame': 'oak_kalibr_imu_frame',
            'gyro_noise_density': ns * float(noise['gyroscope_noise_density']),
            'gyro_random_walk': ns * float(noise['gyroscope_random_walk']),
            'accel_noise_density': ns * float(noise['accelerometer_noise_density']),
            'accel_random_walk': ns * float(noise['accelerometer_random_walk']),
        })
        remaps.append(('visual_slam/imu', '/replay/imu'))

    def converter(side):
        return ComposableNode(
            package='isaac_ros_image_proc', plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
            name=f'replay_{side}_to_mono', parameters=[{'encoding_desired': 'mono8', 'image_width': W, 'image_height': H}],
            remappings=[('image_raw', f'/oak/{side}/image_raw'), ('image', f'/oak/{side}/image_mono')])

    actions.append(ComposableNodeContainer(
        name='replay_vslam_container', namespace='', package='rclcpp_components', executable='component_container_mt',
        composable_node_descriptions=[converter('left'), converter('right'), ComposableNode(
            package='isaac_ros_visual_slam', plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
            name='replay_visual_slam', parameters=[params], remappings=remaps)],
        output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('imu', default_value='false'),
                              DeclareLaunchArgument('noise_scale', default_value='1.0'), OpaqueFunction(function=_build)])
