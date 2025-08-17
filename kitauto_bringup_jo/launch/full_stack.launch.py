#!/usr/bin/env python3
# kitauto_bringup_jo/launch/full_stack.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def try_share(pkg):
    try:
        return get_package_share_directory(pkg)
    except Exception:
        print(f"[kitauto_bringup_jo] WARNING: package '{pkg}' not found — skipping its launch include.")
        return None

def generate_launch_description():
    # ----- args -----
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    rgb_profile_arg  = DeclareLaunchArgument('rgb_profile',  default_value='640x480x30')
    ublox_params_arg = DeclareLaunchArgument('ublox_params', default_value=os.path.expanduser('~/ublox_cfg/m8n.yaml'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    rgb_profile  = LaunchConfiguration('rgb_profile')
    ublox_params = LaunchConfiguration('ublox_params')

    # ----- drivetrain -----
    drivetrain = GroupAction(actions=[
        Node(
            package='drive_steer_bridge', executable='arduino_bridge', name='arduino_bridge',
            parameters=[{'port':'/dev/arduino','baud':115200,'status_request':False,'use_sim_time':use_sim_time}],
            output='screen'
        ),
        Node(
            package='drive_steer_bridge', executable='twist_to_bridge', name='twist_to_bridge',
            parameters=[{'adc_center':455,'steer_span_adc':455,'max_lin':1.0,'max_ang':1.0,'use_sim_time':use_sim_time}],
            output='screen'
        ),
    ])

    # ----- sensors -----
    # RealSense: color+IMU만 사용(사용자 코드 기준으로 유지)
    realsense = Node(
        package='realsense2_camera', executable='realsense2_camera_node', name='camera',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_color': True,
            'rgb_camera.color_profile': rgb_profile,
            'rgb_camera.color_format': 'RGB8',
            'enable_depth': False,
            'enable_gyro': True,
            'enable_accel': True,
            'unite_imu_method': 2,      # gyro+accel fuse -> /camera/imu
        }],
        output='screen'
    )

    sllidar = Node(
        package='sllidar_ros2', executable='sllidar_node', name='sllidar',
        parameters=[{'serial_port':'/dev/ttyUSB1','serial_baudrate':256000,'frame_id':'laser','use_sim_time':use_sim_time}],
        output='screen'
    )

    # ----- static TFs: base_footprint -> base_link -> sensors -----
    static_tfs = GroupAction(actions=[
        # base_footprint -> base_link (차체 기준축 정의)
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='static_basefoot_to_baselink',
            arguments=['0','0','0.05','0','0','0','base_footprint','base_link'], output='screen'
        ),
        # base_link -> laser
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='static_base_to_laser',
            arguments=['0.35','0.0','0.25','0','0','0','base_link','laser'], output='screen'
        ),
        # base_link -> camera_link
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='static_base_to_camera',
            arguments=['0.38','0.0','0.30','0','0','0','base_link','camera_link'], output='screen'
        ),
        # base_link -> gps_link
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='static_base_to_gps',
            arguments=['0.60','0.0','0.45','0','0','0','base_link','gps_link'], output='screen'
        ),
    ])

    # ----- Madgwick -----
    madgwick = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node', name='imu_madgwick',
        parameters=[{
            'use_mag': False,
            'world_frame': 'enu',
            'publish_tf': False,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            # 입력: RealSense IMU (현재 실제 토픽: /camera/camera/imu)
            ('/imu/data_raw', '/camera/camera/imu'),
            # 출력: 표준화된 이름으로 사용
            ('/imu/data', '/imu/filtered'),
        ],
        output='screen',
    )

    # ----- GPS (u-blox) -----
    ublox = Node(
        package='ublox_gps', executable='ublox_gps_node', name='ublox_gps',
        parameters=[ublox_params],
        output='screen'
    )

    # ----- external includes -----
    actions = [use_sim_time_arg, rgb_profile_arg, ublox_params_arg,
               drivetrain, realsense, sllidar, static_tfs, madgwick, ublox]

    # wheel odom
    odom_share = try_share('kitauto_odometry')
    if odom_share:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(odom_share,'launch','odometry.launch.py')),
            launch_arguments={'use_sim_time':use_sim_time}.items()
        ))

    # local,global EKF
    ekf_share = try_share('ekf_stack')
    if ekf_share:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ekf_share,'launch','ekf_only.launch.py')),
            launch_arguments={'use_sim_time':use_sim_time}.items()
        ))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ekf_share,'launch','gps.launch.py')),
            launch_arguments={'use_sim_time':use_sim_time}.items()
        ))

    return LaunchDescription(actions)
