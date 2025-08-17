# ekf_stack/launch/global_gps.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_local   = LaunchConfiguration('odom_local')    # from ekf_odom
    odom_gps     = LaunchConfiguration('odom_gps')      # navsat output
    odom_global  = LaunchConfiguration('odom_global')   # ekf_map output

    cfg_dir      = os.path.join(get_package_share_directory('ekf_stack'), 'config')
    navsat_yaml  = os.path.join(cfg_dir, 'navsat.yaml')
    ekf_map_yaml = os.path.join(cfg_dir, 'ekf_map.yaml')

    # 1) navsat: /fix_ok + /odometry/local (+ optional IMU) -> /odometry/gps
    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_yaml, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/gps/fix',           '/fix_ok'),        # your relay or direct GPS topic
            ('/odometry/filtered', odom_local),       # /odometry/local
            ('/gps/odom',          odom_gps),         # publish -> /odometry/gps
            # If you have an IMU topic, uncomment ONE of these lines:
            # ('/imu', '/imu/data'),
            # ('/imu', '/camera/camera/imu'),
        ],
    )
    navsat_delayed = TimerAction(period=1.0, actions=[navsat])  # wait for /odometry/local TF to exist

    # 2) EKF (map)
    ekf_map = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_map',
        output='screen',
        parameters=[
            ekf_map_yaml,
            {'use_sim_time': use_sim_time},
            {'publish_tf': True},
            {'map_frame': 'map'},
            {'odom_frame': 'odom'},
            {'base_link_frame': 'base_footprint'},
            {'world_frame': 'map'},
        ],
        remappings=[
            ('/odometry/filtered', odom_global),      # /odometry/global
            ('/odometry/gps',      odom_gps),         # /odometry/gps from navsat
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('odom_local',   default_value='/odometry/local'),
        DeclareLaunchArgument('odom_gps',     default_value='/odometry/gps'),
        DeclareLaunchArgument('odom_global',  default_value='/odometry/global'),
        navsat_delayed,
        ekf_map,
    ])
