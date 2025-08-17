from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    cfg_dir      = os.path.join(get_package_share_directory('ekf_stack'), 'config')
    navsat_yaml  = os.path.join(cfg_dir, 'navsat.yaml')
    ekf_map_yaml = os.path.join(cfg_dir, 'ekf_map.yaml')

    ublox = Node(
        package='ublox_gps', executable='ublox_gps_node', name='ublox_gps_node',
        parameters=[os.path.expanduser('~/ublox_cfg/m8n.yaml'),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )

    navsat = Node(
        package='robot_localization', executable='navsat_transform_node',
        name='navsat_transform_node',
        parameters=[navsat_yaml, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/gps/fix', '/fix'),
            ('/odometry/filtered', '/odometry/local'),
            ('/imu', '/camera/camera/imu'),
            ('/gps/odom', '/odometry/gps'),
        ],
        output='screen'
    )

    # Call SetUTMZone with a small delay AFTER navsat is up
    set_zone = ExecuteProcess(
        cmd=[
            'ros2','service','call','/setUTMZone','robot_localization/srv/SetUTMZone',
            "{utm_zone: '52S'}"
        ],
        shell=False,
        output='screen'
    )
    set_zone_later = TimerAction(period=1.0, actions=[set_zone])   # 1s after navsat

    ekf_map = Node(
        package='robot_localization', executable='ekf_node', name='ekf_map',
        parameters=[ekf_map_yaml, {'use_sim_time': use_sim_time},
                    {'publish_tf': True},
                    {'map_frame': 'map'},
                    {'odom_frame': 'odom'},
                    {'base_link_frame': 'base_footprint'},
                    {'world_frame': 'map'}],
        remappings=[
            ('/odometry/filtered', '/odometry/global'),
            ('/odometry/gps', '/odometry/gps_fixed'),   # if you run the shim
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        ublox,
        navsat,
        set_zone_later,   # <-- this kills the UTM 31 spam
        ekf_map,
    ])
