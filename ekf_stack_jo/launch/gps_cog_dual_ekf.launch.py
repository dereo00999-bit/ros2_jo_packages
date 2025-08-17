# ekf_stack/launch/gps_cog_dual_ekf.launch.py
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml

def load_params(path, key_expected):
    try:
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        if isinstance(data, dict) and key_expected in data:
            return data[key_expected].get('ros__parameters', {})
        if isinstance(data, dict) and 'ros__parameters' in data:
            return data['ros__parameters']
    except Exception:
        pass
    return {}

def generate_launch_description():
    share = get_package_share_directory('ekf_stack')
    ekf_local_yaml  = os.path.join(share, 'config', 'ekf_local.yaml')
    ekf_global_yaml = os.path.join(share, 'config', 'ekf_global.yaml')
    navsat_yaml     = os.path.join(share, 'config', 'navsat_transform.yaml')

    ekf_local_params  = load_params(ekf_local_yaml,  'ekf_local')
    ekf_global_params = load_params(ekf_global_yaml, 'ekf_global')
    navsat_params     = load_params(navsat_yaml,     'navsat_transform')

    # --- merge defaults for navsat ---
    navsat_defaults = {
        'world_frame': 'odom',
        'base_link_frame': 'base_footprint',
        'two_d_mode': True,
        'zero_altitude': True,
        'use_odometry_yaw': False,
        'frequency': 30.0,
        'broadcast_cartesian_transform': True,
    }
    if 'broadcast_utm_transform' in navsat_params and 'broadcast_cartesian_transform' not in navsat_params:
        navsat_params['broadcast_cartesian_transform'] = bool(navsat_params['broadcast_utm_transform'])
        navsat_params.pop('broadcast_utm_transform', None)
    navsat_params = {**navsat_defaults, **navsat_params}

    # --- IMU filter ---
    madgwick = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_madgwick',
        parameters=[{'use_mag': False, 'world_frame': 'enu', 'publish_tf': False}],
        remappings=[
            ('imu/data_raw', '/camera/camera/imu'),
            ('imu/data',     '/imu/rpy')
        ],
        output='screen'
    )

    # --- COG fuser ---
    cog_fuser = Node(
        package='cog_orientation',
        executable='cog_imu_fuser',
        name='cog_imu_fuser',
        parameters=[{
            'imu_raw_topic':'/camera/camera/imu',
            'imu_rp_topic':'/imu/rpy',
            'gps_fix_topic':'/fix_ok',
            'odom_topic':'/odom',
            'pub_topic':'/imu/earth',
            'speed_on':0.7, 'speed_off':0.5,
            'yaw_gain_max':0.15, 'yaw_gain_min':0.01,
            'yaw_jump_limit_deg':45.0
        }],
        output='screen'
    )

    # --- EKF local (odom world) ---
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        parameters=[ekf_local_params],
        remappings=[('odometry/filtered','/odometry/local')],
        output='screen'
    )

    # --- navsat_transform (odom world) ---
    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        parameters=[navsat_params],
        remappings=[
            ('imu','/imu/earth'),
            ('odometry/filtered','/odometry/local'),
            ('gps/fix','/fix_ok'),
            ('gps/filtered','/gps/filtered'),
            ('odometry/gps','/odometry/gps')
        ],
        output='screen'
    )

    # --- EKF global (map world) ---
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        parameters=[ekf_global_params],
        remappings=[('odometry/filtered','/odometry/global')],
        output='screen'
    )

    # Start order: imu -> local ekf -> navsat -> global ekf
    return LaunchDescription([
        madgwick,
        cog_fuser,
        TimerAction(period=0.5, actions=[ekf_local]),
        TimerAction(period=1.0, actions=[navsat]),
        TimerAction(period=1.5, actions=[ekf_global]),
    ])
