from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_topic   = LaunchConfiguration('odom_topic')
    imu_topic    = LaunchConfiguration('imu_topic')

    cfg_dir   = os.path.join(get_package_share_directory('ekf_stack'), 'config')
    odom_yaml = os.path.join(cfg_dir, 'odom.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('odom_topic',   default_value='/odom'),
        DeclareLaunchArgument('imu_topic',    default_value='/camera/camera/imu'),

        Node(
            package='robot_localization', executable='ekf_node', name='ekf_odom',
            output='screen',
            parameters=[
                odom_yaml,
                {'use_sim_time': use_sim_time},
                {'odom0': odom_topic},
                {'imu0':  imu_topic},
                {'publish_tf': True},
                {'map_frame': 'map'},
                {'odom_frame': 'odom'},
                {'base_link_frame': 'base_footprint'},
                {'world_frame': 'odom'},
            ],
            remappings=[('odometry/filtered', '/odometry/local')],
        ),
    ])
