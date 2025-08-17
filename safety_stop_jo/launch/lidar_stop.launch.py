# 이 코드 전체를 복사하세요
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('safety_stop_jo')
    params = os.path.join(pkg_share, 'config', 'lidar_stop.yaml')

    return LaunchDescription([
        Node(
            package='safety_stop_jo',
            executable='lidar_stop',      # setup.py의 console_scripts 이름
            name='lidar_stop',
            output='screen',
            parameters=[params]
        )
    ])
