from setuptools import setup

package_name = 'safety_stop_jo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],     # 폴더 safety_stop_jo/ 그대로 인식
    package_dir={'': '.'},       # 루트 레이아웃은 '.' 여야 함
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/safety_stop_jo']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/config', ['config/lidar_stop.yaml']),
        (f'share/{package_name}/launch', ['launch/lidar_stop.launch.py']),
    ],
    install_requires=['setuptools', 'ament_index_python'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='LiDAR one-shot safety stop',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lidar_stop = safety_stop_jo.lidar_stop_node:main',
        ],
    },
)
