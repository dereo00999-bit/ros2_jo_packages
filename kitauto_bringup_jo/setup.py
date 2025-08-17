# ~/ros2_ws/src/kitauto_bringup_jo/setup.py
from setuptools import setup
from glob import glob

package_name = 'kitauto_bringup_jo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py') + glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pw',
    maintainer_email='pw@example.com',
    description='Bringup for full car stack (drivetrain, sensors, tf, ekf, gps)',
    license='MIT',
)

