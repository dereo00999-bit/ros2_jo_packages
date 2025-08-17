from setuptools import setup
from glob import glob
import os

package_name = 'ekf_stack'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),   # <- no hardcoded map.yaml
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pw',
    maintainer_email='pw@todo.todo',
    description='EKF + NavSat stack',
    license='TODO',
)
