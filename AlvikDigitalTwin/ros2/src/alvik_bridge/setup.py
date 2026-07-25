from setuptools import setup
import os
from glob import glob

package_name = 'alvik_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('alvik_bridge/alvik_launch.py')),
        # Config files — URDF, SLAM, Nav2
        (os.path.join('share', package_name),
            glob('alvik_bridge/*.urdf.xacro') +
            glob('alvik_bridge/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'alvik_ros_bridge = alvik_bridge.alvik_ros_bridge:main',
            'alvik_pid_controller = alvik_bridge.alvik_pid_controller:main',
        ],
    },
)
