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
