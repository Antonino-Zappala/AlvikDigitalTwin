"""
alvik_launch.py
===============
Launch file per Alvik Digital Twin.

Avvia:
  - robot_state_publisher  — pubblica TF dall'URDF
  - slam_toolbox           — mappatura (opzionale)
  - nav2_bringup           — navigazione autonoma (opzionale)

Uso:
  ros2 launch alvik_bridge alvik_launch.py
  ros2 launch alvik_bridge alvik_launch.py slam:=true
  ros2 launch alvik_bridge alvik_launch.py slam:=true nav2:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory('alvik_bridge')

    # ── Argomenti ─────────────────────────────────────────────────────────
    slam_arg = DeclareLaunchArgument(
        'slam', default_value='false',
        description='Avvia slam_toolbox')

    nav2_arg = DeclareLaunchArgument(
        'nav2', default_value='false',
        description='Avvia Nav2')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Usa il clock di simulazione')

    slam         = LaunchConfiguration('slam')
    nav2         = LaunchConfiguration('nav2')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── URDF ──────────────────────────────────────────────────────────────
    urdf_file = os.path.join(pkg_dir, 'alvik.urdf.xacro')

    # ── robot_state_publisher ─────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time,
        }]
    )

    # ── slam_toolbox ──────────────────────────────────────────────────────
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch',
                         'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(pkg_dir, 'slam_params.yaml'),
        }.items(),
        condition=IfCondition(slam),
    )

    # ── Nav2 ──────────────────────────────────────────────────────────────
    nav2_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_dir, 'nav2_params.yaml'),
        }.items(),
        condition=IfCondition(nav2),
    )

    return LaunchDescription([
        slam_arg,
        nav2_arg,
        use_sim_time_arg,
        robot_state_publisher,
        slam_launch,
        nav2_launch,
    ])
