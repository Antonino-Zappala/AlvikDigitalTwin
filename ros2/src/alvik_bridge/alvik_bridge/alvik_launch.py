"""
alvik_launch.py
===============
Launch file per Alvik Digital Twin.

Avvia:
  - robot_state_publisher  — pubblica TF dall'URDF
  - joint_state_publisher  — pubblica stato joint ruote
  - slam_toolbox           — mappatura (opzionale)
  - nav2_bringup           — navigazione autonoma (opzionale)
  - nav2 localizzazione    — navigazione con mappa esistente (opzionale)

Uso:
  ros2 launch alvik_bridge alvik_launch.py
  ros2 launch alvik_bridge alvik_launch.py slam:=true
  ros2 launch alvik_bridge alvik_launch.py slam:=true nav2:=true
  ros2 launch alvik_bridge alvik_launch.py localization:=true
  ros2 launch alvik_bridge alvik_launch.py localization:=true map:=/ros2_ws/maps/alvik_map.yaml
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
    """
    Genera la LaunchDescription per Alvik Digital Twin.

    Argomenti disponibili:
      slam=true         → avvia slam_toolbox per la costruzione della mappa
      nav2=true         → avvia Nav2 senza mappa (esplorazione)
      localization=true → avvia Nav2 con mappa esistente (navigazione autonoma)
      map=<path>        → percorso alla mappa .yaml (default: /ros2_ws/config/alvik_map.yaml)
      use_sim_time=true → usa il clock di simulazione (solo per Gazebo)

    Nota: localization=true usa bringup_launch.py che avvia AMCL + Nav2 in un
    singolo launch, evitando il conflitto del behavior_server che si verificava
    avviando localization_launch.py e navigation_launch.py separatamente.
    """

    pkg_dir  = get_package_share_directory('alvik_bridge')
    nav2_dir = get_package_share_directory('nav2_bringup')

    # ── Argomenti ─────────────────────────────────────────────────────────
    slam_arg = DeclareLaunchArgument(
        'slam', default_value='False',
        description='Avvia slam_toolbox')

    nav2_arg = DeclareLaunchArgument(
        'nav2', default_value='False',
        description='Avvia Nav2 senza mappa')

    localization_arg = DeclareLaunchArgument(
        'localization', default_value='False',
        description='Avvia Nav2 con mappa esistente')

    map_arg = DeclareLaunchArgument(
        'map', default_value='/ros2_ws/config/alvik_map.yaml',
        description='Path alla mappa yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Usa il clock di simulazione')

    slam         = LaunchConfiguration('slam')
    nav2         = LaunchConfiguration('nav2')
    localization = LaunchConfiguration('localization')
    map_file     = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── URDF ──────────────────────────────────────────────────────────────
    urdf_file = os.path.join(pkg_dir, 'alvik.urdf.xacro')
    robot_description = Command(['xacro ', urdf_file])

    # ── robot_state_publisher ─────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    # ── joint_state_publisher ─────────────────────────────────────────────
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'robot_description': robot_description,
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

    # ── Nav2 senza mappa ──────────────────────────────────────────────────
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

    # ── Nav2 con mappa esistente (localizzazione) ─────────────────────────
    nav2_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  os.path.join(pkg_dir, 'nav2_params.yaml'),
            'map':          map_file,
        }.items(),
        condition=IfCondition(localization),
    )

    return LaunchDescription([
        slam_arg,
        nav2_arg,
        localization_arg,
        map_arg,
        use_sim_time_arg,
        robot_state_publisher,
        joint_state_publisher,
        slam_launch,
        nav2_launch,
        nav2_localization_launch,
    ])
