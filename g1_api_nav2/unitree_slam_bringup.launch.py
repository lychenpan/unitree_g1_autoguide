"""
Unitree SLAM + Nav2 bringup (Python-only workspace, no colcon build).

Nav2 binaries still come from /opt/ros/foxy (C++). This launch starts:
  - static map->odom TF
  - map_server
  - unitree_relocation_odom_bridge (python3 script)
  - Nav2 navigation stack from nav2_bringup
"""
import os
import sys

_G1_API_DIR = os.path.dirname(os.path.abspath(__file__))
_WS_ROOT = os.path.dirname(_G1_API_DIR)
if _WS_ROOT not in sys.path:
    sys.path.insert(0, _WS_ROOT)

from g1_api_nav2.paths import (  # noqa: E402
    MAP_YAML,
    PARAMS_YAML,
    RELOCATION_ODOM_BRIDGE,
)

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
        },
        convert_types=True,
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '1'),

        DeclareLaunchArgument(
            'params_file',
            default_value=PARAMS_YAML,
            description='Nav2 params yaml',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=MAP_YAML,
            description='Occupancy grid yaml (must match Unitree SLAM map)',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen',
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings,
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['map_server'],
            }],
        ),

        ExecuteProcess(
            cmd=['python3', RELOCATION_ODOM_BRIDGE],
            name='unitree_relocation_odom_bridge',
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py'),
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
                'map_subscribe_transient_local': 'true',
            }.items(),
        ),
    ])
