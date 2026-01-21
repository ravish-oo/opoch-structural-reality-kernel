#!/usr/bin/env python3
"""
ROS2 Launch file for MAPF 12x12 8-agent simulation.

Launches:
- Gazebo with custom world
- 8 TurtleBot3 robots at their start positions
- Nav2 navigation stack for each robot
- MAPF execution node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
import yaml


def generate_launch_description():
    # Get package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Configuration file path
    config_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    robots_config_path = os.path.join(config_dir, 'config', 'robots.yaml')
    world_path = os.path.join(config_dir, 'worlds', 'mapf_grid_12x12.world')

    # Load robot configuration
    with open(robots_config_path, 'r') as f:
        config = yaml.safe_load(f)

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot spawn actions
    spawn_robots = []
    for robot in config['robots']:
        robot_name = robot['name']
        x = robot['start']['world'][0]
        y = robot['start']['world'][1]
        theta = robot['start'].get('theta', 0.0)

        # Spawn robot
        spawn_robot = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', robot_name,
                '-file', os.path.join(
                    get_package_share_directory('turtlebot3_gazebo'),
                    'models', 'turtlebot3_burger', 'model.sdf'
                ),
                '-x', str(x),
                '-y', str(y),
                '-z', '0.01',
                '-Y', str(theta),
                '-robot_namespace', robot_name
            ],
            output='screen'
        )
        spawn_robots.append(spawn_robot)

        # Robot state publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_name,
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_prefix': f'{robot_name}/'
            }],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ],
            output='screen'
        )
        spawn_robots.append(robot_state_publisher)

    # MAPF execution node
    mapf_executor = Node(
        package='mapf_simulation',
        executable='execute_mapf_solution.py',
        name='mapf_executor',
        parameters=[{
            'use_sim_time': use_sim_time,
            'solution_file': os.path.join(config_dir, 'config', 'mapf_solution.json'),
            'time_per_step': 2.0
        }],
        output='screen'
    )

    # Safety monitor node
    safety_monitor = Node(
        package='mapf_simulation',
        executable='safety_monitor.py',
        name='safety_monitor',
        parameters=[{
            'use_sim_time': use_sim_time,
            'min_distance': 0.3
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        gzserver_cmd,
        gzclient_cmd,
        *spawn_robots,
        mapf_executor,
        safety_monitor
    ])


if __name__ == '__main__':
    generate_launch_description()
