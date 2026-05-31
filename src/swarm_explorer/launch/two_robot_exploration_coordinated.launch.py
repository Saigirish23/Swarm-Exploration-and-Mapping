#!/usr/bin/env python3
#
# Launch frontier exploration with coordination and teammate obstacle layers.
#

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Launch coordinated frontier exploration nodes."""

    pkg_share = get_package_share_directory('swarm_explorer')

    robot1_config = os.path.join(pkg_share, 'config', 'robot1_explorer.yaml')
    robot2_config = os.path.join(pkg_share, 'config', 'robot2_explorer.yaml')

    robot1_only_arg = DeclareLaunchArgument(
        'robot1_only',
        default_value='False',
        description='If True, only launch robot1 explorer'
    )

    robot2_only_arg = DeclareLaunchArgument(
        'robot2_only',
        default_value='False',
        description='If True, only launch robot2 explorer'
    )

    robot1_explorer = Node(
        package='swarm_explorer',
        executable='robot_explorer',
        name='robot1_explorer',
        condition=UnlessCondition(LaunchConfiguration('robot2_only')),
        parameters=[robot1_config, {
            'world_min_x': -5.5,
            'world_max_x':  5.5,
            'world_min_y': -5.5,
            'world_max_y':  5.5,
            'save_map_on_complete': False,
        }],
        output='screen',
    )

    robot2_explorer = Node(
        package='swarm_explorer',
        executable='robot_explorer',
        name='robot2_explorer',
        condition=UnlessCondition(LaunchConfiguration('robot1_only')),
        parameters=[robot2_config, {
            'world_min_x': -5.5,
            'world_max_x':  5.5,
            'world_min_y': -5.5,
            'world_max_y':  5.5,
            'save_map_on_complete': False,
        }],
        output='screen',
    )

    coordinator = Node(
        package='swarm_explorer',
        executable='frontier_coordinator',
        name='frontier_coordinator',
        parameters=[{
            'robot_names': ['robot1', 'robot2'],
            'claim_frame': 'map',
            'claim_ttl_sec': 12.0,
            'save_map_on_complete': True,
            'save_map_topic': '/merged_map',
        }],
        output='screen',
    )

    robot1_footprint = Node(
        package='swarm_explorer',
        executable='robot_footprint_broadcaster',
        name='robot1_footprint',
        condition=UnlessCondition(LaunchConfiguration('robot2_only')),
        parameters=[{
            'robot_name': 'robot1',
            'base_frame': 'robot1/base_link',
            'map_frame': 'map',
            'obstacle_topic': '/robot1/robot_obstacle',
        }],
        output='screen',
    )

    robot2_footprint = Node(
        package='swarm_explorer',
        executable='robot_footprint_broadcaster',
        name='robot2_footprint',
        condition=UnlessCondition(LaunchConfiguration('robot1_only')),
        parameters=[{
            'robot_name': 'robot2',
            'base_frame': 'robot2/base_link',
            'map_frame': 'map',
            'obstacle_topic': '/robot2/robot_obstacle',
        }],
        output='screen',
    )

    return LaunchDescription([
        robot1_only_arg,
        robot2_only_arg,
        coordinator,
        robot1_footprint,
        robot2_footprint,
        robot1_explorer,
        robot2_explorer,
    ])
