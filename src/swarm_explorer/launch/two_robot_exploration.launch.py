#!/usr/bin/env python3
#
# Launch two frontier exploration nodes (robot1 and robot2).
# No namespace wrapping - all topics are absolute in the YAML configs.
#

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch two frontier exploration nodes."""

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
        }],
        output='screen',
    )

    return LaunchDescription([
        robot1_only_arg,
        robot2_only_arg,
        robot1_explorer,
        robot2_explorer,
    ])
