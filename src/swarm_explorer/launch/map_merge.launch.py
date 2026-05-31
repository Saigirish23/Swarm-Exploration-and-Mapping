#!/usr/bin/env python3
#
# Launch map merger node with static transforms for RViz.
#

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch map merger node and static transforms."""

    pkg_share = get_package_share_directory('swarm_explorer')
    config_file = os.path.join(pkg_share, 'config', 'map_merge.yaml')

    map_merger = Node(
        package='swarm_explorer',
        executable='map_merger',
        name='map_merger',
        parameters=[config_file],
        output='screen',
    )

    # Static transform: map -> robot1/map (identity — Cartographer handles the actual offset)
    static_tf_robot1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_robot1_map',
        arguments=[
            '--x', '-2.5', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map', '--child-frame-id', 'robot1/map',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Static transform: map -> robot2/map (identity — Cartographer handles the actual offset)
    static_tf_robot2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_robot2_map',
        arguments=[
            '--x', '2.5', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map', '--child-frame-id', 'robot2/map',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        map_merger,
        static_tf_robot1,
        static_tf_robot2,
    ])
