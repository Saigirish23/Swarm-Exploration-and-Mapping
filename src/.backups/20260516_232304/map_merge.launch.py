#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch map merger node."""
    
    map_merger = Node(
        package='swarm_explorer',
        executable='map_merger',
        name='map_merger',
        parameters=[{
            'use_sim_time': True,
            'map1_topic': '/robot1/map',
            'map2_topic': '/robot2/map',
            'merged_map_topic': '/merged_map',
            'robot1_initial_x': -1.0,
            'robot1_initial_y': 0.0,
            'robot1_initial_yaw': 0.0,
            'robot2_initial_x': 1.0,
            'robot2_initial_y': 0.0,
            'robot2_initial_yaw': 0.0,
            'resolution': 0.05,
            'merged_frame': 'map',
        }],
        output='screen',
    )
    
    return LaunchDescription([
        map_merger,
    ])
