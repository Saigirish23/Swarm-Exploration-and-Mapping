#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    """Launch two frontier exploration nodes (robot1 and robot2)."""
    
    robot1_explorer = Node(
        package='swarm_explorer',
        executable='robot_explorer',
        name='explorer',
        namespace='robot1',
        parameters=[{
            'use_sim_time': True,
            'robot_name': 'robot1',
            'map_topic': '/robot1/map',
            'base_frame': 'robot1/base_link',
            'map_frame': 'robot1/map',
            'goal_action': '/robot1/navigate_to_pose',
            'frontier_marker_topic': '/robot1/frontier_markers',
            'exploration_period': 5.0,
        }],
        output='screen',
    )
    
    robot2_explorer = Node(
        package='swarm_explorer',
        executable='robot_explorer',
        name='explorer',
        namespace='robot2',
        parameters=[{
            'use_sim_time': True,
            'robot_name': 'robot2',
            'map_topic': '/robot2/map',
            'base_frame': 'robot2/base_link',
            'map_frame': 'robot2/map',
            'goal_action': '/robot2/navigate_to_pose',
            'frontier_marker_topic': '/robot2/frontier_markers',
            'exploration_period': 5.0,
        }],
        output='screen',
    )
    
    return LaunchDescription([
        GroupAction([
            PushRosNamespace('robot1'),
            robot1_explorer,
        ]),
        GroupAction([
            PushRosNamespace('robot2'),
            robot2_explorer,
        ]),
    ])
