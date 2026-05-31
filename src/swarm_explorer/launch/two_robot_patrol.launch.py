#!/usr/bin/env python3
"""
Launch waypoint patrol nodes with standalone testing arguments.
Supports robot1-only, robot2-only, or both-robot patrol mapping.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments for standalone testing
    robot1_only_arg = DeclareLaunchArgument(
        'robot1_only',
        default_value='False',
        description='If True, only launch robot1 patrol node'
    )

    robot2_only_arg = DeclareLaunchArgument(
        'robot2_only',
        default_value='False',
        description='If True, only launch robot2 patrol node'
    )

    # Robot1: left-side patrol using west corridor to bypass partition2
    # Waypoints designed to avoid partition2, sofa, chair3, and box1.
    # WP0: (-2.0, -2.0) — start in lower-left room
    # WP1: (-2.0, -3.8) — go south to open south corridor
    # WP2: (-3.8, -3.8) — go west to SW corner
    # WP3: (-3.8,  2.5) — go north via west corridor (clears partition2 and sofa)
    # WP4: (-1.5,  2.5) — go east across top (above sofa)
    # WP5: (-3.8,  2.5) — return west
    # WP6: (-3.8, -2.0) — south through west corridor
    # Robot1: left-side patrol using a perfectly collision-free rectangular loop
    # Bypasses the boxes at X=-2.5 and the central pillar at X=-1.2 by using X=-3.8 and X=-0.6 corridors.
    robot1_waypoints = [
        -3.8, -3.5,
        -3.8,  3.5,
        -0.6,  3.5,
        -0.6, -3.5
    ]

    # Robot2: right-side patrol using a perfectly collision-free rectangular loop
    # Bypasses the boxes at X=2.5 and the central pillar at X=1.2 by using X=3.8 and X=0.6 corridors.
    robot2_waypoints = [
        3.8, -3.5,
        3.8,  3.5,
        0.6,  3.5,
        0.6, -3.5
    ]

    # Conditions to determine which nodes run:
    # robot1 runs unless robot2_only is True
    # robot2 runs unless robot1_only is True
    
    robot1_patrol = Node(
        package='swarm_explorer',
        executable='waypoint_patrol',
        name='robot1_patrol',
        condition=UnlessCondition(LaunchConfiguration('robot2_only')),
        parameters=[{
            'use_sim_time': True,
            'robot_name': 'robot1',
            'waypoints': robot1_waypoints,
            'linear_speed': 0.15,
            'angular_speed': 0.6,
            'goal_tolerance': 0.35,
            'heading_tolerance': 0.15,
            'control_rate': 10.0,
            'startup_delay': 5.0,
            'obstacle_stop_dist': 0.22,  # optimized stop distance (robot radius ~10cm)
            'obstacle_slow_dist': 0.45,  # optimized slow distance
        }],
        output='screen',
    )

    robot2_patrol = Node(
        package='swarm_explorer',
        executable='waypoint_patrol',
        name='robot2_patrol',
        condition=UnlessCondition(LaunchConfiguration('robot1_only')),
        parameters=[{
            'use_sim_time': True,
            'robot_name': 'robot2',
            'waypoints': robot2_waypoints,
            'linear_speed': 0.15,
            'angular_speed': 0.6,
            'goal_tolerance': 0.35,
            'heading_tolerance': 0.15,
            'control_rate': 10.0,
            'startup_delay': 5.0,
            'obstacle_stop_dist': 0.22,  # optimized stop distance
            'obstacle_slow_dist': 0.45,  # optimized slow distance
        }],
        output='screen',
    )

    return LaunchDescription([
        robot1_only_arg,
        robot2_only_arg,
        robot1_patrol,
        robot2_patrol,
    ])
