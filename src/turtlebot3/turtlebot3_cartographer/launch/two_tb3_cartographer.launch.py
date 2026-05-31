#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_tb3_cartographer = get_package_share_directory('turtlebot3_cartographer')

    # Robot1 Cartographer node - NO namespace, unique node name
    robot1_cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='robot1_cartographer_node',
        parameters=[{
            'use_sim_time': True,
        }],
        arguments=[
            '-configuration_directory', os.path.join(pkg_tb3_cartographer, 'config'),
            '-configuration_basename', 'robot1_cartographer.lua',
        ],
        remappings=[
            ('scan', '/robot1/scan'),
            ('imu', '/robot1/imu'),
            ('odom', '/robot1/odom'),
            ('submap_list', '/robot1/submap_list'),
            ('/submap_query', '/robot1/submap_query'),
            ('trajectory_node_list', '/robot1/trajectory_node_list'),
            ('landmark_poses_list', '/robot1/landmark_poses_list'),
            ('constraint_list', '/robot1/constraint_list'),
            ('tracked_pose', '/robot1/tracked_pose'),
        ],
        output='screen',
    )

    # Robot1 map publisher (replaces cartographer_occupancy_grid_node)
    robot1_map = Node(
        package='swarm_explorer',
        executable='submap_to_map',
        name='robot1_submap_to_map',
        parameters=[{
            'use_sim_time': True,
            'robot_ns': 'robot1',
            'resolution': 0.05,
            'publish_period_sec': 1.0,
            'map_padding': 1.0,
            'occupied_threshold': 55,
            'free_threshold': 45,
            'speckle_filter_enabled': True,
            'speckle_max_size': 6,
        }],
        output='screen',
    )

    # Robot2 Cartographer node - NO namespace, unique node name
    robot2_cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='robot2_cartographer_node',
        parameters=[{
            'use_sim_time': True,
        }],
        arguments=[
            '-configuration_directory', os.path.join(pkg_tb3_cartographer, 'config'),
            '-configuration_basename', 'robot2_cartographer.lua',
        ],
        remappings=[
            ('scan', '/robot2/scan'),
            ('imu', '/robot2/imu'),
            ('odom', '/robot2/odom'),
            ('submap_list', '/robot2/submap_list'),
            ('/submap_query', '/robot2/submap_query'),
            ('trajectory_node_list', '/robot2/trajectory_node_list'),
            ('landmark_poses_list', '/robot2/landmark_poses_list'),
            ('constraint_list', '/robot2/constraint_list'),
            ('tracked_pose', '/robot2/tracked_pose'),
        ],
        output='screen',
    )

    # Robot2 map publisher (replaces cartographer_occupancy_grid_node)
    robot2_map = Node(
        package='swarm_explorer',
        executable='submap_to_map',
        name='robot2_submap_to_map',
        parameters=[{
            'use_sim_time': True,
            'robot_ns': 'robot2',
            'resolution': 0.05,
            'publish_period_sec': 1.0,
            'map_padding': 1.0,
            'occupied_threshold': 55,
            'free_threshold': 45,
            'speckle_filter_enabled': True,
            'speckle_max_size': 6,
        }],
        output='screen',
    )

    ld = LaunchDescription([
        robot1_cartographer,
        robot1_map,
        robot2_cartographer,
        robot2_map,
    ])

    return ld

