#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy


def generate_launch_description():
    pkg_tb3_cartographer = get_package_share_directory('turtlebot3_cartographer')
    pkg_cartographer_ros = get_package_share_directory('cartographer_ros')

    # QoS profile for transient-local maps (late joiners get latest map)
    map_qos = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )

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
            ('trajectory_node_list', '/robot1/trajectory_node_list'),
            ('landmark_poses_list', '/robot1/landmark_poses_list'),
            ('constraint_list', '/robot1/constraint_list'),
            ('tracked_pose', '/robot1/tracked_pose'),
        ],
        output='screen',
    )

    # Robot1 occupancy grid node - NO namespace, unique node name
    robot1_occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='robot1_occupancy_grid_node',
        parameters=[{
            'use_sim_time': True,
            'resolution': 0.05,
            'publish_period_sec': 1.0,
        }],
        remappings=[
            ('submap_list', '/robot1/submap_list'),
            ('map', '/robot1/map'),
        ],
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
            ('trajectory_node_list', '/robot2/trajectory_node_list'),
            ('landmark_poses_list', '/robot2/landmark_poses_list'),
            ('constraint_list', '/robot2/constraint_list'),
            ('tracked_pose', '/robot2/tracked_pose'),
        ],
        output='screen',
    )

    # Robot2 occupancy grid node - NO namespace, unique node name
    robot2_occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='robot2_occupancy_grid_node',
        parameters=[{
            'use_sim_time': True,
            'resolution': 0.05,
            'publish_period_sec': 1.0,
        }],
        remappings=[
            ('submap_list', '/robot2/submap_list'),
            ('map', '/robot2/map'),
        ],
        output='screen',
    )

    ld = LaunchDescription([
        robot1_cartographer,
        robot1_occupancy_grid,
        robot2_cartographer,
        robot2_occupancy_grid,
    ])

    return ld
