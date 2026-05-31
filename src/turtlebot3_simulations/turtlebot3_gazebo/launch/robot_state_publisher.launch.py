#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_tb3_description = get_package_share_directory('turtlebot3_description')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    frame_prefix = LaunchConfiguration('frame_prefix', default='')

    urdf_path = os.path.join(pkg_tb3_description, 'urdf', 'turtlebot3_burger.urdf')

    with open(urdf_path, 'r') as infp:
        robot_desc_content = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'frame_prefix',
            default_value='',
            description='TF frame prefix for robot'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc_content,
                'use_sim_time': use_sim_time,
                'frame_prefix': frame_prefix,
            }],
            output='screen',
        ),
    ])
