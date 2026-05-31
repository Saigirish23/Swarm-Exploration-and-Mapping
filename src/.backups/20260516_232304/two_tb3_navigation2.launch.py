#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node


def generate_launch_description():
    pkg_tb3_nav2 = get_package_share_directory('turtlebot3_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Robot1 Nav2 stack
    robot1_params = os.path.join(
        pkg_tb3_nav2, 'param', 'robot1_nav2_params.yaml'
    )

    # Robot2 Nav2 stack
    robot2_params = os.path.join(
        pkg_tb3_nav2, 'param', 'robot2_nav2_params.yaml'
    )

    nav2_bringup = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    robot1_nav2 = GroupAction([
        PushRosNamespace('robot1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup),
            launch_arguments={
                'namespace': 'robot1',
                'use_namespace': 'true',
                'use_sim_time': 'true',
                'params_file': robot1_params,
            }.items(),
        ),
    ])

    robot2_nav2 = GroupAction([
        PushRosNamespace('robot2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup),
            launch_arguments={
                'namespace': 'robot2',
                'use_namespace': 'true',
                'use_sim_time': 'true',
                'params_file': robot2_params,
            }.items(),
        ),
    ])

    return LaunchDescription([
        robot1_nav2,
        robot2_nav2,
    ])
