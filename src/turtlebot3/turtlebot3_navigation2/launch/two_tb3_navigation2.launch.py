#!/usr/bin/env python3
#
# Launch Nav2 navigation stack for two TurtleBot3 robots.
# Uses navigation_launch.py ONLY (no AMCL, no map_server).
# Cartographer provides map -> odom TF and live maps.
#

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, SetRemap


def generate_launch_description():
    pkg_tb3_nav2 = get_package_share_directory('turtlebot3_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_navigation = os.path.join(
        pkg_tb3_nav2, 'launch', 'custom_navigation_launch.py'
    )

    # Robot1 Nav2 params
    robot1_params = os.path.join(
        pkg_tb3_nav2, 'param', 'robot1_nav2_params.yaml'
    )

    # Robot2 Nav2 params
    robot2_params = os.path.join(
        pkg_tb3_nav2, 'param', 'robot2_nav2_params.yaml'
    )

    # Robot1 Nav2 navigation stack (no localization, no AMCL)
    # GroupAction + PushRosNamespace ensures all nodes are launched
    # under /robot1 namespace for proper param matching.
    robot1_nav2 = GroupAction([
        PushRosNamespace('robot1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_navigation),
            launch_arguments={
                'namespace': 'robot1',
                'use_sim_time': 'true',
                'autostart': 'true',
                'params_file': robot1_params,
                'use_velocity_smoother': 'false',
                'use_composition': 'False',
            }.items(),
        ),
    ])

    # Robot2 Nav2 navigation stack (no localization, no AMCL)
    robot2_nav2 = GroupAction([
        PushRosNamespace('robot2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_navigation),
            launch_arguments={
                'namespace': 'robot2',
                'use_sim_time': 'true',
                'autostart': 'true',
                'params_file': robot2_params,
                'use_velocity_smoother': 'false',
                'use_composition': 'False',
            }.items(),
        ),
    ])

    return LaunchDescription([
        robot1_nav2,
        robot2_nav2,
    ])
