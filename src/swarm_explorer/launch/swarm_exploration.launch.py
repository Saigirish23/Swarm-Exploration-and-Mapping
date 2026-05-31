#!/usr/bin/env python3
"""
Convenience launch for the full exploration stack.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_swarm = get_package_share_directory('swarm_explorer')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_swarm, 'launch', 'swarm_mapping.launch.py')
            ),
            launch_arguments={
                'use_exploration': 'true',
            }.items(),
        )
    ])
