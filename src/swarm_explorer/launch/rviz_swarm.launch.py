#!/usr/bin/env python3
"""
Launch RViz2 with automatic fresh mapping hot-restart capability.

When launched, this script:
  1. Kills any existing mapping nodes (cartographer, submap_to_map, map_merger, waypoint_patrol).
  2. Resets the Gazebo simulation (coordinates and time) back to zero.
  3. Launches a fresh SLAM and navigation stack so mapping starts fresh from the beginning.
  4. Launches RViz2.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_swarm = get_package_share_directory('swarm_explorer')
    pkg_carto = get_package_share_directory('turtlebot3_cartographer')
    rviz_config = os.path.join(pkg_swarm, 'rviz', 'two_tb3_swarm.rviz')

    # ── 1. Kill any existing active mapping/patrol/rviz processes ──
    kill_old_nodes = ExecuteProcess(
        cmd=['pkill -f "cartographer_node|submap_to_map|map_merger|waypoint_patrol|rviz2|static_transform_publisher" || true'],
        shell=True,
        output='screen'
    )

    # ── 2. Reset the Gazebo Classic simulation poses and time ─
    reset_simulation = ExecuteProcess(
        cmd=['ros2 service call /reset_simulation std_srvs/srv/Empty {} || true'],
        shell=True,
        output='screen'
    )

    # ── 3. Start fresh Cartographer SLAM + submap decoders ────
    cartographer = TimerAction(
        period=1.5,  # Give process manager a moment to free ports
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_carto, 'launch', 'two_tb3_cartographer.launch.py')
                )
            ),
        ]
    )

    # ── 4. Publish static TFs (identity) ──────────────────────
    static_tf_robot1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_robot1_map_tf',
        arguments=['-0.5', '0.5', '0', '0', '0', '0', 'map', 'robot1/map'],
        parameters=[{'use_sim_time': True}],
    )

    static_tf_robot2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_robot2_map_tf',
        arguments=['0.5', '0.5', '0', '0', '0', '0', 'map', 'robot2/map'],
        parameters=[{'use_sim_time': True}],
    )

    # ── 5. Start fresh Waypoint patrols (delayed to let SLAM start)
    patrols = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_swarm, 'launch', 'two_robot_patrol.launch.py')
                )
            ),
        ]
    )

    # ── 6. Start fresh Map merger (delayed to let maps populate) ─
    merger = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='swarm_explorer',
                executable='map_merger',
                name='map_merger',
                parameters=[
                    os.path.join(pkg_swarm, 'config', 'map_merge.yaml'),
                ],
                output='screen',
            ),
        ]
    )

    # ── 7. Launch RViz2 (delayed to prevent concurrent pkill termination) ──
    rviz_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                output='screen',
            ),
        ]
    )

    return LaunchDescription([
        kill_old_nodes,
        reset_simulation,
        cartographer,
        static_tf_robot1,
        static_tf_robot2,
        patrols,
        merger,
        rviz_node,
    ])
