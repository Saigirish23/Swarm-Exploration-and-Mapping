#!/usr/bin/env python3
"""
Master launch file for the TurtleBot3 swarm mapping system.

Launches everything in sequence with appropriate delays:
    1. Gazebo simulation world
    2. Cartographer SLAM + submap decoders (delayed 5s for Gazebo)
    3. Static TF transforms (map -> robotX/map)
    4. Nav2 stack (exploration mode only)
    5. Waypoint patrol OR frontier exploration nodes
    6. Map merger (delayed for map population)
    7. RViz visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_swarm = get_package_share_directory('swarm_explorer')
    pkg_carto = get_package_share_directory('turtlebot3_cartographer')
    pkg_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2 = get_package_share_directory('turtlebot3_navigation2')

    use_exploration_arg = DeclareLaunchArgument(
        'use_exploration',
        default_value='false',
        description='Launch frontier exploration instead of waypoint patrol',
    )

    robot1_only_arg = DeclareLaunchArgument(
        'robot1_only',
        default_value='False',
        description='If True, only launch robot1 explorer'
    )

    robot2_only_arg = DeclareLaunchArgument(
        'robot2_only',
        default_value='False',
        description='If True, only launch robot2 explorer'
    )

    use_coordination_arg = DeclareLaunchArgument(
        'use_coordination',
        default_value='false',
        description='Enable coordinated frontier exploration',
    )

    # ── 1. Gazebo world ─────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'swarm_indoor_world.launch.py')
        )
    )

    # ── 2. Cartographer SLAM + submap decoders (delayed 5s) ─
    cartographer = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_carto, 'launch', 'two_tb3_cartographer.launch.py')
                )
            ),
        ]
    )

    # ── 3. Static TF: map -> robotX/map (identity) ─────────
    static_tf_robot1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_robot1_map_tf',
        arguments=['-2.0', '0.0', '0', '0', '0', '0', 'map', 'robot1/map'],
        parameters=[{'use_sim_time': True}],
    )

    static_tf_robot2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_robot2_map_tf',
        arguments=['2.0', '0.0', '0', '0', '0', '0', 'map', 'robot2/map'],
        parameters=[{'use_sim_time': True}],
    )

    # ── 4. Nav2 stack (exploration only, delayed 8s) ────────
    nav2 = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2, 'launch', 'two_tb3_navigation2.launch.py')
                )
            ),
        ],
        condition=IfCondition(LaunchConfiguration('use_exploration')),
    )

    # ── 5. Waypoint patrols (mapping) OR frontier exploration ──
    patrols = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_swarm, 'launch', 'two_robot_patrol.launch.py')
                )
            ),
        ],
        condition=UnlessCondition(LaunchConfiguration('use_exploration')),
    )

    explorers = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_swarm, 'launch', 'two_robot_exploration.launch.py')
                ),
                launch_arguments={
                    'robot1_only': LaunchConfiguration('robot1_only'),
                    'robot2_only': LaunchConfiguration('robot2_only'),
                }.items(),
            ),
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('use_exploration'), "' == 'true' and '",
            LaunchConfiguration('use_coordination'), "' == 'false'"
        ])),
    )

    explorers_coord = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_swarm, 'launch', 'two_robot_exploration_coordinated.launch.py')
                ),
                launch_arguments={
                    'robot1_only': LaunchConfiguration('robot1_only'),
                    'robot2_only': LaunchConfiguration('robot2_only'),
                }.items(),
            )
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('use_exploration'), "' == 'true' and '",
            LaunchConfiguration('use_coordination'), "' == 'true'"
        ])),
    )

    # ── 6. Map merger (delayed 15s) ─────────────────────────
    merger = TimerAction(
        period=15.0,
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

    # ── 7. RViz ─────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_swarm, 'rviz', 'swarm_mapping.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        use_exploration_arg,
        robot1_only_arg,
        robot2_only_arg,
        use_coordination_arg,
        gazebo,
        cartographer,
        static_tf_robot1,
        static_tf_robot2,
        nav2,
        patrols,
        explorers,
        explorers_coord,
        merger,
        rviz,
    ])
