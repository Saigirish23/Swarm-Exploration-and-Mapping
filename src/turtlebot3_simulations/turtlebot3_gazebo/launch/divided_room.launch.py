#!/usr/bin/env python3

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace


def _create_robot_sdf(base_sdf_path: str, save_path: str, namespace: str):
    tree = ET.parse(base_sdf_path)
    root = tree.getroot()

    for odom_frame_tag in root.iter('odometry_frame'):
        odom_frame_tag.text = f'{namespace}/odom'
    for base_frame_tag in root.iter('robot_base_frame'):
        base_frame_tag.text = f'{namespace}/base_footprint'
    for scan_frame_tag in root.iter('frame_name'):
        scan_frame_tag.text = f'{namespace}/base_scan'

    # Keep topics naturally namespaced by spawn_entity's -robot_namespace.
    sdf_modified = ET.tostring(root, encoding='unicode')
    sdf_modified = '<?xml version="1.0" ?>\n' + sdf_modified

    with open(save_path, 'w', encoding='utf-8') as file:
        file.write(sdf_modified)


def generate_launch_description():
    os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')
    turtlebot3_model = os.environ['TURTLEBOT3_MODEL']

    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    launch_dir = os.path.join(pkg_tb3_gazebo, 'launch')
    bringup_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_bringup'), 'launch'
    )

    world = os.path.join(pkg_tb3_gazebo, 'worlds', 'divided_room.world')
    model_folder = f'turtlebot3_{turtlebot3_model}'
    base_sdf = os.path.join(pkg_tb3_gazebo, 'models', model_folder, 'model.sdf')
    tmp_dir = os.path.join(pkg_tb3_gazebo, 'models', model_folder, 'tmp')
    os.makedirs(tmp_dir, exist_ok=True)

    robot_specs = [
        {'name': 'robot1', 'x': '-2.5', 'y': '0.0'},
        {'name': 'robot2', 'x': '2.5', 'y': '0.0'},
    ]

    robot_actions = []
    generated_sdf_files = []
    for spec in robot_specs:
        ns = spec['name']
        sdf_path = os.path.join(tmp_dir, f'{ns}.sdf')
        _create_robot_sdf(base_sdf, sdf_path, ns)
        generated_sdf_files.append(sdf_path)

        rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_launch_dir, 'turtlebot3_state_publisher.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'namespace': ns,
            }.items(),
        )

        spawner = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'multi_spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': spec['x'],
                'y_pose': spec['y'],
                'robot_name': ns,
                'namespace': ns,
                'sdf_path': sdf_path,
            }.items(),
        )

        robot_actions.append(
            GroupAction([
                PushRosNamespace(ns),
                rsp,
                spawner,
            ])
        )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    def _cleanup(_event, _context):
        for file_path in generated_sdf_files:
            if os.path.exists(file_path):
                os.remove(file_path)
        return []

    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        RegisterEventHandler(OnShutdown(on_shutdown=_cleanup)),
        *robot_actions,
    ])
