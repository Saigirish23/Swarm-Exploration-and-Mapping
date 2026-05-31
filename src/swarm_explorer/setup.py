import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'swarm_explorer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saigirish',
    maintainer_email='185754693+Saigirish23@users.noreply.github.com',
    description='Multi-robot frontier exploration and map merging for TurtleBot3 swarm',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'robot_explorer = swarm_explorer.robot_explorer:main',
            'map_merger = swarm_explorer.map_merger:main',
            'submap_to_map = swarm_explorer.submap_to_map_node:main',
            'waypoint_patrol = swarm_explorer.waypoint_patrol:main',
            'frontier_coordinator = swarm_explorer.frontier_coordinator:main',
            'robot_footprint_broadcaster = swarm_explorer.robot_footprint_broadcaster:main',
        ],
    },
)
