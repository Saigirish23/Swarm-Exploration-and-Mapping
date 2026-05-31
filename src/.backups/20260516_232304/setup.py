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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saigirish',
    maintainer_email='185754693+Saigirish23@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_explorer = swarm_explorer.robot_explorer:main',
            'map_merger = swarm_explorer.map_merger:main',
        ],
    },
)
