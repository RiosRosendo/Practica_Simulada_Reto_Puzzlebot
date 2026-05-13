import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'puzzlebot_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Puzzlebot Team',
    maintainer_email='a01198515@tec.mx',
    description='Autonomous navigation: A* planner, pure-pursuit follower, dynamic obstacle avoidance',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'astar_planner    = puzzlebot_navigation.astar_planner_node:main',
            'path_follower    = puzzlebot_navigation.path_follower_node:main',
            'obstacle_spawner = puzzlebot_navigation.obstacle_spawner_node:main',
        ],
    },
)
