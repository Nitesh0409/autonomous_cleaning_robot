import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Maintainer',
    maintainer_email='robot_user@example.com',
    description='Professional Tactical Navigation for Holonomic Robots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_ui = robot.robot_ui:main',
            'planner_local_apf = robot.planner_local_apf:main',
            'planner_global_astar = robot.planner_global_astar:main',
            'cleaner_mission_manager = robot.cleaner_mission_manager:main',
            'mission_validator = robot.mission_validator:main',
            'calib_verify = robot.calib_verify:main',
            'obstacle_tracker = robot.obstacle_tracker:main',
            'garbage_spawner = robot.garbage_spawner:main',
            'patrol_mission_v2 = robot.patrol_mission_v2:main',
        ],
    },
)
