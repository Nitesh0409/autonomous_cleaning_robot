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
            'planner_local_pid = robot.planner_local_pid:main',
            'planner_local_apf = robot.planner_local_apf:main',
            'planner_global_astar = robot.planner_global_astar:main',
            'verify_nav = robot.tests.verify_nav:main',
            'diagnose_frames = robot.tests.diagnose_frames:main',
            'planner_local_fgm = robot.planner_local_fgm:main',
            'planner_local_simple_apf = robot.planner_local_simple_apf:main'
        ],
    },
)
