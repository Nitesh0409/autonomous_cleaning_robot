import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    cwd = os.getcwd()  # /mnt/f/Projects/Robotics Assignment/Robot

    urdf_file         = os.path.join(cwd, 'dustbin.urdf')
    world_file        = os.path.join(cwd, 'dustbin_world.sdf')
    mastermind_script = os.path.join(cwd, 'mastermind_cv_node.py')

    # Patch URDF: fix Windows paths and plugin extensions for Linux
    urdf_content = open(urdf_file).read()
    urdf_content = urdf_content.replace(
        'file://f:/Projects/Robotics Assignment/Robot/',
        f'file://{cwd}/'
    )
    urdf_content = urdf_content.replace('gazebo_ros_diff_drive.dll', 'libgazebo_ros_diff_drive.so')
    urdf_content = urdf_content.replace('libgazebo_ros_camera.dll', 'libgazebo_ros_camera.so')

    # Write patched URDF to temp file so spawn_entity also uses fixed version
    tmp_urdf = '/tmp/dustbin_patched.urdf'
    with open(tmp_urdf, 'w') as f:
        f.write(urdf_content)

    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_content}]
    )

    # 2. Gazebo Server (headless)
    gazebo = ExecuteProcess(
        cmd=['gzserver', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_file],
        additional_env={
            'GAZEBO_RESOURCE_PATH': '/usr/share/gazebo-11:/opt/ros/humble/share/gazebo_ros',
            'GAZEBO_PLUGIN_PATH':   '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/humble/lib',
            'GAZEBO_MODEL_PATH':    '/usr/share/gazebo-11/models',
        },
        output='screen'
    )

    # 3. Spawn Robot into Gazebo
    spawn_entity = ExecuteProcess(
        cmd=[
            'python3',
            '/opt/ros/humble/lib/gazebo_ros/spawn_entity.py',
            '-entity', 'smart_dustbin',
            '-file', tmp_urdf,
        ],
        output='screen'
    )

    # 4. Mastermind Node
    mastermind = ExecuteProcess(
        cmd=['python3', mastermind_script],
        output='screen'
    )

    # 5. Give trash ball an initial velocity so it arcs through the air
    kick_ball = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/set_entity_state',
            'gazebo_msgs/srv/SetEntityState',
            '{"state": {"name": "trash_ball", "pose": {"position": {"x": 2.0, "y": 0.0, "z": 1.5}, "orientation": {"w": 1.0}}, "twist": {"linear": {"x": -1.5, "y": 0.3, "z": 1.0}}, "reference_frame": "world"}}',
        ],
        output='screen'
    )

    # Delay actions until Gazebo is ready
    spawn_entity_delayed = TimerAction(period=10.0, actions=[spawn_entity])
    mastermind_delayed    = TimerAction(period=12.0, actions=[mastermind])
    kick_ball_delayed     = TimerAction(period=15.0, actions=[kick_ball])

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity_delayed,
        mastermind_delayed,
        kick_ball_delayed,
    ])
