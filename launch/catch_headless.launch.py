import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'robot'
    pkg_share = get_package_share_directory(package_name)

    # 1. World file path
    world_path = os.path.join(pkg_share, 'worlds', 'catch.world')

    # 2. Gazebo Sim Launch (HEADLESS)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-s -r "{world_path}"'}.items(),
    )

    # 3. Robot State Publisher
    urdf_file = os.path.join(pkg_share, 'models', 'mecanum_bot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # 4. Spawn Entity (Robot)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot',
            '-file', urdf_file,
            '-world', 'catch_world',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen',
    )

    # 5. Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml'),
            'use_sim_time': True
        }],
        output='screen'
    )

    # 6. Interceptor Controller
    interceptor_node = Node(
        package='robot',
        executable='interceptor',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 7. Projectile Launcher
    launcher_node = Node(
        package='robot',
        executable='projectile_launcher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 8. True Vision Perception
    vision_node = Node(
        package='robot',
        executable='vision_tracker',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gz_sim,
        rsp,
        spawn,
        bridge,
        interceptor_node,
        launcher_node,
        vision_node
    ])
