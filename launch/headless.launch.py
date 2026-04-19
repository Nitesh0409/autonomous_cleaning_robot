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
    world_path = os.path.join(pkg_share, 'worlds', 'basic.world')

    # 2. Gazebo Sim (Harmonic) Launch - HEADLESS (-s for server only)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -s "{world_path}"'}.items(),
    )

    # 3. Robot State Publisher
    urdf_file = os.path.join(pkg_share, 'models', 'robot.urdf')
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
            '-name', 'smart_dustbin',
            '-file', urdf_file,
            '-world', 'basic_world',
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
            'config_file': os.path.join(pkg_share, 'config', 'bridge.yaml'),
            'use_sim_time': True
        }],
        output='screen'
    )

    # 6. Lino Tactical Navigator (Local Planner)
    lino_node = Node(
        package='robot',
        executable='lino_tactical',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 7. A* Global Navigation
    astar_node = Node(
        package='robot',
        executable='astar_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Note: RViz is OMITTED for headless testing

    return LaunchDescription([
        gz_sim,
        rsp,
        spawn,
        bridge,
        lino_node,
        astar_node
    ])
