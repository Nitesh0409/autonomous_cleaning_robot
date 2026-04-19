import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robot')
    pkg_parent = os.path.dirname(pkg_share)
    
    # Environment Setup
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[pkg_parent]
    )

    # 1. World & Robot paths
    world_path = os.path.join(pkg_share, 'worlds', 'calibration_bay.sdf')
    urdf_file = os.path.join(pkg_share, 'models', 'robot_dual_arm.urdf')

    # 2. Gazebo Headless Server
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -s "{world_path}"'}.items(),
    )

    # 3. Robot State Publisher
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # 4. Spawn & Bridge
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robot', '-file', urdf_file, '-world', 'calibration_bay', '-z', '0.05']
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml'),
            'use_sim_time': True
        }]
    )

    # 5. The Specialized Validation Nodes
    dstar = Node(
        package='robot',
        executable='planner_global_dstar',
        parameters=[{'use_sim_time': True}]
    )
    
    apf = Node(
        package='robot',
        executable='planner_local_apf',
        parameters=[{'use_sim_time': True}]
    )

    validator = Node(
        package='robot',
        executable='mission_validator',
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        rsp,
        spawn,
        bridge,
        dstar,
        apf,
        validator
    ])
