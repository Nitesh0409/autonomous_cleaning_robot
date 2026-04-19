import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch.conditions import IfCondition, LaunchConfigurationEquals

def generate_launch_description():
    package_name = 'robot'
    pkg_share = get_package_share_directory(package_name)
    
    # Declare Launch Arguments
    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='1',
        description='0:Wall, 1:U-Trap, 2:Narrow Gap'
    )
    
    navigator_arg = DeclareLaunchArgument(
        'navigator',
        default_value='lino',
        description='Options: [lino, apf, none]'
    )
    
    # 1. World file path
    world_path = os.path.join(pkg_share, 'worlds', 'basic.world')

    # 2. Gazebo Sim Launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r "{world_path}"'}.items(),
    )

    # 3. Robot State Publisher
    urdf_file = os.path.join(pkg_share, 'models', 'robot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # 4. Spawn Entity
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'smart_dustbin', '-file', urdf_file, '-world', 'basic_world', '-x', '0.0', '-y', '0.0', '-z', '0.2'],
        output='screen',
    )

    # 5. Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': os.path.join(pkg_share, 'config', 'bridge.yaml'), 'use_sim_time': True}],
        output='screen'
    )

    # 6. SLAM Toolbox
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'slam_config.yaml'), {'use_sim_time': True}]
    )

    # 7. Navigator Option A: LinoTactical (New)
    lino_node = Node(
        package='robot',
        executable='lino_tactical',
        output='screen',
        condition=LaunchConfigurationEquals('navigator', 'lino'),
        parameters=[{'use_sim_time': True}]
    )

    # 8. Navigator Option B: PotentialField (Original)
    apf_node = Node(
        package='robot',
        executable='potential_field_node',
        output='screen',
        condition=LaunchConfigurationEquals('navigator', 'apf'),
        parameters=[{'use_sim_time': True}]
    )

    # 9. A* Global Planner
    astar_node = Node(
        package='robot',
        executable='astar_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 10. RViz2
    rviz_config = os.path.join(pkg_share, 'config', 'robot.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        scenario_arg,
        navigator_arg,
        gz_sim,
        rsp,
        spawn,
        bridge,
        slam,
        lino_node,
        apf_node,
        astar_node,
        rviz
    ])
