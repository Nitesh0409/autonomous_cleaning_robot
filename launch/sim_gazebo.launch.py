import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable,
                             DeclareLaunchArgument, ExecuteProcess, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def _gz_launch(context, *args, **kwargs):
    """OpaqueFunction: resolve world path at actual launch time."""
    pkg_share  = get_package_share_directory('robot')
    world_name = LaunchConfiguration('world_name').perform(context)
    if not world_name.endswith('.sdf'):
        world_name += '.sdf'
    world_path = os.path.join(pkg_share, 'worlds', world_name)
    if not os.path.exists(world_path):
        raise FileNotFoundError(f"World not found: {world_path}")
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r "{world_path}"'}.items(),
    )
    return [gz]


def generate_launch_description():
    package_name = 'robot'
    pkg_share    = get_package_share_directory(package_name)
    urdf_file    = os.path.join(pkg_share, 'models', 'robot_dual_arm.urdf')
    pkg_parent   = os.path.dirname(pkg_share)

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[pkg_parent,
               os.path.join(pkg_share, 'worlds'),
               os.path.join(pkg_share, 'models'),
               os.path.join(pkg_share, 'meshes')]
    )

    world_arg   = DeclareLaunchArgument('world_name', default_value='mini_proving_ground')
    planner_arg = DeclareLaunchArgument('planner',    default_value='apf')
    world_config   = LaunchConfiguration('world_name')
    planner_config = LaunchConfiguration('planner')

    # ── Ros─Gz bridge ───────────────────────────────────────────────────
    bridge_cmd = [
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/model/robot/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/model/robot/front_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/model/robot/rear_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '--ros-args',
        '-r', '/model/robot/ground_truth:=/ground_truth',
        '-r', '/model/robot/front_scan:=/front_scan',
        '-r', '/model/robot/rear_scan:=/rear_scan'
    ]
    bridge = ExecuteProcess(cmd=bridge_cmd, output='screen')

    # ── Robot state publisher + Gazebo spawn ──────────────────────────────
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'robot', '-file', urdf_file, '-z', '0.05'],
        output='screen',
    )

    # ── Static TF: map → odom ──────────────────────────────────────────
    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    global_planner_dstar = Node(
        package='robot', executable='planner_global_dstar',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(PythonExpression(["'", planner_config, "' != 'astar'"]))
    )
    global_planner_astar = Node(
        package='robot', executable='planner_global_astar',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(PythonExpression(["'", planner_config, "' == 'astar'"]))
    )

    local_planner_apf = Node(
        package='robot', executable='planner_local_apf',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(PythonExpression(["'", planner_config, "' == 'apf'"]))
    )
    local_planner_dwa = Node(
        package='robot', executable='planner_local_dwa',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(PythonExpression(["'", planner_config, "' in ('dwa', 'astar')"]))
    )

    nav_monitor = Node(package='robot', executable='nav_monitor',
                       parameters=[{'use_sim_time': True}], output='screen')

    rviz_config = os.path.join(pkg_share, 'config', 'simple.rviz')
    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        world_arg, planner_arg,
        set_gz_resource_path,
        OpaqueFunction(function=_gz_launch),
        rsp, spawn_robot, bridge,
        static_tf,
        global_planner_dstar, global_planner_astar,
        local_planner_apf, local_planner_dwa,
        nav_monitor, rviz,
    ])
