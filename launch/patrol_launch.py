import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable,
                             IncludeLaunchDescription, ExecuteProcess, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Spawn positions for up to 6 robots around the 6×6 arena perimeter
SPAWN_POSES = [
    (-2.4, -2.4, 0.785),    # SW corner, facing NE
    ( 2.4, -2.4, 2.356),    # SE corner, facing NW
    ( 2.4,  2.4,-2.356),    # NE corner, facing SW
    (-2.4,  2.4,-0.785),    # NW corner, facing SE
    ( 0.0, -2.4, 1.571),    # South mid, facing N
    ( 0.0,  2.4,-1.571),    # North mid, facing S
]


def patch_urdf(base_urdf: str, robot_name: str) -> str:
    """Replace all robot-specific topics and frame IDs in the URDF string."""
    return (base_urdf
        .replace('/model/robot/',                      f'/model/{robot_name}/')
        .replace('name="robot"',                       f'name="{robot_name}"', 1)
        .replace('<topic>/cmd_vel</topic>',            f'<topic>/{robot_name}/cmd_vel</topic>')
        .replace('<odom_topic>/odom</odom_topic>',     f'<odom_topic>/{robot_name}/odom</odom_topic>')
        .replace('<tf_topic>/tf</tf_topic>',           f'<tf_topic>/{robot_name}/tf</tf_topic>')
        .replace('<frame_id>odom</frame_id>',          f'<frame_id>{robot_name}/odom</frame_id>')
        .replace('<child_frame_id>base_footprint</child_frame_id>',
                 f'<child_frame_id>{robot_name}/base_footprint</child_frame_id>')
    )


def launch_setup(context, *args, **kwargs):
    """OpaqueFunction: evaluated at actual launch time so robot_count is known."""
    pkg_share  = get_package_share_directory('robot')
    urdf_file  = os.path.join(pkg_share, 'models', 'robot_dual_arm.urdf')
    rviz_config = os.path.join(pkg_share, 'config', 'patrol.rviz')

    n = max(1, min(6, int(LaunchConfiguration('robot_count').perform(context))))
    planner = LaunchConfiguration('planner').perform(context)

    with open(urdf_file, 'r') as f:
        base_urdf = f.read()

    nodes        = []
    bridge_args  = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    remap_args   = []

    for i in range(n):
        rname = f'robot_{i}'
        sx, sy, syaw = SPAWN_POSES[i]
        robot_urdf = patch_urdf(base_urdf, rname)

        # ── robot_state_publisher ──────────────────────────────────────────────
        nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=rname,
            parameters=[{
                'robot_description': robot_urdf,
                'use_sim_time': True,
                'frame_prefix': f'{rname}/',
            }],
            output='screen',
        ))

        # ── Spawn in Gazebo ────────────────────────────────────────────────────
        nodes.append(Node(
            package='ros_gz_sim', executable='create',
            arguments=['-name', rname, '-string', robot_urdf,
                       '-x', str(sx), '-y', str(sy), '-z', '0.05',
                       '-Y', str(syaw)],
            output='screen',
        ))

        # ── Static TF: map → robot_i/odom  (CRITICAL for RViz) ────────────────
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', f'{rname}/odom'],
            name=f'static_tf_{rname}',
        ))

        # ── TF bridge: Gazebo robot TF → ROS /tf ──────────────────────────────
        # The MecanumDrive plugin publishes to /{rname}/tf (GZ topic)
        nodes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            arguments=[f'/{rname}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
            name=f'tf_bridge_{rname}',
            output='screen',
        ))

        # ── Global planner (D* Lite OR A*) ───────────────────────────────────
        global_exec = ('planner_global_astar' if planner == 'astar'
                       else 'planner_global_dstar')
        nodes.append(Node(
            package='robot', executable=global_exec,
            namespace=rname,
            remappings=[
                ('/goal_pose',             f'/{rname}/goal_pose'),
                ('/odom',                  f'/{rname}/odom'),
                ('plan',                   f'/{rname}/plan'),
                ('/obstacles/exact_poses', '/obstacles/exact_poses'),
            ],
            parameters=[{'use_sim_time': True}],
        ))

        # ── Local planner (APF / DWA; A* uses DWA) ───────────────────────────
        local_exec = ('planner_local_dwa' if planner in ('dwa', 'astar')
                      else 'planner_local_apf')
        nodes.append(Node(
            package='robot', executable=local_exec,
            namespace=rname,
            remappings=[
                ('/cmd_vel',              f'/{rname}/cmd_vel'),
                ('/odom',                 f'/{rname}/odom'),
                ('/front_scan',           f'/{rname}/front_scan'),
                ('/rear_scan',            f'/{rname}/rear_scan'),
                ('/planner/face_movement',f'/planner/face_movement'),  # shared
                ('/goal_pose',            f'/{rname}/goal_pose'),
                ('/obstacles/exact_poses','/obstacles/exact_poses'),   # shared
                ('plan',                  f'/{rname}/plan'),
            ],
            parameters=[{'use_sim_time': True}],
        ))

        # ── Bridge args ────────────────────────────────────────────────────────
        bridge_args += [
            f'/{rname}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/model/{rname}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/model/{rname}/front_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/model/{rname}/rear_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ]
        remap_args += [
            '-r', f'/model/{rname}/odom:=/{rname}/odom',
            '-r', f'/model/{rname}/front_scan:=/{rname}/front_scan',
            '-r', f'/model/{rname}/rear_scan:=/{rname}/rear_scan',
        ]

    # ── Single shared bridge (cmd_vel + odom + scans + clock) ─────────────────
    bridge = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge']
            + bridge_args + ['--ros-args'] + remap_args,
        output='screen',
    )

    # ── Support nodes ──────────────────────────────────────────────────────────
    obstacle_tracker = Node(package='robot', executable='obstacle_tracker',
                            parameters=[{'use_sim_time': True}], output='screen')
    garbage_spawner  = Node(package='robot', executable='garbage_spawner',
                            parameters=[{'use_sim_time': True}], output='screen')
    patrol_manager   = Node(package='robot', executable='patrol_mission_v2',
                            parameters=[{'use_sim_time': True, 'robot_count': n}],
                            output='screen')

    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}])

    return [bridge, obstacle_tracker, garbage_spawner, patrol_manager, rviz] + nodes


def generate_launch_description():
    pkg_share  = get_package_share_directory('robot')
    pkg_parent = os.path.dirname(pkg_share)

    return LaunchDescription([
        DeclareLaunchArgument('robot_count', default_value='1',
                              description='Number of patrol robots (1-6)'),
        DeclareLaunchArgument('planner', default_value='apf'),

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', [
            pkg_parent,
            os.path.join(pkg_share, 'worlds'),
            os.path.join(pkg_share, 'models'),
        ]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'),
                             'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': f'-r "{os.path.join(pkg_share, "worlds", "patrol_arena.sdf")}"'
            }.items()
        ),

        OpaqueFunction(function=launch_setup),
    ])
