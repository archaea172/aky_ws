import os
import random
import launch

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch.event_handlers import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch.actions import RegisterEventHandler, EmitEvent
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import lifecycle_msgs.msg

def generate_launch_description():
    ld = LaunchDescription()

    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world = os.path.join(
        get_package_share_directory('irc_table'),
        'worlds',
        'irc_table.sdf'
    )

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(
                get_package_share_directory('irc_table'),
                'models'))
    ld.add_action(set_env_vars_resources)

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )
    ld.add_action(gzserver_cmd)

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )
    # ld.add_action(gzclient_cmd)

    spawn_balls_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('irc_table'),
                'launch',
                'spawn_random_balls.launch.py'
            )
        )
    )
    ld.add_action(spawn_balls_cmd)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("inrof_sim"),
                    "urdf",
                    "robot.urdf",
                ]
            ),
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    ROBOT_NUM = 5
    ROBOT_Z = 0.4
    world_name = 'irc_table'
    gz_twist_type = 'ignition.msgs.Twist'
    bridge_arguments = []
    bridge_remappings = []

    min_distance_sq = 0.1 ** 2
    ROBOT_XY = []
    while len(ROBOT_XY) < ROBOT_NUM:
        candidate = (random.uniform(-0.4, -0.1), random.uniform(-0.4, -0.1))
        if all(
            (candidate[0] - x) ** 2 + (candidate[1] - y) ** 2 >= min_distance_sq
            for x, y in ROBOT_XY
        ):
            ROBOT_XY.append(candidate)

    for i in range(ROBOT_NUM):
        robot_name = f"robot_{i}"
        frame_prefix = f"{robot_name}/"
        x = ROBOT_XY[i][0]
        y = ROBOT_XY[i][1]

        robot_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_name,
            name='robot_state_publisher',
            output='screen',
            parameters=[
                robot_description,
                {'frame_prefix': frame_prefix},
            ],
        )
        ld.add_action(robot_node)

        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{robot_name}',
            output='screen',
            arguments=[
                '-world', world_name,
                '-string', robot_description_content,
                '-name', robot_name,
                '-x', str(x),
                '-y', str(y),
                '-z', str(ROBOT_Z),
            ],
        )
        ld.add_action(spawn_node)

        bridge_arguments.extend([
            f'/model/{robot_name}/cmd_vel@geometry_msgs/msg/Twist]{gz_twist_type}',
            f'/model/{robot_name}/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            f'/model/{robot_name}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        ])
        bridge_remappings.extend([
            (f'/model/{robot_name}/cmd_vel', f'/{robot_name}/cmd_vel'),
            (f'/model/{robot_name}/pose', '/tf'),
            (f'/model/{robot_name}/odometry', f'/{robot_name}/odometry'),
        ])

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=bridge_arguments,
        remappings=bridge_remappings,
    )
    ld.add_action(bridge_node)

    map_yaml = os.path.join(
        get_package_share_directory('inrof_swerm'),
        'map',
        'irc.yaml'
    )
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        parameters=[{'yaml_filename': map_yaml}]
    )
    map_configure_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=map_server_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(map_server_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )
                )
            ]
        )
    )
    map_activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(map_server_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )
    ld.add_action(map_server_node)
    ld.add_action(map_configure_event_handler)
    ld.add_action(map_activate_event_handler)

    static_from_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    ld.add_action(static_from_map_to_odom)

    foxglove_bridge_cmd = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml',
            )
        ),
        launch_arguments={
            'port': '8765',
            'use_sim_time': 'true',
        }.items(),
    )

    ld.add_action(foxglove_bridge_cmd)

    return ld
