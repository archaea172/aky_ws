import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world = os.path.join(
        get_package_share_directory('inrof_sim'),
        'worlds',
        'plane_wall.sdf'
    )

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
    ld.add_action(gzclient_cmd)

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
    ROBOT_NUM = 20
    GRID_COLUMNS = 5
    GRID_SPACING = 0.5
    ROBOT_Z = 0.05
    world_name = 'plane_world'
    gz_twist_type = 'ignition.msgs.Twist'
    bridge_arguments = []
    bridge_remappings = []

    for i in range(ROBOT_NUM):
        robot_name = f"robot_{i}"
        frame_prefix = f"{robot_name}/"
        x = (i % GRID_COLUMNS) * GRID_SPACING
        y = (i // GRID_COLUMNS) * GRID_SPACING

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

    return ld
