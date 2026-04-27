import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

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
    GRID_SPACING = 0.1
    ROBOT_Z = 0.34
    gz_twist_type = 'ignition.msgs.Twist' if os.environ.get('ROS_DISTRO') == 'humble' else 'gz.msgs.Twist'

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

        world_name = 'plain_world'

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

        cmd_vel_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=robot_name,
            name='cmd_vel_bridge',
            output='screen',
            arguments=[
                f'/model/{robot_name}/cmd_vel@geometry_msgs/msg/Twist]{gz_twist_type}',
            ],
            remappings=[
                (f'/model/{robot_name}/cmd_vel', f'/{robot_name}/cmd_vel'),
            ],
        )
        ld.add_action(cmd_vel_bridge)

    return ld
