import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # Load parameters for follower and map server
    follower_param = os.path.join(
        get_package_share_directory('inrof_swerm'),
        'config',
        'follower.yaml'
    )

    follower_node = Node(
        package='inrof_swerm',
        executable='follower_node',
        name='follower_node',
        parameters=[follower_param]
    )
    ld.add_action(follower_node)

    leader_rl_server_node = ExecuteProcess(
        cmd=[
            'uv',
            'run',
            'ros2',
            'run',
            'inrof_control',
            'leader_rl_server.py',
        ],
        cwd='/home/t-semi/aky_ws/src/inrof_control',
        output='screen',
    )
    ld.add_action(leader_rl_server_node)

    lifecycle_server_node = Node(
        package='inrof_bt',
        executable='lifecycle_server',
        name='lifecycle_server'
    )
    ld.add_action(lifecycle_server_node)

    bt_node = Node(
        package='inrof_bt',
        executable='inrof_bt_node',
        name='inrof_bt_node'
    )
    ld.add_action(bt_node)

    return ld