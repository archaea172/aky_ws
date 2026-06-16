import os

from launch import LaunchDescription
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

    leader_pos_server_node = Node(
        package='inrof_control',
        executable='leader_pos_server',
        name='leader_pos_server'
    )
    ld.add_action(leader_pos_server_node)

    path_generator_node = Node(
        package='inrof_control',
        executable='path_generate_server',
        name='path_generator_server'
    )
    ld.add_action(path_generator_node)

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