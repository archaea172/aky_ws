import os

import launch
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch.event_handlers import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch.actions import RegisterEventHandler, EmitEvent

import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # Load parameters for follower and map server
    follower_param = os.path.join(
        get_package_share_directory('inrof_swerm'),
        'config',
        'follower.yaml'
    )
    map_yaml = os.path.join(
        get_package_share_directory('inrof_swerm'),
        'map',
        'irc.yaml'
    )

    follower_node = Node(
        package='inrof_swerm',
        executable='follower_node',
        name='follower_node',
        parameters=[follower_param]
    )
    ld.add_action(follower_node)

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