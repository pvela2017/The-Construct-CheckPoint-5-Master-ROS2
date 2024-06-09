from launch import LaunchDescription, actions, substitutions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():

    obstacle_arg = actions.DeclareLaunchArgument('obstacle', default_value='0.3')
    degrees_arg = actions.DeclareLaunchArgument('degrees', default_value='90')
    final_approach_arg = actions.DeclareLaunchArgument('final_approach', default_value='False')

    approach_service = Node(
        package='attach_shelf',
        executable='approach_service_server_node',
        output='screen',
        name='approach_service_server_node',
        parameters=[{'use_sim_time': True}]
        )

    preapproach_node = Node(
        package='attach_shelf',
        executable='pre_approach_v2_node',
        output='screen',
        name='pre_approach_v2_node',
        parameters=[{'use_sim_time': True}, {'obstacle': substitutions.LaunchConfiguration('obstacle')},\
                    {'degrees': substitutions.LaunchConfiguration('degrees')},\
                    {'final_approach': substitutions.LaunchConfiguration('final_approach')}])


    return LaunchDescription(
        [
            obstacle_arg,
            degrees_arg,
            final_approach_arg,
            preapproach_node,
            approach_service,
        ]
    )