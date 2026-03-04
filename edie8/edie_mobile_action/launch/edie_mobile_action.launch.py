import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name='edie_mobile_action'

    edie_mobile_action_node = Node(
        package=package_name,
        executable='edie_mobile_action_node',
        parameters=[{'debug_mode': False}],
    )

    return LaunchDescription([
        edie_mobile_action_node,
    ])