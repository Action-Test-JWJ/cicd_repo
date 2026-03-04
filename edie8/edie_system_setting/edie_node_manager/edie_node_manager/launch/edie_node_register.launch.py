from launch import LaunchDescription
from launch.actions import LogInfo
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():

    node_register = Node(
                package='edie_node_manager',
                executable='edie_node_register',
                output='screen',
    )

    return LaunchDescription([
        node_register,
        LogInfo(msg=['Execute the edie_node_register.']),
    ])
