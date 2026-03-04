from launch import LaunchDescription
from launch.actions import LogInfo
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('edie_node_manager'),
            'param',
            'params.yaml'
        )

    node_manager = Node(
                package='edie_node_manager',
                executable='edie_node_manager',
                output='screen',
                parameters=[config]
    )

    return LaunchDescription([
        node_manager,
        LogInfo(msg=['Execute the edie_node_manager.']),
    ])
