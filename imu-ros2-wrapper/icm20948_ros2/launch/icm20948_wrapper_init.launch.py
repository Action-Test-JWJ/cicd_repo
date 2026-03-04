from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition,UnlessCondition
import os

def generate_launch_description():
    package_name = 'icm20948_ros2'

    icm20948_read_node = Node(
        package=package_name,
        executable='icm20948_read_node',
        output="screen",
    )

    return LaunchDescription([
        icm20948_read_node
    ])