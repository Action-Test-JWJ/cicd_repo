from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition,UnlessCondition
import os

def generate_launch_description():
    package_name = 'mpu9250_ros2'

    imu_read_node = Node(
        package=package_name,
        executable='imu_read_node',
        output="screen",
    )

    imu_offset_init_node = Node(
        package=package_name,
        executable='imu_offset_init_node',
        output="screen",
    )

    return LaunchDescription([
        imu_read_node
        # imu_offset_init_node
    ])