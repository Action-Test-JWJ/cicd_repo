import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    package_name='edie_description'
    rviz2_config_path = os.path.join(get_package_share_directory(package_name), 'rviz', 'check_robot_model.rviz')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    rviz2 = Node(package='rviz2', executable='rviz2',
                        arguments=['-d', rviz2_config_path,],
                        output='screen'
    )

    joint_state_publisher_gui = Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui')

    return LaunchDescription([
        rsp,
        rviz2,
        joint_state_publisher_gui,
    ])