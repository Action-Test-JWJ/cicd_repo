from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the default path to the parameters file
    default_params_file = os.path.join(
        get_package_share_directory('edie_behavior'),
        'config',
        'behavior_parameters.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_mode',
            default_value='false',
            description='Enable simulation mode.'
        ),
        DeclareLaunchArgument(
            'bt_file',
            default_value='edie_bt_robotworld_2025.xml',
            description='Name of the behavior tree XML file.'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the parameters file'
        ),
        DeclareLaunchArgument(
            'laser_stop_distance_mm',
            default_value='400.0',
            description='Distance Threshold to stop the robot.'
        ),
        DeclareLaunchArgument(
            'laser_release_distance_mm',
            default_value='450.0',
            description='Distance Threshold to release the robot.'
        ),
        DeclareLaunchArgument(
            'laser_fresh_timeout',
            # 값↑: 최신성 느슨(오검 증가)하지만 짧은 센서 끊김에 덜 민감.
            # 값↓: 최신성 엄격(오검 감소)하지만 짧은 센서 끊김에 민감
            default_value='0.3',#'0.1',
            description='Timeout for laser data freshness.'
        ),
        DeclareLaunchArgument(
            'min_roi_width',
            default_value='120',
            description='Minimum ROI width to consider a human target (pixels).'
        ),
        DeclareLaunchArgument(
            'robot_mode',
            default_value='remote',
            description='Default robot mode.'
        ),
        Node(
            package='edie_behavior',
            executable='edie_behavior_node',
            parameters=[
                {
                    'sim_mode': LaunchConfiguration('sim_mode'),
                    'bt_file': LaunchConfiguration('bt_file'),
                    'laser_stop_distance_mm': LaunchConfiguration('laser_stop_distance_mm'),
                    'laser_release_distance_mm': LaunchConfiguration('laser_release_distance_mm'),
                    'laser_fresh_timeout': LaunchConfiguration('laser_fresh_timeout'),
                    'min_roi_width': LaunchConfiguration('min_roi_width'),
                    'robot_mode': LaunchConfiguration('robot_mode'),
                },
                LaunchConfiguration('params_file')  # Load the parameters file
            ],
            output='screen'
        ),
    ])