import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name='edie_aruco'
    rviz2_config_path = os.path.join(get_package_share_directory(package_name), 'rviz', 'aruco_visualizer.rviz')

    aruco_params = os.path.join(
        get_package_share_directory('edie_aruco'),
        'config',
        'aruco_parameters.yaml'
        )

    aruco_detection_node = Node(
        package='edie_aruco',
        executable='aruco_detection_node',
        parameters=[aruco_params]
    )

    rviz_marker_visualize_node = Node(
        package='edie_aruco',
        executable='rviz_marker_visualize',
    )

    rviz2 = Node(package='rviz2', executable='rviz2',
                        arguments=['-d', rviz2_config_path,],
                        output='screen'
    )

    return LaunchDescription([
        aruco_detection_node,
        rviz_marker_visualize_node,
        # rviz2,
    ])