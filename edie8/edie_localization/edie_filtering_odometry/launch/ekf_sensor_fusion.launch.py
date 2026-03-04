from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0.0285", "--y", "0","--z", "0.0732",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1", # No Rotation
                   "--frame-id", "base_link",     # chassis
                   "--child-frame-id", "imu_link"],
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("edie_filtering_odometry"), "config", "ekf.yaml")],
        # remappings=[
        #     ('/odometry/filtered', '/edie8/localization/ekf_odom')
        # ]
    )

    filtering_odometry = Node(
        package="edie_filtering_odometry",
        executable="edie_filtering_odom_node",
        output="screen",
    )

    republishing_odom = Node(
        package="edie_filtering_odometry",
        executable="edie_republishing_odom_node",
        output="screen",
    )

    correcting_odom = Node(
        package="edie_filtering_odometry",
        executable="edie_correcting_odom_node",
        output="screen",
    )
    
    return LaunchDescription([
        # static_transform_publisher, 
        robot_localization,
        republishing_odom,
        correcting_odom
        # filtering_odometry,
    ])
