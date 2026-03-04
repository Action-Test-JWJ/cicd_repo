from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='edie_motion',
            executable='edie_motion_node',
        ),
        Node(
            package='edie_motion',
            executable='edie_motion_joy_controller_node',
        ),
        Node(
            package='joystick_ros2',
            executable='joystick_ros2',
        )
    ])