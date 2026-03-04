from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    display_node = Node(
        package="edie_display", 
        executable="edie_display_node",
        output='screen'
    )
    
    motion_node = Node(
        package="edie_motion", 
        executable="edie_motion_handler_node",
        output='screen'
    )
    sound_node = Node(
        package="edie_sound", 
        executable="edie_sound_node"
    )
    action_node = Node(
        package="edie_action_handler", 
        executable="edie_action_handler_node",
        output='screen'
    )
    
    return LaunchDescription([
        display_node,
        motion_node,
        sound_node,
        action_node,
    ])
