#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    algorithm_arg     = DeclareLaunchArgument('algorithm', default_value='MLP',
                                        description='Algorithm for inference')
    max_num_faces_arg     = DeclareLaunchArgument('max_num_faces', default_value='1',
                                        description='Max number of faces')
    show_window_arg     = DeclareLaunchArgument('show_window', default_value='false',
                                        description='Show OpenCV window (true/false)')
    profile_infer_arg     = DeclareLaunchArgument('profile_infer', default_value='false',
                                        description='Profile inference (true/false)')
    draw_mesh_arg         = DeclareLaunchArgument('draw_mesh', default_value='true',
                                        description='Draw emotional mesh landmarks (true/false)')
    neutral_min_conf_arg  = DeclareLaunchArgument('neutral_min_conf', default_value='90.0',
                                        description='Neutral detection min confidence (%). Lower Conf than TH, Neutral')
    neutral_margin_arg    = DeclareLaunchArgument('neutral_margin_th', default_value='20.0', # 20.0 15.0
                                        description='Neutral detectionmin top1-top2 margin (%). Lower Marg than TH, Neutral')


    node = Node(
        package='ml_edie_emotion_detection',
        executable='edie_emotion_detection_node',
        name='ml_edie_emotion_detection',
        output='screen',
        parameters=[{
            'algorithm':        LaunchConfiguration('algorithm'),
            'max_num_faces': LaunchConfiguration('max_num_faces'),
            'show_window': LaunchConfiguration('show_window'),
            'profile_infer': LaunchConfiguration('profile_infer'),
            'neutral_min_conf': LaunchConfiguration('neutral_min_conf'),
            'neutral_margin_th': LaunchConfiguration('neutral_margin_th'),
            'draw_mesh': LaunchConfiguration('draw_mesh'),
        }],
    )

    return LaunchDescription([
        algorithm_arg,
        max_num_faces_arg,
        show_window_arg,
        profile_infer_arg,
        neutral_min_conf_arg,
        neutral_margin_arg,
        draw_mesh_arg,
        node
    ])


