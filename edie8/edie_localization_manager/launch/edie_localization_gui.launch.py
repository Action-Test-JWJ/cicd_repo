#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PythonExpression

def generate_launch_description():
    # 'mode'라는 이름의 런치 인자를 선언합니다.
    # 기본값은 'udp'이고, 'ros' 또는 'udp'를 선택할 수 있습니다.
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='ros',
        description="GUI 실행 모드 선택: 'ros' /'udp' "
    )

    edie_localization_manager = Node(
                    package='edie_localization_manager',
                    executable='edie_localization_manager_node',
                    arguments=[],
                    output="screen",
    )

    # create and return launch description object
    return LaunchDescription(
    [
        mode_arg,
        edie_localization_manager
    ]
)
