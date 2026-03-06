from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    pkg_dir = get_package_share_directory('aeirobot_asr')
    config_file = os.path.join(pkg_dir, 'config', 'audio_config.yaml')

    # config 파일 파라미터 선언
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the YAML config file'
    )

    # ASR 노드 실행
    asr_node = Node(
        package='aeirobot_asr',
        executable='asr_main',
        name='aeirobot_asr',
        parameters=[
            {
                'config_file': LaunchConfiguration('config_file'),
                'result_topic': '/aeirobot/asr/result',
                'language_speaker_topic': '/aeirobot/asr/language_speaker'
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
        asr_node
    ])
