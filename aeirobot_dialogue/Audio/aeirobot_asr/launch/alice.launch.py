from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로 가져오기
    pkg_dir = get_package_share_directory('aeirobot_asr')

    # 기본 config 파일 경로
    default_config_path = os.path.join(pkg_dir, 'config', 'audio_config.yaml')

    # Launch 파라미터 선언
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path,
        description='Path to the config file'
    )

    # Config 파일 로드
    config = os.path.join(
        get_package_share_directory('aeirobot_asr'),
        'config',
        'audio_config.yaml'
    )

    # STT 노드 실행
    stt_node = Node(
        package='aeirobot_asr',
        executable='asr_main',
        name='alice_asr',
        parameters=[
            config,  # Load the YAML file directly
            {
                'config_path': LaunchConfiguration('config_path'),
                'result_topic': '/aeirobot/alice/asr_result',
                'language_speaker_topic': '/aeirobot/alice/asr_language'
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        config_path_arg,
        stt_node
    ])
