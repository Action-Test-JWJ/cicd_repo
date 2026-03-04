from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        # 클라이언트
        Node(
            package='edie_emotion_detection_python',
            executable='emotion_detection_client',
            name='emotion_detection_client',
            output='screen',
            parameters=[{
                'image_topic': '/unused',                      # 비활성 입력
                'active': False,                               # 초기 비활성
                'show_window': False, #True,
                # 서로 fallback(대체제 관계)
                'infer_throttle_ms': 0.0, #100.0, # 최소 간격(ms) # 100[ms] = 10[Hz]
                'infer_target_hz': 0.0, #20.0, # 목표 주파수(Hz) # 이미지 토픽 30hz와 비교했을 때, 다운샘플링(10hz) 하거나 그대로 반영(30hz)
            }]
        ),
    ])
