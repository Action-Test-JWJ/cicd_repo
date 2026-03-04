from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 패키지 디렉터리 경로 가져오기
    package_dir = get_package_share_directory('edie_mobile')

    # 설정 파일 경로
    param_file = os.path.join(package_dir, 'config', 'edie_mobile_params.yaml')
    rviz_config_file = os.path.join(package_dir, 'rviz', 'edie_mobile.rviz')

    # 노드 정의
    edie_mobile_node = Node(
        package='edie_mobile',
        executable='edie_mobile_node',
        output='screen',
        parameters=[param_file]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # LaunchDescription 반환
    return LaunchDescription([
        edie_mobile_node,
        # rviz2
    ])
