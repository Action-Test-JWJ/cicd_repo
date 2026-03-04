from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 패키지 디렉터리 경로 가져오기
    package_dir = get_package_share_directory('edie_motion')

    # 설정 파일 경로
    param_file = os.path.join(package_dir, 'data', 'motion.yaml')

    # 노드 정의
    edie_motion_node = Node(
        package='edie_motion',
        executable='edie_motion_handler_node',
        output='screen',
        parameters=[param_file]
    )

    # LaunchDescription 반환
    return LaunchDescription([
        edie_motion_node,
    ])