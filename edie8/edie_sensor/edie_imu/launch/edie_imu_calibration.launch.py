from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition,UnlessCondition
import os

def generate_launch_description():
    package_name = 'edie_imu'

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 런치 인자를 선언하는 액션을 리스트에 추가합니다.
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Set to "true" to use simulation time'
    )

    # 변환 정보 설정 (에디 8.3 기준)
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0.0285", "--y", "0","--z", "0.0732",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1", # No Rotation
                   "--frame-id", "base_link",     
                   "--child-frame-id", "imu_link"],
    )

    # 실제 환경일 때는 별도 오버라이드 없이 기본값(/edie8/sensor/offset_imu) 사용
    real_edie_imu_node = Node(
        package=package_name, 
        executable='edie_imu_node', 
        output="screen",
        parameters=[
            {'imu_topic': '/edie8/sensor/offset_imu'},
            {'use_sim_time': False},
        ],
        condition=UnlessCondition(use_sim_time),
    )
    # use_sim_time이 true일 때만 실행 (즉, 시뮬레이션 환경)
    sim_edie_imu_node = Node(
        package=package_name,
        executable='edie_imu_node',
        output="screen",
        parameters=[{"imu_topic": "/edie8/sensor/imu"},
                    {'use_sim_time': True},
        ],
        condition=IfCondition(use_sim_time)
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        static_transform_publisher,
        sim_edie_imu_node,
        real_edie_imu_node
    ])