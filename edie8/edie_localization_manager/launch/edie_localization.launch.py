from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    # =================
    # Localization
    # =================
    # 아루코 마커를 이용해 로봇의 절대 위치를 추정하는 노드
    aruco_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('edie_aruco'),
                'launch',
                'aruco_detection_only_pose.launch.py'
            ])
        ])
    )

    # IMU와 Odometry 데이터를 EKF로 퓨전하여 주행 정보를 정밀하게 보정
    ekf_sensor_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('edie_filtering_odometry'),
                'launch',
                'ekf_sensor_fusion.launch.py'
            ])
        ])
    )

    # EKF 산출 토픽 대기 (토픽명은 환경에 맞게)
    ekf_ready = ExecuteProcess(
        cmd=['bash', '-lc',
            'until ros2 topic list | grep -E -q "/edie8/localization/ekf_odom"; do sleep 1.0; done'],
        output='screen'
    )

    # 다양한 위치 추정 소스를 융합하고 최종적인 로봇 위치를 결정하는 매니저 노드
    localization_manager_node = Node(
        package='edie_localization_manager',
        executable='edie_localization_manager_node',
        name='edie_localization_manager',
        parameters=[{'sim_mode': True}]  # or False for real mode
    )


    return LaunchDescription([
        aruco_localization_launch,  
        # ekf_sensor_fusion_launch, # EKF 포함 런치
        ekf_ready,
        # waiter 종료 → 4초 지연 → manager 실행
        RegisterEventHandler(
            OnProcessExit(
                target_action=ekf_ready,
                on_exit=[TimerAction(period=4.0, actions=[localization_manager_node])]
            )
        ),
    ])
