import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    package_name='edie_aruco'
    rviz2_config_path = os.path.join(get_package_share_directory(package_name), 'rviz', 'aruco_visualizer.rviz')

    # 시뮬레이션 모드 파라미터 선언
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Set to true for simulation mode'
    )

    # 시뮬레이션 모드에 따라 파라미터 파일 선택
    sim_mode = LaunchConfiguration('sim_mode')
    
    # 파라미터 파일 경로 설정
    aruco_params_real = os.path.join(
        get_package_share_directory('edie_aruco'),
        'config',
        'aruco_parameters.yaml'  # 실제 로봇용
    )
    
    aruco_params_sim = os.path.join(
        get_package_share_directory('edie_aruco'),
        'config',
        'aruco_parameters_sim.yaml'  # 시뮬레이션용
    )

    # 실제 로봇용 노드
    aruco_detection_node_real = Node(
        package='edie_aruco',
        executable='aruco_detection_only_pose_node',
        parameters=[aruco_params_real, {'sim_mode': False}],
        condition=UnlessCondition(sim_mode),
        name='aruco_detection_node'
    )
    
    # 시뮬레이션용 노드
    aruco_detection_node_sim = Node(
        package='edie_aruco',
        executable='aruco_detection_only_pose_node',
        parameters=[aruco_params_sim, {'sim_mode': True}],
        condition=IfCondition(sim_mode),
        name='aruco_detection_node'
    )

    rviz_marker_visualize_node = Node(
        package='edie_aruco',
        executable='rviz_marker_visualize',
    )

    rviz2 = Node(package='rviz2', executable='rviz2',
                        arguments=['-d', rviz2_config_path,],
                        output='screen'
    )

    return LaunchDescription([
        sim_mode_arg,
        aruco_detection_node_real,
        aruco_detection_node_sim,
        # rviz_marker_visualize_node,
        # rviz2,
    ])