from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch.event_handlers import OnProcessExit, OnExecutionComplete, OnProcessStart
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration, FindExecutable
from launch.conditions import LaunchConfigurationEquals
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction

import os

def generate_launch_description():
    # =================
    # Paths and Files
    # =================
    mobile_pkg_path = get_package_share_directory('edie_mobile')
    mobile_param_file = os.path.join(mobile_pkg_path, 'config', 'edie_mobile_params.yaml')
    robot_controllers = PathJoinSubstitution([
            FindPackageShare('edie_bringup'),
            "config",
            "controllers.yaml"
        ]
    )

    # =================
    # Robot Model
    # =================
    # robot_state_publisher를 실행하여 URDF 모델을 /robot_description 토픽으로 게시합니다.
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('edie_description'),
                'launch',
                'rsp.launch.py'
            ])
        ])
    )

    #  robot_description
    robot_description_content = Command([
        'xacro ',
        # FindExecutable(name='xacro'),
        PathJoinSubstitution([
            FindPackageShare('edie_description'),
            'urdf/real_robot/edie8.urdf.xacro',
        ]),
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    
    # =================
    # Core Logic
    # =================
    # 동작을 결정하는 BehaviorTree 노드
    behavior_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('edie_behavior'),
                'launch',
                'edie_behavior.launch.py'
            ])
        ])
    )
    
    # 감정, 디스플레이, 모션, 사운드 등 다양한 액션 서버를 실행하는 런치 파일
    edie_action_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('edie_bringup'),
                'launch',
                'edie_action.launch.py'
            ])
        ])
    )

    # BT의 네비게이션 명령을 처리하는 액션 서버
    edie_mobile_action_node = Node(
        package='edie_mobile_action',
        executable='edie_mobile_action_node',
    )


    # 주행 명령 발행 노드
    edie_mobile_node = Node(
        package='edie_mobile',
        executable='edie_mobile_node',
        output='screen',
        parameters=[mobile_param_file]
    )

    # =================
    # Sensor
    # =================
    # 카메라 드라이버 노드
    camera_node = Node(
        package="edie_camera", 
        executable="edie_camera_pub", 
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('edie_camera'),
                'config',
                'config.yaml'
            ])
        ]
    )
    
    # 레이저 노드 
    laser_front_node = Node(
        package="edie_laser", 
        executable="edie_laser_node_vl53l0x"
    )
    laser_bottom_node = Node(
        package="edie_laser", 
        executable="edie_laser_node_vl6180x"
    )

    # IMU 센서 드라이버 및 보정 노드
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('edie_imu'),
                'launch',
                'edie_imu_calibration.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    icm20948_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('icm20948_ros2'),
                'launch',
                'icm20948_wrapper_init.launch.py'
            ])
        ])
    )
    
    # FSR 필터링/상태머신 반환 노드
    fsr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('edie_fsr'),
                'launch',
                'edie_fsr_sensor.launch.py'
            ])
        ])
    )
    # 오디오 녹음 노드
    audio_record_node = Node(
        package="edie8_listen", 
        executable="sound_record_node"
    )
    # stt 실행 노드
    wake_up_node = Node(
        package="edie8_listen", 
        executable="wake_up_node"
    )
    stt_node = Node(
        package="edie8_listen", 
        executable="stt_node"
    )

    
    # =================
    # Human Interface
    # =================
    # 조이스틱 드라이버 노드
    joystick_node = Node(
        package="joystick_ros2", 
        executable="joystick_ros2"
    )

    # ROS2 Control
    # =================
    # 컨트롤러를 로드하고 관리하는 controller_manager 노드
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            robot_controllers,
            {"controller_manager.controller_activation_timeout": 30.0}
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        on_exit=Shutdown(),
    )

    # control_node가 시작된 후 컨트롤러들을 순차적으로 로드하고 활성화
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
        #output='screen'
    )
    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'edie_diff_drive_controller'],
        #output='screen'
    )
    load_l_leg_joint_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'edie_l_leg_position_controller'],
        #output='screen'
    )
    load_r_leg_joint_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'edie_r_leg_position_controller'],
        #output='screen'
    )
    load_l_ear_joint_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'edie_l_ear_position_controller'],
        #output='screen'
    )
    load_r_ear_joint_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'edie_r_ear_position_controller'],
        #output='screen'
    )
    load_leg_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'edie_leg_trajectory_controller'],
        #output='screen'
    )
    load_ear_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'edie_ear_trajectory_controller'],
        #output='screen'
    )


    return LaunchDescription([
        # 로봇 모델
        rsp_launch,
        
        # bt 및 액션 서버
        behavior_launch,
        edie_action_launch,
        edie_mobile_action_node,

        # 주행 노드
        edie_mobile_node,

        # 센서 및 인식
        camera_node,
        laser_front_node, 
        laser_bottom_node,
        icm20948_launch,
        imu_launch,
        fsr_launch,

        # audio_record_node,
        # wake_up_node,
        # stt_node,

        # 사용자 입력
        joystick_node,

        # ROS2 컨트롤
        control_node,
        # 컨트롤러 로드를 위한 이벤트 핸들러
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=control_node,
                on_start=[load_diff_drive_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_diff_drive_controller,
                on_exit=[load_l_leg_joint_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_l_leg_joint_controller,
                on_exit=[load_r_leg_joint_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_r_leg_joint_controller,
                on_exit=[load_l_ear_joint_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_l_ear_joint_controller,
                on_exit=[load_r_ear_joint_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_r_ear_joint_controller,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_leg_joint_trajectory_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_leg_joint_trajectory_controller,
                on_exit=[load_ear_joint_trajectory_controller],
            )
        ),

    ])