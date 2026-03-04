import os
import rclpy
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch.substitutions import PathJoinSubstitution
from rclpy.node import Node as RCLNode
from controller_manager_msgs.srv import ListControllers
from launch.actions import IncludeLaunchDescription, ExecuteProcess

class ControllerChecker(RCLNode):
    def __init__(self):
        super().__init__('controller_checker')
        self.client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        while not self.client.wait_for_service(timeout_sec=1.0):
            pass
        self.req = ListControllers.Request()

    def get_loaded_controllers(self):
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            return self.future.result().controller
        else:
            return []

def generate_launch_description():

    pkg_name='edie8_gazebo'
    world_name='edie8_charging_test.world'
    world_file_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', world_name)
    # gazebo_params_file = os.path.join(get_package_share_directory(pkg_name),'config','gazebo_params.yaml')

    gazebo_models_path = os.path.join(get_package_share_directory("edie8_gazebo"), "models")
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ":" + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_models_path

    # Gazebo의 모델 경로 추가
    os.environ['GAZEBO_MODEL_PATH'] += ":/usr/share/gazebo-11/models"

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(pkg_name),'launch','rsp_gazebo.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    #     launch_arguments={'world': world_file_path}.items()
    # )
    gazebo = ExecuteProcess( 
        cmd=['gazebo', '--verbose', world_file_path, '-s', 'libgazebo_ros_init.so',  
        '-s', 'libgazebo_ros_factory.so'], 
        output='screen', 
    )

    mobile_pkg_path = get_package_share_directory('edie_mobile')
    mobile_param_file = os.path.join(mobile_pkg_path, 'config', 'edie_mobile_params_sim.yaml')

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('edie8_gazebo'), 'launch', 'spawn_robot.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["edie_diff_drive_controller"],
    )

    joint_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('edie_imu'),
                'launch',
                'edie_imu_calibration.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    #=================
    # Core Logic
    # =================
    # 동작을 결정하는 BehaviorTree 노드
    behavior_node = Node(
        package="edie_behavior", 
        executable="edie_behavior_node",
        parameters=[{
            'sim_mode': True  # Gazebo 시뮬레이션에서는 sim mode 활성화
        }]
    )

    # BT의 네비게이션 명령을 처리하는 액션 서버
    edie_mobile_action_node = Node(
        package='edie_mobile_action',
        executable='edie_mobile_action_node',
    )

    edie_mobile_node = Node(
        package='edie_mobile',
        executable='edie_mobile_node',
        output='screen',
        parameters=[mobile_param_file]
    )

    # Docking launch
    docking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('edie8_docking'),
                'launch',
                'edie8_docking.launch.py'
            )
        ])
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        # behavior_node,
        edie_mobile_node,
        imu_launch,
        edie_mobile_action_node,
        # yolov8_launch,
        diff_drive_spawner,
        joint_broadcaster_spawner,
        docking_launch,
    ])