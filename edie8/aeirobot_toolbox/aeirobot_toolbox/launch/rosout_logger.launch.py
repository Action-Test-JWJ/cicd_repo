from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from typing import List, Dict, Any, Optional

def process_yaml_and_create_node(context):
    # 런타임에 YAML 처리를 위한 함수

    # 현재 설정 가져오기 (데이터 형식 변환 추가)
    log_dir = LaunchConfiguration('log_dir').perform(context)
    log_level = int(LaunchConfiguration('log_level').perform(context))
    rollover_interval = float(LaunchConfiguration('rollover_interval').perform(context))
    # 불리언 형식으로 변환
    log_to_console = LaunchConfiguration('log_to_console').perform(context).lower() == 'true'
    use_colored_log = LaunchConfiguration('use_colored_log').perform(context).lower() == 'true'
    target_nodes_param = LaunchConfiguration('target_nodes').perform(context)
    log_group_param = LaunchConfiguration('log_group').perform(context)

    # 설정 파일 경로 가져오기
    config_file_path = LaunchConfiguration('config_file_path').perform(context)

    # 기본 노드 파라미터
    node_params = {
        'log_dir': log_dir,
        'log_level': log_level,
        'rollover_interval': rollover_interval,
        'log_to_console': log_to_console,
        'use_colored_log': use_colored_log,
        'target_nodes': target_nodes_param,
        'config_file_path': config_file_path
    }

    # 설정 파일 경로 결정 (파라미터로부터 또는 기본값)
    src_config_file = config_file_path if config_file_path else os.path.join(os.getcwd(), 'src', 'aeirobot_toolbox', 'aeirobot_toolbox', 'config', 'logger_configs.yaml')

    # 로그 그룹 가져오기
    log_groups = [g.strip() for g in log_group_param.split(',') if g.strip()]
    target_nodes = []

    if os.path.exists(src_config_file):
        try:
            with open(src_config_file, 'r') as f:
                config_data = yaml.safe_load(f)
                print(f"Found config file at {src_config_file}")

            # 그룹별 노드 목록 가져오기
            for group_name in log_groups:
                if group_name in config_data.get('log_groups', {}):
                    nodes = config_data['log_groups'][group_name].get('nodes', [])
                    if nodes:
                        target_nodes.extend(nodes)
                        print(f"Added {len(nodes)} nodes from group '{group_name}'")

            if target_nodes:
                print(f"Selected log groups: {log_groups} with {len(target_nodes)} nodes")
                # 배열 형태로 전달(문자열로 변환하지 않음)
                node_params['target_nodes'] = target_nodes
        except Exception as e:
            print(f"Warning: Error processing config file: {str(e)}")
    else:
        print(f"Config file not found: {src_config_file}")
        print("Using default target nodes from launch parameters.")

    # 노드 생성
    node = Node(
        package='aeirobot_toolbox',
        executable='rosout_logger',
        name='rosout_logger',
        output='screen',
        emulate_tty=True,
        parameters=[node_params]
    )

    return [node]

def generate_launch_description():
    # 기본 파라미터 선언
    config_file_path_arg = DeclareLaunchArgument(
        'config_file_path',
        default_value=os.path.join(os.getcwd(), 'src', 'aeirobot_toolbox', 'aeirobot_toolbox', 'config', 'logger_configs.yaml'),
        description='로거 설정 파일 경로'
    )

    log_dir_arg = DeclareLaunchArgument(
        'log_dir',
        default_value='',  # 빈 값이면 자동 생성
        description='로그 파일 저장 디렉토리, 비워두면 자동 생성됨'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='2',  # INFO=2
        description='로깅 레벨 (DEBUG=0, INFO=2, WARN=3, ERROR=4, FATAL=5)'
    )

    rollover_interval_arg = DeclareLaunchArgument(
        'rollover_interval',
        default_value='3600.0',  # 1시간
        description='로그 파일 롤오버 간격(초), 0이면 롤오버 안함'
    )

    log_to_console_arg = DeclareLaunchArgument(
        'log_to_console',
        default_value='false',
        description='로그를 콘솔에도 출력할지 여부'
    )

    # 로그 그룹
    log_group_arg = DeclareLaunchArgument(
        'log_group',
        default_value='localization,control',  # 기본적으로 로컬라이제이션과 컨트롤 그룹 사용
        description='로그할 노드 그룹. 콤마로 구분하여 여러 그룹 지정 가능'
    )

    use_colored_log_arg = DeclareLaunchArgument(
        'use_colored_log',
        default_value='false',
        description='로그에 ANSI 색상 코드 포함 여부'
    )

    target_nodes_arg = DeclareLaunchArgument(
        'target_nodes',
        default_value='[]',  # 기본값 빈 배열(모든 노드 로깅)
        description='파싱할 노드 목록, 예: "[\"node1\", \"node2\"]". 명령행에서는 단일 따옴표로 감싸서 전달'
    )

    # 로그 그룹에 따른 노드 목록 처리
    group_selector = LaunchConfiguration('log_group')
    target_nodes_param = LaunchConfiguration('target_nodes')

    # PythonExpression을 사용하여 로그 그룹에 따라 노드 목록 선택
    # 만약 로그 그룹이 설정되어 있으면 해당 그룹의 노드 목록을 사용
    # 그렇지 않으면 사용자가 직접 입력한 target_nodes 파라미터 사용

    # 기본 파라미터 초기화
    node_parameters = [{
        'log_dir': LaunchConfiguration('log_dir'),
        'log_level': LaunchConfiguration('log_level'),
        'rollover_interval': LaunchConfiguration('rollover_interval'),
        'log_to_console': LaunchConfiguration('log_to_console'),
        'use_colored_log': LaunchConfiguration('use_colored_log'),
        'target_nodes': LaunchConfiguration('target_nodes'),
        'config_file_path': LaunchConfiguration('config_file_path')
    }]

    # 동적 처리를 위해 런타임에 실행되는 기능은 모두 process_yaml_and_create_node 함수로 이동함

    # 동적 처리를 위한 OpaqueFunction 사용
    config_loader = OpaqueFunction(
        function=process_yaml_and_create_node
    )

    # 런치 설명 반환
    return LaunchDescription([
        config_file_path_arg,
        log_dir_arg,
        log_level_arg,
        rollover_interval_arg,
        log_to_console_arg,
        use_colored_log_arg,
        target_nodes_arg,
        log_group_arg,
        config_loader
    ])

# 사용 예시:
# ros2 launch aeirobot_toolbox rosout_logger.launch.py target_nodes:='["alice_localization_manager", "path_planner"]'
# ros2 launch aeirobot_toolbox rosout_logger.launch.py log_level:=4
# ros2 launch aeirobot_toolbox rosout_logger.launch.py log_dir:="/path/to/logs"
